package main

import (
	"fmt"
	"github.com/dominikbraun/graph"
	"github.com/dominikbraun/graph/draw"
	"github.com/fogleman/ln/ln"
	_"github.com/fogleman/gg"
	"math"
	"os"
	"strings"
	"time"
)

type Tr struct {
	V1 ln.Vector
	V2 ln.Vector
	V3 ln.Vector
}

type HalfEdge struct {
	Start ln.Vector
	End   ln.Vector
}

func printTr(tr ln.Triangle) string {
	return fmt.Sprintf("%v, %v, %v", tr.V1, tr.V2, tr.V3)
}

func removeDups(s []int) []int {
	keys := make(map[int]bool)
	ret := []int{}
	for _, ele := range s {
		if _, v := keys[ele]; !v {
			keys[ele] = true
			ret = append(ret, ele)
		}
	}
	return ret
}

func parseMesh(m ln.Mesh) ([]Tr, map[ln.Vector]int, map[Tr]int, map[int]Tr, map[HalfEdge]int) {
	var triangles []Tr
	var vertices = make(map[ln.Vector]int)
	var trVertices = make(map[Tr]int)
	var trByVertex = make(map[int]Tr)
	var halfedges = make(map[HalfEdge]int)

	v := 0
	trV := 0
	for _, tr := range m.Triangles {
		trS := Tr{tr.V1, tr.V2, tr.V3}
		triangles = append(triangles, trS)
		//get triangle vertex
		if _, ok := trVertices[trS]; !ok {
			trVertices[trS] = trV
			trByVertex[trV] = trS
			trV++
		}
		// get vertices
		if _, ok := vertices[tr.V1]; !ok {
			vertices[tr.V1] = v
			v++
		}
		if _, ok := vertices[tr.V2]; !ok {
			vertices[tr.V2] = v
			v++
		}
		if _, ok := vertices[tr.V3]; !ok {
			vertices[tr.V3] = v
			v++
		}
		// get half-edges for current triangle
		// TODO: twins
		he1 := HalfEdge{tr.V1, tr.V2}
		he2 := HalfEdge{tr.V2, tr.V3}
		he3 := HalfEdge{tr.V3, tr.V1}
		trV--
		halfedges[he1] = trV
		halfedges[he2] = trV
		halfedges[he3] = trV
		trV++
	}
	return triangles, vertices, trVertices, trByVertex, halfedges
}

func getAngles(tr *ln.Triangle) []float64 {
	AB := tr.V2.Add(tr.V1.MulScalar(-1))
	AC := tr.V3.Add(tr.V1.MulScalar(-1))
	angleA := math.Acos(AB.Dot(AC)/(AB.Length()*AC.Length())) * 180 / math.Pi
	// (A-B)*(C-B)/|A-B|*|C-B| = cos(<ABC)
	BA := tr.V1.Add(tr.V2.MulScalar(-1))
	BC := tr.V3.Add(tr.V2.MulScalar(-1))
	angleB := math.Acos(BA.Dot(BC)/(BA.Length()*BC.Length())) * 180 / math.Pi

	CA := tr.V1.Add(tr.V3.MulScalar(-1))
	CB := tr.V2.Add(tr.V3.MulScalar(-1))
	angleC := math.Acos(CA.Dot(CB)/(CA.Length()*CB.Length())) * 180 / math.Pi

	angles := []float64{angleA, angleB, angleC}

	return angles
}

func getAdjacencyV(he map[HalfEdge]int) map[int][]int {
	var ret = make(map[int][]int)

	for k, v := range he {
		tr1V := v
		twin := HalfEdge{k.End, k.Start}
		if twinF, ok := he[twin]; ok {
			tr2V := twinF
			if _, ok := ret[tr1V]; ok {
				ret[tr1V] = append(ret[tr1V], tr2V)
			} else {
				ret[tr1V] = []int{tr2V}
			}
			//repeat for tr2?
			if _, ok := ret[tr2V]; ok {
				ret[tr2V] = append(ret[tr2V], tr1V)
			} else {
				ret[tr2V] = []int{tr1V}
			}
		} else {
			continue
		}
	}

	for k, v := range ret {
		v = removeDups(v)
		ret[k] = v
	}

	return ret
}

func findNet(adj map[int][]int, f string) []graph.Edge[int] {
	g := graph.New(graph.IntHash)
	// find dual
	for k, v := range adj {
		_ = g.AddVertex(k)
		for _, e := range v {
			_ = g.AddVertex(e)
			_ = g.AddEdge(k, e)
		}
	}

	gF := f + "-dual.gv"
	gOut, _ := os.Create(gF)
	_ = draw.DOT(g, gOut)
	//find mst
	mstF := f + "-mst.gv"
	mst, err := graph.MinimumSpanningTree(g)
	if err != nil {
		panic(err)
	} else {
		mstOut, _ := os.Create(mstF)
		_ = draw.DOT(mst, mstOut)
		mstEdges, _ := mst.Edges()
		return mstEdges
	}
}

/*
func drawCreasePattern(mst []graph.Edge[int], vTr map[int]Tr, f string) {

	dc := gg.NewContext(1000, 1000)
    dc.SetRGB(0, 0, 0)
	dc.Translate(500, 500)
	rF := 1000 * 0.5
	for _, e := range mst {
		fmt.Printf("%v--%v\n", e.Source, e.Target)
		tr1 := vTr[e.Source]
		tr2 := vTr[e.Target]
		//draw tr1
		dc.DrawLine(tr1.V1.X*resize, tr1.V1.Y*resize, tr1.V2.X*resize, tr1.V2.Y*resize)
		dc.Stroke()
		dc.DrawLine(tr1.V2.X*resize, tr1.V2.Y*resize, tr1.V3.X*resize, tr1.V3.Y*resize)
		dc.Stroke()
		dc.DrawLine(tr1.V3.X*resize, tr1.V3.Y*resize, tr1.V1.X*resize, tr1.V1.Y*resize)
		dc.Stroke()
		//draw tr2
		dc.DrawLine(tr2.V1.X*resize, tr2.V1.Y*resize, tr2.V2.X*resize, tr2.V2.Y*resize)
		dc.Stroke()
		dc.DrawLine(tr2.V2.X*resize, tr2.V2.Y*resize, tr2.V3.X*resize, tr2.V3.Y*resize)
		dc.Stroke()
		dc.DrawLine(tr2.V3.X*resize, tr2.V3.Y*resize, tr2.V1.X*resize, tr2.V1.Y*resize)
		dc.Stroke()
	}


    dc.SavePNG(f)
}
*/

func main() {
	scene := ln.Scene{}
	mesh, err := ln.LoadOBJ("input/cube.obj")
	if err != nil {
		panic(err)
	}
	mesh.UnitCube()
	scene.Add(ln.NewTransformedShape(mesh, ln.Rotate(ln.Vector{0, 1, 0}, 0.5)))

	eye := ln.Vector{2, 1, 2}    // camera position
	center := ln.Vector{0, 0, 0} // camera looks at
	up := ln.Vector{0, 0, 1}     // up direction

	// define rendering parameters
	width := 1024.0  // rendered width
	height := 1024.0 // rendered height
	fovy := 50.0     // vertical field of view, degrees
	znear := 0.1     // near z plane
	zfar := 10.0     // far z plane
	step := 0.01     // how finely to chop the paths for visibility testing

	// compute 2D paths that depict the 3D scene
	paths := scene.Render(eye, center, up, width, height, fovy, znear, zfar, step)

	// render the paths in an image
	ts := time.Now().UTC().Format(time.RFC3339)
	tsF := strings.Replace(strings.Replace(ts, ":", "", -1), "-", "", -1)
	meshF := "output/" + tsF + "-mesh.png"
	paths.WriteToPNG(meshF, width, height)

	triangles, vertices, trVertices, trByV, halfedges := parseMesh(*mesh)
	fmt.Println(len(triangles))
	fmt.Println(len(vertices))
	fmt.Println(len(trVertices))
	fmt.Println(len(trByV))
	fmt.Println(len(mesh.Triangles))
	fmt.Println(len(halfedges))

	adjV := getAdjacencyV(halfedges)
	fmt.Println(len(adjV))
	for k, v := range adjV {
		fmt.Printf("%v", k)
		for _, ele := range v {
			fmt.Printf("--%v", ele)
		}
		fmt.Println()
	}

	adjF := "output/" + tsF
	mst := findNet(adjV, adjF)
	fmt.Println(mst)

	fmt.Println(halfedges)
	/*
	creaseF := "output/" + tsF + "-crease.png"
	drawCreasePattern(mst, trByV, creaseF)
	*/
}
