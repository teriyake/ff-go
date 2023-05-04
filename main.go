package main

import (
	"fmt"
	"github.com/dominikbraun/graph"
	"github.com/dominikbraun/graph/draw"
	_ "github.com/fogleman/gg"
	"github.com/fogleman/ln/ln"
	"math"
	"os"
	"strings"
	"time"
)

type Tr struct {
	V1     ln.Vector
	V2     ln.Vector
	V3     ln.Vector
	Normal ln.Vector
}

type HalfEdge struct {
	Start ln.Vector
	End   ln.Vector
}

func printTr(tr ln.Triangle) string {
	return fmt.Sprintf("%v, %v, %v", tr.V1, tr.V2, tr.V3)
}

func getNormal(v1, v2, v3 ln.Vector) ln.Vector {
	vA := v2.Add(v1.MulScalar(-1))
	vB := v3.Add(v1.MulScalar(-1))
	vN := vA.Cross(vB)
	return vN
}

func unitCross(a, b ln.Vector) ln.Vector {
	return a.Cross(b).Div((a.Cross(b)).Normalize())
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

func parseMesh(m ln.Mesh) ([]Tr, map[ln.Vector]int, map[Tr]int, map[int]Tr, map[HalfEdge]int, map[int][]HalfEdge) {
	var triangles []Tr
	var vertices = make(map[ln.Vector]int)
	var trVertices = make(map[Tr]int)
	var trByVertex = make(map[int]Tr)
	var halfedges = make(map[HalfEdge]int)
	var heByFace = make(map[int][]HalfEdge)

	v := 0
	trV := 0
	for _, tr := range m.Triangles {
		trSN := getNormal(tr.V1, tr.V2, tr.V3)
		trS := Tr{tr.V1, tr.V2, tr.V3, trSN}
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
		heByFace[trV] = append(heByFace[trV], he1, he2, he3)
		trV++
	}
	return triangles, vertices, trVertices, trByVertex, halfedges, heByFace
}

func getAngle(v1, v2 ln.Vector) float64 {
	return math.Acos(v1.Normalize().Dot(v2.Normalize())) * 180 / math.Pi
}

func getAnglesTr(tr *ln.Triangle) []float64 {
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

func rotateTr(tr Tr, ref ln.Vector, rAngle float64) *ln.Triangle {
	rAxis := tr.Normal.Normalize().Cross(ref.Normalize())
	rMat := ln.Rotate(rAxis, rAngle)
	v1 := rMat.MulPosition(tr.V1)
	v2 := rMat.MulPosition(tr.V2)
	v3 := rMat.MulPosition(tr.V3)
	return ln.NewTriangle(v1, v2, v3)
}

func translateTr(tr *ln.Triangle, ref ln.Vector) *ln.Triangle {
	//recover halfedge position first

	tMat := ln.Translate(ref)

	v1 := tMat.MulPosition(tr.V1)
	v2 := tMat.MulPosition(tr.V2)
	v3 := tMat.MulPosition(tr.V3)
	return ln.NewTriangle(v1, v2, v3)
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

func drawCreasePattern(mst []graph.Edge[int], vTr map[int]Tr, adj map[int][]int, he map[int][]HalfEdge, f string) {
	scene := ln.Scene{}

	refT := vTr[mst[0].Source]
	refP := refT.Normal
	fmt.Println(refP)

	//trPrv := ln.NewTriangle(refT.V1, refT.V2, refT.V3)
	drawn := make(map[int]bool)
	rotations := make(map[Tr]float64)

	for _, e := range mst {
		fmt.Printf("%v--%v\n", e.Source, e.Target)
		if _, ok := drawn[e.Target]; ok {
			if _, ok := drawn[e.Source]; ok {
				continue
			}
		}
		tr1 := vTr[e.Source]
		tr2 := vTr[e.Target]
		trs := adj[e.Target]
		for _, tr := range trs {
			if vTr[tr] != tr1 {
				// unfold
				fmt.Println(tr)
			}
		}
		// draw
		// rotating one changes successive trs... maybe recursion??
		// or maybe check adjacency & cut between f1 & f2 is f1 in MST & f2 not? then unfold
		heTr1 := he[e.Source]
		heTr2 := he[e.Target]
		var sharedE HalfEdge
		intersectM := make(map[HalfEdge]bool)
		for _, he1 := range heTr1 {
			intersectM[he1] = true
		}
		for _, he2 := range heTr2 {
			heC := HalfEdge{he2.End, he2.Start}
			if _, ok := intersectM[heC]; ok {
				sharedE = heC
				break
			}
		}
		fmt.Println(sharedE)

		// then use shared E as rAxis
		// diff rAxis for diff tr? direction...

		// another idea:
		// make a map; each time something rotates, add it to the map; each rotation is done after checking the map & get updated position if necessary
		if _, ok := drawn[e.Source]; !ok {
			tr1N := ln.NewTriangle(tr1.V1, tr1.V2, tr1.V3)
			if (refT != tr1) || (tr1.Normal != refP) {
				tr1A := getAngle(refP, tr1.Normal)
				rotations[tr1] = tr1A
				tr1N = rotateTr(tr1, refP, tr1A)
				//tr1N = translateTr(tr1N, ln.RandomUnitVector())
				fmt.Printf("tr1 rotated by: %v\n", tr1A)
			}
			scene.Add(tr1N)
			drawn[e.Source] = true
		}
		if _, ok := drawn[e.Target]; !ok {
			tr2N := ln.NewTriangle(tr2.V1, tr2.V2, tr2.V3)
			if tr2.Normal != refP {
				tr2A := getAngle(refP, tr2.Normal)
				rotations[tr2] = tr2A 
				tr2N = rotateTr(tr2, refP, tr2A)
				//tr2N = translateTr(tr2N, ln.RandomUnitVector())
				fmt.Printf("tr2 rotated by: %v\n", tr2A)
			}

			scene.Add(tr2N)
			drawn[e.Target] = true
		}
	}
	eye := ln.Vector{3, 3, 3}
	center := ln.Vector{0, 0, 0}
	up := refP
	width := 1024.0
	height := 1024.0
	paths := scene.Render(eye, center, up, width, height, 50, 0.1, 100, 0.01)
	paths.WriteToPNG(f, width, height)

	fmt.Println(rotations)
}

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

	triangles, vertices, trVertices, trByV, halfedges, heByF := parseMesh(*mesh)
	fmt.Println(len(triangles))
	fmt.Println(len(vertices))
	fmt.Println(len(trVertices))
	fmt.Println(len(trByV))
	fmt.Println(len(mesh.Triangles))
	fmt.Println(len(halfedges))

	adjFaces := getAdjacencyV(halfedges)
	fmt.Println(len(adjFaces))
	for k, v := range adjFaces {
		fmt.Printf("%v", k)
		for _, ele := range v {
			fmt.Printf("--%v", ele)
		}
		fmt.Println()
	}

	adjF := "output/" + tsF
	mst := findNet(adjFaces, adjF)
	fmt.Println(mst)

	//fmt.Println(halfedges)

	creaseF := "output/" + tsF + "-crease.png"
	drawCreasePattern(mst, trByV, adjFaces, heByF, creaseF)

}
