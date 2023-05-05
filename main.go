package main

import (
	"encoding/json"
	"fmt"
	"github.com/dominikbraun/graph"
	"github.com/dominikbraun/graph/draw"
	"github.com/fogleman/gg"
	"github.com/fogleman/ln/ln"
	"io/ioutil"
	"math"
	"math/rand"
	"os"
	"strconv"
	"strings"
	"time"
)

const (
	W = 1000.0
	H = 1000.0
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

type Edge2D struct {
	X1 float64 `json:"x1"`
	Y1 float64 `json:"y1"`
	X2 float64 `json:"x2"`
	Y2 float64 `json:"y2"`
	He HalfEdge
}

type Tr2D struct {
	E1 Edge2D `json: "e1"`
	E2 Edge2D `json: "e2"`
	E3 Edge2D `json: "e3"`
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

func contains(s []int, v int) bool {
	for _, e := range s {
		if v == e {
			return true
		}
	}
	return false
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

func getSharedEdge(heTr1, heTr2 []HalfEdge, prevTr Tr2D) (Edge2D, int, []HalfEdge) {
	var sharedE HalfEdge
	var usedE []HalfEdge

	intersectM := make(map[HalfEdge]bool)

	for _, he1 := range heTr1 {
		intersectM[he1] = true
	}
	for _, he2 := range heTr2 {
		heC := HalfEdge{he2.End, he2.Start}
		if _, ok := intersectM[heC]; ok {
			sharedE = heC
			usedE = append(usedE, heC)
			break
		}
	}
	sharedE = HalfEdge{sharedE.End, sharedE.Start}
	usedE = append(usedE, sharedE)

	// find the Edge in prevTr that has the same sharedE; get its coords
	prevE1 := prevTr.E1
	prevE2 := prevTr.E2
	prevE3 := prevTr.E3
	if prevE1.He == sharedE {
		return prevE1, 1, usedE
	}
	if prevE2.He == sharedE {
		return prevE2, 2, usedE
	}
	if prevE3.He == sharedE {
		return prevE3, 3, usedE
	}
	fmt.Println("unable to find he")
	return Edge2D{}, -1, []HalfEdge{}
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

func getAngle(v1, v2 ln.Vector) (float64, float64) {
	rad := math.Acos(v1.Normalize().Dot(v2.Normalize()))
	deg := rad * 180 / math.Pi
	return rad, deg
}

func getAnglesTr(tr Tr) []float64 {
	AB := tr.V2.Add(tr.V1.MulScalar(-1))
	AC := tr.V3.Add(tr.V1.MulScalar(-1))
	angleA := math.Acos(AB.Dot(AC) / (AB.Length() * AC.Length()))
	// (A-B)*(C-B)/|A-B|*|C-B| = cos(<ABC)
	BA := tr.V1.Add(tr.V2.MulScalar(-1))
	BC := tr.V3.Add(tr.V2.MulScalar(-1))
	angleB := math.Acos(BA.Dot(BC) / (BA.Length() * BC.Length()))

	CA := tr.V1.Add(tr.V3.MulScalar(-1))
	CB := tr.V2.Add(tr.V3.MulScalar(-1))
	angleC := math.Acos(CA.Dot(CB) / (CA.Length() * CB.Length()))

	angles := []float64{angleA, angleB, angleC}

	return angles
}

func getAngleTr(tr Tr, n int) float64 {
	angles := getAnglesTr(tr)

	return angles[n-1]
}

func getXY(oX, oY float64, a float64, l float64) (float64, float64) {
	x := math.Cos(a)*l + oX
	y := math.Sin(a)*l + oY
	return x, y
}

func rrrrrotateTr(tr Tr, ref ln.Vector, rAngle float64) *ln.Triangle {
	rAxis := tr.Normal.Cross(ref)
	rAxis = rAxis.Normalize()
	v1 := tr.V1.MulScalar(math.Cos(rAngle)).Add(rAxis.Cross(tr.V1)).Add(rAxis.MulScalar(rAxis.Dot(tr.V1)).MulScalar(1 - math.Cos(rAngle)))
	v2 := tr.V2.MulScalar(math.Cos(rAngle)).Add(rAxis.Cross(tr.V2)).Add(rAxis.MulScalar(rAxis.Dot(tr.V2)).MulScalar(1 - math.Cos(rAngle)))
	v3 := tr.V2.MulScalar(math.Cos(rAngle)).Add(rAxis.Cross(tr.V2)).Add(rAxis.MulScalar(rAxis.Dot(tr.V2)).MulScalar(1 - math.Cos(rAngle)))
	return ln.NewTriangle(v1, v2, v3)
}

/*
func rotMat(x ln.Vector, a float64) ln.Matrix {
	sinA := math.Sin(a)
	cosA := math.Cos(a)
	oneMinusCosA := 1 - cosA
	mat := ln.Matrix{(x.X * x.X * oneMinusCosA) + cosA,
		(x.Y * x.X * oneMinusCosA) - (sinA * x.Z),
		(x.Z * x.X * oneMinusCosA) + (sinA * x.Y),
		(x.X * x.Y * oneMinusCosA) + (sinA * x.Z),
		(x.Y * x.Y * oneMinusCosA) + cosA,
		(x.Z * x.Y * oneMinusCosA) - (sinA * x.X),
		(x.Z * x.Z * oneMinusCosA) + cosA,
	0,0,0,1}
	return mat
}
*/

func rotateTr(tr Tr, ref ln.Vector, rAngle float64) *ln.Triangle {
	rAxis := tr.Normal.Cross(ref)
	rAxis = rAxis.Normalize()
	//rMat := rotMat(rAxis, rAngle)
	rMat := ln.Rotate(rAxis, rAngle)
	v1 := rMat.MulPosition(tr.V1)
	v2 := rMat.MulPosition(tr.V2)
	v3 := rMat.MulPosition(tr.V3)
	return ln.NewTriangle(v1, v2, v3)
}

func rrrotateTr(tr Tr, ref ln.Vector, rAngle float64) *ln.Triangle {
	v1N := tr.V1.Normalize()
	v2N := tr.V2.Normalize()
	v3N := tr.V3.Normalize()
	v1 := ref.Mul(v1N).Div(ref)
	v2 := ref.Mul(v2N).Div(ref)
	v3 := ref.Mul(v3N).Div(ref)
	fmt.Println(rAngle)
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

func findNet(adj map[int][]int, f string) ([]graph.Edge[int], graph.Graph[int, int]) {
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
		return mstEdges, mst
	}
}

func testRender(tr *ln.Triangle, f string) {

	scene := ln.Scene{}
	scene.Add(tr)
	eye := ln.Vector{3, 3, 3}
	center := ln.Vector{0, 0, 0}
	up := ln.Vector{0, 1, 0}
	width := 1024.0
	height := 1024.0
	paths := scene.Render(eye, center, up, width, height, 50, 0.1, 100, 0.01)
	paths.WriteToPNG(f, width, height)
}

func getRandColor() string {
	r := rand.New(rand.NewSource(time.Now().UnixNano())).Intn(16777215)
	return strconv.FormatInt(int64(r), 16)
}

func drawCreasePattern(dfs []int, vTr map[int]Tr, adj map[int][]int, he map[int][]HalfEdge, f string) {
	var jsonTrs []Tr2D
	jsonF := f + ".json"

	dc := gg.NewContext(W, H)
	dc.DrawRectangle(0, 0, W, H)
	dc.SetHexColor("#FFFFFF")
	dc.Fill()

	scaleF := math.Min(W, H) / 23
	startX := W / 4
	startY := H / 4
	var x1, y1, x2, y2 float64
	drawn := make(map[int]Tr2D)
	availableE := make(map[HalfEdge]bool)
	var prevTr Tr2D
	prevF := false

	for i, tr := range dfs {
		nn := 2
		trC := vTr[tr]
		trCe1 := trC.V2.Sub(trC.V1)
		trCe2 := trC.V3.Sub(trC.V2)
		trCe3 := trC.V1.Sub(trC.V3)

		if i == 0 {
			x1 = float64(startX)
			y1 = float64(startY)
			x2 = x1 - trCe1.Length()*scaleF
			y2 = y1
			// just draw; no need to check for half edges etc.
		} else {
			for _, adj := range adj[tr] {
				if v, ok := drawn[adj]; ok {
					prevTr = v
					fmt.Printf("adj found: %v--%v\n", tr, adj)
					break
				}
			}

			heTr1A := he[tr]
			heTr2A := []HalfEdge{prevTr.E1.He, prevTr.E2.He, prevTr.E3.He}
			var heTr1 []HalfEdge
			var heTr2 []HalfEdge
			for _, ee := range heTr1A {
				if a, ok := availableE[ee]; a || !ok {
					if !ok {
						availableE[ee] = true
					}
					heTr1 = append(heTr1, ee)
				}
			}
			for _, e := range heTr2A {
				if a, ok := availableE[e]; a || !ok {

					if !ok {
						availableE[e] = true
					}
					heTr2 = append(heTr2, e)
				}
			}
			fmt.Printf("--------updated he---------\ntr1: %v\ntr2: %v\n", heTr1, heTr2)
			sharedE, n, usedE := getSharedEdge(heTr1, heTr2, prevTr)
			x1 = sharedE.X1
			y1 = sharedE.Y1
			x2 = sharedE.X2
			y2 = sharedE.Y2
			nn = n
			fmt.Printf("--------used he------------\n%v\t%v\n", usedE[0], usedE[1])
			availableE[usedE[0]] = false
			availableE[usedE[1]] = false
			//fmt.Printf("--------updated map--------\n%v\n", availableE)

			// now draw
			fmt.Printf("--------shared edge---------\n%v: %v\n", nn, sharedE)
		}
		//draw
		//a12, a12D := getAngle(trCe1, trCe2)

		a := getAngleTr(trC, nn)
		fmt.Printf("angle: %v\n", a*180/math.Pi)

		r := 0.0
		if nn == 1 {
			r = trCe2.Length() * scaleF
		} else if nn == 2 {
			r = trCe3.Length() * scaleF
		} else if nn == 3 {
			r = trCe1.Length() * scaleF
		}
		x3, y3 := getXY(x2, y2, a, r)
		if prevF {
			x3, y3 = getXY(x1, y1, a, r)
			prevF = false
		}
		fmt.Printf("side length: %v\n", r)

		color := getRandColor()

		dc.SetHexColor(color)
		dc.DrawLine(x1, y1, x2, y2)
		dc.Stroke()
		dc.SetHexColor(color)
		dc.DrawLine(x2, y2, x3, y3)
		dc.Stroke()
		dc.SetHexColor(color)
		dc.DrawLine(x3, y3, x1, y1)
		dc.Stroke()

		// update prevTr
		he1 := HalfEdge{trC.V1, trC.V2}
		he2 := HalfEdge{trC.V2, trC.V3}
		he3 := HalfEdge{trC.V3, trC.V1}
		drawnTr := Tr2D{Edge2D{x1, y1, x2, y2, he1}, Edge2D{x2, y2, x3, y3, he2}, Edge2D{x3, y3, x1, y1, he3}}
		drawn[tr] = drawnTr
		prevTr = drawnTr

		fmt.Printf("tr %v drawn at:\n%v\n%v\n%v\n", tr, drawnTr.E1, drawnTr.E2, drawnTr.E3)
		fmt.Println()

		jsonTrs = append(jsonTrs, drawnTr)
	}

	jF, _ := json.MarshalIndent(jsonTrs, "", " ")
	_ = ioutil.WriteFile(jsonF, jF, 0644)
	dc.SavePNG(f)
}

func drawCreasePattern3D(dfs []int, vTr map[int]Tr, adj map[int][]int, he map[int][]HalfEdge, f string) {
	scene := ln.Scene{}

	//trPrv := ln.NewTriangle(refT.V1, refT.V2, refT.V3)
	drawn := make(map[int]bool)

	// another idea:
	// make a map; each time something rotates, add it to the map; each rotation is done after checking the map & get updated position if necessary
	rotations := make(map[Tr]float64)

	prevTrV := dfs[0]
	prevTr := vTr[prevTrV]
	for i, trV := range dfs {
		if _, ok := drawn[trV]; ok {
			continue
		}
		tr := vTr[trV]

		// draw
		//  use shared E as rAxis
		// diff rAxis for diff tr? direction...
		if tr != prevTr {
			heTr1 := he[trV]
			heTr2 := he[prevTrV]
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
		}

		fmt.Printf("tr %v: %v\tref tr %v: %v\n", trV, tr, prevTrV, prevTr)

		trN := ln.NewTriangle(tr.V1, tr.V2, tr.V3)
		// check angle between tr1 & prev tr2
		if tr.Normal != prevTr.Normal {
			// check if tr is adjacent to prevtr
			if !contains(adj[trV], prevTrV) {
				// traverse adj to find the last rotated adj tr??
				for j := i; j >= 0; j-- {
					if contains(adj[trV], dfs[j]) {
						prevTr = vTr[dfs[j]]
						prevTrV = dfs[j]
						break
					}
				}
				fmt.Printf("new ref tr %v: %v\n", prevTr, prevTrV)
			}
			trA, trAD := getAngle(prevTr.Normal, tr.Normal)
			rotations[tr] = trAD
			trN = rotateTr(tr, prevTr.Normal, trA)
			//tr1N = translateTr(tr1N, ln.RandomUnitVector())
			fmt.Printf("tr %v rotated by: %v\tref tr: %v\n", trV, trAD, prevTrV)
			fmt.Printf("new tr %v: %v\n", trV, trN)

			//fmt.Printf("tr %v not rotated\tref tr: %v\n", trV, prevTrV)

		} else {
			fmt.Printf("tr %v not rotated\tsame normal as ref tr: %v\n", trV, prevTrV)
		}

		tf := fmt.Sprintf("output/tr-%v.png", trV)
		testRender(trN, tf)

		scene.Add(trN)
		drawn[trV] = true
		//prevTr = tr
		prevTr = Tr{trN.V1, trN.V2, trN.V3, getNormal(trN.V1, trN.V2, trN.V3)}
		prevTrV = trV
	}

	eye := ln.Vector{3, 3, 3}
	center := ln.Vector{0, 0, 0}
	up := ln.Vector{0, 1, 0}
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

	eye := ln.Vector{3, 3, 3}    // camera position
	center := ln.Vector{0, 0, 0} // camera looks at
	up := ln.Vector{0, 1, 0}     // up direction

	// define rendering parameters
	width := 1024.0  // rendered width
	height := 1024.0 // rendered height
	fovy := 50.0     // vertical field of view, degrees
	znear := 0.1     // near z plane
	zfar := 100.0    // far z plane
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
	mst, mstG := findNet(adjFaces, adjF)
	fmt.Println(mst)

	var dfs []int
	graph.DFS(mstG, mst[0].Source, func(v int) bool {
		dfs = append(dfs, v)
		fmt.Printf("%v--", v)
		return false
	})
	fmt.Println()

	creaseF := "output/" + tsF + "-crease.png"
	drawCreasePattern(dfs, trByV, adjFaces, heByF, creaseF)

}
