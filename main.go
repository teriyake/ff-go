package main

import (
	"fmt"
	"github.com/dominikbraun/graph"
	"github.com/dominikbraun/graph/draw"
	_ "github.com/flywave/go3d/quaternion"
	"github.com/flywave/go3d/vec3"
	_ "github.com/fogleman/gg"
	"github.com/fogleman/ln/ln"
	"gonum.org/v1/gonum/floats/scalar"
	"gonum.org/v1/gonum/num/quat"
	_ "io/ioutil"
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

type Vertex struct {
	X, Y, Z float64
}

type Edge struct {
	S, E Vertex
}

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

func raise(v Vertex) quat.Number {
	return quat.Number{Imag: v.X, Jmag: v.Y, Kmag: v.Z}
}

func qRotate(v Vertex, q quat.Number, s float64) Vertex {
	if l := quat.Abs(q); l != s {
		q = quat.Scale(math.Sqrt(s)/l, q)
	}

	vN := quat.Mul(quat.Mul(q, raise(v)), quat.Conj(q))

	return Vertex{X: vN.Imag, Y: vN.Jmag, Z: vN.Kmag}
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

/*
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
*/

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
	drawCreasePattern3D(dfs, trByV, adjFaces, heByF, creaseF)

	a := vec3.UnitX
	fmt.Println(a)

	alpha := 2 * math.Pi / 3
	q := raise(Vertex{1, 1, 1})
	scale := 1.0
	q = quat.Scale(math.Sin(alpha/2)/quat.Abs(q), q)
	q.Real += math.Cos(alpha / 2)
	for i, v := range []Vertex{
		{X: 0, Y: 0, Z: 0},
		{X: 0, Y: 0, Z: 1},
		{X: 0, Y: 1, Z: 0},
		{X: 0, Y: 1, Z: 1},
		{X: 1, Y: 0, Z: 0},
		{X: 1, Y: 0, Z: 1},
		{X: 1, Y: 1, Z: 0},
		{X: 1, Y: 1, Z: 1},
	} {
		vN := qRotate(v, q, scale)
		vN.X = scalar.Round(vN.X, 2)
		vN.Y = scalar.Round(vN.Y, 2)
		vN.Z = scalar.Round(vN.Z, 2)
		fmt.Printf("%d %+v -> %+v\n", i, v, vN)

		// convert Vertex to ln.Vector

	}

}
