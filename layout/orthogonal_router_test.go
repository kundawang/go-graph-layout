package layout_test

import (
	"fmt"
	"reflect"
	"testing"

	"github.com/gverger/go-graph-layout/layout"
)

func isOrthogonal(path []layout.Position) bool {
	for i := 1; i < len(path); i++ {
		if path[i].X != path[i-1].X && path[i].Y != path[i-1].Y {
			return false
		}
	}
	return true
}

// segmentCrossesNodeInterior reports whether the axis-aligned segment
// ab passes through the interior of n (touching the border is fine).
func segmentCrossesNodeInterior(a, b layout.Position, n layout.Node) bool {
	if a.X == b.X {
		x := a.X
		if x <= n.X || x >= n.X+n.W {
			return false
		}
		lo, hi := min(a.Y, b.Y), max(a.Y, b.Y)
		return max(lo, n.Y) < min(hi, n.Y+n.H)
	}
	if a.Y == b.Y {
		y := a.Y
		if y <= n.Y || y >= n.Y+n.H {
			return false
		}
		lo, hi := min(a.X, b.X), max(a.X, b.X)
		return max(lo, n.X) < min(hi, n.X+n.W)
	}
	return true // non-orthogonal segment: reject conservatively
}

func pathAvoidsNodes(path []layout.Position, g layout.Graph) error {
	for i := 1; i < len(path); i++ {
		for id, n := range g.Nodes {
			if segmentCrossesNodeInterior(path[i-1], path[i], n) {
				return fmt.Errorf("segment %v->%v crosses node %d (%+v)", path[i-1], path[i], id, n)
			}
		}
	}
	return nil
}

func onBoundary(p layout.Position, n layout.Node) bool {
	onV := (p.X == n.X || p.X == n.X+n.W) && p.Y >= n.Y && p.Y <= n.Y+n.H
	onH := (p.Y == n.Y || p.Y == n.Y+n.H) && p.X >= n.X && p.X <= n.X+n.W
	return onV || onH
}

func checkRoutedEdge(t *testing.T, g layout.Graph, e [2]uint64, re layout.RoutedEdge) {
	t.Helper()
	if len(re.Path) < 2 {
		t.Fatalf("edge %v: path too short: %v", e, re.Path)
	}
	if !isOrthogonal(re.Path) {
		t.Errorf("edge %v: path is not orthogonal: %v", e, re.Path)
	}
	if re.Path[0] != re.FromPort || re.Path[len(re.Path)-1] != re.ToPort {
		t.Errorf("edge %v: path endpoints are not the ports", e)
	}
	if !onBoundary(re.FromPort, g.Nodes[e[0]]) {
		t.Errorf("edge %v: from-port %v not on source boundary %+v", e, re.FromPort, g.Nodes[e[0]])
	}
	if !onBoundary(re.ToPort, g.Nodes[e[1]]) {
		t.Errorf("edge %v: to-port %v not on target boundary %+v", e, re.ToPort, g.Nodes[e[1]])
	}
	if err := pathAvoidsNodes(re.Path, g); err != nil {
		t.Errorf("edge %v: %v", e, err)
	}
}

func quadGraph() layout.Graph {
	return layout.Graph{
		Nodes: map[uint64]layout.Node{
			1: {Position: layout.Position{X: 0, Y: 0}, W: 100, H: 40},
			2: {Position: layout.Position{X: 300, Y: 0}, W: 100, H: 40},
			3: {Position: layout.Position{X: 0, Y: 300}, W: 100, H: 40},
			4: {Position: layout.Position{X: 300, Y: 300}, W: 100, H: 40},
		},
		Edges: map[[2]uint64]layout.Edge{
			{1, 4}: {},
			{2, 3}: {},
			{1, 2}: {},
			{3, 4}: {},
		},
	}
}

func TestOrthogonalRouter_CrossingEdges(t *testing.T) {
	g := quadGraph()
	routed := layout.OrthogonalEdgeRouter{}.Route(g)
	if len(routed) != len(g.Edges) {
		t.Fatalf("expected %d routed edges, got %d", len(g.Edges), len(routed))
	}
	for e, re := range routed {
		checkRoutedEdge(t, g, e, re)
		if re.Fallback {
			t.Errorf("edge %v: channel exists, should not fall back", e)
		}
	}
	// diagonals 1->4 and 2->3 must bend around nodes, not go straight
	for _, e := range [][2]uint64{{1, 4}, {2, 3}} {
		if len(routed[e].Path) < 3 {
			t.Errorf("edge %v: expected bends around obstacles, got %v", e, routed[e].Path)
		}
	}
}

func TestOrthogonalRouter_MultiPortAndReverse(t *testing.T) {
	g := layout.Graph{
		Nodes: map[uint64]layout.Node{
			1: {Position: layout.Position{X: 0, Y: 0}, W: 100, H: 100},
			2: {Position: layout.Position{X: 300, Y: 0}, W: 100, H: 40},
			3: {Position: layout.Position{X: 300, Y: 200}, W: 100, H: 40},
		},
		Edges: map[[2]uint64]layout.Edge{
			{1, 2}: {},
			{1, 3}: {},
			{2, 1}: {},
		},
	}
	routed := layout.OrthogonalEdgeRouter{}.Route(g)
	for e, re := range routed {
		checkRoutedEdge(t, g, e, re)
	}
	// two edges leaving the right side of node 1 get distinct ports
	if routed[[2]uint64{1, 2}].FromPort == routed[[2]uint64{1, 3}].FromPort {
		t.Errorf("parallel edges share a port: %v", routed[[2]uint64{1, 2}].FromPort)
	}
	// reverse edge 2->1 is distinguishable from 1->2
	fwd, rev := routed[[2]uint64{1, 2}], routed[[2]uint64{2, 1}]
	if reflect.DeepEqual(fwd.Path, rev.Path) {
		t.Errorf("reverse edge has identical path: %v", fwd.Path)
	}
	if fwd.FromSide == rev.ToSide && fwd.FromPort == rev.ToPort && fwd.ToPort == rev.FromPort {
		t.Errorf("reverse edge reuses the same ports: %v vs %v", fwd, rev)
	}
}

func TestOrthogonalRouter_SelfLoop(t *testing.T) {
	g := layout.Graph{
		Nodes: map[uint64]layout.Node{
			1: {Position: layout.Position{X: 100, Y: 100}, W: 80, H: 40},
		},
		Edges: map[[2]uint64]layout.Edge{{1, 1}: {}},
	}
	re := layout.OrthogonalEdgeRouter{}.Route(g)[[2]uint64{1, 1}]
	checkRoutedEdge(t, g, [2]uint64{1, 1}, re)
	if len(re.Path) < 4 {
		t.Errorf("self loop should go around the node, got %v", re.Path)
	}
	if re.FromSide == re.ToSide {
		t.Errorf("self loop should use two different sides, got %v", re.FromSide)
	}
}

func TestOrthogonalRouter_DegenerateNodes(t *testing.T) {
	g := layout.Graph{
		Nodes: map[uint64]layout.Node{
			1: {Position: layout.Position{X: 50, Y: 50}, W: 0, H: 0},   // zero size
			2: {Position: layout.Position{X: 200, Y: 200}, W: 0, H: 0}, // zero size
			3: {Position: layout.Position{X: 200, Y: 200}, W: 0, H: 0}, // coincident with 2
			4: {Position: layout.Position{X: 400, Y: 50}, W: 60, H: 30},
		},
		Edges: map[[2]uint64]layout.Edge{
			{1, 2}: {},
			{2, 3}: {}, // coincident centers
			{1, 4}: {},
			{2, 2}: {}, // self loop on zero-size node
		},
	}
	routed := layout.OrthogonalEdgeRouter{}.Route(g)
	for e, re := range routed {
		checkRoutedEdge(t, g, e, re)
	}
}

func TestOrthogonalRouter_FallbackWhenEnclosed(t *testing.T) {
	// node 1 is sealed inside a ring of nodes; no channel exists
	g := layout.Graph{
		Nodes: map[uint64]layout.Node{
			1: {Position: layout.Position{X: 100, Y: 100}, W: 20, H: 20},
			2: {Position: layout.Position{X: 40, Y: 40}, W: 140, H: 10},  // top wall
			3: {Position: layout.Position{X: 40, Y: 170}, W: 140, H: 10}, // bottom wall
			4: {Position: layout.Position{X: 40, Y: 40}, W: 10, H: 140},  // left wall
			5: {Position: layout.Position{X: 170, Y: 40}, W: 10, H: 140}, // right wall
			6: {Position: layout.Position{X: 400, Y: 100}, W: 20, H: 20},
		},
		Edges: map[[2]uint64]layout.Edge{{6, 1}: {}},
	}
	re := layout.OrthogonalEdgeRouter{}.Route(g)[[2]uint64{6, 1}]
	if !re.Fallback {
		t.Error("enclosed target should report Fallback")
	}
	if len(re.Path) < 2 || !isOrthogonal(re.Path) {
		t.Fatalf("fallback path must exist and be orthogonal: %v", re.Path)
	}
	if !onBoundary(re.FromPort, g.Nodes[6]) || !onBoundary(re.ToPort, g.Nodes[1]) {
		t.Errorf("fallback path must still connect boundary ports: %v", re.Path)
	}
}

func TestOrthogonalRouter_GridLimitFallback(t *testing.T) {
	g := quadGraph()
	routed := layout.OrthogonalEdgeRouter{MaxGridCells: 10}.Route(g)
	for e, re := range routed {
		if !re.Fallback {
			t.Errorf("edge %v: grid limit should report Fallback", e)
		}
		if len(re.Path) < 2 || !isOrthogonal(re.Path) {
			t.Errorf("edge %v: grid-limit fallback must be orthogonal: %v", e, re.Path)
		}
		if re.Path[0] != re.FromPort || re.Path[len(re.Path)-1] != re.ToPort {
			t.Errorf("edge %v: fallback endpoints are not the ports", e)
		}
	}
}

func TestOrthogonalRouter_DeterministicAndLayoutPreserved(t *testing.T) {
	g := quadGraph()
	g.Edges[[2]uint64{4, 1}] = layout.Edge{}
	g.Edges[[2]uint64{2, 2}] = layout.Edge{}

	before := g.Copy()
	first := layout.OrthogonalEdgeRouter{}.Route(g)
	second := layout.OrthogonalEdgeRouter{}.Route(g)
	if !reflect.DeepEqual(first, second) {
		t.Fatal("repeated runs produced different routes")
	}

	// UpdateGraphLayout rewrites edges only, nodes stay as laid out
	gl := before.Copy()
	layout.OrthogonalEdgeRouter{}.UpdateGraphLayout(gl)
	if !reflect.DeepEqual(gl.Nodes, before.Nodes) {
		t.Fatal("router modified node layout")
	}
	for e, re := range first {
		if !reflect.DeepEqual(gl.Edges[e].Path, re.Path) {
			t.Fatalf("edge %v: UpdateGraphLayout path differs from Route", e)
		}
	}
}

func TestOrthogonalRouter_LargeGraph(t *testing.T) {
	// 12x12 grid of nodes, edges between neighbours and a few long jumps
	g := layout.Graph{
		Nodes: map[uint64]layout.Node{},
		Edges: map[[2]uint64]layout.Edge{},
	}
	id := func(r, c int) uint64 { return uint64(r*12 + c + 1) }
	for r := 0; r < 12; r++ {
		for c := 0; c < 12; c++ {
			g.Nodes[id(r, c)] = layout.Node{
				Position: layout.Position{X: c * 120, Y: r * 80},
				W:        60, H: 30,
			}
		}
	}
	for r := 0; r < 12; r++ {
		for c := 0; c < 12; c++ {
			if c+1 < 12 {
				g.Edges[[2]uint64{id(r, c), id(r, c+1)}] = layout.Edge{}
			}
			if r+1 < 12 {
				g.Edges[[2]uint64{id(r, c), id(r+1, c)}] = layout.Edge{}
			}
		}
	}
	g.Edges[[2]uint64{id(0, 0), id(11, 11)}] = layout.Edge{}
	g.Edges[[2]uint64{id(11, 0), id(0, 11)}] = layout.Edge{}

	routed := layout.OrthogonalEdgeRouter{}.Route(g)
	if len(routed) != len(g.Edges) {
		t.Fatalf("expected %d routed edges, got %d", len(g.Edges), len(routed))
	}
	for e, re := range routed {
		checkRoutedEdge(t, g, e, re)
	}
}
