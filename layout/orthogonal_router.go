package layout

import (
	"container/heap"
	"sort"
)

// Side is a side of a node rectangle that an edge port is attached to.
type Side int

const (
	SideTop Side = iota
	SideRight
	SideBottom
	SideLeft
)

// RoutedEdge is the result of routing one edge: an orthogonal polyline
// plus the boundary ports it starts and ends at.
type RoutedEdge struct {
	Path     []Position // horizontal/vertical polyline, [fromPort ... toPort]
	FromPort Position   // point on the source node boundary
	ToPort   Position   // point on the target node boundary
	FromSide Side
	ToSide   Side
	// Fallback is true when no obstacle-free channel existed (or the
	// grid limit was hit) and Path is a plain L-path that may cross nodes.
	Fallback bool
}

// OrthogonalEdgeRouter replaces edge paths with horizontal/vertical
// polylines that do not cross any node rectangle. It only rewrites
// g.Edges; node positions and sizes are left untouched, so it can be
// appended after any node positioning layout.
//
// Routing is done on a grid built from the (margin-inflated) node
// boundaries plus the edge escape points, searched with A* using a
// bend penalty. All iteration order is sorted, so repeated runs on the
// same input produce identical routes.
//
// Degenerate cases:
//   - self loops exit the right side and re-enter the top side;
//   - zero-size and coincident nodes are handled (their inflated
//     rectangles are still routed around);
//   - when no channel exists, or the grid would exceed MaxGridCells,
//     the edge falls back to a plain orthogonal L-path between its
//     ports (which may cross nodes).
type OrthogonalEdgeRouter struct {
	Margin       int // gap kept between nodes and routes; <=0 uses 10
	PortSpacing  int // distance between ports sharing a node side; <=0 uses 10
	BendPenalty  int // extra cost per 90-degree bend; <=0 uses 8
	MaxGridCells int // routing grid size limit; <=0 uses 4<<20
}

func (r OrthogonalEdgeRouter) margin() int {
	if r.Margin <= 0 {
		return 10
	}
	return r.Margin
}

func (r OrthogonalEdgeRouter) portSpacing() int {
	if r.PortSpacing <= 0 {
		return 10
	}
	return r.PortSpacing
}

func (r OrthogonalEdgeRouter) bendPenalty() int {
	if r.BendPenalty <= 0 {
		return 8
	}
	return r.BendPenalty
}

func (r OrthogonalEdgeRouter) maxGridCells() int {
	if r.MaxGridCells <= 0 {
		return 4 << 20
	}
	return r.MaxGridCells
}

// UpdateGraphLayout routes all edges of g in place. Node layout is preserved.
func (r OrthogonalEdgeRouter) UpdateGraphLayout(g Graph) {
	for e, re := range r.Route(g) {
		g.Edges[e] = Edge{Path: re.Path}
	}
}

// Route computes ports and orthogonal paths for every edge of g without
// modifying g. Edges referencing nodes absent from g.Nodes use a zero
// node at the origin, mirroring DirectEdgesLayout.
func (r OrthogonalEdgeRouter) Route(g Graph) map[[2]NodeID]RoutedEdge {
	margin := r.margin()

	edgeKeys := make([][2]NodeID, 0, len(g.Edges))
	for e := range g.Edges {
		edgeKeys = append(edgeKeys, e)
	}
	sort.Slice(edgeKeys, func(i, j int) bool {
		if edgeKeys[i][0] != edgeKeys[j][0] {
			return edgeKeys[i][0] < edgeKeys[j][0]
		}
		return edgeKeys[i][1] < edgeKeys[j][1]
	})

	// pick exit/entry sides per edge
	type portRef struct {
		edge   [2]NodeID
		isFrom bool
	}
	sides := make(map[[2]NodeID][2]Side, len(edgeKeys))
	groups := make(map[NodeID]map[Side][]portRef)
	addPort := func(n NodeID, side Side, ref portRef) {
		if groups[n] == nil {
			groups[n] = make(map[Side][]portRef)
		}
		groups[n][side] = append(groups[n][side], ref)
	}
	for _, e := range edgeKeys {
		from, to := g.Nodes[e[0]], g.Nodes[e[1]]
		var fs, ts Side
		if e[0] == e[1] {
			fs, ts = SideRight, SideTop
		} else {
			fc, tc := from.CenterXY(), to.CenterXY()
			dx, dy := tc.X-fc.X, tc.Y-fc.Y
			if abs(dx) >= abs(dy) {
				if dx > 0 {
					fs, ts = SideRight, SideLeft
				} else {
					fs, ts = SideLeft, SideRight
				}
			} else {
				if dy > 0 {
					fs, ts = SideBottom, SideTop
				} else {
					fs, ts = SideTop, SideBottom
				}
			}
		}
		sides[e] = [2]Side{fs, ts}
		addPort(e[0], fs, portRef{edge: e, isFrom: true})
		addPort(e[1], ts, portRef{edge: e, isFrom: false})
	}

	// spread ports sharing one side of one node; edgeKeys order makes
	// each group already sorted, so port assignment is deterministic
	ports := make(map[[2]NodeID][2]Position, len(edgeKeys))
	for id, bySide := range groups {
		node := g.Nodes[id]
		for side, refs := range bySide {
			start, end := sideRange(node, side)
			center := (start + end) / 2
			base := center - (len(refs)-1)*r.portSpacing()/2
			for i, ref := range refs {
				offset := clamp(base+i*r.portSpacing(), start, end)
				p := portPosition(node, side, offset)
				if ref.isFrom {
					ports[ref.edge] = [2]Position{p, ports[ref.edge][1]}
				} else {
					ports[ref.edge] = [2]Position{ports[ref.edge][0], p}
				}
			}
		}
	}

	// escape points: just outside the inflated node, straight out of the port
	type escape struct{ from, to Position }
	escapes := make(map[[2]NodeID]escape, len(edgeKeys))
	xsSet, ysSet := map[int]bool{}, map[int]bool{}
	for _, n := range g.Nodes {
		xsSet[n.X-margin] = true
		xsSet[n.X+n.W+margin] = true
		ysSet[n.Y-margin] = true
		ysSet[n.Y+n.H+margin] = true
	}
	for _, e := range edgeKeys {
		p := ports[e]
		s := sides[e]
		ef := moveOut(p[0], s[0], margin)
		et := moveOut(p[1], s[1], margin)
		escapes[e] = escape{from: ef, to: et}
		xsSet[ef.X], xsSet[et.X] = true, true
		ysSet[ef.Y], ysSet[et.Y] = true, true
	}
	xs := sortedInts(xsSet)
	ys := sortedInts(ysSet)

	out := make(map[[2]NodeID]RoutedEdge, len(edgeKeys))

	if len(xs) > 0 && len(ys) > 0 && len(xs)*len(ys) <= r.maxGridCells() {
		hBlock, vBlock := blockedSegments(g, xs, ys, margin)
		xIdx := indexOf(xs)
		yIdx := indexOf(ys)
		for _, e := range edgeKeys {
			esc := escapes[e]
			start := [2]int{xIdx[esc.from.X], yIdx[esc.from.Y]}
			goal := [2]int{xIdx[esc.to.X], yIdx[esc.to.Y]}
			var path []Position
			fallback := false
			if cells, ok := routeAStar(xs, ys, hBlock, vBlock, start, goal, r.bendPenalty()); ok {
				path = make([]Position, 0, len(cells)+2)
				path = append(path, ports[e][0])
				for _, c := range cells {
					path = append(path, Position{X: xs[c[0]], Y: ys[c[1]]})
				}
				path = append(path, ports[e][1])
			} else {
				path = fallbackPath(ports[e], escapes[e].from, escapes[e].to, sides[e])
				fallback = true
			}
			out[e] = RoutedEdge{
				Path:     simplifyPath(path),
				FromPort: ports[e][0],
				ToPort:   ports[e][1],
				FromSide: sides[e][0],
				ToSide:   sides[e][1],
				Fallback: fallback,
			}
		}
		return out
	}

	// grid too large (or empty graph): orthogonal L-paths between ports
	for _, e := range edgeKeys {
		out[e] = RoutedEdge{
			Path:     simplifyPath(fallbackPath(ports[e], escapes[e].from, escapes[e].to, sides[e])),
			FromPort: ports[e][0],
			ToPort:   ports[e][1],
			FromSide: sides[e][0],
			ToSide:   sides[e][1],
			Fallback: true,
		}
	}
	return out
}

// sideRange returns the coordinate range of a node side along its axis.
func sideRange(n Node, side Side) (int, int) {
	switch side {
	case SideTop, SideBottom:
		return n.X, n.X + n.W
	default:
		return n.Y, n.Y + n.H
	}
}

// portPosition places a port on the node boundary at the given offset
// along the side.
func portPosition(n Node, side Side, offset int) Position {
	switch side {
	case SideTop:
		return Position{X: offset, Y: n.Y}
	case SideBottom:
		return Position{X: offset, Y: n.Y + n.H}
	case SideLeft:
		return Position{X: n.X, Y: offset}
	default: // SideRight
		return Position{X: n.X + n.W, Y: offset}
	}
}

func moveOut(p Position, side Side, d int) Position {
	switch side {
	case SideTop:
		return Position{X: p.X, Y: p.Y - d}
	case SideBottom:
		return Position{X: p.X, Y: p.Y + d}
	case SideLeft:
		return Position{X: p.X - d, Y: p.Y}
	default:
		return Position{X: p.X + d, Y: p.Y}
	}
}

// fallbackPath is an orthogonal path between the ports through their
// escape points, used when no obstacle-free channel exists. It may
// cross nodes.
func fallbackPath(ports [2]Position, escFrom, escTo Position, sides [2]Side) []Position {
	path := []Position{ports[0], escFrom}
	if sides[0] == SideLeft || sides[0] == SideRight {
		path = append(path, Position{X: escTo.X, Y: escFrom.Y})
	} else {
		path = append(path, Position{X: escFrom.X, Y: escTo.Y})
	}
	return append(path, escTo, ports[1])
}

// blockedSegments marks unit grid segments whose midpoint lies strictly
// inside a margin-inflated node. Because grid lines sit on every
// inflated node boundary, the midpoint test is exact.
// hBlock[yi][xi]: segment (xi,yi)-(xi+1,yi); vBlock[xi][yi]: (xi,yi)-(xi,yi+1).
func blockedSegments(g Graph, xs, ys []int, margin int) (hBlock, vBlock [][]bool) {
	nx, ny := len(xs), len(ys)
	hBlock = make([][]bool, ny)
	for yi := range hBlock {
		hBlock[yi] = make([]bool, max(nx-1, 0))
		diff := make([]int, nx+1)
		for _, n := range g.Nodes {
			if ys[yi] > n.Y-margin && ys[yi] < n.Y+n.H+margin {
				lo := sort.SearchInts(xs, n.X-margin)
				hi := sort.SearchInts(xs, n.X+n.W+margin)
				diff[lo]++
				diff[hi]--
			}
		}
		acc := 0
		for xi := 0; xi+1 < nx; xi++ {
			acc += diff[xi]
			hBlock[yi][xi] = acc > 0
		}
	}
	vBlock = make([][]bool, nx)
	for xi := range vBlock {
		vBlock[xi] = make([]bool, max(ny-1, 0))
		diff := make([]int, ny+1)
		for _, n := range g.Nodes {
			if xs[xi] > n.X-margin && xs[xi] < n.X+n.W+margin {
				lo := sort.SearchInts(ys, n.Y-margin)
				hi := sort.SearchInts(ys, n.Y+n.H+margin)
				diff[lo]++
				diff[hi]--
			}
		}
		acc := 0
		for yi := 0; yi+1 < ny; yi++ {
			acc += diff[yi]
			vBlock[xi][yi] = acc > 0
		}
	}
	return hBlock, vBlock
}

// A* over grid vertices. Direction state: 0 none, 1 +x, 2 -x, 3 +y, 4 -y.

type astarItem struct {
	f, h       int
	xi, yi, dr int
}

type astarHeap []astarItem

func (h astarHeap) Len() int { return len(h) }

// Less is a total order so ties resolve deterministically.
func (h astarHeap) Less(i, j int) bool {
	a, b := h[i], h[j]
	if a.f != b.f {
		return a.f < b.f
	}
	if a.h != b.h {
		return a.h < b.h
	}
	if a.xi != b.xi {
		return a.xi < b.xi
	}
	if a.yi != b.yi {
		return a.yi < b.yi
	}
	return a.dr < b.dr
}
func (h astarHeap) Swap(i, j int)       { h[i], h[j] = h[j], h[i] }
func (h *astarHeap) Push(x interface{}) { *h = append(*h, x.(astarItem)) }
func (h *astarHeap) Pop() interface{} {
	old := *h
	n := len(old)
	it := old[n-1]
	*h = old[:n-1]
	return it
}

func routeAStar(xs, ys []int, hBlock, vBlock [][]bool, start, goal [2]int, bendPenalty int) ([][2]int, bool) {
	nx := len(xs)
	key := func(xi, yi, dr int) uint32 {
		return uint32((yi*nx+xi)*5 + dr)
	}
	dist := map[uint32]int{key(start[0], start[1], 0): 0}
	prev := map[uint32]uint32{}

	pq := &astarHeap{astarItem{
		f:  abs(xs[goal[0]]-xs[start[0]]) + abs(ys[goal[1]]-ys[start[1]]),
		h:  abs(xs[goal[0]]-xs[start[0]]) + abs(ys[goal[1]]-ys[start[1]]),
		xi: start[0], yi: start[1],
	}}

	// moves: dr, dxi, dyi
	moves := [4][3]int{{1, 1, 0}, {2, -1, 0}, {3, 0, 1}, {4, 0, -1}}

	var goalKey uint32
	found := false
	for pq.Len() > 0 {
		cur := heap.Pop(pq).(astarItem)
		ck := key(cur.xi, cur.yi, cur.dr)
		d, ok := dist[ck]
		if !ok || cur.f-cur.h != d {
			continue // stale heap entry
		}
		if cur.xi == goal[0] && cur.yi == goal[1] {
			goalKey = ck
			found = true
			break
		}
		for _, m := range moves {
			dr, dxi, dyi := m[0], m[1], m[2]
			nxi, nyi := cur.xi+dxi, cur.yi+dyi
			if nxi < 0 || nyi < 0 || nxi >= nx || nyi >= len(ys) {
				continue
			}
			if dxi > 0 && hBlock[cur.yi][cur.xi] {
				continue
			}
			if dxi < 0 && hBlock[cur.yi][nxi] {
				continue
			}
			if dyi > 0 && vBlock[cur.xi][cur.yi] {
				continue
			}
			if dyi < 0 && vBlock[cur.xi][nyi] {
				continue
			}
			step := abs(dxi)*(abs(xs[nxi]-xs[cur.xi])) + abs(dyi)*(abs(ys[nyi]-ys[cur.yi]))
			if cur.dr != 0 && cur.dr != dr {
				step += bendPenalty
			}
			nk := key(nxi, nyi, dr)
			nd := d + step
			if old, seen := dist[nk]; !seen || nd < old {
				dist[nk] = nd
				prev[nk] = ck
				h := abs(xs[goal[0]]-xs[nxi]) + abs(ys[goal[1]]-ys[nyi])
				heap.Push(pq, astarItem{f: nd + h, h: h, xi: nxi, yi: nyi, dr: dr})
			}
		}
	}
	if !found {
		return nil, false
	}

	var rev [][2]int
	for k := goalKey; ; k = prev[k] {
		xi := int(k/5) % nx
		yi := int(k / 5 / uint32(nx))
		rev = append(rev, [2]int{xi, yi})
		if k == key(start[0], start[1], 0) {
			break
		}
	}
	cells := make([][2]int, len(rev))
	for i, c := range rev {
		cells[len(rev)-1-i] = c
	}
	return cells, true
}

// simplifyPath drops duplicate points and points collinear with their
// neighbours, keeping endpoints.
func simplifyPath(path []Position) []Position {
	out := make([]Position, 0, len(path))
	for _, p := range path {
		if n := len(out); n > 0 && out[n-1] == p {
			continue
		}
		out = append(out, p)
		for len(out) >= 3 {
			a, b, c := out[len(out)-3], out[len(out)-2], out[len(out)-1]
			if (a.X == b.X && b.X == c.X) || (a.Y == b.Y && b.Y == c.Y) {
				out[len(out)-2] = out[len(out)-1]
				out = out[:len(out)-1]
			} else {
				break
			}
		}
	}
	return out
}

func sortedInts(set map[int]bool) []int {
	out := make([]int, 0, len(set))
	for v := range set {
		out = append(out, v)
	}
	sort.Ints(out)
	return out
}

func indexOf(sorted []int) map[int]int {
	idx := make(map[int]int, len(sorted))
	for i, v := range sorted {
		idx[v] = i
	}
	return idx
}

func clamp(v, lo, hi int) int {
	if lo > hi {
		return (lo + hi) / 2
	}
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

func abs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}
