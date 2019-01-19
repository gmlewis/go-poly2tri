/*
 * Poly2Tri Copyright (c) 2009-2011, Poly2Tri Contributors
 * http://code.google.com/p/poly2tri/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of Poly2Tri nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package poly2tri

import (
	"container/list"
	"log"
	"sort"
)

// Point represents a point.
type Point struct {
	X, Y  float64
	edges []*Edge
}

// NewPoint returns a new point.
func NewPoint(x, y float64) *Point {
	return &Point{x, y, []*Edge{}}
}

// PointArray attaches the methods of Interface to []*Point, sorting in increasing order.
type PointArray []*Point

func (p PointArray) Len() int { return len(p) }

func (p PointArray) Less(i, j int) bool {
	if p[i].Y < p[j].Y {
		return true
	} else if p[i].Y == p[j].Y {
		// Make sure q is point with greater x value
		if p[i].X < p[j].X {
			return true
		}
	}
	return false
}

func (p PointArray) Swap(i, j int) { p[i], p[j] = p[j], p[i] }

// Sort is a convenience method.
func (p PointArray) Sort() { sort.Sort(p) }

// Edge represents an edge.
type Edge struct{ p, q *Point }

// Triangle represents a triangle.
type Triangle struct {
	// Triangle Point
	Point [3]*Point
	// neighbor list
	neighbor [3]*Triangle
	// Has t triangle been marked as an interior triangle?
	interior bool
	// Flags to determine if an edge is a Constrained edge
	constrainedEdge [3]bool
	// Flags to determine if an edge is a Delauney edge
	delaunayEdge [3]bool

	eref *list.Element
}

// TriArray represents a triangle array.
type TriArray []*Triangle

func (t *Triangle) init(p1, p2, p3 *Point) {
	t.Point[0] = p1
	t.Point[1] = p2
	t.Point[2] = p3
}

func (e *Edge) init(p1, p2 *Point) {
	e.p = p1
	e.q = p2
	if p1.Y > p2.Y {
		e.q = p1
		e.p = p2
	} else if p1.Y == p2.Y {
		if p1.X > p2.X {
			e.q = p1
			e.p = p2
		} else if p1.X == p2.X {
			log.Fatalf("repeat Point at x=%v", p1.X)
		}
	}
	e.q.edges = append(e.q.edges, e)
}

func (t *Triangle) containsPoint(p *Point) bool {
	return p == t.Point[0] || p == t.Point[1] || p == t.Point[2]
}

func (t *Triangle) containsEdge(e Edge) bool {
	return t.containsPoint(e.p) && t.containsPoint(e.q)
}

func (t *Triangle) containsPoints(p, q *Point) bool {
	return t.containsPoint(p) && t.containsPoint(q)
}

// Update neighbor pointers
func (t *Triangle) markNeighbor(p1, p2 *Point, t2 *Triangle) {
	if (p1 == t.Point[2] && p2 == t.Point[1]) ||
		(p1 == t.Point[1] && p2 == t.Point[2]) {
		t.neighbor[0] = t2
	} else if (p1 == t.Point[0] && p2 == t.Point[2]) ||
		(p1 == t.Point[2] && p2 == t.Point[0]) {
		t.neighbor[1] = t2
	} else if (p1 == t.Point[0] && p2 == t.Point[1]) ||
		(p1 == t.Point[1] && p2 == t.Point[0]) {
		t.neighbor[2] = t2
	} else {
		log.Fatalf("invalid triangles: p1=%#v, p2=%#v, t2=%#v", *p1, *p2, *t2)
	}
}

// Exhaustive search to update neighbor pointers
func (t *Triangle) markNeighbor2(t2 *Triangle) {
	if t2.containsPoints(t.Point[1], t.Point[2]) {
		t.neighbor[0] = t2
		t2.markNeighbor(t.Point[1], t.Point[2], t)
	} else if t2.containsPoints(t.Point[0], t.Point[2]) {
		t.neighbor[1] = t2
		t2.markNeighbor(t.Point[0], t.Point[2], t)
	} else if t2.containsPoints(t.Point[0], t.Point[1]) {
		t.neighbor[2] = t2
		t2.markNeighbor(t.Point[0], t.Point[1], t)
	}
}

func (t *Triangle) markConstrainedEdge(index int) {
	t.constrainedEdge[index] = true
}

func (t *Triangle) markConstrainedEdge2(edge Edge) {
	t.markConstrainedEdge3(edge.p, edge.q)
}

// Mark edge as constrained
func (t *Triangle) markConstrainedEdge3(p, q *Point) {
	if (q == t.Point[0] && p == t.Point[1]) ||
		(q == t.Point[1] && p == t.Point[0]) {
		t.constrainedEdge[2] = true
	} else if (q == t.Point[0] && p == t.Point[2]) ||
		(q == t.Point[2] && p == t.Point[0]) {
		t.constrainedEdge[1] = true
	} else if (q == t.Point[1] && p == t.Point[2]) ||
		(q == t.Point[2] && p == t.Point[1]) {
		t.constrainedEdge[0] = true
	}
}

/**
 * Clears all references to all other triangles and Point
 */
func (t *Triangle) clear() {
	t.clearNeighbors()
	t.Point[0] = nil
	t.Point[1] = nil
	t.Point[2] = nil
}

func (t *Triangle) clearNeighbor(t2 *Triangle) {
	if t.neighbor[0] == t2 {
		t.neighbor[0] = nil
	} else if t.neighbor[1] == t2 {
		t.neighbor[1] = nil
	} else {
		t.neighbor[2] = nil
	}
}

func (t *Triangle) clearNeighbors() {
	t.neighbor[0] = nil
	t.neighbor[1] = nil
	t.neighbor[2] = nil
}

func (t *Triangle) clearDelunayEdges() {
	t.delaunayEdge[0] = false
	t.delaunayEdge[1] = false
	t.delaunayEdge[2] = false
}

// The point counter-clockwise to given point
func (t *Triangle) pointCW(point *Point) *Point {
	switch {
	case point == t.Point[0]:
		return t.Point[2]
	case point == t.Point[1]:
		return t.Point[0]
	case point == t.Point[2]:
		return t.Point[1]
	default:
		log.Fatalf("invalid Point: %#v", *point)
	}
	return nil
}

// The point counter-clockwise to given point
func (t *Triangle) pointCCW(point *Point) *Point {
	switch {
	case point == t.Point[0]:
		return t.Point[1]
	case point == t.Point[1]:
		return t.Point[2]
	case point == t.Point[2]:
		return t.Point[0]
	default:
		log.Fatalf("invalid Point: %#v", *point)
	}
	return nil
}

func (t *Triangle) oppositePoint(t2 *Triangle, p *Point) *Point {
	cw := t2.pointCW(p)
	return t.pointCW(cw)
}

// Legalize triangle by rotating clockwise around point(0).
func (t *Triangle) legalize(point *Point) {
	t.Point[1] = t.Point[0]
	t.Point[0] = t.Point[2]
	t.Point[2] = point
}

// Legalize triagnle by rotating clockwise around oPoint.
func (t *Triangle) legalize2(opoint, npoint *Point) {
	if opoint == t.Point[0] {
		t.Point[1] = t.Point[0]
		t.Point[0] = t.Point[2]
		t.Point[2] = npoint
	} else if opoint == t.Point[1] {
		t.Point[2] = t.Point[1]
		t.Point[1] = t.Point[0]
		t.Point[0] = npoint
	} else if opoint == t.Point[2] {
		t.Point[0] = t.Point[2]
		t.Point[2] = t.Point[1]
		t.Point[1] = npoint
	} else {
		log.Fatalf("invalid point %#v", *opoint)
	}
}

func (t *Triangle) index(p *Point) int {
	switch {
	case p == t.Point[0]:
		return 0
	case p == t.Point[1]:
		return 1
	case p == t.Point[2]:
		return 2
	default:
		log.Fatalf("invalid point: %#v", *p)
	}
	return -1
}

func (t *Triangle) edgeIndex(p1, p2 *Point) int {
	if t.Point[0] == p1 {
		if t.Point[1] == p2 {
			return 2
		} else if t.Point[2] == p2 {
			return 1
		}
	} else if t.Point[1] == p1 {
		if t.Point[2] == p2 {
			return 0
		} else if t.Point[0] == p2 {
			return 2
		}
	} else if t.Point[2] == p1 {
		if t.Point[0] == p2 {
			return 1
		} else if t.Point[1] == p2 {
			return 0
		}
	}
	return -1
}

// The neighbor clockwise to given point
func (t *Triangle) neighborCW(point *Point) *Triangle {
	if point == t.Point[0] {
		return t.neighbor[1]
	} else if point == t.Point[1] {
		return t.neighbor[2]
	}
	return t.neighbor[0]
}

// The neighbor counter-clockwise to given point
func (t *Triangle) neighborCCW(point *Point) *Triangle {
	if point == t.Point[0] {
		return t.neighbor[2]
	} else if point == t.Point[1] {
		return t.neighbor[0]
	}
	return t.neighbor[1]
}

func (t *Triangle) getConstrainedEdgeCCW(p *Point) bool {
	if p == t.Point[0] {
		return t.constrainedEdge[2]
	} else if p == t.Point[1] {
		return t.constrainedEdge[0]
	}
	return t.constrainedEdge[1]
}

func (t *Triangle) getConstrainedEdgeCW(p *Point) bool {
	if p == t.Point[0] {
		return t.constrainedEdge[1]
	} else if p == t.Point[1] {
		return t.constrainedEdge[2]
	}
	return t.constrainedEdge[0]
}

func (t *Triangle) setConstrainedEdgeCCW(p *Point, ce bool) {
	if p == t.Point[0] {
		t.constrainedEdge[2] = ce
	} else if p == t.Point[1] {
		t.constrainedEdge[0] = ce
	} else {
		t.constrainedEdge[1] = ce
	}
}

func (t *Triangle) setConstrainedEdgeCW(p *Point, ce bool) {
	if p == t.Point[0] {
		t.constrainedEdge[1] = ce
	} else if p == t.Point[1] {
		t.constrainedEdge[2] = ce
	} else {
		t.constrainedEdge[0] = ce
	}
}

func (t *Triangle) getDelunayEdgeCCW(p *Point) bool {
	if p == t.Point[0] {
		return t.delaunayEdge[2]
	} else if p == t.Point[1] {
		return t.delaunayEdge[0]
	}
	return t.delaunayEdge[1]
}

func (t *Triangle) getDelunayEdgeCW(p *Point) bool {
	if p == t.Point[0] {
		return t.delaunayEdge[1]
	} else if p == t.Point[1] {
		return t.delaunayEdge[2]
	}
	return t.delaunayEdge[0]
}

func (t *Triangle) setDelunayEdgeCCW(p *Point, e bool) {
	if p == t.Point[0] {
		t.delaunayEdge[2] = e
	} else if p == t.Point[1] {
		t.delaunayEdge[0] = e
	} else {
		t.delaunayEdge[1] = e
	}
}

func (t *Triangle) setDelunayEdgeCW(p *Point, e bool) {
	if p == t.Point[0] {
		t.delaunayEdge[1] = e
	} else if p == t.Point[1] {
		t.delaunayEdge[2] = e
	} else {
		t.delaunayEdge[0] = e
	}
}

// The neighbor across to given point
func (t *Triangle) neighborAcross(opoint *Point) *Triangle {
	if opoint == t.Point[0] {
		return t.neighbor[0]
	} else if opoint == t.Point[1] {
		return t.neighbor[1]
	}
	return t.neighbor[2]
}
