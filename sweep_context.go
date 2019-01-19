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

import "container/list"

// TODO: make everything in here private

// Inital t factor, seed t will extend 30% of
// PointSet width to both left and right.
const alpha = 0.3

// SweepContext represents a sweep context.
type SweepContext struct {
	basin     *Basin
	edgeEvent *EdgeEvent
	edges     []*Edge

	triangles *list.List
	points    PointArray
	tmap      *list.List

	// Advancing front
	front *AdvancingFront
	// head and tail point used with advancing front
	head, tail *Point

	afHead, afMiddle, afTail *Node
}

// Basin represents a basin.
type Basin struct {
	leftNode, bottomNode, rightNode *Node
	width                           float64
	leftHighest                     bool
}

// EdgeEvent represents an edge event.
type EdgeEvent struct {
	constrainedEdge *Edge
	right           bool
}

// Clear clears the basin.
func (b *Basin) Clear() {
	b.leftNode = nil
	b.bottomNode = nil
	b.rightNode = nil
	b.width = 0.0
	b.leftHighest = false
}

func (s *SweepContext) init(polyline []*Point) {
	s.triangles = list.New()
	s.edges = []*Edge{}
	s.tmap = list.New()
	s.basin = new(Basin)
	s.edgeEvent = new(EdgeEvent)
	s.points = polyline
	s.initEdges(s.points)
}

func (s *SweepContext) initTriangulation() {
	xmax := s.points[0].X
	xmin := s.points[0].X
	ymax := s.points[0].Y
	ymin := s.points[0].Y

	// Calculate bounds.
	for i := 0; i < len(s.points); i++ {
		p := s.points[i]
		if p.X > xmax {
			xmax = p.X
		}
		if p.X < xmin {
			xmin = p.X
		}
		if p.Y > ymax {
			ymax = p.Y // Was p.X !!!
		}
		if p.Y < ymin {
			ymin = p.Y
		}
	}

	dx := alpha * (xmax - xmin)
	dy := alpha * (ymax - ymin)
	s.head = &Point{X: xmax + dx, Y: ymin - dy}
	s.tail = &Point{X: xmin - dx, Y: ymin - dy}

	// Sort points along y-axis.
	s.points.Sort()
}

func (s *SweepContext) initEdges(polyline []*Point) {
	numPoints := len(polyline)
	for i := 0; i < numPoints; i++ {
		var j int
		if i < numPoints-1 {
			j = i + 1
		}
		p1 := polyline[i]
		p2 := polyline[j]
		e := new(Edge)
		e.init(p1, p2)
		s.edges = append(s.edges, e)
	}
}

func (s *SweepContext) addHole(polyline []*Point) {
	s.initEdges(polyline)
	n := len(polyline)
	for i := 0; i < n; i++ {
		s.points = append(s.points, polyline[i])
	}
}

func (s *SweepContext) addPoint(point *Point) {
	s.points = append(s.points, point)
}

func (s *SweepContext) locateNode(point *Point) *Node {
	// TODO implement search tree
	return s.front.locateNode(point.X)
}

// TODO: why pass nodes into this function?
func (s *SweepContext) createAdvancingFront() {
	// Initial t
	t := new(Triangle)
	t.init(s.points[0], s.tail, s.head)

	t.eref = s.tmap.PushBack(t)

	s.afHead = &Node{point: t.Point[1], triangle: t, value: t.Point[1].X}
	s.afMiddle = &Node{point: t.Point[0], triangle: t, value: t.Point[0].X}
	s.afTail = &Node{point: t.Point[2], value: t.Point[2].X}
	s.front = new(AdvancingFront)
	s.front.init(s.afHead, s.afTail)

	// TODO: More intuitive if head is middle's next and not previous?
	//       so swap head and tail
	s.afHead.next = s.afMiddle
	s.afMiddle.next = s.afTail
	s.afMiddle.prev = s.afHead
	s.afTail.prev = s.afMiddle
}

// RemoveNode removes a node.
func (s *SweepContext) RemoveNode(node *Node) {
	node = nil
}

func (s *SweepContext) mapTriangleToNodes(t *Triangle) {
	for i := 0; i < 3; i++ {
		if t.neighbor[i] == nil {
			n := s.front.locatePoint(t.pointCW(t.Point[i]))
			if n != nil {
				n.triangle = t
			}
		}
	}
}

// RemoveFromMap removes a triangle from the map.
func (s *SweepContext) RemoveFromMap(t *Triangle) {
	s.tmap.Remove(t.eref)
}

func (s *SweepContext) meshClean(t *Triangle) {
	if t != nil && !t.interior {
		t.interior = true
		s.triangles.PushBack(t)
		for i := 0; i < 3; i++ {
			if !t.constrainedEdge[i] {
				s.meshClean(t.neighbor[i])
			}
		}
	}
}
