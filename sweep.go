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
	"fmt"
	"log"
	"math"
)

// Triangulate simple polygon with holes
func triangulate(tcx *SweepContext) {
	//s.nodes = new(vector.Vector)
	tcx.initTriangulation()
	tcx.createAdvancingFront()
	// Sweep points; build mesh
	sweepPoints(tcx)
	// Clean up
	finalizationPolygon(tcx)
}

func sweepPoints(tcx *SweepContext) {
	for i := 1; i < len(tcx.points); i++ {
		point := tcx.points[i]
		node := pointEvent(tcx, point)
		for i := 0; i < len(point.edges); i++ {
			initSweepEdgeEvent(tcx, point.edges[i], node)
		}
	}
}

// TODO: Move this function to sweep_context?
func finalizationPolygon(tcx *SweepContext) {
	// Get an Internal triangle to start with
	t := tcx.front.head.next.triangle
	p := tcx.front.head.next.point
	for t != nil && !t.getConstrainedEdgeCW(p) {
		t = t.neighborCCW(p)
	}

	// Collect interior triangles constrained by edges
	tcx.meshClean(t)
}

func pointEvent(tcx *SweepContext, point *Point) *Node {

	node := tcx.locateNode(point)
	newNode := newFrontTriangle(tcx, point, node)

	// Only need to check +Epsilon since point never have smaller
	// x value than node due to how we fetch nodes from the front
	if point.X <= node.point.X+Epsilon {
		fill(tcx, node)
	}

	fillAdvancingFront(tcx, newNode)
	return newNode
}

func newFrontTriangle(tcx *SweepContext, point *Point, node *Node) *Node {

	triangle := new(Triangle)
	triangle.init(point, node.point, node.next.point)

	triangle.markNeighbor2(node.triangle)
	triangle.eref = tcx.tmap.PushBack(triangle)

	newNode := &Node{point: point, value: point.X}
	//s.nodes.Push(newNode)

	newNode.next = node.next
	newNode.prev = node
	node.next.prev = newNode
	node.next = newNode

	if !legalize(tcx, triangle) {
		tcx.mapTriangleToNodes(triangle)
	}

	return newNode
}

func legalize(tcx *SweepContext, t *Triangle) bool {

	// To legalize a triangle we start by finding if any of the three edges
	// violate the Delaunay condition
	for i := 0; i < 3; i++ {
		if t.delaunayEdge[i] {
			continue
		}

		ot := t.neighbor[i]

		if ot != nil {

			p := t.Point[i]
			op := ot.oppositePoint(t, p)
			oi := ot.index(op)

			// If this is a Constrained Edge or a Delaunay Edge(only during recursive legalization)
			// then we should not try to legalize
			if ot.constrainedEdge[oi] || ot.delaunayEdge[oi] {
				t.constrainedEdge[i] = ot.constrainedEdge[oi]
				continue
			}

			inside := incircle(p, t.pointCCW(p), t.pointCW(p), op)

			if inside {
				// Lets mark this shared edge as Delaunay
				t.delaunayEdge[i] = true
				ot.delaunayEdge[oi] = true

				// Lets rotate shared edge one vertex CW to legalize it
				rotateTrianglePair(t, p, ot, op)

				// We now got one valid Delaunay Edge shared by two triangles
				// This gives us 4 new edges to check for Delaunay

				// Make sure that triangle to node mapping is done only one time for a specific triangle
				notLegalized := !legalize(tcx, t)
				if notLegalized {
					tcx.mapTriangleToNodes(t)
				}

				notLegalized = !legalize(tcx, ot)
				if notLegalized {
					tcx.mapTriangleToNodes(ot)
				}

				// Reset the Delaunay edges, since they only are valid Delaunay edges
				// until we add a new triangle or point.
				// XXX: need to think about this. Can these edges be tried after we
				//      return to previous recursive level?
				t.delaunayEdge[i] = false
				ot.delaunayEdge[oi] = false

				// If triangle have been legalized no need to check the other edges since
				// the recursive legalization will handles those so we can end here.
				return true
			}
		}
	}
	return false
}

func incircle(pa, pb, pc, pd *Point) bool {

	adx := pa.X - pd.X
	ady := pa.Y - pd.Y
	bdx := pb.X - pd.X
	bdy := pb.Y - pd.Y

	adxbdy := adx * bdy
	bdxady := bdx * ady
	oabd := adxbdy - bdxady

	if oabd <= 0 {
		return false
	}

	cdx := pc.X - pd.X
	cdy := pc.Y - pd.Y

	cdxady := cdx * ady
	adxcdy := adx * cdy
	ocad := cdxady - adxcdy

	if ocad <= 0 {
		return false
	}

	bdxcdy := bdx * cdy
	cdxbdy := cdx * bdy

	alift := adx*adx + ady*ady
	blift := bdx*bdx + bdy*bdy
	clift := cdx*cdx + cdy*cdy

	det := alift*(bdxcdy-cdxbdy) + blift*ocad + clift*oabd

	return det > 0
}

func rotateTrianglePair(t *Triangle, p *Point, ot *Triangle, op *Point) {
	var n1, n2, n3, n4 *Triangle
	n1 = t.neighborCCW(p)
	n2 = t.neighborCW(p)
	n3 = ot.neighborCCW(op)
	n4 = ot.neighborCW(op)

	var ce1, ce2, ce3, ce4 bool
	ce1 = t.getConstrainedEdgeCCW(p)
	ce2 = t.getConstrainedEdgeCW(p)
	ce3 = ot.getConstrainedEdgeCCW(op)
	ce4 = ot.getConstrainedEdgeCW(op)

	var de1, de2, de3, de4 bool
	de1 = t.getDelunayEdgeCCW(p)
	de2 = t.getDelunayEdgeCW(p)
	de3 = ot.getDelunayEdgeCCW(op)
	de4 = ot.getDelunayEdgeCW(op)

	t.legalize2(p, op)
	ot.legalize2(op, p)

	// Remap delaunayEdge
	ot.setDelunayEdgeCCW(p, de1)
	t.setDelunayEdgeCW(p, de2)
	t.setDelunayEdgeCCW(op, de3)
	ot.setDelunayEdgeCW(op, de4)

	// Remap constrainedEdge
	ot.setConstrainedEdgeCCW(p, ce1)
	t.setConstrainedEdgeCW(p, ce2)
	t.setConstrainedEdgeCCW(op, ce3)
	ot.setConstrainedEdgeCW(op, ce4)

	// Remap neighbors
	// XXX: might optimize the markNeighbor by keeping track of
	//      what side should be assigned to what neighbor after the
	//      rotation. Now mark neighbor does lots of testing to find
	//      the right side.
	t.clearNeighbors()
	ot.clearNeighbors()
	if n1 != nil {
		ot.markNeighbor2(n1)
	}
	if n2 != nil {
		t.markNeighbor2(n2)
	}
	if n3 != nil {
		t.markNeighbor2(n3)
	}
	if n4 != nil {
		ot.markNeighbor2(n4)
	}
	t.markNeighbor2(ot)
}

func initSweepEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {

	tcx.edgeEvent.constrainedEdge = edge
	tcx.edgeEvent.right = (edge.p.X > edge.q.X)

	if isEdgeSideOfTriangle(node.triangle, edge.p, edge.q) {
		return
	}

	// For now we will do all needed filling
	// TODO: integrate with flip process might give some better performance
	//       but for now this avoid the issue with cases that needs both flips and fills
	fillEdgeEvent(tcx, edge, node)
	sweepEdgeEvent(tcx, edge.p, edge.q, node.triangle, edge.q)
}

func fillAdvancingFront(tcx *SweepContext, n *Node) {
	// fill right holes
	node := n.next
	for node.next != nil {
		var angle = holeAngle(node)
		if angle > PiOver2 || angle < -PiOver2 {
			break
		}
		fill(tcx, node)
		node = node.next
	}

	// fill left holes
	node = n.prev
	for node.prev != nil {
		var angle = holeAngle(node)
		if angle > PiOver2 || angle < -PiOver2 {
			break
		}
		fill(tcx, node)
		node = node.prev
	}

	// fill right basins
	if n.next != nil && n.next.next != nil {
		var angle = basinAngle(n)
		if angle < Pi3Over4 {
			fillBasin(tcx, n)
		}
	}
}

func sweepEdgeEvent(tcx *SweepContext, ep, eq *Point, triangle *Triangle, point *Point) {
	if isEdgeSideOfTriangle(triangle, ep, eq) {
		return
	}

	p1 := triangle.pointCCW(point)
	o1 := orient2d(eq, p1, ep)
	if o1 == COLLINEAR {
		if triangle.containsPoints(eq, p1) {
			triangle.markConstrainedEdge3(eq, p1)
			// We are modifying the constraint maybe it would be better to
			// not change the given constraint and just keep a variable for the new constraint
			tcx.edgeEvent.constrainedEdge.q = p1
			triangle = triangle.neighborAcross(point)
			sweepEdgeEvent(tcx, ep, p1, triangle, p1)
		} else {
			panic(fmt.Sprintf("EdgeEvent - collinear points not supported"))
		}
		return
	}

	p2 := triangle.pointCW(point)
	o2 := orient2d(eq, p2, ep)
	if o2 == COLLINEAR {
		if triangle.containsPoints(eq, p2) {
			triangle.markConstrainedEdge3(eq, p2)
			// We are modifying the constraint maybe it would be better to
			// not change the given constraint and just keep a variable for the new constraint
			tcx.edgeEvent.constrainedEdge.q = p2
			triangle = triangle.neighborAcross(point)
			sweepEdgeEvent(tcx, ep, p2, triangle, p2)
		} else {
			panic(fmt.Sprintf("EdgeEvent - collinear points not supported"))
		}
		return
	}

	if o1 == o2 {
		// Need to decide if we are rotating CW or CCW to get to a triangle
		// that will cross edge
		if o1 == CW {
			triangle = triangle.neighborCCW(point)
		} else {
			triangle = triangle.neighborCW(point)
		}
		sweepEdgeEvent(tcx, ep, eq, triangle, point)
	} else {
		// This triangle crosses constraint so lets flippin start!
		flipEdgeEvent(tcx, ep, eq, triangle, point)
	}
}

func isEdgeSideOfTriangle(t *Triangle, ep, eq *Point) bool {
	index := t.edgeIndex(ep, eq)
	if index != -1 {
		t.markConstrainedEdge(index)
		var tn = t.neighbor[index]
		if tn != nil {
			tn.markConstrainedEdge3(ep, eq)
		}
		return true
	}
	return false
}

func fill(tcx *SweepContext, node *Node) {
	t := new(Triangle)
	t.init(node.prev.point, node.point, node.next.point)

	// TODO: should copy the constrainedEdge value from neighbor triangles
	//       for now constrainedEdge values are copied during the legalize
	t.markNeighbor2(node.prev.triangle)
	t.markNeighbor2(node.triangle)

	t.eref = tcx.tmap.PushBack(t)

	// Update the advancing front
	node.prev.next = node.next
	node.next.prev = node.prev

	// If it was legalized the triangle has already been mapped
	if !legalize(tcx, t) {
		tcx.mapTriangleToNodes(t)
	}

}

func basinAngle(node *Node) float64 {
	ax := node.point.X - node.next.next.point.X
	ay := node.point.Y - node.next.next.point.Y
	return math.Atan2(ay, ax)
}

func holeAngle(node *Node) float64 {
	/* Complex plane
	 * ab = cosA +i*sinA
	 * ab = (ax + ay*i)(bx + by*i) = (ax*bx + ay*by) + i(ax*by-ay*bx)
	 * atan2(y,x) computes the principal value of the argument function
	 * applied to the complex number x+iy
	 * Where x = ax*bx + ay*by
	 *       y = ax*by - ay*bx
	 */
	ax := node.next.point.X - node.point.X
	ay := node.next.point.Y - node.point.Y
	bx := node.prev.point.X - node.point.X
	by := node.prev.point.Y - node.point.Y
	return math.Atan2(ax*by-ay*bx, ax*bx+ay*by)
}

func fillBasin(tcx *SweepContext, node *Node) {

	if orient2d(node.point, node.next.point, node.next.next.point) == CCW {
		tcx.basin.leftNode = node.next.next
	} else {
		tcx.basin.leftNode = node.next
	}

	// Find the bottom and right node
	tcx.basin.bottomNode = tcx.basin.leftNode
	for tcx.basin.bottomNode.next != nil &&
		tcx.basin.bottomNode.point.Y >= tcx.basin.bottomNode.next.point.Y {
		tcx.basin.bottomNode = tcx.basin.bottomNode.next
	}
	if tcx.basin.bottomNode == tcx.basin.leftNode {
		// No valid basin
		return
	}

	tcx.basin.rightNode = tcx.basin.bottomNode
	for tcx.basin.rightNode.next != nil &&
		tcx.basin.rightNode.point.Y < tcx.basin.rightNode.next.point.Y {
		tcx.basin.rightNode = tcx.basin.rightNode.next
	}
	if tcx.basin.rightNode == tcx.basin.bottomNode {
		// No valid basins
		return
	}

	tcx.basin.width = tcx.basin.rightNode.point.X - tcx.basin.leftNode.point.X
	tcx.basin.leftHighest = tcx.basin.leftNode.point.Y > tcx.basin.rightNode.point.Y

	fillBasinReq(tcx, tcx.basin.bottomNode)
}

func fillBasinReq(tcx *SweepContext, node *Node) {
	// if shallow stop filling
	if isShallow(tcx, node) {
		return
	}

	fill(tcx, node)

	if node.prev == tcx.basin.leftNode && node.next == tcx.basin.rightNode {
		return
	} else if node.prev == tcx.basin.leftNode {
		o := orient2d(node.point, node.next.point, node.next.next.point)
		if o == CW {
			return
		}
		node = node.next
	} else if node.next == tcx.basin.rightNode {
		o := orient2d(node.point, node.prev.point, node.prev.prev.point)
		if o == CCW {
			return
		}
		node = node.prev
	} else {
		// Continue with the neighbor node with lowest Y value
		if node.prev.point.Y < node.next.point.Y {
			node = node.prev
		} else {
			node = node.next
		}
	}

	fillBasinReq(tcx, node)
}

func isShallow(tcx *SweepContext, node *Node) bool {
	var height float64
	if tcx.basin.leftHighest {
		height = tcx.basin.leftNode.point.Y - node.point.Y
	} else {
		height = tcx.basin.rightNode.point.Y - node.point.Y
	}

	// if shallow, stop filling
	if tcx.basin.width > height {
		return true
	}
	return false
}

func fillEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	if tcx.edgeEvent.right {
		fillRightAboveEdgeEvent(tcx, edge, node)
	} else {
		fillLeftAboveEdgeEvent(tcx, edge, node)
	}
}

func fillRightAboveEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	for node.next.point.X < edge.p.X {
		// Check if next node is below the edge
		if orient2d(edge.q, node.next.point, edge.p) == CCW {
			fillRightBelowEdgeEvent(tcx, edge, node)
		} else {
			node = node.next
		}
	}
}

func fillRightBelowEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	if node.point.X < edge.p.X {
		if orient2d(node.point, node.next.point, node.next.next.point) == CCW {
			// Concave
			fillRightConcaveEdgeEvent(tcx, edge, node)
		} else {
			// Convex
			fillRightConvexEdgeEvent(tcx, edge, node)
			// Retry this one
			fillRightBelowEdgeEvent(tcx, edge, node)
		}
	}
}

func fillRightConcaveEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	fill(tcx, node.next)
	if node.next.point != edge.p {
		// Next above or below edge?
		if orient2d(edge.q, node.next.point, edge.p) == CCW {
			// Below
			if orient2d(node.point, node.next.point, node.next.next.point) == CCW {
				// Next is concave
				fillRightConcaveEdgeEvent(tcx, edge, node)
			} else {
				// Next is convex
			}
		}
	}
}

func fillRightConvexEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	// Next concave or convex?
	if orient2d(node.next.point, node.next.next.point, node.next.next.next.point) == CCW {
		// Concave
		fillRightConcaveEdgeEvent(tcx, edge, node.next)
	} else {
		// Convex
		// Next above or below edge?
		if orient2d(edge.q, node.next.next.point, edge.p) == CCW {
			// Below
			fillRightConvexEdgeEvent(tcx, edge, node.next)
		} else {
			// Above
		}
	}
}

func fillLeftAboveEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	for node.prev.point.X > edge.p.X {
		// Check if next node is below the edge
		if orient2d(edge.q, node.prev.point, edge.p) == CW {
			fillLeftBelowEdgeEvent(tcx, edge, node)
		} else {
			node = node.prev
		}
	}
}

func fillLeftBelowEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	if node.point.X > edge.p.X {
		if orient2d(node.point, node.prev.point, node.prev.prev.point) == CW {
			// Concave
			fillLeftConcaveEdgeEvent(tcx, edge, node)
		} else {
			// Convex
			fillLeftConvexEdgeEvent(tcx, edge, node)
			// Retry this one
			fillLeftBelowEdgeEvent(tcx, edge, node)
		}
	}
}

func fillLeftConvexEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	// Next concave or convex?
	if orient2d(node.prev.point, node.prev.prev.point, node.prev.prev.prev.point) == CW {
		// Concave
		fillLeftConcaveEdgeEvent(tcx, edge, node.prev)
	} else {
		// Convex
		// Next above or below edge?
		if orient2d(edge.q, node.prev.prev.point, edge.p) == CW {
			// Below
			fillLeftConvexEdgeEvent(tcx, edge, node.prev)
		} else {
			// Above
		}
	}
}

func fillLeftConcaveEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	fill(tcx, node.prev)
	if node.prev.point != edge.p {
		// Next above or below edge?
		if orient2d(edge.q, node.prev.point, edge.p) == CW {
			// Below
			if orient2d(node.point, node.prev.point, node.prev.prev.point) == CW {
				// Next is concave
				fillLeftConcaveEdgeEvent(tcx, edge, node)
			} else {
				// Next is convex
			}
		}
	}
}

func flipEdgeEvent(tcx *SweepContext, ep, eq *Point, t *Triangle, p *Point) {
	ot := t.neighborAcross(p)
	op := ot.oppositePoint(t, p)

	if ot == nil {
		// If we want to integrate the fillEdgeEvent do it here
		// With current implementation we should never get here
		panic(fmt.Sprintf("[BUG:FIXME] FLIP failed due to missing triangle"))
	}

	if inScanArea(p, t.pointCCW(p), t.pointCW(p), op) {
		// Lets rotate shared edge one vertex CW
		rotateTrianglePair(t, p, ot, op)
		tcx.mapTriangleToNodes(t)
		tcx.mapTriangleToNodes(ot)

		if p == eq && op == ep {
			if eq == tcx.edgeEvent.constrainedEdge.q && ep == tcx.edgeEvent.constrainedEdge.p {
				t.markConstrainedEdge3(ep, eq)
				ot.markConstrainedEdge3(ep, eq)
				legalize(tcx, t)
				legalize(tcx, ot)
			} else {
				// XXX: I think one of the triangles should be legalized here?
			}
		} else {
			o := orient2d(eq, op, ep)
			t = nextFlipTriangle(tcx, o, t, ot, p, op)
			flipEdgeEvent(tcx, ep, eq, t, p)
		}
	} else {
		newP := nextFlipPoint(ep, eq, ot, op)
		flipScanEdgeEvent(tcx, ep, eq, t, ot, newP)
		sweepEdgeEvent(tcx, ep, eq, t, p)
	}
}

func nextFlipTriangle(tcx *SweepContext, o int, t, ot *Triangle, p, op *Point) *Triangle {
	if o == CCW {
		// ot is not crossing edge after flip
		edgeIndex := ot.edgeIndex(p, op)
		ot.delaunayEdge[edgeIndex] = true
		legalize(tcx, ot)
		ot.clearDelunayEdges()
		return t
	}

	// t is not crossing edge after flip
	edgeIndex := t.edgeIndex(p, op)

	t.delaunayEdge[edgeIndex] = true
	legalize(tcx, t)
	t.clearDelunayEdges()
	return ot
}

func nextFlipPoint(ep, eq *Point, ot *Triangle, op *Point) *Point {
	o2d := orient2d(eq, op, ep)
	if o2d == CW {
		// Right
		return ot.pointCCW(op)
	} else if o2d == CCW {
		// Left
		return ot.pointCW(op)
	}
	log.Fatalf("[Unsupported] Opposing point on constrained edge")
	return nil
}

func flipScanEdgeEvent(tcx *SweepContext, ep, eq *Point, flipTriangle, t *Triangle, p *Point) {
	ot := t.neighborAcross(p)
	op := ot.oppositePoint(t, p)

	if t.neighborAcross(p) == nil {
		// If we want to integrate the fillEdgeEvent do it here
		// With current implementation we should never get here
		//throw new RuntimeException( "[BUG:FIXME] FLIP failed due to missing triangle");
		panic(fmt.Sprintf("[BUG:FIXME] FLIP failed due to missing triangle"))
	}

	if inScanArea(eq, flipTriangle.pointCCW(eq), flipTriangle.pointCW(eq), op) {
		// flip with new edge op.eq
		flipEdgeEvent(tcx, eq, op, ot, op)
		// TODO: Actually I just figured out that it should be possible to
		//       improve this by getting the next ot and op before the the above
		//       flip and continue the flipScanEdgeEvent here
		// set new ot and op here and loop back to inScanArea test
		// also need to set a new flipTriangle first
		// Turns out at first glance that this is somewhat complicated
		// so it will have to wait.
	} else {
		newP := nextFlipPoint(ep, eq, ot, op)
		flipScanEdgeEvent(tcx, ep, eq, flipTriangle, ot, newP)
	}
}
