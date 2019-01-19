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

// Package poly2tri is a port of Poly2Tri to Go.
package poly2tri

/**
 * Sweep-line, Constrained Delauney Triangulation (CDT) See: Domiter, V. and
 * Zalik, B.(2008)'Sweep-line algorithm for constrained Delaunay triangulation',
 * International Journal of Geographical Information Science
 *
 * "FlipScan" Constrained Edge Algorithm invented by Thomas Åhlén, thahlen@gmail.com
 */

// New initializes the polyline.
func New(polyline PointArray) *SweepContext {
	tcx := &SweepContext{}
	tcx.init(polyline)
	return tcx
}

// Triangulate returns the contstrained triangles.
func (s *SweepContext) Triangulate() TriArray {
	triangulate(s)

	triangles := make(TriArray, s.triangles.Len())
	for e, i := s.triangles.Front(), 0; e != nil; e, i = e.Next(), i+1 {
		triangles[i] = e.Value.(*Triangle)
	}
	return triangles
}

// AddHole adds a hole.
func (s *SweepContext) AddHole(polyline PointArray) {
	s.addHole(polyline)
}

// AddPoint adds a point.
func (s *SweepContext) AddPoint(p *Point) {
	s.addPoint(p)
}

// Mesh returns the entire triangle mesh for debugging purposes.
func (s *SweepContext) Mesh() TriArray {
	triangles := make(TriArray, s.tmap.Len())
	for e, i := s.tmap.Front(), 0; e != nil; e, i = e.Next(), i+1 {
		triangles[i] = e.Value.(*Triangle)
	}
	return triangles
}
