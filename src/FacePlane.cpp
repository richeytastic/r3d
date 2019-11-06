/************************************************************************
 * Copyright (C) 2019 Richard Palmer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ************************************************************************/

#include <FacePlane.h>
#include <cassert>
#include <cmath>
using r3d::FacePlane;
using r3d::Mesh;
using r3d::Vec3f;


FacePlane::FacePlane( const Mesh& src, int fid, const Vec3f& p, const Vec3f& n)
    : _mesh(src), _fid(fid), _fvidxs( _mesh.fvidxs(fid)), _p(p), _n(n), _ain(true), _nih(_vertexInHalf()) { }   // end ctor


Vec3f FacePlane::abIntersection() const
{
    assert( _nih == 0);
    return linePlaneIntersection( _p, _n, va(), vb());
}   // end abIntersection


Vec3f FacePlane::acIntersection() const
{
    assert( _nih == 0);
    return linePlaneIntersection( _p, _n, va(), vc());
}   // end acIntersection


// Returns 1 if all vertices of the face are in the half of the space pointing into by _n.
// Returns -1 if all vertices of the face are in the half of the space opposite to _n.
// Returns 0 if face crosses the boundary, sets a to be the index into fvidxs on the
// side in which only that single vertex resides, and sets the sign of n to point
// into the half that this vertex resides.
int FacePlane::_vertexInHalf()   // On return, a is on {0,1,2}
{
    const Vec3f& x0 = _mesh.vtx(_fvidxs[0]);
    const Vec3f& x1 = _mesh.vtx(_fvidxs[1]);
    const Vec3f& x2 = _mesh.vtx(_fvidxs[2]);

    // If the given point is exactly the same as one of the vertices then this function won't return +/- 1
    const Vec3f& p = _p;
    const Vec3f dv0 = x0 - p;
    const Vec3f dv1 = x1 - p;
    const Vec3f dv2 = x2 - p;

    int s0 = dv0.squaredNorm() > 0 ? int(copysign( 1, _n.dot(dv0))) : 0;
    int s1 = dv1.squaredNorm() > 0 ? int(copysign( 1, _n.dot(dv1))) : 0;
    int s2 = dv2.squaredNorm() > 0 ? int(copysign( 1, _n.dot(dv2))) : 0;

    _a = 0;
    _b = 1;
    _c = 2;

    // If all of the direction signs are the same (i.e., all 1 or all -1) then all vertices
    // of this triangle are either within the required half (+1) or outside of it (-1).
    if ( fabs(s0 + s1 + s2) == 3)   // s0 == s1 == s2
        return s0;  // Will be (+/-)1.

    // If any of {s0,s1,s2} are zero then the intersecting plane crosses through the respective
    // vertices. It is only possible for 1 or 2 of {s0,s1,s2} to be zero (i.e. to be crossed through
    // by the intersecting plane). If this is the case, the triangle must ALWAYS be split - it CANNOT
    // be the case that all vertices are in the same half (otherwise we could encounter the possibility
    // of the plane sitting exactly between two triangles along their joining edge and there would be
    // no intersecting positions!

    // There are 9 possible cases for the plane with respect to the triangle. The first two cases
    // (and the most common) are where all three of the vertices are on one side of the plane. These
    // two cases are taken care of by the above return statement. The remaining 7 cases are:
    // 1) The plane (line) crosses two edges (by far the most typical case of intersection)
    //    meaning that two of {s0,s1,s2} == {-1,1} (2 cases).
    // 2) The plane crosses through a single vertex of the triangle (rare case) meaning that the
    //    respective si value is zero and the other two sj,sk are either 1 or -1 (3 cases).
    // 3) The plane crosses through two vertices of the triangle (extremely rare case) meaning that
    //    two of {s0,s1,s2} == 0 (2 cases).

    // 1) and 3) above (4 cases) and 2 cases of 2) are accounted for by the first three conditionals.
    // The last conditional deals with the case where the plane goes through a single vertex and an edge.
    if ( s0 == s1)
    {
        _a = 2;
        _ain = s2 > 0 || s0 < 0;
    }   // end if
    else if ( s1 == s2)
    {
        _a = 0;
        _ain = s0 > 0 || s1 < 0;
    }   // end else if
    else if ( s2 == s0)
    {
        _a = 1;
        _ain = s1 > 0 || s2 < 0;
    }   // end else if
    else
    {
        _ain = true;
        if ( s1 == 1)
            _a = 1;
        else if ( s2 == 1)
            _a = 2;
    }   // end else

    _b = (_a+1)%3;
    _c = (_a+2)%3;
    return 0;
}   // end _vertexInHalf
