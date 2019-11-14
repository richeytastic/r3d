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
#include <cstdlib>
#include <cmath>
using r3d::FacePlane;
using r3d::Mesh;
using r3d::Vec3f;

namespace {
float calcw( const Vec3f &n, const Vec3f &ac, float dp)
{
    static const float EPS = 1e-4f; // DO NOT CHANGE TO BE ANY SMALLER!
    float w = n.dot(ac);
    if ( fabsf(w) > EPS)
        w = (w - dp)/w; // Always +ve since numerator and denominator cancel
    if ( w <= EPS)
        w = 0.0f;
    return w;
}   // end calcwA


int setSide( float dp)
{
    int s = 0;
    if ( std::fpclassify(dp) != FP_ZERO)
    //if ( fabsf(dp) > 1e-5f)
        s = int(std::copysignf( 1.0f, dp));
    return s;
}   // end setSide

}   // end namespace


FacePlane::FacePlane( const Mesh& src, int fid, const Vec3f& p, const Vec3f& n)
    : _mesh(src), _fid(fid), _fvidxs( _mesh.fvidxs(fid)),
      _ain(false), _straddleId(-1), _a(0), _b(1), _c(2), _nih(0)
{
    assert( !p.array().isNaN().any());
    assert( !n.array().isNaN().any());

    const Vec3f &x0 = _mesh.vtx(_fvidxs[0]);
    const Vec3f &x1 = _mesh.vtx(_fvidxs[1]);
    const Vec3f &x2 = _mesh.vtx(_fvidxs[2]);

    const float dp[3] = { n.dot(x0-p), n.dot(x1-p), n.dot(x2-p)};
    const int s[3] = { setSide( dp[0]), setSide( dp[1]), setSide( dp[2])};
    // Note that for dot products NEAR zero, the sign bit is still set correctly so that
    // even if vertices lie exactly on the boundary, they are still treated as being on one side of it.
    // However, if p is EXACTLY incident with one of the vertices, then the dot product will be exactly
    // zero. Because the copysign function still looks at the sign bit, we need to be careful and
    // explicitly check for zero (in the setSide function).
    // This situation can happen with a single vertex, or two vertices along an edge.
    // In theory, it can happen with all three vertices if the area of the face is zero, but
    // the design of the Mesh class disallows this since vertices are not allowed to be conincident,
    // and the edges of a face must be linearly independent.

    // If all direction signs are same then all vertices are either in required half (+1) or outside it (-1).
    if ( abs(s[0] + s[1] + s[2]) == 3)   // s0 == s1 == s2
        _nih = s[0];  // Will be (+/-)1.
    else
    {
        if ( s[0] == 0) // _a will be zero unless another vertex is also on the edge
        {
            // If either of the other vertices are also on the boundary, then _a is set
            // to the vertex that is off the boundary. Note that the design of Mesh precludes
            // all three vertices being zero since this implies a face with zero area and therefore
            // all of the vertices being superimposed upon one another (which is disallowed by
            // virtue of the Mesh class checking a hash of vertex positions).
            assert( s[1] != 0 || s[2] != 0);

            if ( s[1] == 0)
                _a = 2;
            else if ( s[2] == 0)
                _a = 1;
            else if ( s[1] != s[2])
            {
                // However, if _a (at zero) is on a vertex AND the other vertices are on different
                // sides of the boundary (i.e. stradle it), then _a is set to be one of these other
                // vertices. If this is not done then the intersection of the boundary with two
                // edges that connect _a cannot be performed since the boundary passes through
                // the vertex at _a and its opposite edge.
                _straddleId = _fvidxs[_a];
                _a = 1;
            }   // end else
        }   // end else if
        else if ( s[1] == 0)
        {
            _a = 1;
            // No need to check if s[0] == 0 here since was checked above, but still need
            // to check if the other vertices are on either side of the boundary.
            if ( s[2] == 0 || s[0] != s[2])
            {
                _straddleId = _fvidxs[_a];
                _a = 0;
            }   // end if
        }   // end else if
        else if ( s[2] == 0)
        {
            _a = 2;
            // Again, no need to check the other vertices if they're zero at this point.
            // But still must check if the other two vertices stradle the boundary.
            if ( s[0] != s[1])
            {
                _straddleId = _fvidxs[_a];
                _a = 0;
            }   // end if
        }   // end else if
        else if ( s[0] == s[1]) // Look at vertex pairs on the same side of the boundary
            _a = 2;
        else if ( s[2] == s[0])
            _a = 1;
        /* // Don't need to check this situation since _a defaults to zero
        else if ( s[1] == s[2])
            _a = 0;
        */

        _b = (_a+1)%3;
        _c = (_a+2)%3;

        // Set the intersecting points on lines a-b and a-c
        const Vec3f &v0 = va();
        const Vec3f &v1 = vb();
        const Vec3f &v2 = vc();

        // Note that the calculation of the intersecting points
        // is always in terms of how far the point is AWAY FROM A.
        // This is important because in some pathological cases, all
        // vertices of the triangle are going to be exactly on the boundary
        // and in such cases, the intersecting points on the two edges
        // should be coincident with vertex a.

        // Note also that it doesn't matter which direction n is in - the signs
        // in the numerator and denominator of the coefficient cancel.
        const Vec3f ab = v1 - v0;
        _abx = v0 + calcw( n, ab, dp[_b]) * ab;    // _abx is proportion of vector ab FROM a (v0)
        const Vec3f ac = v2 - v0;
        _acx = v0 + calcw( n, ac, dp[_c]) * ac;    // _acx is proportion of vector ac FROM a (v0)
    }   // end else

    _ain = std::copysignf( 1.0f, dp[_a]) > 0;
}   // end ctor
