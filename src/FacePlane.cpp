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
    static const float EPS = 1e-4f; // DO NOT CHANGE TO ANY LOWER!
    float w = n.dot(ac);
    if ( fabsf(w) > EPS)
        w = (w - dp)/w; // Always +ve since numerator and denominator cancel
    if ( w <= EPS)
        w = 0.0f;
    return w;
}   // end calcw
}   // end namespace


FacePlane::FacePlane( const Mesh& src, int fid, const Vec3f& p, const Vec3f& n)
    : _mesh(src), _fid(fid), _fvidxs( _mesh.fvidxs(fid)),
      _ain(false), _a(0), _b(1), _c(2), _nih(0)
{
    const Vec3f &x0 = _mesh.vtx(_fvidxs[0]);
    const Vec3f &x1 = _mesh.vtx(_fvidxs[1]);
    const Vec3f &x2 = _mesh.vtx(_fvidxs[2]);

    const float dp[3] = {n.dot(x0-p), n.dot(x1-p), n.dot(x2-p)};
    const int s[3] = {int(std::copysignf( 1.0f, dp[0])), 
                      int(std::copysignf( 1.0f, dp[1])), 
                      int(std::copysignf( 1.0f, dp[2]))};
    // Note that for dot products of zero (or -zero), the sign bit is still set correctly so that
    // even if vertices lie exactly on the boundary, they are still treated as being on one side of it.

    // If all direction signs are same then all vertices are either in required half (+1) or outside it (-1).
    if ( abs(s[0] + s[1] + s[2]) == 3)   // s0 == s1 == s2
        _nih = s[0];  // Will be (+/-)1.
    else
    {
        // Find vertex _a by looking at vertex pairs to see if they're on the same side of the boundary.
        if ( s[0] == s[1])
            _a = 2;
        else if ( s[1] == s[2])
            _a = 0;
        else if ( s[2] == s[0])
            _a = 1;
        else
            assert(false);

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

    _ain = s[_a] > 0;
}   // end ctor
