/************************************************************************
 * Copyright (C) 2021 Richard Palmer
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

#include <FacetCut.h>
#include <cstdlib>
#include <cassert>
using r3d::FacetCut;
using r3d::Mesh;
using r3d::Vec3f;

namespace {
float calcw( const Vec3f &n, const Vec3f &ac, float d)
{
    float w = n.dot(ac);
    if ( fabsf(w) > 0.0f)
        w = (w - d)/w;
    return w;
}   // end calcw
}   // end namespace


FacetCut::FacetCut( const Mesh& mesh, int fid, const Vec3f& p, const Vec3f& n)
    : _mesh(mesh), _fid(fid), _a(0), _b(1), _c(2), _h(0),
    _aid(-1), _bid(-1), _cid(-1), _ain(false), _abx(0,0,0), _acx(0,0,0)
{
    assert( !p.array().isNaN().any());
    assert( !n.array().isNaN().any());
    assert( !n.isZero());

    const int *fvidxs = mesh.fvidxs(fid);
    const float d[3] = { n.dot(mesh.vtx(fvidxs[0])-p),
                         n.dot(mesh.vtx(fvidxs[1])-p),
                         n.dot(mesh.vtx(fvidxs[2])-p)};
    int s[3] = { d[0] < 0.0f ? -1 : 1,
                 d[1] < 0.0f ? -1 : 1,
                 d[2] < 0.0f ? -1 : 1};
    const int ssum = std::abs(s[0] + s[1] + s[2]);
    assert( ssum == 1 || ssum == 3);

    if ( ssum == 3) // All vertices in the same half?
        _h = s[0];  // Will be (+/-)1
    else
    {
        if ( s[0] == s[1]) // Look at vertex pairs on the same side of the boundary
        {
            _a = 2;
            _b = 0;
            _c = 1;
        }   // end if
        else if ( s[2] == s[0])
        {
            _a = 1;
            _b = 2;
            _c = 0;
        }   // end else if

        assert( s[_b] == s[_c]);    // b and c are on the same side of the boundary

        // Account for 1 or 2 vertices placed exactly on the boundary
        // and ensure they're all flagged as being on the same side.
        if ( s[_b] == -1 && d[_a] == 0.0f)  // _a incident with the slicing plane?
            s[_a] = _h = -1;
        else if ( d[_b] == 0.0f && d[_c] == 0.0f)   // _b & _c incident with the slicing plane?
            s[_b] = s[_c] = _h = -1;
    }   // end else

    // _a, _b, _c now have 1,2,3 set correctly
    _aid = fvidxs[_a];
    _bid = fvidxs[_b];
    _cid = fvidxs[_c];

    _abx = _acx = va();
    if ( _h == 0) // Straddling the plane? Set the edge intersection points.
    {
        // Calculation is always how far the point is away from a.
        // Note that it doesn't matter which direction n is in; the signs cancel.
        const Vec3f ab = vb() - va();
        _abx += calcw( n, ab, d[_b]) * ab;    // _abx is proportion of vector ab FROM va()
        const Vec3f ac = vc() - va();
        _acx += calcw( n, ac, d[_c]) * ac;    // _acx is proportion of vector ac FROM va()
    }   // end else

    _ain = s[_a] == 1;
}   // end ctor
