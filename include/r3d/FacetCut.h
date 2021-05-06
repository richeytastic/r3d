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

#ifndef R3D_FACET_CUT_H
#define R3D_FACET_CUT_H

/**
 * Find how to split an Face into two pieces and know which side is on the "inside" of
 * a space divided by an arbitrary plane.
 */

#include "Mesh.h"

namespace r3d {

class r3d_EXPORT FacetCut
{
public:
    // f: id of facet to test,
    // p: point on the plane,
    // n: the plane's unit normal vector - points inside the half space to keep.
    FacetCut( const Mesh&, int fid, const Vec3f& p, const Vec3f& n);

    // Returns -1 if all three facet vertices are in the outside half,
    // 1 if all three facet vertices are in the inside half, and 0 if the
    // facet stradles the plane. If 0 is returned, then va() is the vertex
    // by itself on one side of the plane and vb() and vc() are the two on
    // the other side. va() is in the inside half if inside() returns true.
    inline int half() const { return _h;}

    // Returns true iff va() is inside half space defined by n (i.e. (a-p).n is +ve).
    inline bool inside() const { return _ain;}

    // Returns point on edge ab that intersects with the plane.
    // Only use if half() returns zero.
    inline const Vec3f& abIntersection() const { return _abx;}

    // Returns point on edge ac that intersects with the plane.
    // Only use if half() returns zero.
    inline const Vec3f& acIntersection() const { return _acx;}

    inline int vaid() const { return _aid;}
    inline int vbid() const { return _bid;}
    inline int vcid() const { return _cid;}

    inline const Vec3f& va() const { return _mesh.vtx(_aid);}
    inline const Vec3f& vb() const { return _mesh.vtx(_bid);}
    inline const Vec3f& vc() const { return _mesh.vtx(_cid);}

    inline const Vec2f& uva() const { return _mesh.faceUV(_fid, _a);}
    inline const Vec2f& uvb() const { return _mesh.faceUV(_fid, _b);}
    inline const Vec2f& uvc() const { return _mesh.faceUV(_fid, _c);}

private:
    const Mesh &_mesh;
    int _fid, _a, _b, _c, _h, _aid, _bid, _cid;
    bool _ain;
    Vec3f _abx, _acx;
};  // end class

}   // end namespace

#endif
