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

#ifndef R3D_FACE_PLANE_H
#define R3D_FACE_PLANE_H

/**
 * Find how to split an Face into two pieces and know which side is on the "inside" of
 * a space divided by an arbitrary plane.
 */

#include "Mesh.h"

namespace r3d {

class r3d_EXPORT FacePlane
{
public:
    // f is the face to test, p is a point on the plane and n is a normal vector of unit length giving
    // the plane's orientation with it pointing in the direction of "inside" the half space we want to keep.
    FacePlane( const Mesh& src, int fid, const Vec3f& p, const Vec3f& n);

    // Returns -1 if all vertices on this face are in the "outside" half, 1 if all vertices are in the "inside" half
    // and 0 if the vertices stradle the plane (then use findPlaneVertices to find where the plane intersects the
    // edges of the triangle). If 0 is returned, then a() is the vertex that is by itself on one side of the plane
    // (with b() and c() the other two vertices). If inside() returns true, then vertex 'a' is
    // is "inside" the half space defined by the plane normal.
    inline int inhalf() const { return _nih;}

    // If true, then vertex a is inside the half space pointed into by vector n.
    // (meaning that dot product (a-p).n is positive).
    inline bool inside() const { return _ain;}

    // If non-negative, then the returned vertex ID passes EXACTLY through the half space
    // boundary AND the other two vertices of the triangle are on either side of the boundary.
    // Note that this will never be vaid() since the design of Mesh disallows zero area faces.
    inline int straddleId() const { return _straddleId;}

    // Returns the point on edge ab of the polygon that intersect with the plane.
    // Only use if inhalf() returns zero.
    inline const Vec3f& abIntersection() const { return _abx;}

    // Returns the point on edge ac of the polygon that intersect with the plane.
    // Only use if inhalf() returns zero.
    inline const Vec3f& acIntersection() const { return _acx;}

    inline int vaid() const { return _fvidxs[_a];}
    inline int vbid() const { return _fvidxs[_b];}
    inline int vcid() const { return _fvidxs[_c];}

    inline const Vec3f& va() const { return _mesh.vtx(vaid());}
    inline const Vec3f& vb() const { return _mesh.vtx(vbid());}
    inline const Vec3f& vc() const { return _mesh.vtx(vcid());}

    inline const Vec2f& uva() const { return _mesh.faceUV(_fid, _a);}
    inline const Vec2f& uvb() const { return _mesh.faceUV(_fid, _b);}
    inline const Vec2f& uvc() const { return _mesh.faceUV(_fid, _c);}

private:
    const Mesh &_mesh;
    int _fid;
    const int* _fvidxs;
    bool _ain;
    int _straddleId;
    int _a, _b, _c;
    int _nih;
    Vec3f _abx, _acx;
};  // end class

}   // end namespace

#endif
