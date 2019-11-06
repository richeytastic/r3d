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

/**
 * Linear search for points on the surface of a mesh starting from some arbitrary vertex.
 */

#ifndef R3D_SURFACE_POINT_FINDER_H
#define R3D_SURFACE_POINT_FINDER_H

#include "Mesh.h"

namespace r3d {

class r3d_EXPORT SurfacePointFinder
{
public:
    explicit SurfacePointFinder( const Mesh& m) : _mesh(m) {}

    // Finds the point on the surface of the mesh closest to input point t. vidx must be set as the
    // vertex from which to start searching over the surface for the point closest to t.
    // On return, point fv will either be in the plane of one of the polygons attached to vidx,
    // in which case fid will be set to the ID of this face and vidx will be -1, or fv will be
    // in the same position as vertex vidx in which case vidx will be unchanged and fid will be set
    // to -1. Parameters fid and fv may be set to anything (their correct values will be set upon return).
    // Safe to pass in same argument as both t and fv (if don't want to keep t).
    // Returns squared l2-norm of (fv-t).
    float find( const Vec3f& t, int& vidx, int& fid, Vec3f& fv) const;

    /**
     * Same as above but the closet point to t on the surface is returned and the vertex ID and
     * polygon IDs are optional. If the starting vertex ID is not specified, an undefined mesh
     * vertex is used. This can result in the closest point on the surface being further away
     * than the absolute closest point because a local hill/valley may be preventing progress.
     */
    Vec3f find( const Vec3f& t, int vidx=-1, int* fid=nullptr) const;

private:
    const Mesh& _mesh;
};  // end class

}   // end namespace

#endif
