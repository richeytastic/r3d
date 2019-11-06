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

#ifndef R3D_SLICER_H
#define R3D_SLICER_H

/**
 * Copy out one half of a source mesh along the defined plane creating new
 * vertices and faces where necessary to ensure that the triangles intersect
 * cleanly at the planar boundary.
 **/

#include "Mesh.h"

namespace r3d {

class r3d_EXPORT Slicer
{
public:
    // Provide the source mesh. Note that the source mesh's transform matrix should not be set,
    // but that the inverse of its transform will be used anyway when creating the sliced half.
    explicit Slicer( const Mesh&);

    // Provide a plane defined by a point and a vector pointing into the half of the mesh wanted.
    Mesh::Ptr operator()( const Vec3f& pt, const Vec3f& vec) const;

private:
    const Mesh &_mesh;
    Slicer( const Slicer&) = delete;
    void operator=( const Slicer&) = delete;
};  // end class

}   // end namespace

#endif
