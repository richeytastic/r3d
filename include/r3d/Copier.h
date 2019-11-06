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

#ifndef R3D_COPIER_H
#define R3D_COPIER_H

/**
 * Copy a subsection of a mesh by specifying the individual triangles to copy.
 * The source object's transform matrix is also copied over.
 * For making full copies, use Mesh::deepCopy instead.
 **/

#include "Mesh.h"

namespace r3d {

class r3d_EXPORT Copier
{
public:
    /**
     * Create a new empty mesh that triangles can be progressively copied to.
     * Copies over the source mesh's transformation matrix.
     */
    explicit Copier( const Mesh& source);

    /**
     * Copy over the given triangle to the output mesh.
     */
    void add( int fid);

    /**
     * Returns the new mesh.
     */
    Mesh::Ptr copiedMesh() const { return _cmesh;}

private:
    const Mesh& _mesh;
    Mesh::Ptr _cmesh;
    Copier( const Copier&) = delete;
    void operator=( const Copier&) = delete;
};  // end class

}   // end namespace

#endif
