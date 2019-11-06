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

#ifndef R3D_TETRAHEDRON_REPLACER_H
#define R3D_TETRAHEDRON_REPLACER_H

#include "Mesh.h"

namespace r3d {

class r3d_EXPORT TetrahedronReplacer
{
public:
    TetrahedronReplacer( Mesh::Ptr);

    // Finds vertices that are the points of three adjacent polygons and removes these polygons
    // replacing them with a single triangle (if one does not already exist). Returns the number
    // of vertices removed.
    int removeTetrahedrons();

private:
    Mesh::Ptr _mesh;
};  // end class

}   // end namespace

#endif

