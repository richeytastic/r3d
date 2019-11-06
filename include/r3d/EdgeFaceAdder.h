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

#ifndef R3D_EDGE_FACE_ADDER_H
#define R3D_EDGE_FACE_ADDER_H

#include "Mesh.h"

namespace r3d {

class r3d_EXPORT EdgeFaceAdder
{
public:
    explicit EdgeFaceAdder( Mesh &mod) : _mesh(mod) {}

    // The vertex IDs in xyset must already be present in the mesh.
    void addFaces( const std::unordered_map<int,IntSet>& xyset);

private:
    Mesh &_mesh;
    std::unordered_map<int, std::unordered_map<int, int> > _edgeUse; // Count number of times each edge is used

    bool _setFace( int x, int y, int z);
    bool _areSharedFacesJoined( int x, int y, int z) const;
    void _init( int x, int y);
    void _addTriangle( int x, int y, int z);
};  // end class

}   // end namespace

#endif
