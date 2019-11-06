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

#ifndef R3D_VERTEX_ADDER_H
#define R3D_VERTEX_ADDER_H

#include "Mesh.h"

namespace r3d {

class r3d_EXPORT VertexAdder
{
public:
    explicit VertexAdder( Mesh&);

    // Adds vertices until there are no more triangles in the object
    // having area greater than maxTriangleArea. Returns the number
    // of new vertices added.
    int addVerticesToMaxTriangleArea( float maxTriangleArea);

    // Subdivides triangles having areas larger than given, but then merges
    // newly created triangles along existing edges - joinging together pairs
    // of new introduced vertices. This DOES change the morphology of the
    // object (making it flatter on average), but tends to distribute
    // vertices into more equally spaced locations.
    int subdivideAndMerge( float maxTriangleArea);

private:
    Mesh &_mesh;
};  // end class

}   // end namespace

#endif
