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

#ifndef R3D_REFLECTOR_H
#define R3D_REFLECTOR_H

#include "Mesh.h"

namespace r3d {

class r3d_EXPORT Reflector
{
public:
    explicit Reflector( Mesh&);

    // Reflect the mesh through the plane defined by the given point and vector.
    void reflect( const Vec3f& point, const Vec3f& planev);

    // Reflect an arbitrary point through the given plane defined by point and plane vector.
    // NB planev MUST BE NORMALIZED before calling this function!
    static Vec3f reflectPoint( const Vec3f &toReflect, const Vec3f &planePoint, const Vec3f& planeVector);

private:
    Mesh& _mesh;
};  // end class

}   // end namespace

#endif
