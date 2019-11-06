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

#include <Reflector.h>
#include <algorithm>
using r3d::Reflector;
using r3d::Mesh;
using r3d::Vec3f;

Reflector::Reflector( Mesh::Ptr mesh) : _mesh(mesh) {}


void Reflector::reflectPoint( Vec3f& v, const Vec3f& pt, const Vec3f& pvec)
{
    const float d = pvec.dot( pt - v);  // Project to plane vector to give orthogonal distance to plane
    v = v + 2.0f * d * pvec;            // Reflect point through plane
}   // end reflectPoint
                                                        

void Reflector::reflect( const Vec3f& pt, const Vec3f& plane)
{
    Vec3f pvec = plane; // Ensure plane vector is normalized
    pvec.normalize();

    const IntSet& vids = _mesh->vtxIds();
    for ( int vidx : vids)
    {
        Vec3f v = _mesh->vtx(vidx);    // Copy out
        reflectPoint( v, pt, pvec);
        _mesh->adjustVertex( vidx, v);
    }   // end for

    // Also need to flip normals on all the mesh's faces
    const IntSet& fids = _mesh->faces();
    for ( int fid : fids)
        _mesh->reverseFaceVertices(fid);
}   // end reflect
