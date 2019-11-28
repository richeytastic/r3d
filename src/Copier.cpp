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

#include <Copier.h>
using r3d::Copier;
using r3d::Mesh;


Copier::Copier( const Mesh& source) : _mesh(source)
{
    _cmesh = Mesh::create();
    _cmesh->copyInMaterials( source);    // Copy in all the material data
}   // end ctor


void Copier::add( int fid)
{
    const int* vids = _mesh.fvidxs(fid);

    // Get the raw (untransformed) vertices.
    const Vec3f& va = _mesh.uvtx( vids[0]);
    const Vec3f& vb = _mesh.uvtx( vids[1]);
    const Vec3f& vc = _mesh.uvtx( vids[2]);

    const int v0 = _cmesh->addVertex( va);
    const int v1 = _cmesh->addVertex( vb);
    const int v2 = _cmesh->addVertex( vc);
    const int nfid = _cmesh->addFace( v0, v1, v2);

    const int mid = _mesh.faceMaterialId(fid);  // Will be -1 if no material for this face
    if ( mid >= 0)
    {
        const int* uvids = _mesh.faceUVs(fid);
        _cmesh->setOrderedFaceUVs( mid, nfid, _mesh.uv(mid, uvids[0]),
                                               _mesh.uv(mid, uvids[1]),
                                               _mesh.uv(mid, uvids[2]));
    }   // end if
}   // end add
