/************************************************************************
 * Copyright (C) 2017 Richard Palmer
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

#include <TetrahedronReplacer.h>
using r3d::TetrahedronReplacer;
using r3d::Mesh;
#include <cassert>


TetrahedronReplacer::TetrahedronReplacer( Mesh::Ptr m) : _mesh(m) {}


int TetrahedronReplacer::removeTetrahedrons()
{
    int removedVertexCount = 0;
    int bfid;
    int vidxs[3];
    int fids[3];

    const IntSet& vids = _mesh->vtxIds();
    for ( int vidx : vids)
    {
        const IntSet& sfs = _mesh->faces(vidx);
        if ( sfs.size() != 3)
            continue;

        // Get the three face IDs
        IntSet::const_iterator fit = sfs.begin();
        fids[0] = *fit;
        fids[1] = *(++fit);
        fids[2] = *(++fit);

        const IntSet& cvs = _mesh->cvtxs(vidx); // Get the vertices that connect to vidx.
        // If there are four, vidx is a boundary vertex so it's ignored.
        if ( cvs.size() == 4)
            continue;

        assert( cvs.size() == 3);
        IntSet::const_iterator it = cvs.begin();
        vidxs[0] = *it;
        vidxs[1] = *(++it);
        vidxs[2] = *(++it);

        // Is there an existing face as the base of these three polygons?
        // If so, don't need to add one and can rely upon existing material setting.
        bfid = _mesh->face( vidxs[0], vidxs[1], vidxs[2]);
        if ( bfid < 0)
        {
            // Set material info based on the three polygons (they must shared the same material)
            const int m0 = _mesh->faceMaterialId(fids[0]);
            const int m1 = _mesh->faceMaterialId(fids[1]);
            const int m2 = _mesh->faceMaterialId(fids[2]);

            bfid = _mesh->addFace( vidxs);

            // Do we need to assign material offsets for this new face?
            if ( m0 >= 0 && m0 == m1 && m1 == m2)
            {
                // Find which of the texture offsets from polygon fids[0] to use (and in what order)
                const int* f0vorder = _mesh->fvidxs( fids[0]);
                const int* uvis0 = _mesh->faceUVs( fids[0]);

                IntSet cvs2 = cvs;  // Copy out
                Vec2f tnoffset[3];
                int j = 0;   // Will be position in tnoffset that needs an offset stored via reference of one of the other two polygons
                for ( int i = 0; i < 3; ++i)
                {
                    if ( f0vorder[i] == vidx)
                        j = i;
                    else
                    {
                        tnoffset[i] = _mesh->uv( m0, uvis0[i]);
                        cvs2.erase( f0vorder[i]);
                    }   // end if
                }   // end for

                const int vj = *cvs2.begin();
                const int* f1vorder = _mesh->fvidxs(fids[1]);
                const int* uvis1 = _mesh->faceUVs(fids[1]);
                for ( int i = 0; i < 3; ++i)
                {
                    if ( f1vorder[i] == vj)
                    {
                        tnoffset[j] = _mesh->uv( m1, uvis1[i]);
                        break;
                    }   // end if
                }   // end for

                _mesh->setOrderedFaceUVs( m0, bfid, tnoffset);
            }   // end if
        }   // end if

        // Remove the three faces
        _mesh->removeFace( fids[0]);
        _mesh->removeFace( fids[1]);
        _mesh->removeFace( fids[2]);
        _mesh->removeVertex( vidx); // Remove the shared vertex
        removedVertexCount++;
    }   // end for

    return removedVertexCount;
}   // end removeTetrahedrons
