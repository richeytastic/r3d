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

#include <Slicer.h>
#include <Copier.h>
#include <FacePlane.h>
#include <cmath>
using r3d::Slicer;
using r3d::Mesh;
using r3d::Vec3f;


// public
Slicer::Slicer( const Mesh& src) : _mesh(src)
{
    if ( src.transformMatrix() != Mat4f::Identity())
        std::cerr << "[WARNING] r3d::Slicer::ctor: Supplied mesh's transform is not identity!" << std::endl;
}   // end ctor


Mesh::Ptr Slicer::operator()( const Vec3f& p, const Vec3f& vec) const
{
    Vec3f n = vec;    // Ensure the plane vector is normalised
    n.normalize();

    const Mesh& mod = _mesh;
    Copier copier( mod);
    // The copied mesh has the source mesh's transform matrix.
    Mesh::Ptr hmod = copier.copiedMesh();

    // We need the inverse of the transform matrix because we must transform the discovered
    // plane vertices to their "untransformed" positions since when discovered they are transformed.
    const Mat4f imat = hmod->transformMatrix().inverse();

    int mid;
    Vec2f uvyb, uvyc;
    Vec3f yb, yc;
    const IntSet& fids = _mesh.faces();
    for ( int fid : fids)
    {
        const FacePlane pp( mod, fid, p, n);
        const int nihs = pp.inhalf();

        if ( nihs == -1) // All face vertices in the wrong half so ignore
            continue;
        else if ( nihs == 1)
            copier.add(fid);    // All face vertices in right half so copy in normally (uses the untransformed vertices)
        else
        {
            yb = pp.abIntersection();
            yc = pp.acIntersection();
            transform( imat, yb);
            transform( imat, yc);
            const int y = hmod->addVertex( yb);
            const int z = hmod->addVertex( yc);

            mid = mod.faceMaterialId(fid);
            if ( mid >= 0)
            {
                uvyb = mod.calcTextureCoords( fid, yb);
                uvyc = mod.calcTextureCoords( fid, yc);
            }   // end if

            if ( pp.inside()) // Only a single vertex is in the half space so new triangle easy
            {
                const int x = hmod->addVertex( transform( imat, pp.va()));   // In half space
                const int nfid = hmod->addFace( x, y, z);
                if ( mid >= 0)
                    hmod->setOrderedFaceUVs( mid, nfid, pp.uva(), uvyb, uvyc);
            }   // end if
            else    // Two vertices in the half space so need to add two new triangles
            {
                const Vec3f& xb = pp.vb();   // In half space
                const Vec3f& xc = pp.vc();   // In half space
                const int v = hmod->addVertex( transform( imat, xb));
                const int x = hmod->addVertex( transform( imat, xc));

                // What's the shortest diagonal to split the two new triangles? yb,xc or yc,xb?
                if ( ( yb-xc).norm() < ( yc-xb).norm())
                {
                    const int nfid0 = hmod->addFace( y, v, x);
                    const int nfid1 = hmod->addFace( x, z, y);
                    if ( mid >= 0)
                    {
                        const Vec2f& uvb = pp.uvb();
                        const Vec2f& uvc = pp.uvc();
                        hmod->setOrderedFaceUVs( mid, nfid0, uvyb, uvb, uvc);
                        hmod->setOrderedFaceUVs( mid, nfid1, uvc, uvyc, uvyb);
                    }   // end if
                }   // end if
                else
                {
                    const int nfid0 = hmod->addFace( v, x, z);
                    const int nfid1 = hmod->addFace( z, y, v);
                    if ( mid >= 0)
                    {
                        const Vec2f& uvb = pp.uvb();
                        const Vec2f& uvc = pp.uvc();
                        hmod->setOrderedFaceUVs( mid, nfid0, uvb, uvc, uvyc);
                        hmod->setOrderedFaceUVs( mid, nfid1, uvyc, uvyb, uvb);
                    }   // end if
                }   // end else
            }   // end else
        }   // end else
    }   // end for

    return hmod;
}   // end operator()
