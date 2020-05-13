/************************************************************************
 * Copyright (C) 2020 Richard Palmer
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
    if ( !src.hasFixedTransform())
        std::cerr << "[WARNING] r3d::Slicer::ctor: Mesh's transform is not fixed!" << std::endl;
}   // end ctor


Mesh::Ptr Slicer::operator()( const Vec3f& p, const Vec3f& vec) const
{
    Vec3f n = vec;    // Ensure the plane vector is normalised
    n.normalize();

    const Mesh& mesh = _mesh;
    Copier copier( mesh);
    // The copied mesh has the source mesh's transform matrix.
    Mesh::Ptr hmesh = copier.copiedMesh();

    // We need the inverse of the transform matrix because we must transform the discovered
    // plane vertices to their "untransformed" positions since when discovered they are transformed.
    const Mat4f& imat = hmesh->inverseTransformMatrix();

    Vec2f uvyb, uvyc;
    const IntSet& fids = _mesh.faces();
    for ( int fid : fids)
    {
        const FacePlane fp( mesh, fid, p, n);
        const int nihs = fp.inhalf();

        if ( nihs == -1) // All face vertices in the wrong half so ignore
            continue;
        else if ( nihs == 1)
            copier.add(fid);    // All face vertices in right half so copy in normally (uses the untransformed vertices)
        else
        {
            const Vec3f& abi = fp.abIntersection();
            const Vec3f& aci = fp.acIntersection();

            // If the plane goes exactly through vertex a, then the intersection points will meet vertex a.
            // This will always be the case since FacePlane always calculates the intersections to be points
            // along the edges FROM vertex a.
            // If the intersecting points meet vertex a, then depending on what side of the plane the other
            // vertices are in, we either add, or reject the triangle. We can deduce this because FacePlane
            // always tries to split the triangle meaning that if fp.inside() is true then the other two
            // vertices must be in the wrong half, meaning that the whole of the triangle is outside and
            // should be rejected. If fp.inside() is false then the other two vertices are in the correct
            // half so the triangle is copied in. In some pathological cases, the triangle has zero area
            // and all vertices lie on the boundary. In such cases, the triangle can be rejected.
            static const float EPS = FLT_MIN;
            if ( (abi - aci).squaredNorm() < EPS)
            {
                const float farea = mesh.calcFaceArea(fid);
                if ( !fp.inside() && farea > EPS)
                    copier.add(fid);
                continue;
            }   // end if

            const bool xbOnBoundary = (fp.vb() - abi).squaredNorm() < EPS;
            const bool xcOnBoundary = (fp.vc() - aci).squaredNorm() < EPS;

            // If the intersecting points are both coincident with their edge points b and c then the entirety
            // of one edge lies along the boundary and a similar check of fp.inside() will yield if we need
            // to keep or reject the triangle.
            if ( xbOnBoundary && xcOnBoundary)
            {
                const float farea = mesh.calcFaceArea(fid);
                if ( fp.inside() && farea > EPS)
                    copier.add(fid);
                continue;
            }   // end if

            const Vec3f yb = transform( imat, abi);
            const Vec3f yc = transform( imat, aci);
            const int y = hmesh->addVertex( yb);
            const int z = hmesh->addVertex( yc);
            assert( y != z);

            int mid = mesh.faceMaterialId(fid);
            if ( mid >= 0)
            {
                uvyb = mesh.calcTextureCoords( fid, yb);
                uvyc = mesh.calcTextureCoords( fid, yc);
            }   // end if

            if ( fp.inside()) // Copy across a single triangle formed by vertex a and the intersection points
            {
                const int x = hmesh->addVertex( transform( imat, fp.va()));   // In half space
                assert( x != y && x != z);
                const int nfid = hmesh->addFace( x, y, z);
                if ( mid >= 0)
                    hmesh->setOrderedFaceUVs( mid, nfid, fp.uva(), uvyb, uvyc);
            }   // end if
            else    // Two vertices in the half space so add two new triangles
            {
                // Note that they can't both be on the boundary since this is dealt with above
                assert( !xbOnBoundary || !xcOnBoundary);

                const int v = hmesh->addVertex( transform( imat, fp.vb()));
                const int x = hmesh->addVertex( transform( imat, fp.vc()));
                assert( v != x);

                // If only one is on the boundary, then making the triangle is easy
                if ( xbOnBoundary)
                {
                    assert( v == y);
                    const int nfid = hmesh->addFace( x, z, y);
                    if ( mid >= 0)
                        hmesh->setOrderedFaceUVs( mid, nfid, fp.uvc(), uvyc, uvyb);
                }   // end if
                else if ( xcOnBoundary)
                {
                    assert( x == z);
                    const int nfid = hmesh->addFace( y, v, x);
                    if ( mid >= 0)
                        hmesh->setOrderedFaceUVs( mid, nfid, uvyb, fp.uvb(), fp.uvc());
                }   // end else if
                else
                {
                    // Neither is on the boundary.
                    // What's the shortest diagonal to split the two new triangles? (yb - fp.vc()) or (yc - fp.vb())?
                    if ( ( yb - fp.vc()).squaredNorm() < ( yc - fp.vb()).squaredNorm())
                    {
                        assert( x != v);
                        assert( x != y);
                        assert( x != z);
                        assert( y != v);
                        assert( y != z);

                        const int nfid0 = hmesh->addFace( y, v, x);
                        const int nfid1 = hmesh->addFace( x, z, y);
                        if ( mid >= 0)
                        {
                            hmesh->setOrderedFaceUVs( mid, nfid0,     uvyb, fp.uvb(), fp.uvc());
                            hmesh->setOrderedFaceUVs( mid, nfid1, fp.uvc(),     uvyc,     uvyb);
                        }   // end if
                    }   // end if
                    else
                    {
                        assert( x != z && z != v && x != v);
                        assert( v != y && y != z && v != z);
                        const int nfid0 = hmesh->addFace( v, x, z);
                        const int nfid1 = hmesh->addFace( z, y, v);
                        if ( mid >= 0)
                        {
                            hmesh->setOrderedFaceUVs( mid, nfid0, fp.uvb(), fp.uvc(),     uvyc);
                            hmesh->setOrderedFaceUVs( mid, nfid1,     uvyc,     uvyb, fp.uvb());
                        }   // end if
                    }   // end else
                }   // end else
            }   // end else
        }   // end else
    }   // end for

    return hmesh;
}   // end operator()
