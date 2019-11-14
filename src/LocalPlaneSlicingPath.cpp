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

#include <LocalPlaneSlicingPath.h>
using r3d::LocalPlaneSlicingPath;
using r3d::Vec3f;


Vec3f LocalPlaneSlicingPath::faceSlicingPlane( int tfid, const Vec3f& v) const
{
    assert( !v.array().isNaN().any());
    assert(_endPath);
    const Vec3f& q = _endPath->lastVertexAdded();
    assert( !q.array().isNaN().any());

    Vec3f v2q = q - v;
    assert( !v2q.isZero());

    v2q.normalize();    // There might seem like there are more normalisations going on here than necessary,
    // and indeed - in theory - there are. However, I noticed that I get better accuracy normalising a vector
    // before involving it in a cross product with another normal vector, so that's the reason for all the
    // normalising going on. This is probably something to do with the Eigen library doing some caching of
    // values or something for unit vectors. Who knows. Also, I tried making everything double precision first
    // (except the input and output vertices) and that doesn't increase accuracy so everything is calculated
    // using single precision.

    // v is the crossing point on the entry edge of this face (i.e. the point on the edge shared with
    // faces tfid and lastParsedFace where the path crosses from face lastParsedFace into face tfid).
    const Mesh &msh = mesh();
    Vec3f zvec = msh.calcFaceVector( tfid, true/*calc transformed*/);
    zvec.normalize();
    Vec3f xvec = v2q.cross( zvec);
    xvec.normalize();
    if ( lastParsedFace() != tfid)
    {
        // Get the vector to the current endpoint from v.
        // After rotating it into the plane of face tfid, its direction MUST cause it to cut
        // across the surface of face tfid otherwise the path will bounce back out into the
        // adjacent face which has already been parsed causing the algorithm to prematurely end.
        // If it is found that this will happen, the face slicing plane normal is instead given
        // as the vector that is orthogonal to the entry edge and this face's normal.
        // That will cause the path to propogate along the entry edge.
        
        // Get the standard orientation for the face
        const Edge *edge = msh.commonEdge( tfid, lastParsedFace());
        assert( edge != nullptr);
        Vec3f yvec = msh.vtx(edge->at(1)) - msh.vtx(edge->at(0));
        yvec.normalize();
        Vec3f rvec = yvec.cross(zvec);
        rvec.normalize();

        // Pop into an homogeneous matrix along with the edge entry vertex 
        Mat4f T;
        T.col(0) << rvec, 0;
        T.col(1) << yvec, 0;
        T.col(2) << zvec, 0;
        T.col(3) << v, 1;
        const Mat4f invT = T.inverse();

        // Transform the non edge vertex of the face into standard position ...
        const int oppVtxId = msh.face(tfid).opposite( edge->at(1), edge->at(0));
        const Vec3f fvtx = transform( invT, msh.vtx( oppVtxId));
        // ... and the current endpoint given by the last vertex added by the paired slicing path
        const Vec3f tq = transform( invT, q);

        /*
        std::cerr << "LocalPlaneSlicingPath::faceSlicingPlane: " << std::hex << this << std::dec << std::endl;
        std::cerr << "    Face: " << tfid << " with entered edge vertices "
                  << edge->at(1) << " and " << edge->at(0) << " (opposite " << oppVtxId << ")" << std::endl;
        */
        // Now easily check if the path to the endpoint crosses the face by seeing if the sign bit
        // for the x coordinate of this vector matches with the non edge vertex of the transformed
        // face. If so, we can safely use v2q as the direction vector for calculating the slicing
        // plane's normal since the projected path intersects the face. Note that if the x-coord
        // of tq is zero, then the cross product is undefined so use the existing face edge.
        if ( std::signbit(tq[0]) != std::signbit(fvtx[0]) || std::fpclassify(tq[0]) == FP_ZERO)
        {
            xvec = rvec;
            //std::cerr << "    EDGE defined slicing plane normal of ";
        }   // end if
        //else
        //    std::cerr << "    PATH defined slicing plane normal of ";
        //std::cerr << xvec.transpose() << std::endl;
    }   // end if

    assert( !xvec.isZero());
    return xvec;
}   // end faceSlicingPlane
