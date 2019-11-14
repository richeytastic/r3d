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
    // v is the crossing point on the entry edge of this face (i.e. the point on the edge shared with
    // faces tfid and lastParsedFace where the path crosses from face lastParsedFace into face tfid).
    assert(_endPath);

    // Get the vector to the current endpoint from v.
    // After rotating it into the plane of face tfid, its direction MUST cause it to cut
    // across the surface of face tfid otherwise the path will bounce back out into the
    // adjacent face which has already been parsed causing the algorithm to prematurely end.
    // If it is found that this will happen, the face slicing plane normal is instead given
    // as the vector that is orthogonal to the entry edge and this face's normal.
    // That will cause the path to propogate along the entry edge.
    
    const Mesh &msh = mesh();
    // Get the standard orientation for the face
    Vec3f zvec = msh.calcFaceNorm( tfid, true/*calc transformed*/);
    const Edge *edge = msh.commonEdge( tfid, lastParsedFace());
    assert( edge != nullptr);
    Vec3f yvec = msh.vtx(edge->at(1)) - msh.vtx(edge->at(0));
    yvec.normalize();
    Vec3f xvec = yvec.cross(zvec);
    assert( xvec.norm() == 1);

    // Pop into an homogeneous matrix along with the edge entry vertex 
    Mat4f T;
    T.col(0) << xvec, 0;
    T.col(1) << yvec, 0;
    T.col(2) << zvec, 0;
    T.col(3) << v, 1;
    const Mat4f invT = T.inverse();

    // Transform the non edge vertex of the face into standard position ...
    const Vec3f fvtx = transform( invT, msh.vtx( msh.face(tfid).opposite( edge->at(1), edge->at(0))));
    // ... and the vector to the current endpoint given as the last vertex added by the paired slicing path
    const Vec3f v2q = _endPath->lastVertexAdded() - v;
    const Vec3f tv2q = transform( invT, v2q);

    // Now easily check if the path to the endpoint crosses the face by seeing if the sign bit
    // for the x coordinate of this vector matches with the non edge vertex of the transformed
    // face. If so, we can safely use v2q as the direction vector for calculating the slicing
    // plane's normal since the projected path intersects the face.
    if ( std::signbit(tv2q[0]) == std::signbit(fvtx[0]))
    {
        xvec = v2q.cross(zvec);
        xvec.normalize();
    }   // end if

    return xvec;
}   // end faceSlicingPlane
