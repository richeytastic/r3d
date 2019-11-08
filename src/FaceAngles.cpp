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

#include <FaceAngles.h>
#include <cassert>
#include <cmath>
using r3d::FaceAngles;
using r3d::Mesh;
using r3d::Vec3f;


// public static
float FaceAngles::calcAngle( const Vec3f& u0, const Vec3f& u1)
{
    return acosf( u0.dot(u1) / (u0.norm() * u1.norm()));
}   // end calcAngle


// public static
float FaceAngles::calcInnerAngle( const Mesh &mesh, int fid, int u0)
{
    const Face& face = mesh.face( fid);
    int u1, u2;
    if ( !face.opposite( u0, u1, u2))
        return -1;
    const Vec3f& v0 = mesh.vtx( u0);
    const Vec3f& v1 = mesh.vtx( u1);
    const Vec3f& v2 = mesh.vtx( u2);
    return calcAngle( v1 - v0, v2 - v0);
}   // end calcInnerAngle


FaceAngles::FaceAngles( const Mesh& m) : _mesh(m), _fangles( m.numFaces(), 3)
{
    assert( m.hasSequentialFaceIds());
    const size_t N = m.numFaces();
    for ( size_t i = 0; i < N; ++i)
    {
        const int *fvidxs = m.fvidxs(int(i));
        const Vec3f v02 = m.uvtx(fvidxs[0]) - m.uvtx(fvidxs[2]);
        const Vec3f v10 = m.uvtx(fvidxs[1]) - m.uvtx(fvidxs[0]);
        const Vec3f v21 = m.uvtx(fvidxs[2]) - m.uvtx(fvidxs[1]);
        _fangles.row(i) << calcAngle( v10, -v02), calcAngle( v21, -v10), calcAngle( v02, -v21);
    }   // end for
}   // end ctor

