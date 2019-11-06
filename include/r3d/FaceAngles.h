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

#ifndef R3D_FACE_ANGLES_H
#define R3D_FACE_ANGLES_H

/**
 * Calculates the inner angles of Mesh faces.
 * Mesh must have sequential face ids!
 */
#include "Mesh.h"

namespace r3d {

class r3d_EXPORT FaceAngles
{
public:
    explicit FaceAngles( const Mesh&);
    FaceAngles( const FaceAngles&) = default;
    FaceAngles& operator=( const FaceAngles&) = default;

    const Mesh &mesh() const { return _mesh;}

    // Return complete matrix (face per row, angles per column in corresponding face stored vertex order)
    inline const MatX3f &matrix() { return _fangles;}

    // Return the inner angle of vidx inside triangle fid.
    inline float operator()( int fid, int vidx) const { return _fangles(fid, _mesh.face(fid).index(vidx));}

    // Static version of above function.
    static float calcInnerAngle( const Mesh&, int fid, int vidx);

    // Calculate inner angle where two edges of arbitrary length meet.
    static float calcAngle( const Vec3f&, const Vec3f&);

private:
    const Mesh &_mesh;
    MatX3f _fangles;
};  // end class

}   // end namespace

#endif
