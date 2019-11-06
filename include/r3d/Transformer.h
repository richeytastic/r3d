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

#ifndef R3D_TRANSFORMER_H
#define R3D_TRANSFORMER_H

#ifdef _WIN32
#pragma warning( disable : 4251)
#endif

#include "r3dTypes.h"

namespace r3d {

class r3d_EXPORT Transformer
{
public:
    Transformer();                          // Identity matrix
    Transformer( const Vec3f& translation); // Create translation matrix only (no rotation).
    Transformer( const Mat4f& transform);   // Set entire transformation matrix directly.

    // The following constructors allow for the optional incorporatation of a translatation AFTER the desired rotation.

    // Set rotation submatrix directly with (optional) subsequent translation.
    Transformer( const Mat3f& rotMat, const Vec3f& t=Vec3f::Zero());
    // Rotation from positive Z (normal) and positive Y (up) vectors with (optional) subsequent translation.
    Transformer( const Vec3f& posZ, const Vec3f& posY, const Vec3f& t=Vec3f::Zero());
    // Rotation with angle about given axis with (optional) subsequent translation.
    Transformer( float radians, const Vec3f& axis, const Vec3f& t=Vec3f::Zero());

    // Perform a transform prior to the existing transform.
    // Necessary for rotations where the object is not already at the origin.
    void prependTranslation( const Vec3f&);

    // Append parameter's matrix by this one and set in this one (returning self).
    // The parameter matrix will be applied AFTER the matrix in this mover.
    Transformer& operator*( const Transformer&);

    inline const Mat4f& matrix() const { return _tmat;}   // Returns Transformer's transformation matrix.
    inline const Mat4f& operator()() const { return _tmat;} // Synonym

    Vec3f transform( const Vec3f&) const;   // Return transformed
    void transform( Vec3f&) const;          // In-place transform

    // Apply just the rotation submatrix (don't translate).
    Vec3f rotate( const Vec3f&) const;
    void rotate( Vec3f&) const;     // In-place rotation of given vertex

private:
    Mat4f _tmat;  // Transformation matrix as homogeneous coordinates
};  // end class

}   // end namespace

#endif
