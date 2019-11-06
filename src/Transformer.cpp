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

#include <Transformer.h>
using r3d::Transformer;
using r3d::Vec3f;
using r3d::Mat3f;
using r3d::Mat4f;


Transformer::Transformer() : _tmat( Mat4f::Identity()) {}

Transformer::Transformer( const Mat4f& t) : _tmat(t) {}

Transformer::Transformer( const Vec3f& t) : _tmat( Mat4f::Identity())
{
    _tmat.block<3,1>(0,3) = t;
}   // end ctor

Transformer::Transformer( const Mat3f& R, const Vec3f& t) : _tmat( Mat4f::Identity())
{
    _tmat.block<3,3>(0,0) = R;
    _tmat.block<3,1>(0,3) = t;
}   // end ctor


Transformer::Transformer( const Vec3f& vnorm, const Vec3f& vup, const Vec3f& t)
{
    // Ensure normalized
    Vec3f uvec = vup;
    uvec.normalize();
    
    Vec3f zvec = vnorm;
    zvec.normalize();

    Vec3f xvec = uvec.cross(zvec);
    Vec3f yvec = zvec.cross(xvec);   // Don't trust that vup and vnorm actually are orthogonal

    _tmat.col(0) << xvec, 0;
    _tmat.col(1) << yvec, 0;
    _tmat.col(2) << zvec, 0;
    _tmat.col(3) << t, 1;
}   // end ctor


namespace {

void _initMatrix( float rads, const Vec3f& axis, const Vec3f& t, Mat4f& tmat)
{
    Vec3f u = axis;    // Ensure normalised axis
    u.normalize();

    const float x = u[0];
    const float y = u[1];
    const float z = u[2];
    const float ct = cosf(rads);
    const float mct = 1.0f-ct;
    const float st = sinf(rads);
    const float xst = x*st;
    const float yst = y*st;
    const float zst = z*st;

    // Set the 3x3 upper left submatrix with the rotation params
    tmat(0,0) = x*x*mct + ct;
    tmat(0,1) = x*y*mct - zst;
    tmat(0,2) = x*z*mct + yst;
    tmat(0,3) = t[0];

    tmat(1,0) = y*x*mct + zst;
    tmat(1,1) = y*y*mct + ct;
    tmat(1,2) = y*z*mct - xst;
    tmat(1,3) = t[1];

    tmat(2,0) = z*x*mct - yst;
    tmat(2,1) = z*y*mct + xst;
    tmat(2,2) = z*z*mct + ct;
    tmat(2,3) = t[2];

    tmat(3,0) = 0.0f;
    tmat(3,1) = 0.0f;
    tmat(3,2) = 0.0f;
    tmat(3,3) = 1.0f;
}   // end _initMatrix

}   // end namespace


Transformer::Transformer( float rads, const Vec3f& axis, const Vec3f& t)
{
    _initMatrix( rads, axis, t, _tmat);
}   // end ctor


void Transformer::prependTranslation( const Vec3f& t)
{
    Mat4f T = Mat4f::Identity();
    T.block<3,1>(0,3) = t;
    _tmat = _tmat * T;  // Note that this prepends because applied transform will be (_tmat * T * v) for some vector v
}   // end prependTranslation


Transformer& Transformer::operator*( const Transformer& m)
{
    _tmat = m.matrix() * _tmat;
    return *this;
}   // end operator*


void Transformer::transform( Vec3f& v) const
{
    const Vec4f nv = _tmat * Vec4f( v[0], v[1], v[2], 1);
    v = nv.segment<3>(0);
}   // end transform


Vec3f Transformer::transform( const Vec3f& v) const
{
    return (_tmat * Vec4f( v[0], v[1], v[2], 1)).segment<3>(0);
}   // end transform


void Transformer::rotate( Vec3f& v) const
{
    v = _tmat.block<3,3>(0,0) * v;
}   // end rotate


Vec3f Transformer::rotate( const Vec3f& v) const
{
    return _tmat.block<3,3>(0,0) * v;
}   // end rotate
