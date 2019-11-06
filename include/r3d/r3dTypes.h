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

#ifndef R3D_TYPES_H
#define R3D_TYPES_H

#include "r3d_Export.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace r3d {

/**
 * A feature matrix has rows for each vertex (sequentially ordered IDs)
 * with the first three columns being the point's position, and the last
 * three columns being the point's normal vector.
 */
using FeatMat = Eigen::Matrix<float, Eigen::Dynamic, 6>;

/**
 * A face matrix has rows for each face (sequentially ordered IDs)
 * with the columns being the ORDERED vertex indices of the face
 * (i.e. defining the orientation of the face normal by right hand rule).
 */
using FaceMat = Eigen::Matrix<int, Eigen::Dynamic, 3>;

using Mat3f = Eigen::Matrix3f;
using Mat3d = Eigen::Matrix3d;
using Mat4f = Eigen::Matrix4f;
using Mat4d = Eigen::Matrix4d;
using MatXf = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using MatXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;
using Vec2f = Eigen::Vector2f;
using Vec2d = Eigen::Vector2d;
using Vec3f = Eigen::Vector3f;
using Vec3d = Eigen::Vector3d;
using Vec4f = Eigen::Vector4f;
using Vec4d = Eigen::Vector4d;
using Vec6f = Eigen::Matrix<float, 6, 1>;
using Vec6d = Eigen::Matrix<double, 6, 1>;

using VecXi = Eigen::VectorXi;
using VecXf = Eigen::VectorXf;
using VecXd = Eigen::VectorXd;

using MatX3f = Eigen::Matrix<float, Eigen::Dynamic, 3>;
using MatX3d = Eigen::Matrix<double, Eigen::Dynamic, 3>;

using Vec3iMap = Eigen::Map<const Vec3i>;

using Vec2fMap = Eigen::Map<const Vec2f>;
using Vec2dMap = Eigen::Map<const Vec2d>;
using Vec3fMap = Eigen::Map<const Vec3f>;
using Vec3dMap = Eigen::Map<const Vec3d>;
using Vec4fMap = Eigen::Map<const Vec4f>;
using Vec4dMap = Eigen::Map<const Vec4d>;

// Mapping buffers assumes row major ordering (which is OpenCV storage order)
using Mat3fMap = Eigen::Map<const Eigen::Matrix<float,  3, 3, Eigen::RowMajor> >;
using Mat3dMap = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >;
using Mat4fMap = Eigen::Map<const Eigen::Matrix<float,  4, 4, Eigen::RowMajor> >;
using Mat4dMap = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> >;

// Map memory in OpenCV structures to Eigen format (i.e. without copying buffer)
r3d_EXPORT const Vec3iMap asEigen( const int*);

r3d_EXPORT const Vec2fMap asEigen( const cv::Vec2f&);
r3d_EXPORT const Vec2dMap asEigen( const cv::Vec2d&);
r3d_EXPORT const Vec3fMap asEigen( const cv::Vec3f&);
r3d_EXPORT const Vec3dMap asEigen( const cv::Vec3d&);
r3d_EXPORT const Vec4fMap asEigen( const cv::Vec4f&);
r3d_EXPORT const Vec4dMap asEigen( const cv::Vec4d&);

r3d_EXPORT const Mat3fMap asEigen( const cv::Matx33f&);
r3d_EXPORT const Mat3dMap asEigen( const cv::Matx33d&);
r3d_EXPORT const Mat4fMap asEigen( const cv::Matx44f&);
r3d_EXPORT const Mat4dMap asEigen( const cv::Matx44d&);

// Copy Eigen format to OpenCV format and back again (i.e. copy buffer)
r3d_EXPORT cv::Matx44d toCV( const Mat4d&);
r3d_EXPORT Mat4d toEigen( const cv::Matx44d&);


// Find and return the vertex along line segment {xp,x} that intersects plane {p,n} where p is a point
// in the plane and n is a perpendicular normal (unit length) vector pointing into one half of the space.
r3d_EXPORT Vec3f linePlaneIntersection( const Vec3f &p, const Vec3f &n, const Vec3f &xp, const Vec3f &x);


// v0 = a-i
// v1 = b-i
// Returns inner angle at i as v0.dot(v1)/(norm(v0)*norm(v1)) to determine direction similarily of vectors.
// With i as the root of the vectors, if a-i and b-i point in exactly the same direction, 1 is returned.
// -1 is returned if in opposite directions. 0 returned if exactly orthogonal.
r3d_EXPORT float cosi( const Vec3f &i, const Vec3f &a, const Vec3f &b);


// Takes two positions vroot and vp, and a normal direction vector (doesn't need to be unit length) and returns the
// magnitude of vp-vroot after projecting into the plane that goes through vroot and is orthogonal to nv.
// If pvp is not null it will be set to the projection of vp into the plane (so that the returned magnitude is
// that of the vector *pvp-vroot).
r3d_EXPORT float projectIntoPlane( const Vec3f& vroot, const Vec3f& vp, const Vec3f& nv, Vec3f *pvp=nullptr);


// Given two lines specified by point pairs, calculate and return the point in space at which they intersect.
// If the point pairs represent line segment endpoints and the caller wants to know if the returned point sits
// within both line segments, use isPointOnBothLineSegments below.
r3d_EXPORT Vec3f intersection( const Vec3f& v0, const Vec3f& v1,  // Points describing first line
                               const Vec3f& u0, const Vec3f& u1); // Points describing first line


// Given a point p which is incident with the lines described by both endpoint pairs, check if it actually
// sits within the endpoints of both the lines (and is therefore an "actual" line segment intersection point).
// Note that point p is NOT FIRST CHECKED to see that it shares the same directions as both lines!
r3d_EXPORT bool isPointOnBothLineSegments( const Vec3f& v0, const Vec3f& v1, // Endpoints of 1st line segment
                                           const Vec3f& u0, const Vec3f& u1, // Endpoints of 2nd line segment
                                           const Vec3f& x,    // Point to test
                                           float TOLERANCE=0);   // Increase segment length by this much at both ends


template <typename T>
Eigen::Matrix<T, 3, 1> transform( const Eigen::Matrix<T, 4, 4> &M, const Eigen::Matrix<T, 3, 1> &p)
{
    auto r = M * Eigen::Matrix<T, 4, 1>( p[0], p[1], p[2], 1);
    return r.head(3);
}   // end transform


template <typename T>
T calcArea( const Eigen::Matrix<T,3,1> &a, const Eigen::Matrix<T,3,1> &b) { return a.cross(b).norm() / 2;}


template <typename T>
T calcArea( const Eigen::Matrix<T,3,1> &a, const Eigen::Matrix<T,3,1> &b, const Eigen::Matrix<T,3,1> &c) { return calcArea<T>( (a-c), (b-c));}


template <typename T>
Eigen::Matrix<T, 3, 1> calcBarycentric( const Eigen::Matrix<T, 3, 1> &A,
                                        const Eigen::Matrix<T, 3, 1> &B,
                                        const Eigen::Matrix<T, 3, 1> &C,
                                        const Eigen::Matrix<T, 3, 1> &P)
{
    const auto PC = P-C;
    const auto AC = A-C;
    const auto BC = B-C;
    const T denom = (A-B).cross(AC).norm();    // Area of parallelogram but no need to divide by 2 since they cancel.
    const T u = PC.cross(BC).norm() / denom;
    const T v = AC.cross(PC).norm() / denom;
    Eigen::Matrix<T, 3, 1> vec;
    vec << u, v, 1-u-v;
    return vec;
}   // end calcBarycentric


template <typename T>
void reorderAscending( T &v0, T &v1, T &v2)
{
    if ( v0 > v1)
        std::swap( v0, v1);
    if ( v1 > v2)
        std::swap( v1, v2);
    if ( v0 > v1)
        std::swap( v0, v1);
}   // end reorderAscending


template <typename T>
T roundndp( T x, size_t ndp)
{
    const double E = pow(10,ndp);
    const double y = double(long(x));
    const double z = double(long((x-y)*E + 0.5));
    return static_cast<T>(y + z/E);
}   // end roundndp

}   // end namespace

#endif
