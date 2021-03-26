/************************************************************************
 * Copyright (C) 2021 Richard Palmer
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
using MatX6f = Eigen::Matrix<float, Eigen::Dynamic, 6>;

/**
 * A face matrix has rows for each face (sequentially ordered IDs)
 * with the columns being the ORDERED vertex indices of the face
 * (i.e. defining the orientation of the face normal by right hand rule).
 */
using FaceMat = Eigen::Matrix<int, Eigen::Dynamic, 3>;

using Mat3f = Eigen::Matrix3f;
using Mat4f = Eigen::Matrix4f;
using MatXf = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using Mat3d = Eigen::Matrix3d;
using Mat4d = Eigen::Matrix4d;
using MatXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;
using Vec4i = Eigen::Vector4i;
using Vec5i = Eigen::Matrix<int, 5, 1>;
using Vec6i = Eigen::Matrix<int, 6, 1>;
using Vec2f = Eigen::Vector2f;
using Vec3f = Eigen::Vector3f;
using Vec4f = Eigen::Vector4f;
using Vec5f = Eigen::Matrix<float, 5, 1>;
using Vec6f = Eigen::Matrix<float, 6, 1>;
using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Vec4d = Eigen::Vector4d;
using Vec5d = Eigen::Matrix<double, 5, 1>;
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


// Find and return the vertex along line segment {x0,x1} that intersects plane {p,n} where p is a point
// in the plane and n is a perpendicular normal (unit length) vector pointing into one half of the space.
r3d_EXPORT Vec3f linePlaneIntersection( const Vec3f &p, const Vec3f &n, const Vec3f &x0, const Vec3f &x1);


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


r3d_EXPORT Vec3f transform( const Mat4f &M, const Vec3f &p);
r3d_EXPORT Vec3d transform( const Mat4d &M, const Vec3d &p);


r3d_EXPORT float calcArea( const Vec3f &a, const Vec3f &b);
r3d_EXPORT float calcArea( const Vec3f &a, const Vec3f &b, const Vec3f &c);


r3d_EXPORT Vec3f calcBarycentric( const Vec3f &A, const Vec3f &B, const Vec3f &C, const Vec3f &P);


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
