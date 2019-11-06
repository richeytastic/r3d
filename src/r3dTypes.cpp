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

#include <r3dTypes.h>
using namespace r3d;

const Vec3iMap r3d::asEigen( const int *v) { return Vec3iMap(v);}
const Vec2fMap r3d::asEigen( const cv::Vec2f &v) { return Vec2fMap(&v[0]);}
const Vec2dMap r3d::asEigen( const cv::Vec2d &v) { return Vec2dMap(&v[0]);}
const Vec3fMap r3d::asEigen( const cv::Vec3f &v) { return Vec3fMap(&v[0]);}
const Vec3dMap r3d::asEigen( const cv::Vec3d &v) { return Vec3dMap(&v[0]);}
const Vec4fMap r3d::asEigen( const cv::Vec4f &v) { return Vec4fMap(&v[0]);}
const Vec4dMap r3d::asEigen( const cv::Vec4d &v) { return Vec4dMap(&v[0]);}
const Mat3fMap r3d::asEigen( const cv::Matx33f &m) { return Mat3fMap(&m(0,0));}
const Mat3dMap r3d::asEigen( const cv::Matx33d &m) { return Mat3dMap(&m(0,0));}
const Mat4fMap r3d::asEigen( const cv::Matx44f &m) { return Mat4fMap(&m(0,0));}
const Mat4dMap r3d::asEigen( const cv::Matx44d &m) { return Mat4dMap(&m(0,0));}


cv::Matx44d r3d::toCV( const Mat4d& tmat)
{
    cv::Matx44d t;

    t(0,0) = tmat(0,0);
    t(0,1) = tmat(0,1);
    t(0,2) = tmat(0,2);
    t(0,3) = tmat(0,3);

    t(1,0) = tmat(1,0);
    t(1,1) = tmat(1,1);
    t(1,2) = tmat(1,2);
    t(1,3) = tmat(1,3);

    t(2,0) = tmat(2,0);
    t(2,1) = tmat(2,1);
    t(2,2) = tmat(2,2);
    t(2,3) = tmat(2,3);

    t(3,0) = tmat(3,0);
    t(3,1) = tmat(3,1);
    t(3,2) = tmat(3,2);
    t(3,3) = tmat(3,3);

    return t;
}   // end toCV


Mat4d r3d::toEigen( const cv::Matx44d &m)
{
    Mat4d em;
    em << m(0,0), m(0,1), m(0,2), m(0,3),
          m(1,0), m(1,1), m(1,2), m(1,3),
          m(2,0), m(2,1), m(2,2), m(2,3),
          m(3,0), m(3,1), m(3,2), m(3,3);
    return em;
}   // end toEigen


Vec3f r3d::linePlaneIntersection( const Vec3f& p, const Vec3f& n, const Vec3f& xp, const Vec3f& x)
{
    const Vec3f xv = xp-x;
    return xp - xv * (xp-p).dot(n) / xv.dot(n);
}   // end linePlaneIntersection



float r3d::cosi( const Vec3f& i, const Vec3f& a, const Vec3f& b)
{
    Vec3f va = a-i;
    va.normalize();
    Vec3f vb = b-i;
    vb.normalize();
    return va.dot(vb);
}   // end cosi


float r3d::projectIntoPlane( const Vec3f& r, const Vec3f& p, const Vec3f& dn, Vec3f *pp)
{
    const Vec3f dv = p - r; // Difference
    const Vec3f du = dv.cross(dn);
    Vec3f dr = dn.cross(du);
    dr.normalize();
    const float l = dv.dot(dr);
    if ( pp)
        *pp = r + l*dr;
    return l;
}   // end projectIntoPlane


Vec3f r3d::intersection( const Vec3f& p0, const Vec3f& p1, const Vec3f& q0, const Vec3f& q1)
{
    const Vec3f p10 = p1 - p0;
    const float pnorm = p10.norm();
    assert( pnorm >= 0);
    if ( pnorm <= 0)
        return Vec3f::Zero();

    const Vec3f q01 = q0 - q1;
    const float qnorm = q01.norm();
    assert( qnorm >= 0);
    if ( qnorm <= 0)
        return Vec3f::Zero();

    const Vec3f u = p10 / pnorm;
    const Vec3f v = q01 / qnorm;
    const Vec3f p2 = p0 + ((q1 - p0).dot(u)) * u;
    const Vec3f pq = p2 - q1;
    const float pqnorm = pq.norm();
    const Vec3f w = pq / pqnorm;
    const float beta = pqnorm / w.dot(v);
    return q1 + beta*v;
}   // end intersection


bool r3d::isPointOnBothLineSegments( const Vec3f& p0, const Vec3f& p1,
                                     const Vec3f& q0, const Vec3f& q1,
                                     const Vec3f& x, float TOLERANCE)
{
    const float np = (p0-p1).squaredNorm() + TOLERANCE;
    const float nq = (q0-q1).squaredNorm() + TOLERANCE;
    const float d0 = (x-p0).squaredNorm();
    const float d1 = (x-p1).squaredNorm();
    const float d2 = (x-q0).squaredNorm();
    const float d3 = (x-q1).squaredNorm();
    return d0 <= np && d1 <= np && d2 <= nq && d3 <= nq;
}   // end isPointOnBothLineSegments
