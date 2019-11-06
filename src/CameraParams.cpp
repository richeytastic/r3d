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

#include <CameraParams.h>
#include <Eigen/Geometry>
using r3d::CameraParams;
using r3d::Vec3f;
#include <cassert>


CameraParams::CameraParams( const Vec3f& p, const Vec3f& f, const Vec3f& u, float fv)
    : pos(p), focus(f), up(u), fov(fv)
{
    up.normalize(); // Ensure up vector is unit length
    assert( fov > 0.0 && fov <= 180.0);
    Vec3f tvec = focus - pos;
    tvec.normalize();
    assert( fabsf(tvec.dot(up)) < 0.0005f);    // up vector must always be orthogonal to focus - position vector
}  // end ctor


void CameraParams::setPositionFromFocus( float r, const Vec3f* newfocus)
{
    if ( newfocus)
        focus = *newfocus;
    Vec3f dirVec = pos - focus;
    dirVec.normalize();
    pos = float(r)*dirVec + focus;
}   // end setPositionFromFocus


void CameraParams::rotateAboutUpAxis( float degs)
{
    Vec3f axis = up;
    axis.normalize();
    Eigen::Quaternionf r;
    r = Eigen::AngleAxisf( degs * (float)(CV_PI/180), axis);
    r.normalize();

    Eigen::Quaternionf qpos;
    qpos.w() = 0;
    qpos.vec() = pos;

    Eigen::Quaternionf rotPos = r * qpos * r.inverse();
    pos = rotPos.vec();
}   // end rotateAboutUpAxis


cv::Point2f CameraParams::project( const Vec3f& t) const
{
    assert( up.norm() == 1);

    Vec3f nrmFocVec = focus - pos;    // Relative to the camera position
    nrmFocVec.normalize();
    assert( nrmFocVec.dot(up) == 0);    // up vector must always be orthogonal to focus - position vector

    Vec3f rt = t - pos; // Target position relative to the camera
    const float ty = up.dot(rt);  // Amount up (+Y) (real world coord metric)

    Vec3f rightVec = nrmFocVec.cross(up);
    rightVec.normalize();
    const float tx = rightVec.dot(rt); // Amount right (+X) (real world coord metric)

    // The view plane is assumed here to have unit height (can be scaled later by client)
    const float rfov = float( CV_PI * fov / 180);  // Convert degrees to radians
    const float focLen = 0.5f/tanf(rfov/2);

    const float realDistance = distance( t);
    float viewPlaneUp = focLen * ty/realDistance;  // From centre of view plane
    float viewPlaneRight = focLen * tx/realDistance;   // From centre of view plane

    // Translate offsets to be from upper left instead of centre
    return cv::Point2f( viewPlaneRight + 0.5f, 0.5f - viewPlaneUp);
}   // end project


cv::Point CameraParams::project( const Vec3f& t, const cv::Size2i& dims) const
{
    const cv::Point2f pp = project( t);
    return cv::Point( (int)cvRound(dims.width * pp.x), (int)cvRound(dims.height * pp.y));
}   // end project


Vec3f CameraParams::project( const cv::Point2f& p, float Z) const
{
    const float S = Z/calcFocalLength();
    return Vec3f( S*p.x, S*p.y, Z);
}   // end project


float CameraParams::distance( const Vec3f& t) const
{
    Vec3f nrmFocVec = focus - pos;
    nrmFocVec.normalize();
    return (t - pos).dot( nrmFocVec);
}   // end distance


float CameraParams::fovRads() const { return fov * float(CV_PI/180);}


float CameraParams::calcFocalLength() const
{
    const float hfov = fovRads()/2;
    return cosf(hfov)/sinf(hfov);
}   // end calcFocalLength


std::ostream& operator<<( std::ostream& os, const CameraParams& cp)
{
    os << "CAM_POS:   " << cp.pos.transpose() << std::endl;
    os << "CAM_FOCUS: " << cp.focus.transpose() << std::endl;
    os << "CAM_UP:    " << cp.up.transpose() << std::endl;
    os << "CAM_FOV:   " << cp.fov << std::endl;
    return os;
}   // end operator
