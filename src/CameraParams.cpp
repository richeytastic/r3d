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

#include <CameraParams.h>
#include <Eigen/Geometry>
using r3d::CameraParams;
using r3d::Vec3f;
#include <cassert>


CameraParams::CameraParams( const Vec3f& p, const Vec3f& f, const Vec3f& u, float fv)
    : _pos(p), _foc(f), _upv(u), _fov(fv), _pscale(0.0f)
{
    _upv.normalize(); // Ensure up vector is unit length
    assert( fv > 0.0 && fv < 180.0);
    Vec3f tvec = _foc - _pos;
    tvec.normalize();
    assert( fabsf(tvec.dot(_upv)) < 0.0005f);    // up vector must always be orthogonal to focus - position vector
}  // end ctor


bool CameraParams::set( const Vec3f &foc, const Vec3f &pos)
{
    // Ensure up vector remains orthogonal to position - focus
    Vec3f cvec = pos - foc;
    cvec.normalize();
    Vec3f rvec = cvec.cross(_upv);
    if ( rvec.isZero())
        return false;
    _upv = rvec.cross(cvec);
    _upv.normalize();
    _foc = foc;
    _pos = pos;
    return true;
}   // end set


void CameraParams::setPositionFromFocus( float r, const Vec3f* newfocus)
{
    if ( newfocus)
        _foc = *newfocus;
    Vec3f dirVec = _pos - _foc;
    dirVec.normalize();
    _pos = r*dirVec + _foc;
}   // end setPositionFromFocus


namespace {

Vec3f rotateAboutAxis( const Vec3f &pos, Vec3f axis, float degs)
{
    axis.normalize();
    Eigen::Quaternionf r;
    r = Eigen::AngleAxisf( degs * (float)(EIGEN_PI/180), axis);
    r.normalize();

    Eigen::Quaternionf qpos;
    qpos.w() = 0;
    qpos.vec() = pos;

    Eigen::Quaternionf rotPos = r * qpos * r.inverse();
    return rotPos.vec();
}   // end rotateAboutAxis

}   // end namespace


void CameraParams::rotate( float degs)
{
    const Vec3f npos = rotateAboutAxis( _pos, _upv, degs);
    set( _foc, npos);
}   // end rotate


void CameraParams::tilt( float degs)
{
    Vec3f cvec = _pos - _foc;
    cvec.normalize();
    const Vec3f rvec = cvec.cross(_upv);
    const Vec3f npos = rotateAboutAxis( _pos, rvec, degs);
    set( _foc, npos);
}   // end tilt


cv::Point2f CameraParams::project( const Vec3f& t) const
{
    assert( _upv.norm() == 1);

    Vec3f nrmFocVec = _foc - _pos;    // Relative to the camera position
    nrmFocVec.normalize();
    assert( nrmFocVec.dot(_upv) == 0);    // up vector must always be orthogonal to focus - position vector

    Vec3f rt = t - _pos; // Target position relative to the camera
    const float ty = _upv.dot(rt);  // Amount up (+Y) (real world coord metric)

    Vec3f rightVec = nrmFocVec.cross(_upv);
    rightVec.normalize();
    const float tx = rightVec.dot(rt); // Amount right (+X) (real world coord metric)

    // The view plane is assumed here to have unit height (can be scaled later by client)
    const float rfov = fovRads();
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
    Vec3f nrmFocVec = _foc - _pos;
    nrmFocVec.normalize();
    return (t - _pos).dot( nrmFocVec);
}   // end distance


void CameraParams::setFoV( float v)
{
    _fov = std::max( FLT_MIN, std::min( v, 180.0f - FLT_MIN));
}   // end setFoV


void CameraParams::setViewRadius( float r) { setFoV( (180.0/EIGEN_PI) * atan2f( r, distance()));}


float CameraParams::fovRads() const { return float(_fov * EIGEN_PI/180);}


float CameraParams::calcFocalLength() const { return float(1.0/tan( _fov * EIGEN_PI/360.0));}


std::ostream& r3d::operator<<( std::ostream& os, const CameraParams& cp)
{
    os << "POSITION:   " << cp.pos().transpose() << std::endl;
    os << "FOCUS: " << cp.focus().transpose() << std::endl;
    os << "UP:    " << cp.up().transpose() << std::endl;
    if ( cp.isParallel())
        os << "PARALLEL_SCALE: " << cp.parallelScale() << std::endl;
    else
        os << "FOV:   " << cp.fov() << std::endl;
    return os;
}   // end operator<<
