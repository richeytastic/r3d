/************************************************************************
 * Copyright (C) 2017 Richard Palmer
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

#ifndef R3D_CAMERA_PARAMS_H
#define R3D_CAMERA_PARAMS_H

#ifdef _WIN32
#pragma warning( disable : 4251)
#endif

#include "r3dTypes.h"
#include <iostream>

namespace r3d {

class r3d_EXPORT CameraParams
{
public:
    CameraParams( const Vec3f &pos=Vec3f(0,0,1),
                  const Vec3f &focus=Vec3f(0,0,0),
                  const Vec3f &up=Vec3f(0,1,0),     // +ve Y
                  float fov=30);   // 30 degrees vertical field of view

    // Return the camera position.
    inline const Vec3f& pos() const { return _pos;}

    // Where the camera is focused.
    inline const Vec3f& focus() const { return _foc;}

    // The "up" vector for the camera view.
    inline const Vec3f& up() const { return _upv;}

    // The vetical field of view in degrees in (0,180].
    inline float fov() const { return _fov;}

    // Get the distance between the camera position and its focus.
    inline float distance() const { return (pos() - focus()).norm();}

    // Set a new focus and position on the camera.
    // Returns false if unable to set because the (postion - focus)
    // vector is equal to the current up vector.
    bool set( const Vec3f &focus, const Vec3f &position);

    // Set a new field of view in degrees (constrained to be in (0,180]).
    void setFoV( float v);

    // Given a view radius for the image plane, set a new field
    // of view while keeping the camera position fixed.
    void setViewRadius( float r);

    // Reposition the camera r units from the focus (or a new focus if given)
    // along the line between the existing camera position and the focus.
    void setPositionFromFocus( float r, const Vec3f* newFocus=nullptr);

    // Project t's location onto the view plane as x,y proportional coordinates from the top left.
    cv::Point2f project( const Vec3f& t) const;

    // Convenience function which takes the proportional coordinates and returns actual coords.
    cv::Point project( const Vec3f& t, const cv::Size2i& viewPortSize) const;

    // Do perspective projection to return a world position with the X and Y values calculated and Z as given.
    Vec3f project( const cv::Point2f& p, float Z) const;

    // Get the distance from t to the plane cooincident with the camera.
    float distance( const Vec3f& t) const;

    // Set a new position based on a clockwise rotation along the up vector.
    void rotateAboutUpAxis( float degs);

    // Convenience calculations
    float fovRads() const; // fov * pi/180

    // Returns the standardised focal length between 0 and 1.
    float calcFocalLength() const; // cos(fovRads(fov)/2)/sin(fovRads(fov)/2)

private:
    Vec3f _pos; // Camera position
    Vec3f _foc; // Camera focus
    Vec3f _upv; // Camera up vector
    float _fov; // Vertical field of view in degrees (must be > 0 and <= 180)
};  // end class


r3d_EXPORT std::ostream& operator<<( std::ostream&, const r3d::CameraParams&);

}   // end namespace

#endif
