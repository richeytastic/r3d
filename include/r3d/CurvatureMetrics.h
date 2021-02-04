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

#ifndef R3D_CURVATURE_METRICS_H
#define R3D_CURVATURE_METRICS_H

#include "Curvature.h"

namespace r3d {

class r3d_EXPORT CurvatureMetrics
{
public:
    explicit CurvatureMetrics( const Curvature&);

    // The first order derivative of the surface function is simply the curvature function.
    float faceKP1FirstOrder( int fid) const;    // Max curvature
    float faceKP2FirstOrder( int fid) const;    // Min curvature
    float vertexKP1FirstOrder( int vid) const;  // Max curvature
    float vertexKP2FirstOrder( int vid) const;  // Min curvature

    float faceDeterminant( int fid) const;
    float vertexDeterminant( int vid) const;
    Vec3f vertexNormal( int vid) const;

private:
    const Curvature &_cmap;
};  // end class

}   // end namespace

#endif
