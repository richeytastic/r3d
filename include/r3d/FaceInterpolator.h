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

#ifndef R3D_FACE_INTERPOLATOR_H
#define R3D_FACE_INTERPOLATOR_H

/**
 * Used with Remesher to correctly interpolate points on input mesh triangles.
 */

#include "FastMarcher.h"

namespace r3d {

class FaceInterpolator
{
public:
    FaceInterpolator( const Mesh &inputMesh,
                      const Mesh &outputMesh,
                      const VecXi &nearestSources,
                      const std::unordered_map<int, std::unordered_map<int, float> > &itimes);

    // Given vidx on the input mesh, find a suitable Face to interpolate
    // over (with vidx as one of its vertices), returning the interpolated position.
    Vec3f interpolate( int vidx) const;

    // Given two external points X and Y and known arrival times of X and Y at A and B, 
    // calculate the time at which X and Y meet along a line segment with A and B as endpoints.
    static float calcLambda( float txa, float txb, float tya, float tyb);

    // Given the value of lambda above, get the point along the line segment between vA and vB.
    // This is calculated simply as (1.0 - lambda)*vA + lambda*vB.
    static Vec3f calcLambdaPoint( float lambda, const Vec3f& vA, const Vec3f& vB);

private:
    const Mesh &_inmod;
    const Mesh &_outmod;
    const VecXi &_nearestSources;
    const std::unordered_map<int, std::unordered_map<int, float> > &_itimes;
    void _printDebug( int,int) const;
};  // end class

}   // end namespace

#endif
