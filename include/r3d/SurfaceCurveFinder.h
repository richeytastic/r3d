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

#ifndef R3D_SURFACE_CURVE_FINDER_H
#define R3D_SURFACE_CURVE_FINDER_H

#include "SurfacePathFinder.h"

namespace r3d {

class r3d_EXPORT SurfaceCurveFinder : public SurfacePathFinder
{
public:
    explicit SurfaceCurveFinder( const KDTree&);

    // Creates a path between v0 and v1 where the orientation of the curve is determined
    // by the local curvature of every polygon crossed between the two points.
    // Returns the length of the path if found or a negative value if not found.
    float findPath( const Vec3f& v0, const Vec3f& v1) override;

private:
    int _getOppositeEdge( const Vec3f&, const Vec3f&, int) const;
};  // end class

}   // end namespace

#endif
