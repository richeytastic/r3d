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

#ifndef R3D_ITERATIVE_SURFACE_PATH_FINDER_H
#define R3D_ITERATIVE_SURFACE_PATH_FINDER_H

#include "SurfaceGlobalPlanePathFinder.h"

namespace r3d {

class r3d_EXPORT IterativeSurfacePathFinder : public SurfacePathFinder
{
public:
    explicit IterativeSurfacePathFinder( const KDTree&);

    float findPath( const Vec3f& startPos, const Vec3f& endPos) override;

private:
    const Vec3f _u;
};  // end class

}   // end namespace

#endif
