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

#ifndef R3D_SURFACE_PLANAR_PATH_FINDER_H
#define R3D_SURFACE_PLANAR_PATH_FINDER_H

#include "SurfacePathFinder.h"

namespace r3d {

class r3d_EXPORT SurfacePlanarPathFinder : public SurfacePathFinder
{
public:
    /**
     * Vector u defines the orientation of a fixed (global) view plane of the measurement
     * (the slicing plane is defined to be orthogonal to the view plane). Vector u should be
     * linearly independent of the path vector (p1-p0) since the slicing plane's orientation
     * is defined as u.cross(p1-p0). Note that u = -u for these purposes.
     */
    SurfacePlanarPathFinder( const KDTree&, const Vec3f& u);

    /**
     * Find a path on the mesh between the given endpoints which must be on the mesh's surface
     * returning the path sum and causing the path returned from lastPath() to be updated.
     * If a negative value is returned, no path could be found between the points.
     */
    float findPath( const Vec3f& startPos, const Vec3f& endPos) override;

private:
    const Vec3f _u;
};  // end class

}   // end namespace

#endif
