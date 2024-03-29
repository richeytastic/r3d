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

#ifndef R3D_SURFACE_PATH_FINDER_H
#define R3D_SURFACE_PATH_FINDER_H

/**
 * Finds a path over the surface of the given mesh between a start and end point.
 * The default implementation of findPath uses DijkstraShortestPathFinder.
 */

#include "KDTree.h"

namespace r3d {

class r3d_EXPORT SurfacePathFinder
{
public:
    explicit SurfacePathFinder( const KDTree&);

    /**
     * Returns last path calculated from findPath. All points lie on the mesh surface.
     * The first and last points will be the points on the mesh closest to the start
     * and end points provided to findPath. If an empty vector is returned, no calls
     * to findPath have been received.
     */
    const std::vector<Vec3f>& lastPath() const { return _lpath;}

    /**
     * Find a path between the given endpoints returning the path sum and updating
     * the path returned from lastPath(). If a negative value is returned, no path
     * could be found between the points.
     */
    virtual float findPath( const Vec3f& startPos, const Vec3f& endPos);

protected:
    const KDTree &_kdt;
    std::vector<Vec3f> _lpath;
};  // end class

/**
 * Convenience function that uses the SurfacePointFinder to project starting points into their faces.
 * If the given points are incident with mesh vertices (so don't project to faces), faces are chosen
 * in the direction of the straight line between the given points.
 */
r3d_EXPORT void findInitialFaces( const KDTree&, const Vec3f &p0, int &f0, const Vec3f &p1, int &f1);

/**
 * Returns the sum of the distances between consecutive points in the given vector.
 */
r3d_EXPORT float calcPathLength( const std::vector<Vec3f>&);

}   // end namespace

#endif
