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

#ifndef R3D_SURFACE_GLOBAL_PLANE_PATH_FINDER_H
#define R3D_SURFACE_GLOBAL_PLANE_PATH_FINDER_H

/**
 * Finds the shortest string of points on a surface that lies within a plane defined by
 * the two path end points and a fixed (global) vector which must be linearly independent
 * of the vector formed by the path end points. The slicing plane defines how 'tilted'
 * the path across the surface is. For a non-flat surface, choosing different directions
 * for u will change how curvy the surface path is and thus its length.
 * This suggests an optimisation algorithm for finding the shortest path (in the plane)
 * by iteratively changing the orientation of the fixed slicing plane.
 */

#include "SurfacePathFinder.h"
#include "GlobalPlaneSlicingPath.h"

namespace r3d {

class r3d_EXPORT SurfaceGlobalPlanePathFinder : public SurfacePathFinder
{
public:
    /**
     * The slicing path uses a fixed (global) plane (path points will be incident with the plane).
     * Vector u defines the orientation of the view plane of the measurement (the slicing plane is
     * defined to be orthogonal to the view plane). Vector u should be linearly independent of the
     * path vector (p1-p0) since the slicing plane's orientation is defined as u.cross(p1-p0).
     * The magnitude of given vector u is irrelevant.
     */
    SurfaceGlobalPlanePathFinder( const KDTree&, const Vec3f& u);

    /**
     * Find a path on the mesh between the given endpoints which must be on the mesh's surface
     * returning the path sum and causing the path returned from lastPath() to be updated.
     * If a negative value is returned, no path could be found between the points.
     */
    float findPath( const Vec3f& startPos, const Vec3f& endPos) override;

private:
    const Vec3f _u;
};  // end class


/**
 * Take two paths stemming from a pair of end points and see if they join over the surface.
 * Returns > 0 if the given vector was filled with path vertices.
 */
r3d_EXPORT float findPathOption( PlaneSlicingPath&, int, PlaneSlicingPath&, int, std::vector<Vec3f>&);


/**
 * Given two slicing paths, find and return the shortest (best) surface path that splices them together.
 */
r3d_EXPORT std::vector<Vec3f> findBestPath( PlaneSlicingPath&, PlaneSlicingPath&);

/**
 * Convenience function that uses the SurfacePointFinder to project starting points into their faces.
 * If the given points are incident with mesh vertices (so don't project to faces), faces are chosen in
 * the direction of the straight line between the given points.
 */
r3d_EXPORT void findInitialVertices( const KDTree&, Vec3f &p0, int &f0, Vec3f &p1, int &f1);

}   // end namespace

#endif
