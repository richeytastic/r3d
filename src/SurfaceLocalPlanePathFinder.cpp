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

#include <SurfaceLocalPlanePathFinder.h>
#include <SurfaceGlobalPlanePathFinder.h>
#include <LocalPlaneSlicingPath.h>
using r3d::SurfaceLocalPlanePathFinder;
using r3d::SurfacePathFinder;
using r3d::KDTree;


SurfaceLocalPlanePathFinder::SurfaceLocalPlanePathFinder( const KDTree& k) : SurfacePathFinder(k) {}


float SurfaceLocalPlanePathFinder::findPath( const Vec3f& spos, const Vec3f& fpos)
{
    // Find the start and finish end points on the surface of the model as v0 and v1 respectively.
    int f0, f1;
    Vec3f v0 = spos;
    Vec3f v1 = fpos;
    findInitialVertices( _kdt, v0, f0, v1, f1);

    // If the start and end vertices are on the same face, the returned list of points is simply the start and end vertex
    if ( f0 == f1)
        _lpath = {v0,v1};
    else
    {
        LocalPlaneSlicingPath sp0( _kdt.mesh(), f0, v0);
        LocalPlaneSlicingPath sp1( _kdt.mesh(), f1, v1);
        sp0.setEndPath( sp1);
        sp1.setEndPath( sp0);
        _lpath = findBestPath( sp0, sp1);
    }   // end else

    return calcPathLength( _lpath);
}   // end findPath
