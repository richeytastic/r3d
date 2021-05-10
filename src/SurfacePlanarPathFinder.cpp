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

#include <SurfacePlanarPathFinder.h>
#include <PlanarSlicingPath.h>
#include <SurfacePointFinder.h>
#include <utility>
using r3d::SurfacePlanarPathFinder;
using r3d::SurfacePathFinder;
using r3d::Vec3f;


SurfacePlanarPathFinder::SurfacePlanarPathFinder( const r3d::KDTree& k, const Vec3f& u)
    : SurfacePathFinder(k), _u(u)
{
    assert(!u.isZero());
}   // end ctor


float SurfacePlanarPathFinder::findPath( const Vec3f& v0, const Vec3f& v1)
{
    int f0, f1;
    findInitialFaces( _kdt, v0, f0, v1, f1);
    float plen = FLT_MAX;
    // If start and end vertices are on same facet, the returned
    // list of points is simply the start and end vertex
    if ( f0 == f1)
    {
        _lpath = {v0,v1};
        plen = (v1 - v0).norm();
    }   // end if
    else
    {
        Vec3f n = _u.cross(v1-v0);
        assert( !n.isZero());
        n.normalize();
        const std::function<Vec3f(int)> fn = [=](int){ return n;};
        _lpath = PlanarSlicingPath( _kdt.mesh(), f0, v0, f1, v1, fn).shortestPath( &plen);
    }   // end else

    return plen;
}   // end findPath
