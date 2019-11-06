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

#include <SurfacePathFinder.h>
#include <SurfacePointFinder.h>
#include <AStarSearch.h>
#include <cassert>
using r3d::SurfacePathFinder;
using r3d::KDTree;
using r3d::Mesh;
using r3d::Vec3f;


SurfacePathFinder::SurfacePathFinder( const KDTree& k) : _kdt(k) {}


float SurfacePathFinder::findPath( const Vec3f& spos, const Vec3f& fpos)
{
    _lpath.clear();

    // Find the start and end point vertices on the mesh's surface.
    SurfacePointFinder spf( _kdt.mesh());
    int f0, f1;
    int p0 = _kdt.find(spos); // First vertex
    int p1 = _kdt.find(fpos); // Last vertex
    const Vec3f v0 = spf.find( spos, p0, &f0);
    const Vec3f v1 = spf.find( fpos, p1, &f1);

    /*
    // Ensure that p0 and p1 are valid vertex indices on the mesh
    p0 = findClosestVertexIndex( _kdt.mesh(), p0, f0, v0);
    p1 = findClosestVertexIndex( _kdt.mesh(), p1, f1, v1);
    */

    AStarSearch ass( _kdt.mesh());
    ass.setEndPointVertexIndices( p0, p1);
    std::vector<int> pvids;
    ass.findShortestPath( pvids);
    if ( pvids.size() < 1)
        return -1;

    size_t nps = pvids.size();
    assert( nps >= 2);

    // If the point closest to spos is not incident with a vertex, then
    // check the polygon it's incident with to determine if this is the
    // same polygon as shared by the first two path vertices. If so, then
    // we skip the first discovered path vertex because this leads us
    // away from the desired path direction.
    size_t i = 0;  // Will be the modified start vertex for determined path
    // If the shared face of the first two discovered path vertices == f0, then
    // v0 is incident with face f0 and the first path vertex can be ignored.
    if (( f0 >= 0) && ( _kdt.mesh().sfaces(pvids[0], pvids[1]).count(f0) > 0))
    {
        _lpath.push_back(v0);   // Initial vertex
        i = 1;
    }   // end if

    // Similarly for the end point
    bool pushv1 = false;
    if (( f1 >= 0) && ( _kdt.mesh().sfaces(pvids[nps-1], pvids[nps-2]).count(f1) > 0))
    {
        nps--;
        pushv1 = true;
    }   // end if

    for ( ; i < nps; ++i)
        _lpath.push_back( _kdt.mesh().vtx( pvids[i]));

    if ( pushv1)    // If ended early
        _lpath.push_back(v1);   // Last vertex

    return calcPathLength(_lpath);
}   // end findPath


float SurfacePathFinder::calcPathLength( const std::vector<Vec3f>& path)
{
    float psum = 0;
    if ( !path.empty())
    {
        Vec3f tv = path.front();
        for ( const Vec3f& v : path)
        {
            psum += (v - tv).norm();
            tv = v;
        }   // end for
    }   // end if
    return psum;
}   // end calcPathLength
