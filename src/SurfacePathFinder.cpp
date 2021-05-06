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


float r3d::calcPathLength( const std::vector<Vec3f>& path)
{
    float psum = 0;
    if ( !path.empty())
    {
        const Vec3f *tv = &path.front();
        for ( const Vec3f& v : path)
        {
            psum += (v - *tv).norm();
            tv = &v;
        }   // end for
    }   // end if
    return psum;
}   // end calcPathLength


namespace {

int findFace( const r3d::Mesh& mesh, const Vec3f& pos, int &vid)
{
    r3d::SurfacePointFinder spf( mesh);
    int fid;
    Vec3f v;
    spf.find( pos, vid, fid, v);
    return fid;
}   // end findFace


int findBestFace( const r3d::Mesh& mesh, int v0, const Vec3f& p1)
{
    assert( v0 >= 0);
    const Vec3f& p0 = mesh.vtx(v0);
    Vec3f dv = p1 - p0;            // Local direction vector to p1
    dv.normalize();
    const IntSet& fids = mesh.faces( v0);  // Get the faces that are attached
    int bfid = *fids.begin();
    float bdp = -1.0f;

    for ( int fid : fids)
    {
        int v1, v2;
        mesh.face( fid).opposite( v0, v1, v2);
        Vec3f fv = 0.5f * (mesh.vtx(v1) + mesh.vtx(v2)) - p0;
        fv.normalize();
        const float dp = fv.dot(dv);
        if ( dp > bdp)
        {
            bdp = dp;
            bfid = fid;
        }   // end if
    }   // end for

    return bfid;
}   // end findBestFace

}   // end namespace


void r3d::findInitialFaces( const r3d::KDTree& kdt, const Vec3f &p0, int &f0, const Vec3f &p1, int &f1)
{
    const Mesh &mesh = kdt.mesh();
    int v0 = kdt.find(p0);
    int v1 = kdt.find(p1);
    f0 = findFace( mesh, p0, v0);
    if ( f0 < 0)    // Try to find a viable starting facet for point 0
        f0 = findBestFace( mesh, v0, p1);
    f1 = findFace( mesh, p1, v1);
    if ( f1 < 0)    // Try to find a viable starting facet for point 1
        f1 = findBestFace( mesh, v1, p0);
}   // end findInitalFaces
