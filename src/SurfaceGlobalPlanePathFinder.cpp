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

#include <SurfaceGlobalPlanePathFinder.h>
#include <SurfacePointFinder.h>
using r3d::SurfaceGlobalPlanePathFinder;
using r3d::SurfacePathFinder;
using r3d::GlobalPlaneSlicingPath;
using r3d::KDTree;
using r3d::Mesh;
using r3d::Vec3f;


SurfaceGlobalPlanePathFinder::SurfaceGlobalPlanePathFinder( const KDTree& k, const Vec3f& u)
    : SurfacePathFinder(k), _u(u)
{
    assert( u.squaredNorm() > 0);   // Cannot be zero vector
}   // end ctor


namespace {

Vec3f findInitialVertex( const KDTree& kdt, const Vec3f& pos, int &fid)
{
    r3d::SurfacePointFinder spf( kdt.mesh());
    int vid = kdt.find(pos);
    Vec3f v;
    spf.find( pos, vid, fid, v);
    return v;
}   // end findInitialVertex


int findWithinFace( const Mesh& mesh, int v0, const Vec3f& p1)
{
    assert( v0 >= 0);
    const Vec3f& p0 = mesh.vtx(v0);
    int v1, v2;
    const IntSet& fids = mesh.faces( v0);  // Get the faces that are attached
    for ( int fid : fids)
    {
        mesh.face( fid).opposite( v0, v1, v2);
        // Is pp within the angle made by the two edges of the triangle rooted at p?
        const Vec3f v20 = mesh.vtx(v2) - p0;
        const Vec3f v10 = mesh.vtx(v1) - p0;
        const Vec3f pp = mesh.projectToFacePlane( fid, p1); // Project the other point into the face.
        const Vec3f pp0 = pp - p0;  // pp0 may possibly be the zero vector (very unlikely though)!

        // Dot product will be zero if pp0 is colinear with either of the two edges (or the zero vector).
        if ( v10.cross(pp0).dot( v20.cross(pp0)) <= 0)
            return fid;
    }   // end for

    // It will only be possible to reach here if p1 projects into a space where there isn't a face 
    // attached to vertex p0. In this case we return the first face attached to p0 - it's likely
    // that path finding will fail in this case.
    std::cerr << "[WARNING] r3d::findInitialVertices: Path starts on vertex " << v0 << " and no candidate attached face"
              << "could be found that endpoint " << p1 << " projects into the plane of!" << std::endl;
    return *fids.begin();
}   // end findWithinFace

}   // end namespace


void r3d::findInitialVertices( const KDTree& kdt, Vec3f &p0, int &f0, Vec3f &p1, int &f1)
{
    p0 = findInitialVertex( kdt, p0, f0);
    if ( f0 < 0)
    {
        f0 = findWithinFace( kdt.mesh(), kdt.find(p0), p1);
        assert( f0 >= 0);
    }   // end if
    p1 = findInitialVertex( kdt, p1, f1);
    if ( f1 < 0)
    {
        f1 = findWithinFace( kdt.mesh(), kdt.find(p1), p0);
        assert( f1 >= 0);
    }   // end if
}   // end findInitalVertices


float r3d::findPathOption( PlaneSlicingPath& sp0, int nfid0, PlaneSlicingPath& sp1, int nfid1, std::vector<Vec3f>& path)
{
    sp0.init( nfid0);
    if ( !sp0.canSplice( sp1))
        sp1.init( nfid1);

    //int ex0count = 0; // DEBUG
    //int ex1count = 0; // DEBUG
    while ( !sp0.canSplice(sp1) && sp0.canExtend() && sp1.canExtend())
    {
        //ex0count++;   // DEBUG
        sp0.extend();
        if ( !sp0.canSplice(sp1))
        {
            //ex1count++;   // DEBUG
            sp1.extend();
        }   // end if
    }   // end while

    /*
    std::cerr << " ------------- " << std::endl;
    std::cerr << "Extended sp0 " << ex0count << " times" << std::endl;
    std::cerr << "sp0.initFace() = " << sp0.initFace() << std::endl;
    std::cerr << "sp0.firstFace() = " << sp0.firstFace() << std::endl;
    std::cerr << "sp0.nextFace() = " << sp0.nextFace() << std::endl;
    std::cerr << " ------------- " << std::endl;
    std::cerr << "Extended sp1 " << ex1count << " times" << std::endl;
    std::cerr << "sp1.initFace() = " << sp1.initFace() << std::endl;
    std::cerr << "sp1.firstFace() = " << sp1.firstFace() << std::endl;
    std::cerr << "sp1.nextFace() = " << sp1.nextFace() << std::endl;
    */

    float psum = 0;
    // Joined up path found?
    if ( sp0.canSplice(sp1))
    {
        sp0.splice( sp1, path);
        psum = SurfacePathFinder::calcPathLength( path);

        /*
        std::cerr << "Spliced path:" << std::endl;
        for ( const Vec3f& v : path)
            std::cerr << "  " << v << std::endl;
        std::cerr << "  Path sum = " << psum << std::endl;
        */
    }   // end if

    return psum;
}   // end findPathOption


std::vector<Vec3f> r3d::findBestPath( PlaneSlicingPath& sp0, PlaneSlicingPath& sp1)
{
    // There are four possible paths to take from the two endpoints.
    std::vector<Vec3f> path0, path1;

    //std::cerr << "\n\nFIND_BEST_PATH SET: " << std::endl;
    //std::cerr << "\n * findPathOption 0 *" << std::endl;

    sp0.reset();
    sp1.reset();
    float psum0 = findPathOption( sp0, -1, sp1, -1, path0);
    const int sp0Afid = sp0.firstFace();    // First face that path took from v0 (direction from f0)
    const int sp1Afid = sp1.firstFace();    // First face that path took from v1 (direction from f1)

    //std::cerr << "\n * findPathOption 1 *" << std::endl;

    sp0.reset();
    sp1.reset();
    float psum1 = findPathOption( sp0, sp0Afid, sp1, sp1Afid, path1);  // Choose a different direction to go in for the endpoint
    const int sp0Bfid = sp0.firstFace();    // Second face direction that path took from v0 (direction from f0)
    const int sp1Bfid = sp1.firstFace();    // Second face direction that path took from v1 (direction from f1)

    //std::cerr << "\n * findPathOption 2 *" << std::endl;

    sp0.reset();
    sp1.reset();
    if ( psum0 == 0)
        psum0 = findPathOption( sp0, sp0Bfid, sp1, sp1Afid, path0);
    else if ( psum1 == 0)
        psum1 = findPathOption( sp0, sp0Bfid, sp1, sp1Afid, path1);
    
    //std::cerr << "\n * findPathOption 3 *" << std::endl;

    sp0.reset();
    sp1.reset();
    if ( psum0 == 0)
        psum0 = findPathOption( sp0, sp0Afid, sp1, sp1Bfid, path0);
    else if ( psum1 == 0)
        psum1 = findPathOption( sp0, sp0Afid, sp1, sp1Bfid, path1);

    if ( psum0 == 0)
        psum0 = FLT_MAX;
    if ( psum1 == 0)
        psum1 = FLT_MAX;

    return psum0 < psum1 ? path0 : path1;
}   // end findBestPath


float SurfaceGlobalPlanePathFinder::findPath( const Vec3f& spos, const Vec3f& fpos)
{
    // Find the start and finish end points on the surface of the mesh as v0 and v1 respectively.
    int f0, f1;
    Vec3f v0 = spos;
    Vec3f v1 = fpos;
    findInitialVertices( _kdt, v0, f0, v1, f1);

    // If the start and end vertices are on the same face, the returned list of points is simply the start and end vertex
    if ( f0 == f1)
        _lpath = {v0,v1};
    else
    {
        Vec3f n = _u.cross(v1-v0);
        n.normalize();
        // If it's not possible to define the global slicing plane orientation (because _u does not make an angle with
        // the path vector), then we can't find the path. _u should always be linearly independent of the path vector.
        if ( n.squaredNorm() == 0)
        {
            std::cerr << "[ERROR] r3d::SurfaceGlobalPlanePathFinder::findPath: Unable to define plane slicing orientation!" << std::endl;
            return 0;
        }   // end if

        GlobalPlaneSlicingPath sp0( _kdt.mesh(), f0, v0, n);
        GlobalPlaneSlicingPath sp1( _kdt.mesh(), f1, v1, n);
        _lpath = findBestPath( sp0, sp1);
    }   // end else

    return calcPathLength( _lpath);
}   // end findPath
