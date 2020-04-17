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

#include <IterativeSurfacePathFinder.h>
#include <SurfacePointFinder.h>
using r3d::IterativeSurfacePathFinder;
using r3d::SurfacePathFinder;
using r3d::KDTree;
using r3d::Vec3f;


IterativeSurfacePathFinder::IterativeSurfacePathFinder( const KDTree& k) : SurfacePathFinder(k) {}


namespace {
int findVectorId( const KDTree &kdt, const Vec3f &p)
{
    int id = -1;
    const int vid = kdt.find(p);
    r3d::SurfacePointFinder( kdt.mesh()).find( p, vid, &id);
    if ( id < 0)   // p is incident with vid
        id = -(vid+1);  // Return vertex ID (+1)
    else
        id++;   // Return face ID (+1)
    return id;
}   // end findVectorId


Vec3f calcVertexVector( const r3d::Mesh& mesh, std::unordered_map<int, Vec3f> &vectors, int vid)
{
    Vec3f u = Vec3f::Zero();
    const IntSet &fids = mesh.faces(vid);
    for ( int fid : fids)
    {
        const int id = fid+1;   // Ids for faces are +1
        if ( vectors.count(id) == 0)
            vectors[id] = mesh.calcFaceVector(fid);
        u += vectors.at(id);
    }   // end for
    u.normalize();
    return u;
}   // end calcVertexVector

}   // end namespace


float IterativeSurfacePathFinder::findPath( const Vec3f& spos, const Vec3f& fpos)
{
    // Find the start and finish end points on the surface of the model as v0 and v1 respectively.
    int f0, f1;
    Vec3f v0 = spos;
    Vec3f v1 = fpos;
    findInitialVertices( _kdt, v0, f0, v1, f1);

    // Cache calculated face vectors
    std::unordered_map<int, Vec3f> vectors;
    vectors[f0] = _kdt.mesh().calcFaceVector(f0);
    vectors[f1] = _kdt.mesh().calcFaceVector(f1);

    // Set initial normal as the mean normal of the two endpoint faces.
    // Note use of face area (intrinsic in vector magnitudes) to weight vector.
    Vec3f u = vectors[f0] + vectors[f1];

    // Now basically iterate using the global path finder, updating the global plane normal each time
    // using a new estimate from the previous path, until the faces that are crossed no longer change.
    IntSet crossedSet0;
    IntSet crossedSet1;
    IntSet *oldCrossedSet = &crossedSet0;
    IntSet *newCrossedSet = &crossedSet1;

/*
#ifndef NDEBUG
    std::cout << "--------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "Looking for path between " << v0.transpose() << " and " << v1.transpose() << std::endl;
#endif
*/
    static const size_t MAX_ITERS = 10;
    size_t numIts = 0;
    while ( true)
    {
        numIts++;
        newCrossedSet->clear();

        u.normalize();
        SurfaceGlobalPlanePathFinder gpfinder( _kdt, u);
        gpfinder.findPath( v0, v1);
        const std::vector<Vec3f>& fpath = gpfinder.lastPath();

        bool diffFaceAdded = false;

        u = Vec3f::Zero();
        for ( const Vec3f& v : fpath)
        {
            const int id = findVectorId( _kdt, v);
            if ( vectors.count(id) == 0)
                vectors[id] = id > 0 ? _kdt.mesh().calcFaceVector(id-1) : calcVertexVector( _kdt.mesh(), vectors, -(id+1));
            u += vectors[id];

            // Note here that it's possible that the face id membership of
            // the two sets may flip back and forth swapping in and out the
            // same pair of face ids so need another check.
            if ( oldCrossedSet->count(id) == 0)
                diffFaceAdded = true;
            newCrossedSet->insert(id);
        }   // end for

        if ( diffFaceAdded && numIts < MAX_ITERS)
            std::swap( oldCrossedSet, newCrossedSet);
        else
        {
            _lpath = fpath;
            break;
        }   // end else
    }   // end while
/*
#ifndef NDEBUG
    std::cout << "Found path after " << numIts << " iterations" << std::endl;
#endif
*/
    return calcPathLength( _lpath);
}   // end findPath
