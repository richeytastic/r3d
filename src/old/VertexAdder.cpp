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

#include <VertexAdder.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <cassert>
#include <queue>
using r3d::VertexAdder;
using r3d::Mesh;

namespace {

void findLargeTriangles( std::queue<int>& fids, const IntSet& fset, const Mesh &mesh, float maxArea)
{
    for ( int fid : fset)
    {
        if ( mesh.calcFaceArea(fid) > maxArea)
            fids.push(fid);
    }   // end for
}   // end findLargeTriangles

}   // end namespace


VertexAdder::VertexAdder( Mesh &m) : _mesh(m) {}
                                                        

int VertexAdder::addVerticesToMaxTriangleArea( float maxTriangleArea)
{
    std::queue<int> fids;
    finddLargeTriangles( fids, _mesh.faces(), _mesh, maxTriangleArea);

    int nadded = 0;
    while ( !fids.empty())
    {
        const int fid = fids.front();
        fids.pop();

        // Subdivision position as face centroid
        const int* vidxs = _mesh.fvidxs(fid);
        const Vec3f npos = (_mesh.uvtx(vidxs[0]) + _mesh.uvtx(vidxs[1]) + _mesh.uvtx(vidxs[2])) / 3;
        int nvidx = _mesh.subDivideFace( fid, npos);

        // Check the newly created subdivided faces to see if they need to be subdivided also...
        findLargeTriangles( fids, _mesh.faces(nvidx), _mesh, maxTriangleArea);
        nadded++;
    }   // end while

    return nadded;
}   // end addVerticesToMaxTriangleArea


int ObjModelVertexAdder::subdivideAndMerge( float maxTriangleArea)
{
    int nvadded = 0;

    // Find the set of faces with area greater than maxTriangleArea
    IntSet* fset = new IntSet;
    for ( int fid : _mesh.faces())
    {
        if ( _mesh.calcFaceArea(fid) > maxTriangleArea)
            fset->insert(fid);
    }   // end for

    // fedges will be the set of all edges that may be "edge-flipped"
    unordered_set<Edge, HashEdge> *fedges = new unordered_set<Edge, HashEdge>;
    while ( !fset->empty()) // While no more faces larger than the maximum triangle area...
    {
        for ( int fid : *fset)
        {
            // Get the set of edges that may be flipped from this triangle
            // (includes edges not sharing a single pair of texture coordinates).
            const int* vidxs = _mesh.fvidxs(fid);

            if ( _mesh.nsfaces( vidxs[0], vidxs[1]) == 2)
                fedges->insert( Edge( vidxs[0], vidxs[1]));
            if ( _mesh.nsfaces( vidxs[1], vidxs[2]) == 2)
                fedges->insert( Edge( vidxs[1], vidxs[2]));
            if ( _mesh.nsfaces( vidxs[2], vidxs[0]) == 2)
                fedges->insert( Edge( vidxs[2], vidxs[0]));

            // Subdivide this face into three new faces with new vertex at the centre.
            const Vec3f npos = (_mesh.uvtx(vidxs[0]) + _mesh.uvtx(vidxs[1]) + _mesh.uvtx(vidxs[2])) / 3;
            _mesh.subDivideFace( fid, npos);
            assert( _mesh.faces().count(fid) == 0);
            nvadded++;   // New vertex added
        }   // end for
        fset->clear();

        for ( const Edge& edge : *fedges)
        {
            const int v0 = edge[0];
            const int v1 = edge[1];
            const IntSet& sfids = _mesh.sfaces( v0, v1);
            const int f0 = *sfids.begin();
            const int f1 = *(++sfids.begin());

            // If 1-to-1 mapping of geometry edge to texture edge, then just flip the face pair
            // normally. However, if there's a larger number of texture edges mapped to the
            // geometry edge, need to introduce a new vertex which adds new faces.
            if ( _mesh.numTextureEdges( v0, v1) <= 1)
            {
                // Flip edge join for face pairs where the existing edge join is along the longer pair of opposing vertices.
                int v2 = _mesh.face(f0).opposite( v0, v1);
                int v3 = _mesh.face(f1).opposite( v0, v1);
                if ( ( _mesh.uvtx(v2) - _mesh.uvtx(v3)).squaredNorm() < ( _mesh.uvtx(v0) - _mesh.uvtx(v1)).squaredNorm())
                    _mesh.flipFacePair( v0, v1);

                assert( _mesh.faces().count(f0) > 0);
                assert( _mesh.faces().count(f1) > 0);

                // Note that the face indices don't change after flipping (though the areas of the faces may now be different).
                if ( fset->count(f0) == 0 && _mesh.calcFaceArea( f0) > maxTriangleArea)
                    fset->insert( f0);
                if ( fset->count(f1) == 0 && _mesh.calcFaceArea( f1) > maxTriangleArea)
                    fset->insert( f1);
            }   // end if
            else
            {
                // Since edge v0-->v1 is being subdivided, the old face on this edge will be removed
                // from the model so they also need to be removed from fset.
                fset->erase(f0);
                fset->erase(f1);

                const Vec3f npos = (_mesh.uvtx(v0) + _mesh.uvtx(v1)) * 0.5f;  // New vertex midway between the edge vertices.
                const int nvidx = _mesh.addVertex( npos);
                _mesh.subDivideEdge( v0, v1, nvidx);
                nvadded++;
                for ( int nfid : _mesh.faces(nvidx))
                {
                    if ( fset->count(nfid) == 0 && _mesh.calcFaceArea( nfid) > maxTriangleArea)
                        fset->insert( nfid);
                }   // end for
            }   // end else
        }   // end for
        fedges->clear();
    }   // end while

    delete fset;
    delete fedges;

    return nvadded;
}   // end subdivideAndMerge
