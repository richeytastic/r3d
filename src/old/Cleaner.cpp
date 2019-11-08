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

#include <Cleaner.h>
#include <FeatureUtils.h>
#include <cassert>
#include <cstring>
#include <stack>
using RFeatures::ObjModelCleaner;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;


namespace {

cv::Vec3f removeFlattestPair( const ObjModel* model, IntSet& fids)
{
    cv::Vec3f mnrm(0,0,0);

    // If there's only one element in fids, just remove it and return the zero vector.
    if ( fids.size() == 1)
    {
        fids.clear();
        return mnrm;
    }   // end if

    int bf0 = -1;
    int bf1 = -1;
    double bangle = -1;  // Best cosine of the angle between vectors (maximise)

    std::vector<int> vfids( fids.begin(), fids.end());
    const int n = static_cast<int>(vfids.size());

    for ( int i = 0; i < n-1; ++i)
    {
        const int bfid = vfids[i];
        const cv::Vec3f bnrm = model->calcFaceNorm(bfid);    // Norm for the base poly

        for ( int j = i+1; j < n; ++j)
        {
            const int tfid = vfids[j];
            const cv::Vec3f tnrm = model->calcFaceNorm(tfid);
            const double cosAngle = bnrm.dot(tnrm);
            if ( cosAngle > bangle)
            {
                bangle = cosAngle;
                bf0 = bfid;
                bf1 = tfid;
                mnrm = (bnrm + tnrm) * 0.5f;    // Compute the mean normal of these two polygons
            }   // end if
        }   // end for
    }   // end for

    // Remove the two faces identified as the "flattest" pair (i.e. with normals most pointing in the same direction) from fids.
    fids.erase(bf0);
    fids.erase(bf1);

    return mnrm;
}   // end removeFlattestPair


int replaceJoiningEdges( ObjModel::Ptr model)
{
    const ObjModel* cmodel = model.get();

    IntSet pedges;  // Identify all edges shared by more than two triangles.
    for ( int eid : cmodel->edgeIds())
    {
        if ( cmodel->getSharedFaces(eid).size() > 2)
            pedges.insert(eid);
    }   // end for

    const int SPATIAL_PRECISION = cmodel->spatialPrecision();
    const float DELTA_MAG = powf(10, -SPATIAL_PRECISION);   // Minimum magnitude of shift required for an existing vertex to hash to a new one.
    int fverts[3];

    int vadded = 0;
    for ( int eid : pedges)
    {
        int oldv = cmodel->edge(eid).v0;  // Get a vertex on this edge to change (just need to change one).

        IntSet fids = cmodel->getSharedFaces(eid);    // Copy out the shared polygons
        do
        {
            // The 2 triangles using this edge that lie closest to being in the same plane are removed
            // and a fraction of their mean normal (returned) used to create a new vertex which is mapped
            // to the remaining polygons.
            const cv::Vec3f vdelta = removeFlattestPair( cmodel, fids);

            // All of the remaining triangles in fids have their vertex oldv replaced with a new vertex
            // (which is only added to the model if there are polygons left).
            if ( !fids.empty()) // New vertex
            {
                const cv::Vec3f pos = cmodel->vtx(oldv) + DELTA_MAG * vdelta;
                assert( toKey( cmodel->vtx(oldv), SPATIAL_PRECISION) != toKey( pos, SPATIAL_PRECISION));
                const int newv = model->addVertex(pos);
                assert( newv >= 0);
                assert( newv > oldv);
                vadded++;

                IntSet nfids;   // Will be the set of newly added replacement faces
                for ( int fid : fids)
                {
                    const ObjPoly& poly = cmodel->poly(fid);
                    memcpy( fverts, poly.fvindices, sizeof(int)*3);    // Copy out the vertices
                    fverts[poly.index(oldv)] = newv;    // Replace the vertex index
                    model->removeFace(fid);    // poly no longer valid
                    const int nfid = model->addFace(fverts);
                    assert( nfid >= 0);
                    assert( nfid > fid);
                    nfids.insert(nfid);
                }   // end for

                oldv = newv;
                fids = nfids;
            }   // end if
        } while ( !fids.empty());
    }   // end for

    return vadded;
}   // end replaceJoiningEdges

}   // end namespace


ObjModelCleaner::ObjModelCleaner( ObjModel::Ptr m) : _model(m) {}

int ObjModelCleaner::makeSurface()
{
    int vadded = replaceJoiningEdges( _model);   // Ensure no edges share more than two triangles.

    IntSet pverts;  // Collect problem vertices
    for ( int vid : _model->vertexIds())
    {
    }   // end for

    return vadded;
}   // end makeSurface




void getComplexTopology( const ObjModel* cmodel, int vid)
{
    const IntSet& cvs = cmodel->cvtxs( vid);
    assert( !cvs.empty());

    // If the region around the vertex is a single surface, it should be possible to start
    // from an arbitrary connected vertex and discover all of the vertice's adjacent faces.
    std::stack<int> xplr;       // Exploration front of vertices to check next.
    xplr.push( *cvs.begin());   // Arbitrary connected vertex.

    IntSet fcvs;    // All found connected vertices.
    IntSet ffis;    // All found polygon (face) ids.

    while ( !xplr.empty())
    {
        int cv = xplr.top(); xplr.pop();
        fcvs.insert(cv);

        // Look at the shared faces of vid,cv to explore other edges.
        for ( int fid : _model->spolys( vid, cv))
        {
            if ( ffis.count(fid) == 0)
            {
                ffis.insert(fid);
                const int nv = _model->poly(fid).opposite( vid, cv); // Other vertex on poly not vid or cv
                if ( fcvs.count(nv) == 0)
                    xplr.push(nv);
            }   // end if
        }   // end for
    }   // end while
}   // end getComplexTopology

