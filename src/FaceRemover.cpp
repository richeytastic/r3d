/************************************************************************
 * Copyright (C) 2017 Richard Palmer
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

#include <ObjModelPolygonCollapser.h>
using RFeatures::ObjModelPolygonCollapser;
using RFeatures::ObjPolySet;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <cassert>
#include <cstdlib>


ObjModelPolygonCollapser::ObjModelPolygonCollapser( ObjModel::Ptr m) : _model(m) {}


namespace {
// On all attached polygons, replace vertex rvid with vertex nvid
// (adjusting material texture offsets where needed).
void replaceVertex( ObjModel::Ptr m, int rvid, int nvid, int matId, const cv::Vec2f& tx)
{
    int nvs[3];
    const cv::Vec2f* txs[3];
    const ObjPolySet cfs = m->faces( rvid);    // Copied out since removing polygons
    for ( const ObjPoly* f : cfs)
    {
        // Get the vertex IDs for the new face to add (identifying and replacing the removed vertex)
        nvs[0] = f->at(0) == rvid ? nvid : f->at(0);
        nvs[1] = f->at(1) == rvid ? nvid : f->at(1);
        nvs[2] = f->at(2) == rvid ? nvid : f->at(2);

        if ( m->face( nvs[0], nvs[1], nvs[2]))
        {
            const ObjPoly* ff = m->face( nvs[0], nvs[1], nvs[2]);
            const int ffid = m->faceId(ff);
            const int fid = m->faceId(f);
            std::cerr << "Found face ID " << ffid << " with vertices " << *ff << std::endl;
            std::cerr << "Looking to replace face " << fid << " with vertices " << *f << std::endl;
            std::cerr << "Replacing vertex " << rvid << " with new vertex " << nvid << std::endl;
            continue;
        }   // end if

        const int newfid = m->addFace( nvs);

        // If this polygon has the same material ID, set the new texture offsets.
        if ( matId >= 0 && m->faceMaterialId(f) == matId)
        {
            const int* uvis = m->faceUVs(f);
            if ( f->at(0) == rvid)
            {
                txs[0] = &tx;
                txs[1] = &m->uv( matId, uvis[1]);
                txs[2] = &m->uv( matId, uvis[2]);
            }   // end if
            else if ( f->at(1) == rvid)
            {
                txs[0] = &m->uv( matId, uvis[0]);
                txs[1] = &tx;
                txs[2] = &m->uv( matId, uvis[2]);
            }   // end else if
            else
            {
                assert( f->at(2) == rvid);
                txs[0] = &m->uv( matId, uvis[0]);
                txs[1] = &m->uv( matId, uvis[1]);
                txs[2] = &tx;
            }   // end else if

            assert( m->faceMaterialId( newfid) < 0);
            m->setOrderedFaceUVs( matId, newfid, *txs[0], *txs[1], *txs[2]);
        }   // end if

        m->removeFace(f); // Finally, remove the old polygon.
    }   // end for
}   // end replaceVertex


void collapseEdge( ObjModel::Ptr m, int rv0, int rv1, int matId, int rt0, int rt1)
{
    const cv::Vec3f nvert = ( m->vtx(rv0) + m->vtx(rv1)) * 1.0f/2;  // New interpolated vertex on edge
    const int nvidx = m->addVertex( nvert);
    cv::Vec2f tx2(0,0);
    if ( matId >= 0)
        tx2 = (m->uv(matId, rt0) + m->uv(matId, rt1)) * 0.5f;
    replaceVertex( m, rv0, nvidx, matId, tx2);
    replaceVertex( m, rv1, nvidx, matId, tx2);
    m->removeVertex( rv0);
    m->removeVertex( rv1);
}   // end collapseEdge

}   // end namespace


int ObjModelPolygonCollapser::collapse( int fid)
{
    ObjModel::Ptr m = _model;
    assert( fid >= 0 && fid < m->numFaces());
    if ( fid < 0 || fid >= m->numFaces())
        return -1;

    const int* vindices = m->fvidxs(fid);  // Always in texture offset order if fid maps a texture

    // If this polygon is not a part of a local triangulated mesh, cannot remove the it!
    if ( m->nspolys( vindices[0], vindices[1]) > 2
      || m->nspolys( vindices[1], vindices[2]) > 2
      || m->nspolys( vindices[2], vindices[0]) > 2)
        return -2;

    int vids[3];   // Copy out since removing the face
    memcpy( vids, vindices, 3*sizeof(int));

    // Get the texture coordinates for the polygon being removed.
    // This will be used if adjacent polys with vertices being moved also have the same material (generally the case).
    const int matId = m->faceMaterialId( fid);

    // Copy out the UV indices since removing the face
    int uvis[3] = {-1,-1,-1};
    if ( matId >= 0)
        memcpy( uvis, m->faceUVs(fid), 3*sizeof(int));

    m->removeFace(fid);  // Remove the polygon

    // If the polygon shares no other polygons with any of its edges, or shares polygons with only one
    // of its edges, collapsing the polygon is the same as removing it since no other polygons will be affected.
    const ObjPolySet& sfids0 = m->spolys( vids[0], vids[1]);
    const ObjPolySet& sfids1 = m->spolys( vids[1], vids[2]);
    const ObjPolySet& sfids2 = m->spolys( vids[2], vids[0]);
    const size_t polyEdgeCount = sfids0.size() + sfids1.size() + sfids2.size();
    assert( polyEdgeCount < 4);

    // If the polygon shares vertices with all three of its edges, the polygon will be collapsed to a point.
    // This is the usual behaviour for the vast majority of polygons in a triangulated mesh.
    if ( polyEdgeCount == 3)
    {
        // Remove polygons that share edges of the removed polygon. The gaps left behind will be
        // filled when the 3 vertices of the removed polygon are reduced to a single newly inserted vertex.
        if ( !sfids0.empty()) m->removeFace( *sfids0.begin());
        if ( !sfids1.empty()) m->removeFace( *sfids1.begin());
        if ( !sfids2.empty()) m->removeFace( *sfids2.begin());

        // The removed polygon's vertices are reduced to a new single vertex found as the mean of these vertices.
        const cv::Vec3f nvert = (m->vtx(vids[0]) + m->vtx(vids[1]) + m->vtx(vids[2])) * 1.0f/3;
        const int nvidx = m->addVertex( nvert);
        cv::Vec2f tx3;  // New mean texture UV
        if ( matId >= 0)
            tx3 = (m->uv(matId, uvis[0]) + m->uv(matId, uvis[1]) + m->uv(matId, uvis[2])) * 1.f/3;

        // For the remaining polygons attached to rvids, the corresponding vertex needs to be replaced with nvidx
        replaceVertex( m, vids[0], nvidx, matId, tx3);
        replaceVertex( m, vids[1], nvidx, matId, tx3);
        replaceVertex( m, vids[2], nvidx, matId, tx3);
        m->removeVertex( vids[0]);
        m->removeVertex( vids[1]);
        m->removeVertex( vids[2]);
    }   // end if
    else if ( polyEdgeCount == 2)
    {
        // New vertex will be positioned halfway along the edge that has no other polygons shared with it.
        if ( !sfids0.empty() && !sfids1.empty())
            collapseEdge( m, vids[0], vids[2], matId, uvis[0], uvis[2]);
        if ( !sfids1.empty() && !sfids2.empty())
            collapseEdge( m, vids[0], vids[1], matId, uvis[0], uvis[1]);
        if ( !sfids0.empty() && !sfids2.empty())
            collapseEdge( m, vids[1], vids[2], matId, uvis[1], uvis[2]);
    }   // end else if
    else if ( polyEdgeCount == 1)
    {
        // Need to remove the vertex opposite the edge that is being shared.
        if ( !sfids0.empty())
            m->removeVertex( vids[2]);
        else if ( !sfids1.empty())
            m->removeVertex( vids[0]);
        else if ( !sfids2.empty())
            m->removeVertex( vids[1]);
    }   // end else if
    else
    {   // polyEdgeCount == 0
        // Polygon fid is a single isolated polygon so its vertices are no longer required.
        m->removeVertex( vids[0]);
        m->removeVertex( vids[1]);
        m->removeVertex( vids[2]);
    }   // end else

    return (int)polyEdgeCount;
}   // end collapse
