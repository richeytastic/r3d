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

#include <FaceUnfoldingVertexSearcher.h>
#include <FaceAngles.h>
#include <FaceUnfolder.h>
using r3d::FaceUnfoldingVertexSearcher;
using r3d::FaceUnfolder;
using r3d::Mesh;
using r3d::Vec3f;
#include <iostream>
#include <cassert>

namespace {
// Get the other face in fids that isn't fid. Returns fid if only fid present.
int getOther( const IntSet& fcs, int fid)
{
    assert( fcs.size() <= 2);
    for ( int f : fcs)
        if ( f != fid)
            return f;
    return fid;
}   // end getOther
}   // end namespace


FaceUnfoldingVertexSearcher::FaceUnfoldingVertexSearcher( const Mesh &m) : _mesh(m) {}


int FaceUnfoldingVertexSearcher::operator()( int ui, int T, Vec3f& upos)
{
    const float theta = FaceAngles::calcInnerAngle( _mesh, T, ui);
    return operator()( ui, T, theta, upos);
}   // end operator()


int FaceUnfoldingVertexSearcher::operator()( int ui, int T, float theta, Vec3f& upos)
{
    assert( theta >= float(CV_PI/2));
    FaceUnfolder polyUnfolder( _mesh, T);

    const Face& face = _mesh.face( T);
    int uj, uk;
    face.opposite( ui, uj, uk);

    const Vec3f& vi = _mesh.vtx( ui);   // The position of the search vector
    const Vec3f& vj = _mesh.vtx( uj);
    const Vec3f& vk = _mesh.vtx( uk);

    _cVec = vi;

    Vec3f v0 = vj - vi;
    v0.normalize();
    Vec3f v1 = vk - vi;
    v1.normalize();
    _secDirVec = v0 + v1;
    _secDirVec.normalize();

    // theta is defined to be the angle between v0 and v1
    // The valid section angle defined to be the allowed angle between _secDirVec and any candidate vector.
    _alpha = float(CV_PI - theta)/2;

    _initEdgePos = vj;
    _initUnitEdge = vk - vj;
    _halfInitEdgeNorm = _initUnitEdge.norm() / 2;
    _initUnitEdge.normalize();

    _recursionLim = 0;
    _parsedTriangles.clear();
    _parsedTriangles.insert(T);
    const int nextT = getOther( _mesh.sfaces( uj, uk), T);
    const int ufound = _searchForVertexInUnfoldingSection( &polyUnfolder, uj, uk, nextT);
    if ( ufound >= 0)
        upos = (Vec3f)polyUnfolder.uvtx(ufound);

    return ufound;
}   // end operator()


int FaceUnfoldingVertexSearcher::_searchForVertexInUnfoldingSection( FaceUnfolder* polyUnfolder, int u0, int u1, int T)
{
    if ( _parsedTriangles.count(T) > 0)
        return -1;

    _parsedTriangles.insert(T);

    const int u2 = polyUnfolder->unfold( T, u0, u1);
    const Vec3f &v2 = polyUnfolder->uvtx( u2);

    // If the translated position of u2 (v2) is within the planar section search area, we're done!
    Vec3f v2dir = v2 - _cVec; // Direction of v2 from the initial search point
    v2dir.normalize();
    const float r0 = std::min( std::max( -1.0f, v2dir.dot(_secDirVec)), 1.0f);
    const float theta = acosf( r0);  // Angle between the vectors
    if ( theta < _alpha)
        return u2;

    // Next triangle to be unfolded runs along edge u0->u2
    int ui = u0;
    int uj = u2;
    // Unless the better search area is the triangle along the other edge
    const float v2Proj = (v2 - _initEdgePos).dot(_initUnitEdge);
    if ( v2Proj < _halfInitEdgeNorm)  // Projection of edge e1 along the original edge
    {
        ui = u2;
        uj = u1;
    }   // end if

    // If still recursing after 1000 triangles, something is seriously wrong!
    _recursionLim++;
    if ( _recursionLim >= 1000)
    {
        std::cerr << "[WARNING] r3d::FaceUnfoldingVertexSearcher::_searchForVertexInUnfoldingSection: Exceeded recursion limit!" << std::endl;
        return -1;
    }   // end if

    const int nextT = getOther( polyUnfolder->mesh().sfaces( ui, uj), T);
    // Ordering of vertices ensures direction vectors calculated correctly.
    return _searchForVertexInUnfoldingSection( polyUnfolder, ui, uj, nextT);
}   // end _searchForVertexInUnfoldingSection
