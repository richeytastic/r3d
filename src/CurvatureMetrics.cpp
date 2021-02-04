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

#include <CurvatureMetrics.h>
#include <functional>
#include <cassert>
using r3d::CurvatureMetrics;
using r3d::Curvature;
using r3d::Mesh;

namespace {

float vertexValue( const Mesh& mesh, int vid, const std::function<float(int)> &fn)
{
    float v = 0;
    const IntSet& fids = mesh.faces(vid);
    for ( int fid : fids)
        v += fn(fid);
    return v / fids.size();
}   // end vertexValue

}   // end namespace


CurvatureMetrics::CurvatureMetrics( const Curvature &cmap) : _cmap(cmap) {}


float CurvatureMetrics::faceDeterminant( int fid) const
{
    const int *fvidxs = _cmap.mesh().fvidxs( fid);
    assert( fvidxs);
    const Vec3f n0 = vertexNormal( fvidxs[0]);
    const Vec3f n1 = vertexNormal( fvidxs[1]);
    const Vec3f n2 = vertexNormal( fvidxs[2]);
    return n2.dot(n0.cross(n1));   // Calculate determinant as the scalar triple product
}   // end faceDeterminant


float CurvatureMetrics::vertexDeterminant( int vid) const
{
    return vertexValue( _cmap.mesh(), vid, [this](int fid){ return faceDeterminant(fid);});
}   // end vertexDeterminant


r3d::Vec3f CurvatureMetrics::vertexNormal( int vid) const { return _cmap.vertexNormal(vid);}


float CurvatureMetrics::faceKP1FirstOrder( int fid) const
{
    const int *fvidxs = _cmap.mesh().fvidxs( fid);
    assert( fvidxs);
    float ka, kb, kc;
    _cmap.vertexPC1( fvidxs[0], ka);
    _cmap.vertexPC1( fvidxs[1], kb);
    _cmap.vertexPC1( fvidxs[2], kc);
    // Face curvature is the average of the curvature at the 3 corner vertices. Note that these
    // curvatures have already been calculated using weights corresponding to the relative area
    // of this polygon with the sum of the area of the polygons connected to each of the vertices.
    return (ka + kb + kc)/3;
}   // end faceKP1FirstOrder


float CurvatureMetrics::faceKP2FirstOrder( int fid) const
{
    const int *fvidxs = _cmap.mesh().fvidxs( fid);
    assert( fvidxs);
    float ka, kb, kc;
    _cmap.vertexPC2( fvidxs[0], ka);
    _cmap.vertexPC2( fvidxs[1], kb);
    _cmap.vertexPC2( fvidxs[2], kc);
    // Face curvature is the average of the curvature at the 3 corner vertices. Note that these
    // curvatures have already been calculated using weights corresponding to the relative area
    // of this polygon with the sum of the area of the polygons connected to each of the vertices.
    return (ka + kb + kc)/3;
}   // end faceKP2FirstOrder


float CurvatureMetrics::vertexKP1FirstOrder( int vid) const
{
    float ka;
    _cmap.vertexPC1( vid, ka);
    return ka;
    //return vertexValue( _cmap.mesh(), vid, [this](int fid){ return faceKP1FirstOrder(fid);});
}   // end vertexKP1FirstOrder


float CurvatureMetrics::vertexKP2FirstOrder( int vid) const
{
    //return vertexValue( _cmap.mesh(), vid, [this](int fid){ return faceKP2FirstOrder(fid);});
    float ka;
    _cmap.vertexPC2( vid, ka);
    return ka;
}   // end vertexKP2FirstOrder
