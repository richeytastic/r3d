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

#include <Curvature.h>
#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>
using r3d::Curvature;
using r3d::Mesh;
using r3d::Vec3f;
using r3d::Mat3f;


Vec3f Curvature::vertexPC1( int vi, float& kp1) const
{
    kp1 = _vtxCurvature(vi,3);
    return _vtxCurvature.block<1,3>(vi, 0);
}   // end vertexPC1


Vec3f Curvature::vertexPC2( int vi, float& kp2) const
{
    kp2 = _vtxCurvature(vi,7);
    return _vtxCurvature.block<1,3>(vi, 3);
}   // end vertexPC2


// static
Vec3f Curvature::calcVertexNormal( const Mesh& m, int vidx)
{
    Vec3f nrm = Vec3f::Zero();
    const IntSet& fids = m.faces(vidx);
    for ( int fid : fids)
    {
        const int *fvidxs = m.fvidxs(fid);
        const Vec3f& vA( m.uvtx(fvidxs[0]));
        const Vec3f& vB( m.uvtx(fvidxs[1]));
        const Vec3f& vC( m.uvtx(fvidxs[2]));
        nrm += (vB - vA).cross(vC - vB);  // Magnitude of cross product vector is twice triangle area
    }   // end for
    nrm.normalize();
    return nrm;
}   // end calcVertexNormal


// public static
Curvature::Ptr Curvature::create( Mesh& m)
{
    return Ptr( new Curvature(m), [](Curvature* x){delete x;});
}   // end create


void Curvature::_updateFace( int fid, float mult)
{
    const int *fvidxs = _mesh.fvidxs(fid);
    const int a = fvidxs[0];
    const int b = fvidxs[1];
    const int c = fvidxs[2];
    const Vec3f& vA( _mesh.uvtx(a));
    const Vec3f& vB( _mesh.uvtx(b));
    const Vec3f& vC( _mesh.uvtx(c));

    const Vec3f fnrm = mult * (vB-vA).cross(vC-vB);    // Magnitude is twice area of triangle

    _vtxNormals.row(a) += fnrm;
    _vtxNormals.row(b) += fnrm;
    _vtxNormals.row(c) += fnrm;

    const float A = mult * fnrm.norm() / 2; // Triangle area

    _edgeFaceSums[_mesh.edgeId(a, b)] += A;
    _edgeFaceSums[_mesh.edgeId(b, c)] += A;
    _edgeFaceSums[_mesh.edgeId(c, a)] += A;

    _vtxAdjFacesSum[a] += A;
    _vtxAdjFacesSum[b] += A;
    _vtxAdjFacesSum[c] += A;
}   // end _updateFace


Curvature::Curvature( Mesh &m)
    : _mesh(m),
      _vtxNormals( MatX3f::Zero( m.numVtxs(), 3)),
      _edgeFaceSums( VecXf::Zero( m.numEdges())),
      _vtxAdjFacesSum( VecXf::Zero( m.numVtxs())),
      _vtxCurvature( m.numVtxs(), 8)
{
    assert( m.hasSequentialIds());
    const size_t NV = m.numVtxs();
    const size_t NF = m.numFaces();

    for ( size_t fid = 0; fid < NF; ++fid)
        _updateFace( fid, 1.0f);

    // Normalise vertex norms
    for ( size_t i = 0; i < NV; ++i)
        _vtxNormals.row(i).normalize();

    // Set curvature
    for ( size_t i = 0; i < NV; ++i)
        _setVertexCurvature( i);
}   // end ctor


void Curvature::adjustVertex( int vidx, const Vec3f &npos)
{
    const IntSet &fids = _mesh.faces(vidx);     // Associated polys

    // Remove the old values from associated vertices and faces
    for ( int fid : fids)
        _updateFace( fid, -1.0f);

    // Update the vertex's position
    _mesh.adjustVertex( vidx, npos[0], npos[1], npos[2]);

    // Add back in new values for the associated vertices and faces
    for ( int fid : fids)
        _updateFace( fid, 1.0f);

    const IntSet &cvtxs = _mesh.cvtxs(vidx);    // Connected vertices

    // Normalise vertex norms
    _vtxNormals.row(vidx).normalize();
    for ( int i : cvtxs)
        _vtxNormals.row(i).normalize();

    // Set curvature
    _setVertexCurvature( vidx);
    for ( int i : cvtxs)
        _setVertexCurvature( i);
}   // end adjustVertex


void Curvature::_setVertexCurvature( int vi)
{
    Mat3f M = Mat3f::Zero();
    const IntSet &cvtxs = _mesh.cvtxs(vi);
    for ( int vj : cvtxs)
        _addEdgeCurvature( vi, vj, M);

    const Vec3f &N = _vtxNormals.row(vi);
    Vec3f W(1,0,0);
    if ( (W - N).norm() > (W + N).norm())
        W -= N;
    else
        W += N;
    W.normalize();

    const Mat3f Q = Eigen::Matrix3f::Identity() - 2 * W * W.transpose();  // Householder matrix
    // First column of Q is N (or -N depending on above +/- conditional).
    // The second and thirds columns of Q define an orthonormal basis of the tangent space
    // (but not necessarily the principal directions).
    const Vec3f Ti = Q.col(1);
    const Vec3f Tj = Q.col(2);

    const Mat3f H = Q.transpose() * M * Q;
    const float a = H(1,1);  // Eigenvalue of principal curvature
    const float b = H(2,2);  // Eigenvalue of principal curvature
    float c, s;
    calcGivensRotation( a, b, c, s);

    _vtxCurvature.block<1,3>(vi,0) = c*Ti - s*Tj;
    _vtxCurvature(vi,3) = 3*a - b;  // kp1
    _vtxCurvature.block<1,3>(vi,4) = s*Ti + c*Tj;
    _vtxCurvature(vi,7) = 3*b - a;  // kp2
}   // end _setVertexCurvature


void Curvature::_addEdgeCurvature( int vi, int vj, Mat3f &M) const
{
    const Vec3f &N = _vtxNormals.row(vi);   // Normal to the surface at vi
    const Vec3f &ui = _mesh.uvtx(vi);
    const Vec3f &uj = _mesh.uvtx(vj);
    Vec3f uji = uj - ui;

    Vec3f Tij = (Eigen::Matrix3f::Identity() - N*N.transpose()) * -uji; // Tij is the unit vector in the tangent plane to the surface at point vi
    Tij.normalize();

    uji.normalize();
    const float k = 2.0f * N.dot(uji); // Approximate the directional curvature 

    // Calc edge weight ensuring that sum of the edge weights for all vertices connected to vi == 1
    const float w = _edgeFaceSums[_mesh.edgeId(vi,vj)] / _vtxAdjFacesSum[vi];

    M += w * k * Tij * Tij.transpose(); // Add the weighted curvature matrix
}   // end _addEdgeCurvature


// public static
float Curvature::calcGivensRotation( float a, float b, float &c, float &s)
{
    float r;
    //if ( fabs(b) < DBL_MIN)
    if ( b == 0.0)
    {
        c = copysign(1.0f, a);
        s = 0.0f;
        r = fabsf(a);
    }   // end if
    //else if ( fabs(a) < DBL_MIN)
    else if ( a == 0.0f)
    {
        c = 0.0f;
        s = -copysign(1.0f, b);
        r = fabsf(b);
    }   // end else if
    else if ( fabsf(a) > fabsf(b))
    {
        float t = b/a;
        float u = copysign(1.0f, a) * fabsf(sqrtf(1.0f + t*t));
        c = 1.0f/u;
        s = -c * t;
        r = a * u;
    }   // end else if
    else
    {
        float t = a/b;
        float u = copysign(1.0f, b) * fabsf(sqrtf(1.0f + t*t));
        s = -1.0f/u;
        c = -s * t;
        r = b * u;
    }   // end else
    return r;
}   // end calcGivensRotation

