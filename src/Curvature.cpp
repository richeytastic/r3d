/************************************************************************
 * Copyright (C) 2020 Richard Palmer
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


// public static
Curvature::Ptr Curvature::create( const Mesh& m)
{
    return Ptr( new Curvature(m), [](Curvature* x){delete x;});
}   // end create


void Curvature::_updateFace( const r3d::Mesh &mesh, int fid, float mult)
{
    const int *fvidxs = mesh.fvidxs(fid);
    const int a = fvidxs[0];
    const int b = fvidxs[1];
    const int c = fvidxs[2];
    const Vec3f& vA( mesh.uvtx(a));
    const Vec3f& vB( mesh.uvtx(b));
    const Vec3f& vC( mesh.uvtx(c));

    const Vec3f fnrm = mult * (vB-vA).cross(vC-vB);    // Magnitude is twice area of triangle

    _vtxNormals.row(a) += fnrm;
    _vtxNormals.row(b) += fnrm;
    _vtxNormals.row(c) += fnrm;

    const float A = mult * fnrm.norm() / 2; // Triangle area

    _edgeFaceSums[mesh.edgeId(a, b)] += A;
    _edgeFaceSums[mesh.edgeId(b, c)] += A;
    _edgeFaceSums[mesh.edgeId(c, a)] += A;

    _vtxAdjFacesSum[a] += A;
    _vtxAdjFacesSum[b] += A;
    _vtxAdjFacesSum[c] += A;
}   // end _updateFace


Curvature::Curvature( const Mesh &mesh)
    : _mesh( &mesh),
      _vtxNormals( MatX3f::Zero( mesh.numVtxs(), 3)),
      _edgeFaceSums( VecXf::Zero( mesh.numEdges())),
      _vtxAdjFacesSum( VecXf::Zero( mesh.numVtxs())),
      _vtxCurvature( mesh.numVtxs(), 8)
{
    assert( mesh.hasSequentialIds());
    const int NV = int(mesh.numVtxs());
    const int NF = int(mesh.numFaces());

    for ( int fid = 0; fid < NF; ++fid)
        _updateFace( mesh, fid, 1.0f);

    // Normalise vertex norms
    for ( int i = 0; i < NV; ++i)
        _vtxNormals.row(i).normalize();

    // Set curvature
    for ( int i = 0; i < NV; ++i)
        _setVertexCurvature( mesh, i);
}   // end ctor


void Curvature::adjustRawVertex( Mesh &mesh, int vidx, const Vec3f &npos)
{
    const IntSet &fids = mesh.faces(vidx);     // Associated polys

    // Remove the old values from associated vertices and faces
    for ( int fid : fids)
        _updateFace( mesh, fid, -1.0f);

    // Update the vertex's raw (untransformed) position
    mesh.adjustRawVertex( vidx, npos);

    // Add back in new values for the associated vertices and faces
    for ( int fid : fids)
        _updateFace( mesh, fid, 1.0f);

    const IntSet &cvtxs = mesh.cvtxs(vidx);    // Connected vertices

    // Normalise vertex norms
    _vtxNormals.row(vidx).normalize();
    for ( int i : cvtxs)
        _vtxNormals.row(i).normalize();

    // Set curvature
    _setVertexCurvature( mesh, vidx);
    for ( int i : cvtxs)
        _setVertexCurvature( mesh, i);
}   // end adjustRawVertex


void Curvature::_setVertexCurvature( const Mesh &mesh, int vi)
{
    Mat3f M = Mat3f::Zero();
    const IntSet &cvtxs = mesh.cvtxs(vi);
    for ( int vj : cvtxs)
        _addEdgeCurvature( mesh, vi, vj, M);

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


void Curvature::_addEdgeCurvature( const Mesh &mesh, int vi, int vj, Mat3f &M) const
{
    const Vec3f &N = _vtxNormals.row(vi);   // Normal to the surface at vi
    const Vec3f &ui = mesh.uvtx(vi);
    const Vec3f &uj = mesh.uvtx(vj);
    Vec3f uji = uj - ui;

    Vec3f Tij = (Eigen::Matrix3f::Identity() - N*N.transpose()) * -uji; // Tij is the unit vector in the tangent plane to the surface at point vi
    Tij.normalize();

    uji.normalize();
    const float k = 2.0f * N.dot(uji); // Approximate the directional curvature 

    // Calc edge weight ensuring that sum of the edge weights for all vertices connected to vi == 1
    const float w = _edgeFaceSums[mesh.edgeId(vi,vj)] / _vtxAdjFacesSum[vi];

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

