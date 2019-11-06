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

#include <VertexCrossingTimeCalculator.h>
#include <FaceUnfoldingVertexSearcher.h>
#include <algorithm>    // std::swap
#include <cassert>
#include <iostream>
using r3d::VertexCrossingTimeCalculator;
using r3d::FaceUnfoldingVertexSearcher;
using r3d::FaceAngles;
using std::unordered_map;


struct VertexCrossingTimeCalculator::VCrossingTimes
{
    explicit VCrossingTimes( r3d::VecXf &tm) : _srcID(-1), _stimes(&tm), _mtimes(nullptr) {}   // end ctor


    VCrossingTimes( unordered_map<int, unordered_map<int, float> > &tm, int srcID)
        : _srcID(srcID), _stimes(nullptr), _mtimes(&tm)
    {
        assert( srcID >= 0);
    }   // end ctor


    float at( int i)
    {
        assert( i >= 0);
        if ( _stimes)
        {
            if ( i >= int(_stimes->size()))
                (*_stimes)[i] = FLT_MAX;
            return (*_stimes)[i];
        }   // end if

        if ( !_mtimes->count(i) || !_mtimes->at(i).count(_srcID))
            (*_mtimes)[i][_srcID] = FLT_MAX;
        return _mtimes->at(i).at(_srcID);
    }   // end at()

private:
    const int _srcID;    // For _mtimes only
    r3d::VecXf *_stimes; // Times from a single source per vertex
    unordered_map<int, unordered_map<int, float> > *_mtimes; // Times from multiple sources per vertex
};  // end struct



VertexCrossingTimeCalculator::VertexCrossingTimeCalculator( const FaceAngles &fa, r3d::VecXf &tm)
    : _angles(fa), _vtimes( new VertexCrossingTimeCalculator::VCrossingTimes( tm))
{}   // end ctor


VertexCrossingTimeCalculator::VertexCrossingTimeCalculator( const FaceAngles &fa,
                                                            unordered_map<int, unordered_map<int, float> > &tm,
                                                            int srcID)
    : _angles(fa), _vtimes( new VertexCrossingTimeCalculator::VCrossingTimes( tm, srcID))
{}   // end ctor


VertexCrossingTimeCalculator::~VertexCrossingTimeCalculator() { delete _vtimes;}



// public
float VertexCrossingTimeCalculator::operator()( int C, float F)
{
    const Mesh &mesh = _angles.mesh();
    static const float HALF_PI = float(CV_PI/2);
    float tC = _vtimes->at(C);

    assert( mesh.vtxIds().count(C) > 0);
    const Vec3f& vC = mesh.vtx(C);

    // Over each triangle that vertex C is a corner of, calculate the time at which the front arrives
    // based on the arrival time for the other two vertices of the triangle. If the angle is acute, the adjacent
    // triangles must be "unfolded" until a pseudo vertex can be found which gives an acute triangulation.
    const IntSet& fs = mesh.faces(C);
    for ( int fid : fs)
    {
        const float theta = _angles( fid, C); // Inner angle on face fid at C

        int A, B;
        mesh.face(fid).opposite( C, A, B); // For each triangle connected to C, get the two vertex IDs on the opposite edge.

        const float tA = _vtimes->at( A);
        const float tB = _vtimes->at( B);

        const Vec3f& vA = mesh.vtx(A);
        const Vec3f& vB = mesh.vtx(B);
        const Vec3f vBC = vB - vC;
        const Vec3f vAC = vA - vC;
        float a = vBC.norm();
        float b = vAC.norm();

        float t2v = tC;
        if ( tA  == FLT_MAX && tB < FLT_MAX)
            t2v = F*a + tB;
        else if ( tB == FLT_MAX && tA < FLT_MAX)
            t2v = F*b + tA;
        else if ( tA < FLT_MAX && tB < FLT_MAX)
        {
            t2v = std::min( F*a + tB, F*b + tA);
            if ( theta < HALF_PI)
                t2v = calcTimeAtC( tB, tA, a, b, theta, F);
            else
            {
                // "Unfold" adjacent triangles until a vertex is found that is within the planar section defined by triangle
                // fid that allows for a segmentation of triangle fid into two acute triangles. Then calculate the time to C
                // using these two psuedo triangles and use the lesser of the two times.
                Vec3f vP;
                int P = FaceUnfoldingVertexSearcher( mesh)( C, fid, theta, vP);
                if ( P >= 0)
                {
                    const Vec3f vPC = vP - vC;
                    const float p = (vP - vC).norm();
                    const float tP = _vtimes->at(P);
                    const float r0 = std::min( std::max( -1.0f, vPC.dot(vBC)), 1.0f);
                    const float r1 = std::min( std::max( -1.0f, vPC.dot(vAC)), 1.0f);
                    const float thetaBP = acosf( r0 / (p*a));
                    const float thetaAP = acosf( r1 / (p*b));
                    const float t2v0 = calcTimeAtC( tB, tP, a, p, thetaBP, F);
                    const float t2v1 = calcTimeAtC( tA, tP, b, p, thetaAP, F);
                    t2v = std::min( t2v0, t2v1);
                }   // end if
            }   // end else
        }   // end if

        tC = std::min( t2v, tC);
    }   // end for

    return tC;
}   // end operator()


// public static
// Calculate the time to vertex C. See breakdown on page 8433 of "Computing Geodesic Paths on Manifolds".
float VertexCrossingTimeCalculator::calcTimeAtC( float tB, float tA, float a, float b, float thetaAtC, float F)
{
    if ( tB < tA)
    {
        std::swap( tB, tA);
        std::swap( a, b);
    }   // end if

    // Comments are example values produced
    const float u = tB - tA;   // 0
    assert( u >= 0.0);
    const float cosTheta = cosf(thetaAtC);  // 0

    const float aSq = powf(a,2);    // 1
    const float bSq = powf(b,2);    // 1

    // Get the 3 quadratic terms
    const float qA = aSq + bSq - 2*a*b*cosTheta;   // 2
    const float qB = 2*b*u*(a*cosTheta - b);       // 0
    const float qC = bSq*(powf(u,2) - powf(F,2)*aSq*powf(sinf(thetaAtC),2));   // -1

    // Quadratic calc
    const float sRT = sqrtf( std::max(0.0f, powf(qB,2) - 4*qA*qC));   // 2*sqrt(2) = 2.828427
    const float t = (sRT - qB)/(2*qA); // sqrt(2)/2

    float CD = b*(t-u);
    if ( t != 0.0)
        CD /= t;

    float tC;
    if ( u < t && a*cosTheta < CD && CD*cosTheta < a)
        tC = tA + t;
    else
        tC = std::min( F*b + tA, F*a + tB);

    return tC;
}   // end calcTimeAtC
