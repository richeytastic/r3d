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

#include <FaceInterpolator.h>
#include <algorithm>
#include <iomanip>
#include <cassert>
using r3d::FaceInterpolator;
using r3d::Mesh;
using r3d::Vec3f;
using r3d::VecXi;


FaceInterpolator::FaceInterpolator( const Mesh &imod,
                                    const Mesh &omod,
                                    const VecXi &nsources,
                                    const std::unordered_map<int, std::unordered_map<int, float> >& itimes)
                                
    : _inmod(imod), _outmod(omod), _nearestSources(nsources), _itimes(itimes)
{
}   // end ctor


// static
float FaceInterpolator::calcLambda( float txa, float txb, float tya, float tyb)
{
    assert( tya >= txa);
    float lambda = 0.5f;
    const float divisor = txb - txa + tya - tyb;
    if ( divisor != 0.0f)
        lambda = (tya - txa)/divisor;
    return lambda;
}   // end calcLambda


// static
Vec3f FaceInterpolator::calcLambdaPoint( float lambda, const Vec3f& vA, const Vec3f& vB)
{
    return (1.0f - lambda)*vA + lambda*vB;
}   // end calcLambdaPoint


void FaceInterpolator::_printDebug( int A, int a) const
{
    const int x = _nearestSources[a];
    const Vec3f vA = _inmod.vtx(a);
    const Vec3f vX = _outmod.vtx(x);

    if ( A == a)
        std::cerr << " #";
    else
        std::cerr << "  ";
    std::cerr << " (" << a << ") near [" << x << "]" << std::endl;
    std::cerr << "     (" << std::fixed << std::setprecision(2) << std::setw(7)
              << vA[0] << ", " << vA[1] << ")"
              << " near [" << vX[0] << ", " << vX[1] << "]" << std::endl;
}   // end _printDebug


Vec3f FaceInterpolator::interpolate( int A) const
{
    const int X = _nearestSources[A];  // -1 before any sources
    if ( X < 0)
        return _inmod.vtx(A);

    const Vec3f &vA = _inmod.vtx(A);
    const float txa = _itimes.at(A).at(X);

    int icount = 4; // Found empirically to result in more equalateral triangles than 5,3,2, or 1.
    Vec3f mpos = icount*vA;

    const IntSet& cuvs = _inmod.cvtxs( A);
    for ( int B : cuvs)
    {
        const int Y = _nearestSources[B];
        if ( Y == X)
            continue;

        const Vec3f &vB = _inmod.vtx(B);
        const float tya = _itimes.at(A).at(Y);
        const float txb = _itimes.at(B).at(X);
        const float tyb = _itimes.at(B).at(Y);
        const float l0 = calcLambda( txa, txb, tya, tyb);  // lambda position between A and B
        mpos += calcLambdaPoint( l0, vA, vB);
        icount++;
    }   // end foreach

    return mpos / icount;
}   // end interpolate
