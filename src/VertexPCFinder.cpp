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

#include <VertexPCFinder.h>
using r3d::VertexPCFinder;
using r3d::MatX3f;
using r3d::Mat3f;


namespace {
Mat3f createCovariance( const MatX3f& P)
{
    const size_t N = P.rows();
    Mat3f C = Mat3f::Zero();
    for ( size_t i = 0; i < N; ++i)
        C += P.row(i).transpose() * P.row(i);
    return C / N;
}   // end createCovariance
}   // end namespace


VertexPCFinder::VertexPCFinder( const MatX3f &m)
{
    const size_t N = m.rows();
    const Vec3f mean = m.colwise().sum() / N;   // Mean position
    MatX3f P( N, 3);
    for ( size_t i = 0; i < N; ++i)
        P.row(i) = (Vec3f)m.row(i) - mean;  // Transform to origin
    const Mat3f C = createCovariance( P);
    Eigen::EigenSolver<Mat3f> eig(C, true);
    _evs = eig.eigenvectors().real();
}   // end ctor


Mat3f VertexPCFinder::eigenVectors2RotationMatrix( const Mat3f &evs)
{
    // Find which column for which vector and ensure positive unit vectors
    // congruent with desired i,j,k unit directions in 3 space.

    // First find the column vector with the largest absolute X coefficient:
    int xi = 0;
    if ( fabsf(evs(0,1)) > fabsf(evs(0,0)))
        xi = 1;
    if ( fabsf(evs(0,2)) > fabsf(evs(0,xi)))
        xi = 2;
    Vec3f xvec = evs.col(xi);

    // Then get the Y and Z vectors - swapping them if the absolute value of
    // the Z coefficient in yvec is larger than that of zvec:
    Vec3f yvec = evs.col((xi+1)%3);
    Vec3f zvec = evs.col((xi+2)%3);
    if ( fabsf(yvec[2]) > fabsf(zvec[2]))
        std::swap( yvec, zvec);

    // Now ensure the vector directions are positive in the canonical directions.
    if ( xvec[0] < 0)
        xvec *= -1;
    if ( yvec[1] < 0)
        yvec *= -1;
    if ( zvec[2] < 0)
        zvec *= -1;

    Mat3f T;
    T.block<3,1>(0,0) = xvec;
    T.block<3,1>(0,1) = yvec;
    T.block<3,1>(0,2) = zvec;
    return T;
}   // end eigenVectors2RotationMatrix
