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
