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

#include <ProcrustesSuperimposition.h>
#include <Eigen/SVD>
#include <cassert>
using r3d::ProcrustesSuperimposition;
using r3d::VecXf;
using r3d::Vec3f;
using r3d::Mat4f;
using r3d::Mat4f;
using r3d::MatX3f;

namespace {

Vec3f calcWeightedMean( const MatX3f &vrows, const VecXf &w)
{
    const size_t N = vrows.rows();
    assert( N == size_t(w.size()));
    Vec3f vbar = Vec3f::Zero();
    for ( size_t i = 0; i < N; ++i)
        vbar += w[i] * vrows.row(i);
    return vbar / N;
}   // end calcWeightedMean


float centreAndScale( MatX3f &outRows, const MatX3f &vrows, const VecXf &w, const Vec3f &vbar)
{
    const size_t N = vrows.rows();
    assert( N == size_t(w.size()));
    outRows = MatX3f( N, 3);
    float s = 0.0f;
    for ( size_t i = 0; i < N; ++i)
    {
        outRows.row(i) = (Vec3f)vrows.row(i) - vbar; // Centre the distribution
        s += (w[i] * outRows.row(i)).squaredNorm(); // Weighted scaling
    }   // end for
    s = sqrtf( s/N);    // Scale factor
    outRows /= s;       // Normalise range
    return s;
}   // end centreAndScale

}   // end namespace


ProcrustesSuperimposition::ProcrustesSuperimposition( const MatX3f& vrows, const VecXf& vw, bool scaleUp)
    : _W(vw), _scaleUp(scaleUp)
{
    _vbar = calcWeightedMean( vrows, vw); // Calculate weighted mean (how much each point matters in the distribution)
    _s = centreAndScale( _A, vrows, vw, _vbar);
}   // end ctor


Mat4f ProcrustesSuperimposition::operator()( const MatX3f& inB) const
{
    const Vec3f vbar = calcWeightedMean( inB, _W);
    MatX3f B;
    const float sB = centreAndScale( B, inB, _W, vbar);

    // Weight the rows of B
    const size_t N = inB.rows();
    for ( size_t i = 0; i < N; ++i)
        B.row(i) *= _W[i];

    // Compute the covariance between B and A (not normalised?)
    const Mat3f C = B.transpose() * _A;

    Eigen::JacobiSVD<Mat3f> svd( C, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Mat4f t0 = Mat4f::Identity();
    const float scaleFactor = _scaleUp ? _s/sB : 1.0f;
    t0.block<3,3>(0,0) = (svd.matrixV().transpose() * svd.matrixU().transpose()) * scaleFactor;   // 3x3 rotation matrix with scaling
    t0.block<3,1>(0,3) = _vbar;

    Mat4f t1 = Mat4f::Identity();
    t1.block<3,1>(0,3) = -_vbar;

    return t0 * t1;
}   // end operator()
