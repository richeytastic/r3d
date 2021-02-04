/************************************************************************
 * Copyright (C) 2021 Richard Palmer
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
using r3d::Mat4f;
using r3d::MatX3f;


ProcrustesSuperimposition::ProcrustesSuperimposition( const MatX3f& inA, const VecXf &w, bool s2t)
    : _W(&w), _scale2Tgt(s2t)
{
    _init(inA);
}   // end ctor


ProcrustesSuperimposition::ProcrustesSuperimposition( const MatX3f& inA, bool s2t)
    : _autoW( VecXf::Ones( inA.rows())), _W(nullptr), _scale2Tgt(s2t)
{
    _W = &_autoW;
    _init(inA);
}   // end ctor


void ProcrustesSuperimposition::_init( const MatX3f &inA)
{
    const VecXf &W = *_W;
    _wsum = W.sum();
    assert( inA.rows() == W.rows());    // Number of vertices must match number of vertex weights
    MatX3f wA = inA.array().colwise() * W.array(); // Weight the vertices and use to calculate ...
    _vbar = wA.colwise().sum() / _wsum; // ... the weighted mean position which is used to obtain ...
    const MatX3f A = inA.rowwise() - _vbar.transpose(); // ... the mean centred input vertices.
    wA = A.array().colwise() * W.array();   // Calc the weighted mean centred vectors before obtaining ...
    _s = wA.rowwise().norm().sum() / _wsum; // ... the scale as their mean magnitude.
    _A = wA / _s;  // Finally, normalise the weighted vectors.
}   // end _init


Mat4f ProcrustesSuperimposition::operator()( const MatX3f& inB) const
{
    const VecXf &W = *_W;
    MatX3f wB = inB.array().colwise() * W.array();   // Weight the input vertices
    const Vec3f vbar = wB.colwise().sum() / _wsum;  // The weighted mean position
    MatX3f B = inB.rowwise() - vbar.transpose();    // The mean centred input vertices
    wB = B.array().colwise() * W.array();
    const float sB = wB.rowwise().norm().sum() / _wsum;
    B = wB / sB;

    const Mat3f C = _A.transpose() * B; // Covariance between B and A
    Eigen::JacobiSVD<Mat3f> svd( C, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Mat4f t0 = Mat4f::Identity();
    const float sf = _scale2Tgt ? _s/sB : 1.0f; // Scale factor
    t0.block<3,3>(0,0) = (svd.matrixV() * svd.matrixU().transpose()) * sf;   // 3x3 rotation with scaling
    t0.block<3,1>(0,3) = vbar;
    Mat4f t1 = Mat4f::Identity();
    t1.block<3,1>(0,3) = -_vbar;
    return t0 * t1;
}   // end operator()
