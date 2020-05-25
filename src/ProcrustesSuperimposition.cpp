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

#include <ProcrustesSuperimposition.h>
#include <Eigen/SVD>
#include <cassert>
using r3d::ProcrustesSuperimposition;
using r3d::VecXf;
using r3d::Mat4f;
using r3d::MatX3f;


ProcrustesSuperimposition::ProcrustesSuperimposition( const MatX3f& vrows, const VecXf& w, bool scaleUp)
    : _W(w), _wsum(w.sum()), _scaleUp(scaleUp)
{
    assert( vrows.rows() == w.rows());
    const MatX3f wvs = vrows.array().colwise() * w.array(); // N rows x 3 cols
    _vbar = wvs.colwise().sum() / _wsum;
    const MatX3f A = vrows.rowwise() - _vbar.transpose();
    _s = wvs.rowwise().norm().sum() / _wsum;
    _A = (A.array().colwise() * w.array()) / _s;    // Note weighing of A again here for covariance matrix
}   // end ctor


Mat4f ProcrustesSuperimposition::operator()( const MatX3f& inB) const
{
    const MatX3f wvs = inB.array().colwise() * _W.array();
    const Vec3f vbar = wvs.colwise().sum() / _wsum;
    MatX3f B = inB.rowwise() - vbar.transpose();
    const float sB = wvs.rowwise().norm().sum() / _wsum;
    B /= sB;

    const Mat3f C = _A.transpose() * B; // Covariance between B and A
    Eigen::JacobiSVD<Mat3f> svd( C, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Mat4f t0 = Mat4f::Identity();
    const float scaleFactor = _scaleUp ? _s/sB : 1.0f;
    t0.block<3,3>(0,0) = (svd.matrixV() * svd.matrixU().transpose()) * scaleFactor;   // 3x3 rotation matrix with scaling
    t0.block<3,1>(0,3) = vbar;
    Mat4f t1 = Mat4f::Identity();
    t1.block<3,1>(0,3) = -_vbar;
    return t0 * t1;
}   // end operator()
