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

#include <PatchBendingEnergy.h>
#include <cassert>
#include <cmath>
using r3d::PatchBendingEnergy;
using r3d::Mesh;
using r3d::VecXf;
using r3d::MatXf;

namespace {
size_t setCoordinateVectors( const Mesh &mesh, const IntSet &vset, VecXf &x, VecXf &y, VecXf &z)
{
    const size_t m = vset.size();
    x.resize(m);
    y.resize(m);
    z.resize(m);
    int i = 0;
    for ( int vidx : vset)
    {
        const r3d::Vec3f &v = mesh.uvtx(vidx);
        x[i] = v[0];
        y[i] = v[1];
        z[i] = v[2];
        i++;
    }   // end for
    return m;
}   // end setCoordinateVectors


MatXf setOmega( const VecXf &x, const VecXf &y, const VecXf &z)
{
    const size_t m = x.size();
    assert( m == size_t(y.size()));
    assert( m == size_t(z.size()));

    MatXf outMat( m+4, m+4);

    // Create matrix K
    for ( size_t i = 0; i < m; ++i)
    {
        for ( size_t j = 0; j < m; ++j)
        {
            const float sqDiff = powf(x[i]-x[j],2) + powf(y[i]-y[j],2) + powf(z[i]-z[j],2);
            outMat(i,j) = sqDiff > 0.0f ? sqDiff * logf( sqDiff) : 0.0f;
        }   // end for
    }   // end for

    outMat.block(0,m,m,1) = VecXf::Ones(m);
    outMat.block(0,m,m+1,1) = x;
    outMat.block(0,m,m+2,1) = y;
    outMat.block(0,m,m+3,1) = z;
    outMat.block(m,0,4,m) = outMat.block(0,m,m,4).transpose();  // Okay to do this since different slices of memory
    outMat.block(m,m,4,4) = r3d::Mat4f::Zero();

    return outMat.inverse().block(0,0,m,m);    // m * m upper left
}   // end setOmega

}   // end namespace


PatchBendingEnergy::PatchBendingEnergy( const Mesh &m, const Mesh &n) : _m(m), _n(n) {}


float PatchBendingEnergy::operator()( const IntSet& p, const IntSet& q) const
{
    VecXf qx, qy, qz;
    const int m = setCoordinateVectors( _n, q, qx, qy, qz);
    assert( m == (int)p.size());
    if ( m != (int)p.size())
        return -1;

    VecXf px, py, pz;
    setCoordinateVectors( _m, p, px, py, pz);

    const MatXf omega = setOmega( px, py, pz);
    return ((qx.transpose() * omega * qx) + (qy.transpose() * omega * qy) + (qz.transpose() * omega * qz))(0,0);
}   // end operator()


// static
float PatchBendingEnergy::calc( const MatX3f& p, const MatX3f& q)
{
    assert( p.rows() == q.rows());

    const VecXf qx = q.col(0);
    const VecXf qy = q.col(1);
    const VecXf qz = q.col(2);

    const VecXf px = p.col(0);
    const VecXf py = p.col(1);
    const VecXf pz = p.col(2);

    const MatXf omega = setOmega( px, py, pz);
    return ((qx.transpose() * omega * qx) + (qy.transpose() * omega * qy) + (qz.transpose() * omega * qz))(0,0);
}   // end calc

