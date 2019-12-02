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

#include <Bounds.h>
using r3d::Bounds;
using r3d::Mesh;
using r3d::Face;
using r3d::Vec3f;
using r3d::Vec6f;
using r3d::Mat4f;

Bounds::Ptr Bounds::create( const Vec3f& minc, const Vec3f& maxc, const Mat4f& m)
{
    return Ptr( new Bounds( minc, maxc, m));
}   // end create


Bounds::Ptr Bounds::create( const Mesh& mesh, const Mat4f& m, const IntSet* vset)
{
    return Ptr( new Bounds( mesh, m, vset));
}   // end create


Bounds::Ptr Bounds::create( const Mesh& mesh, const Mat4f& m, const IntSet& pset)
{
    return Ptr( new Bounds( mesh, m, pset));
}   // end create


void Bounds::_init( const Mesh& mesh, const Mat4f& m, const IntSet& vidxs)
{
    _minc[0] = _minc[1] = _minc[2] = FLT_MAX;
    _maxc[0] = _maxc[1] = _maxc[2] = -FLT_MAX;

    _tmat = m;
    _imat = m.inverse();

    for ( int vidx : vidxs)
    {
        const Vec3f v = transform( _imat, mesh.vtx(vidx));

        const float x = v[0];
        if ( x < _minc[0])
            _minc[0] = x;
        if ( x > _maxc[0])
            _maxc[0] = x;

        const float y = v[1];
        if ( y < _minc[1])
            _minc[1] = y;
        if ( y > _maxc[1])
            _maxc[1] = y;

        const float z = v[2];
        if ( z < _minc[2])
            _minc[2] = z;
        if ( z > _maxc[2])
            _maxc[2] = z;
    }   // end for

    _calcExtents();
}   // end _init


void Bounds::_calcExtents()
{
    // Get the unit vectors along the edges of the cuboid in three directions
    Vec3f ux = _tmat.block<3,1>(0,0);
    Vec3f uy = _tmat.block<3,1>(0,1);
    Vec3f uz = _tmat.block<3,1>(0,2);
    ux.normalize();
    uy.normalize();
    uz.normalize();

    const float dx = xlen();
    const float dy = ylen();
    const float dz = zlen();

    const Vec3f minc = minCorner(); // Transformed
    const Vec3f maxc = maxCorner(); // Transformed

    // Get the other possible corners in the transformed space:
    const Vec3f c0 = minc + ux*dx;
    const Vec3f c1 = minc + uy*dy;
    const Vec3f c2 = minc + uz*dz;
    const Vec3f c3 = maxc - ux*dx;
    const Vec3f c4 = maxc - uy*dy;
    const Vec3f c5 = maxc - uz*dz;

    _mine = Vec3f( std::min(minc[0], std::min(c0[0], std::min(c1[0], std::min(c2[0], std::min(c3[0], std::min(c4[0], std::min(c5[0], maxc[0]))))))),
                   std::min(minc[1], std::min(c0[1], std::min(c1[1], std::min(c2[1], std::min(c3[1], std::min(c4[1], std::min(c5[1], maxc[1]))))))),
                   std::min(minc[2], std::min(c0[2], std::min(c1[2], std::min(c2[2], std::min(c3[2], std::min(c4[2], std::min(c5[2], maxc[2]))))))));

    _maxe = Vec3f( std::max(minc[0], std::max(c0[0], std::max(c1[0], std::max(c2[0], std::max(c3[0], std::max(c4[0], std::max(c5[0], maxc[0]))))))),
                   std::max(minc[1], std::max(c0[1], std::max(c1[1], std::max(c2[1], std::max(c3[1], std::max(c4[1], std::max(c5[1], maxc[1]))))))),
                   std::max(minc[2], std::max(c0[2], std::max(c1[2], std::max(c2[2], std::max(c3[2], std::max(c4[2], std::max(c5[2], maxc[2]))))))));
}   // end _calcExtents


Bounds::Bounds( const Vec3f& minc, const Vec3f& maxc, const Mat4f& m)
{
    _tmat = m;
    _imat = m.inverse();
    _minc = transform( _imat, minc);
    _maxc = transform( _imat, maxc);
    _calcExtents();
}   // end ctor


Bounds::Bounds( const Mesh& mesh, const Mat4f& m, const IntSet* vset)
{
    if ( !vset)
        vset = &mesh.vtxIds();
    _init( mesh, m, *vset);
}   // end ctor


Bounds::Bounds( const Mesh& mesh, const Mat4f& m, const IntSet& pset)
{
    IntSet vset;
    for ( int fid : pset)
    {
        const Face& p = mesh.face(fid);
        vset.insert(p[0]);
        vset.insert(p[1]);
        vset.insert(p[2]);
    }   // end for
    _init( mesh, m, vset);
}   // end ctor


Bounds::Ptr Bounds::deepCopy() const
{
    return Ptr( new Bounds(*this));
}   // end deepCopy


void Bounds::setTransformMatrix( const Mat4f& tmat)
{
    _tmat = tmat;
    _imat = tmat.inverse();
    _calcExtents();
}   // end setTransformMatrix


void Bounds::addTransformMatrix( const Mat4f& tmat)
{
    setTransformMatrix( tmat * _tmat);
}   // end addTransformMatrix


void Bounds::fixTransformMatrix()
{
    // The extents will be the same as the new corners given that we're resetting to the identity matrix.
    _mine = _minc = minCorner();
    _maxe = _maxc = maxCorner();
    _tmat = _imat = Mat4f::Identity();
}   // end fixTransformMatrix


void Bounds::encompass( const Bounds& ob)
{
    Bounds tob = ob;
    tob.addTransformMatrix( _imat);
    const Vec3f& min1c = tob.minExtent();
    const Vec3f& max1c = tob.maxExtent();

    // Update
    _minc[0] = std::min( _minc[0], min1c[0]);
    _minc[1] = std::min( _minc[1], min1c[1]);
    _minc[2] = std::min( _minc[2], min1c[2]);

    _maxc[0] = std::max( _maxc[0], max1c[0]);
    _maxc[1] = std::max( _maxc[1], max1c[1]);
    _maxc[2] = std::max( _maxc[2], max1c[2]);

    _calcExtents();
}   // end encompass


bool Bounds::intersects( const Bounds& ob) const
{
    // Transform the parameter bounds into standard position with respect to these bounds.
    Bounds tob = ob;
    tob.addTransformMatrix( _imat);
    const Vec3f& min1c = tob.minExtent();
    const Vec3f& max1c = tob.maxExtent();

    Vec6f a;
    a << _minc[0], _maxc[0], _minc[1], _maxc[1], _minc[2], _maxc[2];
    Vec6f b;
    b << min1c[0], max1c[0], min1c[1], max1c[1], min1c[2], max1c[2];

    const bool xAinB = ((a[0] >= b[0] && a[0] <= b[1]) || (a[1] >= b[0] && a[1] <= b[1]));    // x edges of A in B
    const bool xBinA = ((b[0] >= a[0] && b[0] <= a[1]) || (b[1] >= a[0] && b[1] <= a[1]));    // x edges of B in A
    const bool yAinB = ((a[2] >= b[2] && a[2] <= b[3]) || (a[3] >= b[2] && a[3] <= b[3]));    // y edges of A in B
    const bool yBinA = ((b[2] >= a[2] && b[2] <= a[3]) || (b[3] >= a[2] && b[3] <= a[3]));    // y edges of B in A
    const bool zAinB = ((a[4] >= b[4] && a[4] <= b[5]) || (a[5] >= b[4] && a[5] <= b[5]));    // y edges of A in B
    const bool zBinA = ((b[4] >= a[4] && b[4] <= a[5]) || (b[5] >= a[4] && b[5] <= a[5]));    // y edges of B in A

    const bool xint = xAinB || xBinA;
    const bool yint = yAinB || yBinA;
    const bool zint = zAinB || zBinA;

    return xint && yint && zint;
}   // end intersects


Vec3f Bounds::minCorner() const { return transform( _tmat, _minc);}
Vec3f Bounds::maxCorner() const { return transform( _tmat, _maxc);}
Vec3f Bounds::centre() const
{
    const Vec3f c = 0.5f * (_minc + _maxc);
    return transform( _tmat, c);
}   // end centre


float Bounds::xlen() const
{
    // Get sX as the scale factor along the x axis.
    const float sX = _tmat.block<3,1>(0,0).norm();
    return sX * fabsf(_minc[0] - _maxc[0]);
}   // end xlen


float Bounds::ylen() const
{
    // Get sY as the scale factor along the y axis.
    const float sY = _tmat.block<3,1>(0,1).norm();
    return sY * fabsf(_minc[1] - _maxc[1]);
}   // end ylen


float Bounds::zlen() const
{
    // Get sZ as the scale factor along the z axis.
    const float sZ = _tmat.block<3,1>(0,2).norm();
    return sZ * fabsf(_minc[2] - _maxc[2]);
}   // end zlen


float Bounds::width() const { return _maxe[0] - _mine[0];}
float Bounds::height() const { return _maxe[1] - _mine[1];}
float Bounds::depth() const { return _maxe[2] - _mine[2];}


float Bounds::diagonal() const { return (maxCorner() - minCorner()).norm();}


// Note that the raw non-transformed vertices are being used here!
Vec6f Bounds::cornersAs6f() const { return as6f(_minc, _maxc);}

Vec6f Bounds::extentsAs6f() const { return as6f(_mine, _maxe);}


// static
Vec6f Bounds::as6f( const Vec3f& a, const Vec3f& b)
{
    Vec6f rv;

    if ( a[0] < b[0])
    {
        rv[0] = a[0];
        rv[1] = b[0];
    }   // end if
    else
    {
        rv[0] = b[0];
        rv[1] = a[0];
    }   // end else

    if ( a[1] < b[1])
    {
        rv[2] = a[1];
        rv[3] = b[1];
    }   // end if
    else
    {
        rv[2] = b[1];
        rv[3] = a[1];
    }   // end else

    if ( a[2] < b[2])
    {
        rv[4] = a[2];
        rv[5] = b[2];
    }   // end if
    else
    {
        rv[4] = b[2];
        rv[5] = a[2];
    }   // end else

    return rv;
}   // end as6f
