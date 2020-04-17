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

#include <FaceUnfolder.h>
#include <cassert>
using r3d::FaceUnfolder;
using r3d::Mesh;
using r3d::Face;
using r3d::Vec3f;


FaceUnfolder::FaceUnfolder( const Mesh &mesh) : _mesh(mesh), _pnorm(Vec3f::Zero()) {}


FaceUnfolder::FaceUnfolder( const Mesh &mesh, int T) : _mesh(mesh), _pnorm(Vec3f::Zero())
{
    reset(T);
}   // end ctor


FaceUnfolder::FaceUnfolder( const FaceUnfolder& unf, int T)
    : _mesh(unf.mesh()), _pnorm(unf.norm())
{
    const int* vidxs = _mesh.fvidxs(T);
    if ( vidxs)
    {
        const int r = vidxs[0];
        const int a = vidxs[1];
        const int b = vidxs[2];
        _uvtxs[r] = unf.uvtx(r);
        _uvtxs[a] = unf.uvtx(a);
        _uvtxs[b] = unf.uvtx(b);
        _upolys.insert(T);
    }   // end if
}   // end ctor


Vec3f FaceUnfolder::calcUnfoldedPoint( int T, const Vec3f& p) const
{
    assert( isFaceUnfolded(T));
    const Vec3f pp = _mesh.toBarycentric( T, p);
    const int* vidxs = _mesh.fvidxs(T);
    const Vec3f& v0 = uvtx( vidxs[0]);
    const Vec3f& v1 = uvtx( vidxs[1]);
    const Vec3f& v2 = uvtx( vidxs[2]);

    const Vec3f vi = v1 - v0;
    const Vec3f vj = v2 - v1;
    Vec3f z = vi.cross(vj);
    z.normalize();

    // The triangle area used is the ORIGINAL area of the triangle (because unfolding can warp the area)
    const float A = _mesh.calcFaceArea(T);

    return v0 + pp[0]*(v1-v0) + pp[1]*(v2-v1) + pp[2]*sqrtf(2*A)*z;
}   // end calcUnfoldedPoint


void FaceUnfolder::reset( int T)
{
    _uvtxs.clear();
    _upolys.clear();
    _pnorm = Vec3f::Zero();
    if ( _mesh.faces().count(T) == 0)
        return;

    const int* vidxs = _mesh.fvidxs(T);
    assert(vidxs);
    const int r = vidxs[0];
    const int a = vidxs[1];
    const int b = vidxs[2];
    _uvtxs[r] = _mesh.vtx(r);
    _uvtxs[a] = _mesh.vtx(a);
    _uvtxs[b] = _mesh.vtx(b);
    _upolys.insert(T);
    _pnorm = _mesh.calcFaceNorm(T);
}   // end reset


int FaceUnfolder::unfold( int T, int r, int a)
{
    if ( _mesh.faces().count(T) == 0)
        return -1;

    const Face& face = _mesh.face( T);
    const int b = face.opposite( r, a);

    if ( _uvtxs.count(r) == 0 || _uvtxs.count(a) == 0)
    {
        reset( T);
        return b;
    }   // end if

    face.opposite( b, r, a);    // Ensure correct order of a,r
    const Vec3f& va = _uvtxs.at(a);
    const Vec3f& vr = _uvtxs.at(r);
    Vec3f u = va - vr;    // Unit vector unfolding edge
    u.normalize();
    Vec3f v = _pnorm.cross(u);
    v.normalize();  // Unit vector orthogonal to plane and folding edge (sign to be determined)

    const Vec3f br = _mesh.vtx(b) - _mesh.vtx(r);
    Vec3f ou = _mesh.vtx(a) - _mesh.vtx(r);
    ou.normalize();
    const float m = br.dot(ou);           // Amount along u
    const float n = (br - m*ou).norm();  // Amount along v

    _uvtxs[b] = vr + m*u + n*v;
    _upolys.insert(T);

    return b;
}   // end unfold


void FaceUnfolder::unfoldPath( const std::vector<int>& spvids, int T, int fT)
{
    assert( spvids.size() >= 1);
    assert( _mesh.faces(spvids[0]).count( T) > 0);
    assert( _mesh.faces(*spvids.rbegin()).count( fT) > 0);
    reset();

    std::vector<int>::const_iterator it = spvids.begin();
    // a,b are consecutive point pair vertex indices in spvids.
    int a = *it++;
    int b = it != spvids.end() ? *it : -1;
    int x; // Don't care which edge of T to start on
    _mesh.face(T).opposite( a, x, x);

    while ( true)
    {
        x = unfold(T, a, x);
        if ( T == fT)
            break;

        int nT = _mesh.oppositeFace( T, a, x);
        if ( nT >= 0)   // No need to do anything else if nT < 0: will go back the other way!
            T = nT;

        if ( x == b)    // Concertina around the other way!
        {
            x = a;
            a = *it++;
            b = it != spvids.end() ? *it : -1;
        }   // end if
    }   // end while
}   // unfoldPath
