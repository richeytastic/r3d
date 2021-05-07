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

#include <SurfacePointFinder.h>
using r3d::SurfacePointFinder;
using r3d::Mesh;
using r3d::Vec3f;

namespace {

struct FaceFinder
{
    FaceFinder( const Mesh& mesh, const Vec3f& t) : _mesh(mesh), _t(t), _minsd(FLT_MAX), _bv(t), _bfid(-1)
    {
        _sfids.insert(-1);  // Because oppositeFace returns -1 if no opposite poly found.
    }   // end ctor

    float find( int fid)
    {
        _sfids.insert( fid);   // Don't check this face again
        const Vec3f u = _mesh.nearestPositionWithinFace( fid, _t);    // Project t into polygon fid
        const float sd = (u - _t).squaredNorm(); // Repositioned difference

        if ( sd <= _minsd)    // At least as close to t on repositioning
        {
            _minsd = sd;
            _bv = u;
            _bfid = fid;

            const int *vidxs = _mesh.fvidxs(fid); // Check adjacent polygons

            const int f0 = _mesh.oppositeFace( fid, vidxs[0], vidxs[1]);
            if ( _sfids.count(f0) == 0)
                find( f0);

            const int f1 = _mesh.oppositeFace( fid, vidxs[1], vidxs[2]);
            if ( _sfids.count(f1) == 0)
                find( f1);

            const int f2 = _mesh.oppositeFace( fid, vidxs[2], vidxs[0]);
            if ( _sfids.count(f2) == 0)
                find( f2);
        }   // end if

        return _minsd;
    }   // end find

    const Vec3f& vertex() const { return _bv;}

    int poly() const { return _bfid;}

private:
    const Mesh& _mesh;
    const Vec3f _t;
    float _minsd;
    Vec3f _bv;
    int _bfid;
    IntSet _sfids;
};  // end struct

}   // end namespace


float SurfacePointFinder::find( const Vec3f& t, int& vidx, int& fid, Vec3f& fv) const
{
    assert( !_mesh.faces(vidx).empty());
    float sd = 0;
    if ( _mesh.vtx(vidx) == t)
    {
        fv = t;
        fid = -1;
    }   // end if
    else
    {
        fid = *_mesh.faces(vidx).begin();
        FaceFinder pfinder( _mesh, t);
        sd = pfinder.find( fid);
        fv = pfinder.vertex();
        fid = pfinder.poly();
        vidx = -1;
    }   // end else
    return sd;
}   // end find


Vec3f SurfacePointFinder::find( const Vec3f& t, int vidx, int* fid) const
{
    int f = 0;
    Vec3f fv = t;
    if ( vidx < 0)
        vidx = *_mesh.vtxIds().begin();
    find( t, vidx, f, fv);
    if ( fid)
        *fid = f;
    return fv;
}   // end find

