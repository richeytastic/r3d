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

#include <PlanarSlicingPath.h>
#include <SurfacePathFinder.h>  // For calcPathLength
#include <FacetCut.h>
using r3d::PlanarSlicingPath;
using r3d::Vec3f;
using r3d::Mesh;


PlanarSlicingPath::PlanarSlicingPath( const Mesh& m, int ifid, const Vec3f& v0,
                                                     int efid, const Vec3f &v1,
                                                     const std::function<Vec3f(int)> &fsp)
    : _mesh(m), _ifid(ifid), _v0(v0), _efid(efid), _v1(v1), _facetSlicePlaneFn( fsp) {}


int PlanarSlicingPath::_setEdgeCrossing( int fid)
{
    // Cut this facet
    const Vec3f u = _facetSlicePlaneFn( fid);
    const FacetCut fc( _mesh, fid, _vtxs.back(), u);
    if ( fc.half() != 0)
        return -1;

    const int nfidAB = _mesh.oppositeFace( fid, fc.vaid(), fc.vbid());
    const int nfidAC = _mesh.oppositeFace( fid, fc.vaid(), fc.vcid());

    if ( _pfids.count(nfidAB) == 0)
    {
        _vtxs.push_back( fc.abIntersection());
        fid = nfidAB;
    }   // end if
    else if ( _pfids.count(nfidAC) == 0)
    {
        _vtxs.push_back( fc.acIntersection());
        fid = nfidAC;
    }   // end else if
    else
        fid = -1;

    return fid;
}   // end _setEdgeCrossing


bool PlanarSlicingPath::_extend()
{
    _pfids.insert(-1);
    if ( _fids.size() > 1)
        _pfids.insert(_fids.at(1));
    _fids.clear();
    _vtxs.clear();
    _vtxs.push_back(_v0);
    int fid = _ifid;
    while ( fid >= 0 && fid != _efid)
    {
        _fids.push_back(fid);
        _pfids.insert(fid);
        fid = _setEdgeCrossing( fid);
    }   // end while
    if ( fid == _efid)
    {
        _fids.push_back(fid);
        _vtxs.push_back(_v1);
    }   // end if
    return fid == _efid;
}   // end _extend


std::vector<Vec3f> PlanarSlicingPath::shortestPath( float *plen)
{
    float len = FLT_MAX;
    std::vector<Vec3f> spath;
    if ( _extend()) // Found a path?
    {
        spath = std::move( _vtxs);
        len = calcPathLength( spath);
    }   // end if

    if ( _extend()) // Is there a path in the other direction?
    {
        const float len2 = calcPathLength( _vtxs);
        if ( len2 < len)  // Is it shorter?
        {
            spath = std::move( _vtxs);
            len = len2;
        }   // end if
    }   // end if

    if ( plen)
        *plen = len;
    return spath;
}   // end shortestPath
