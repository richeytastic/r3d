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

#include <PlaneSlicingPath.h>
#include <FacePlane.h>
#include <cassert>
using r3d::PlaneSlicingPath;
using r3d::Mesh;
using r3d::Vec3f;


PlaneSlicingPath::PlaneSlicingPath( const Mesh& m, int ifid, const Vec3f& ip)
    : _mesh(m), _ip(ip), _ifid(ifid)
{
    assert( !ip.array().isNaN().any());
    reset();
}   // end ctor


void PlaneSlicingPath::reset()
{
    _ffid = -1;
    _nfid = _ifid;
    _evtxs.clear();
    _evtxs.push_back(_ip);
    _pfids.clear();
    _pfids.insert(_ifid);
    _lastParsedFace = _ifid;
}   // end reset


void PlaneSlicingPath::init( int notThisFid)
{
    reset();

    // Derived class implementation defines the calculation to find the plane that
    // crosses at right angles to the path direction and goes through triangle _ifid.
    const Vec3f u = faceSlicingPlane( _ifid, _ip);
    const FacePlane pp( _mesh, _ifid, _ip, u);
    assert( pp.inhalf() == 0);

    int nfid = -1;

    const int nfidab = _mesh.oppositeFace( _ifid, pp.vaid(), pp.vbid());
    if ( nfidab >= 0 && nfidab != notThisFid)
    {
        nfid = nfidab;
        assert( !pp.abIntersection().array().isNaN().any());
        _evtxs.push_back( pp.abIntersection());
    }   // end if

    const int nfidac = _mesh.oppositeFace( _ifid, pp.vaid(), pp.vcid());
    if ( nfid < 0 && nfidac >= 0 && nfidac != notThisFid)
    {
        nfid = nfidac;
        assert( !pp.acIntersection().array().isNaN().any());
        _evtxs.push_back( pp.acIntersection());
    }   // end else

    _nfid = _ffid = nfid;
}   // end init


bool PlaneSlicingPath::canSplice( const PlaneSlicingPath& psp) const
{
    return (canExtend() && psp.nextFace() == _nfid) || (psp.lastVertexAdded() - lastVertexAdded()).squaredNorm() < 1e-4f;
}   // end canSplice


void PlaneSlicingPath::_pushOnInitialToBack( std::vector<Vec3f>& path) const
{
    int iidx = 0;
    const int n = static_cast<int>(_evtxs.size());
    while ( iidx < n)
        path.push_back( _evtxs[iidx++]);
}   // _pushOnInitialToBack


void PlaneSlicingPath::_pushOnBackToInitial( std::vector<Vec3f>& path) const
{
    int iidx = static_cast<int>(_evtxs.size()) - 1;
    while ( iidx >= 0)
    {
        const Vec3f &v = _evtxs[iidx--];
        if ( path.back() != v)  // No duplicate
            path.push_back( v);
    }   // end if
}   // _pushOnBackToInitial


void PlaneSlicingPath::splice( const PlaneSlicingPath& psp, std::vector<Vec3f>& path) const
{
    path.clear();
    if ( canSplice(psp))
    {
        _pushOnInitialToBack( path);
        psp._pushOnBackToInitial( path);
    }   // end if
}   // end splice


int PlaneSlicingPath::_findNextFaceEdgeVertex( int fid, const Vec3f& v, Vec3f& ev)
{
    _pfids.insert(fid);     // Record that we've parsed this face
    const Vec3f u = faceSlicingPlane( fid, v);
    _lastParsedFace = fid;
    assert( !u.isZero());

    const FacePlane pp( _mesh, fid, v, u);

    // if inhalf is not zero, then the calculated plane slicing norm does
    // not cut through the face and we're done.
    if ( pp.inhalf() != 0)
        return -1;

    int nfid = -1;
    // If vertex v is EXACTLY incident with a vertex on this face, then pp.straddleId()
    // will return the straddling vertex ID and in this case the opposite face is the one
    // adjacent to the edge opposite the straddling vertex.
    if ( pp.straddleId() >= 0)
    {
        //std::cerr << "  - Got straddle vertex " << pp.straddleId() << std::endl;
        assert( pp.vaid() != pp.straddleId());
        nfid = _mesh.oppositeFace( fid, pp.vaid(), pp.straddleId());
        if ( pp.straddleId() == pp.vbid())
            ev = pp.abIntersection();
        else
            ev = pp.acIntersection();
    }   // end if
    else
    {
        nfid = _mesh.oppositeFace( fid, pp.vaid(), pp.vbid());
        if ( _pfids.count(nfid) == 0)
            ev = pp.abIntersection();
        else
        {
            nfid = _mesh.oppositeFace( fid, pp.vaid(), pp.vcid());
            ev = pp.acIntersection();
        }   // end else
    }   // end else
    // If we've already parsed nfid, then return -1, otherwise return nfid (which could still be -1 if an edge was reached)
    return _pfids.count(nfid) > 0 ? -1 : nfid;
}   // end _findNextFaceEdgeVertex


bool PlaneSlicingPath::extend()
{
    assert( _nfid >= 0);
    Vec3f ev;
    _nfid = _findNextFaceEdgeVertex( _nfid, _evtxs.back(), ev);

    if ( _nfid >= 0)
    {
        // If the opposite face is the initial face then we've encountered a loop so fail
        if ( _nfid == _ifid)
            _nfid = -1;
        else
        {
            assert( !ev.array().isNaN().any());
            _evtxs.push_back( ev);
        }   // end else
    }   // end if

    return _nfid >= 0;
}   // end extend
