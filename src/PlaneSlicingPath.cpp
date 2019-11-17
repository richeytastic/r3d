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
    const Vec3f u = faceSlicingPlane( _ifid);
    const FacePlane pp( _mesh, _ifid, _ip, u);
    assert( pp.inhalf() == 0);

    int nfid = -1;

    const int nfidab = _mesh.oppositeFace( _ifid, pp.vaid(), pp.vbid());
    if ( nfidab >= 0 && nfidab != notThisFid)
    {
        nfid = nfidab;
        assert( !pp.abIntersection().array().isNaN().any());
        if ( pp.abIntersection() != _evtxs.back())
            _evtxs.push_back( pp.abIntersection());
    }   // end if

    const int nfidac = _mesh.oppositeFace( _ifid, pp.vaid(), pp.vcid());
    if ( nfid < 0 && nfidac >= 0 && nfidac != notThisFid)
    {
        nfid = nfidac;
        assert( !pp.acIntersection().array().isNaN().any());
        if ( pp.acIntersection() != _evtxs.back())
            _evtxs.push_back( pp.acIntersection());
    }   // end else

    _nfid = _ffid = nfid;
}   // end init


bool PlaneSlicingPath::canSplice( const PlaneSlicingPath& psp) const
{
    return (canExtend() && psp.nextFace() == _nfid) || (psp.edgeCrossings().back() - edgeCrossings().back()).squaredNorm() < 1e-4f;
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
        path.push_back( _evtxs[iidx--]);
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


int PlaneSlicingPath::_findNextFaceEdgeVertex( Vec3f& ev)
{
    int fid = _nfid;

    const Vec3f &v = _evtxs.back();
    _pfids.insert(fid);     // Record that we've parsed this face
    const Vec3f u = faceSlicingPlane( fid);
    assert( !u.isZero());

    const FacePlane pp( _mesh, fid, v, u);

    // if inhalf is not zero, then the calculated plane slicing norm does
    // not cut through the face and we're done.
    if ( pp.inhalf() != 0)
        return -1;

    // If vertex v is EXACTLY incident with a vertex on this face, then pp.straddleId()
    // will return the straddling vertex ID. In this case, it is necessary to check which
    // direction the path entered the straddling vertex and choose the next face based on this.
    if ( pp.straddleId() >= 0)
    {
#ifndef NDEBUG
        std::cerr << "  - Got straddle vertex " << pp.straddleId() << std::endl;
#endif
        assert( pp.vaid() != pp.straddleId());
        // If the last face parsed is adjacent to the straddle vertex, then the next
        // face should be the one opposite the straddle vertex since the path entered
        // via the straddle vertex.
        const int adjFace0 = _mesh.oppositeFace( fid, pp.straddleId(), pp.vaid());
        const int notStraddleAndNotVaidVtx = _mesh.face(fid).opposite( pp.straddleId(), pp.vaid());
        const int adjFace1 = _mesh.oppositeFace( fid, pp.straddleId(), notStraddleAndNotVaidVtx);
        if ( _lastParsedFace == adjFace0 || _lastParsedFace == adjFace1)
        {
#ifndef NDEBUG
        std::cerr << "  -  Path entered via straddle vertex" << std::endl;
#endif
        assert( pp.vaid() != pp.straddleId());
            fid = _mesh.oppositeFace( fid, pp.vaid(), notStraddleAndNotVaidVtx);
            // Get the intersection point on the edge opposite to the straddle vertex
            if ( notStraddleAndNotVaidVtx == pp.vbid())
                ev = pp.abIntersection();
            else
                ev = pp.acIntersection();
        }   // end if
        else
        {
#ifndef NDEBUG
        std::cerr << "  -  Path entered via opposite edge" << std::endl;
#endif
            // Otherwise, the path entered through the edge opposite
            // to the straddle vertex and so the next face can be either of the
            // two faces adjacent to the straddle vertex and this face.
            fid = _mesh.oppositeFace( fid, pp.straddleId(), pp.vaid());
            // And the next vertex on the path is the position of the straddle vertex.
            ev = _mesh.vtx( pp.straddleId());
        }   // end else
    }   // end if
    else
    {
        // Get the two possible next faces
        const int nfidAB = _mesh.oppositeFace( fid, pp.vaid(), pp.vbid());
        const int nfidAC = _mesh.oppositeFace( fid, pp.vaid(), pp.vcid());

        // If one of these is already a parsed face, then the choice is easy:
        // it is only a parsed face because the path came into this face from
        // that edge. However, it is possible that neither face is in the
        // parsed set; this can happen if the edge is "bounced" from entering
        // the current face and the path is given cause to run along the
        // edge through the edge vertices. In this case we need to select
        // the next face based on its local direction from the start vertex.
        bool useAC = false;
        if ( _pfids.count(nfidAB) > 0)
            useAC = true;
        else if ( _pfids.count(nfidAC) == 0)
        {
            // Select based on the last step vector being congruent in direction
            // with the chosen vertex (or coincidental with it).
            const Vec3f lstep = v - _evtxs[_evtxs.size()-2];
            assert( !lstep.isZero());   // Last step vector can't be zero
            useAC  = (_mesh.vtx(pp.vcid()) - v).dot(lstep) >= 0;
        }   // end else

        if ( useAC)
        {
            ev = pp.acIntersection();
            fid = nfidAC;
        }   // end if
        else
        {
            ev = pp.abIntersection();
            fid = nfidAB;
        }   // end else if
    }   // end else

    // If we've already parsed nfid, then return -1, otherwise return nfid (which could still be -1 if an edge was reached)
    return _pfids.count(fid) > 0 ? -1 : fid;
}   // end _findNextFaceEdgeVertex


bool PlaneSlicingPath::extend()
{
    assert( _nfid >= 0);
    Vec3f ev;
    int lfid = _nfid;
    _nfid = _findNextFaceEdgeVertex( ev);
    _lastParsedFace = lfid;

    if ( _nfid >= 0)
    {
        // If the opposite face is the initial face then we've encountered a loop so fail
        if ( _nfid == _ifid)
            _nfid = -1;
        else
        {
            assert( !ev.array().isNaN().any());
            if ( ev != _evtxs.back())   // Ensure a step of non-zero magnitude is recorded
                _evtxs.push_back( ev);
        }   // end else
    }   // end if

    return _nfid >= 0;
}   // end extend
