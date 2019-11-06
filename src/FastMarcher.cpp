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

#include <FastMarcher.h>
#include <VertexCrossingTimeCalculator.h>
using r3d::FastMarcher;
using r3d::FaceAngles;
using r3d::MinVertex;
using r3d::MaxVertex;
#include <algorithm>
#include <cassert>
#include <iomanip>
#include <cfloat>


// Returns true if v0 is considered to be less than v1 for a max-heap (v0 goes below v1)
bool r3d::VertexMinTimeComparator::operator()( const MinVertex* v0, const MinVertex* v1) const
{
    return v0->time >= v1->time;  // Reversed because boost creates a max-heap
}   // end operator()


// Returns true if v1 is considered to be less than v0 for a max-heap (v0 goes above v1)
bool r3d::VertexMaxTimeComparator::operator()( const MaxVertex* v0, const MaxVertex* v1) const
{
    return v0->time <= v1->time;  // Reversed because boost creates a max-heap
}   // end operator()


// public static
FastMarcher::Ptr FastMarcher::create( const FaceAngles &fa, const FastMarcher::SpeedFn &sf)
{
    return Ptr( new FastMarcher( fa, sf));
}   // end create


FastMarcher::FastMarcher( const FaceAngles &fa, const FastMarcher::SpeedFn &sf) : _fangles(fa), _speedFn(sf)
{
    assert( fa.mesh().hasSequentialIds());
    _time = VecXf::Ones( fa.mesh().numVtxs()) * FLT_MAX;
}   // end ctor


int FastMarcher::operator()( int vidx, float t)
{
    assert( _fangles.mesh().vtxIds().count(vidx) > 0);

    _narrowBand = new std::unordered_map<int, MinVertex*>;
    _fixed = new std::unordered_set<int>;
    _minHeap = new MinHeap;

    _addVertex( vidx, t);
    int lastReached = vidx;
    while ( !_narrowBand->empty())
        lastReached = _expandFront();

    delete _narrowBand;
    delete _fixed;
    delete _minHeap;

    return lastReached;
}   // end operator()


int FastMarcher::_expandFront()
{
    const Mesh &mesh = _fangles.mesh();

    const int vidx = _popVertex();
    const IntSet& cvs = mesh.cvtxs( vidx);
    for ( int c : cvs)
    {
        if ( _fixed->count(c) > 0)  // Ignore vertices with already fixed arrival times
            continue;

        const float F = _speedFn(c);  // Speed of progression at the point being updated
        const float t = VertexCrossingTimeCalculator( _fangles, _time)( c, F);

        if ( t < _time[c])
        {
            if ( _narrowBand->count(c) == 0)    // Is C in the propagation front?
                _addVertex( c, t);
            else
                _updateVertex( c, t);  // Update with faster crossing time
        }   // end if
    }   // end for

    return vidx;
}   // end _expandFront


int FastMarcher::_popVertex()
{
    MinVertex* v = _minHeap->top();  // Reached "first" by current front
    _minHeap->pop();
    const int vidx = v->vidx;
    _fixed->insert( vidx);  // Fix this vertex
    _narrowBand->erase( vidx);
    delete v;
    return vidx;
}   // end _popVertex


void FastMarcher::_addVertex( int vi, float t)
{
    MinVertex* v = (*_narrowBand)[vi] = new MinVertex( vi, t);
    _time[vi] = t;
    v->minHeapHandle = _minHeap->push( v); // O(log(N))
}   // end _addVertex


void FastMarcher::_updateVertex( int vi, float t)
{
    MinVertex* v = _narrowBand->at(vi);
    _time[vi] = v->time = t;
    _minHeap->increase( v->minHeapHandle);   // O(log(N))
}   // end _updateVertex
