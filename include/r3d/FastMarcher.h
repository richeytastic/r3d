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

#ifndef R3D_FAST_MARCHER_H
#define R3D_FAST_MARCHER_H

/**
 * Implements Sethian's Fast Marching algorithm on a triangulated manifold.
 * Runs in ~ O(N log(N))
 */

#include "FaceAngles.h"
#include <functional>
#include <boost/heap/fibonacci_heap.hpp>

namespace r3d {

struct MinVertex;
struct MaxVertex;
struct VertexMinTimeComparator { bool operator()( const MinVertex* v0, const MinVertex* v1) const;};
struct VertexMaxTimeComparator { bool operator()( const MaxVertex* v0, const MaxVertex* v1) const;};
using MinHeap = boost::heap::fibonacci_heap<MinVertex*, boost::heap::compare<VertexMinTimeComparator> >;
using MaxHeap = boost::heap::fibonacci_heap<MaxVertex*, boost::heap::compare<VertexMaxTimeComparator> >;

struct MinVertex
{
    MinVertex( int ui, float t) : vidx(ui), time(t) { }   // end ctor
    int vidx;       // The vertex on the mesh that this MinVertex is closest to.
    float time;     // The time at which this vertex is crossed
    MinHeap::handle_type minHeapHandle;
};  // end struct

struct MaxVertex
{
    MaxVertex( int ui, float t) : vidx(ui), time(t) { }   // end ctor
    int vidx;       // The vertex on the mesh that this MinVertex is closest to.
    float time;     // The time at which this vertex is crossed
    MaxHeap::handle_type maxHeapHandle;
};  // end struct


class r3d_EXPORT FastMarcher
{
public:
    // The basic speed function may be designed to respond to the surface function of the
    // underlying mesh (e.g. curvature at a vertex). By default, a uniform speed is used.
    using SpeedFn = std::function<float( int)>;

    using Ptr = std::shared_ptr<FastMarcher>;
    static Ptr create( const FaceAngles&, const SpeedFn& sfn = [](int){return 1.0f;});
    FastMarcher( const FaceAngles&, const SpeedFn& sfn = [](int){return 1.0f;});

    inline const Mesh &mesh() const { return _fangles.mesh();}
    inline const FaceAngles &angles() const { return _fangles;}
    inline const SpeedFn &speedFn() const { return _speedFn;}

    // Update the vertex time crossing map using a fast marching front
    // that propagates outwards from the starting vertex (initial entry on _minHeap).
    // On return, provided tmap will have been updated only with the vertices that
    // can be reached from vidx. Returns the ID of the unique vertex reached
    // last by the propagating front which will be the most distant from vidx
    // t is the initial time of arrival at vertex vidx. Typically 0 unless making
    // multiple sequentially updating calls to the time map.
    int operator()( int vidx, float=0.0f);

    // Return the vertex time crossings. Only vertices reachable from vidx have non FLT_MAX times.
    // Subsequent calls to propagateFront will update this mapping.
    inline const VecXf &crossings() const { return _time;}

private:
    const FaceAngles &_fangles;
    const SpeedFn &_speedFn;
    VecXf _time;    // Time when front passed vertices

    std::unordered_map<int, MinVertex*> *_narrowBand;  // narrow band of vertices constituting the propagating front
    std::unordered_set<int> *_fixed;                   // members of _time with fixed times
    MinHeap *_minHeap;                  // Priority queue of _narrowBand allows for O(1) access to vertex closest to fixed

    int _expandFront();
    int _popVertex();
    void _addVertex( int ui, float t);
    void _updateVertex( int ui, float t);

    FastMarcher( const FastMarcher&) = delete;
    void operator=( const FastMarcher&) = delete;
};  // end class

}   // end namespace

#endif
