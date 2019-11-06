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

#include <Remesher.h>
#include <VertexCrossingTimeCalculator.h>
#include <cmath>
#include <cassert>
#include <iomanip>
#include <sstream>
#include <queue>
using r3d::Remesher;
using r3d::FastMarcher;
using r3d::Mesh;
using r3d::Face;
using r3d::FaceAngles;
using r3d::Vec3f;
using r3d::VecXi;
using std::unordered_map;

/*
// DEBUG - private
void Remesher::printTime( int v, const unordered_map<int,int>& srcs) const
{
    const int s = srcs.at(v);
    std::cerr << " (" << std::right << std::setw(2) << v << ")[" << std::right << std::setw(2) << s
              << "]:" << std::fixed << std::setprecision(2) << _vTimes.at(v).at(s) << "   ";
    //std::cerr << " (" << std::right << std::setw(2) << v << ")[" << std::right << std::setw(2) << s << "] ";
    //std::cerr << " [" << std::right << std::setw(2) << s << "] ";
}   // end printTime


// DEBUG - private
void Remesher::printVertexTimes( const unordered_map<int,int>& srcs) const
{
    const IntSet& vidxs = _fm.mesh()->getVertexIds();
    const int ndim = (int)sqrtf((float)vidxs.size());
    std::vector<std::vector<int> > m(ndim);
    for ( int i = 0; i < ndim; ++i)
        m[i].resize(ndim);

    for ( int vi : vidxs)
    {
        const Vec3f& v = _fm.mesh()->getVertex(vi);
        m[(int)v[0]][(int)v[1]] = vi;
    }   // end foreach

    for ( int i = ndim-1; i >= 0; --i)
    {
        std::cerr << "    ";
        for ( int j = 0; j < ndim; ++j)
            printTime( m[j][i], srcs);
        std::cerr << std::endl;
    }   // end for
}   // end printVertexTimes


// DEBUG
size_t printFaces( const Mesh* mesh)
{
    const IntSet& fids = mesh->getFaceIds();
    if ( fids.empty())
        return 0;
    std::cerr << "\t\t " << fids.size() << " faces" << std::endl;
    for ( int fid : fids)
    {
        const Face& face = mesh->getFace( fid);
        std::cerr << "\t\t   F" << fid << ") " << face.vindices[0] << "," << face.vindices[1] << "," << face.vindices[2] << std::endl;
    }   // end foreach
    return fids.size();
}   // end printFaces


// DEBUG
void listSaddlePoints( const unordered_map<int,IntSet>& xymap) // DEBUG
{
    int n = 0;
    typedef std::pair<int,IntSet> XY;
    for ( const XY& xy : xymap)
    {
        const int x = xy.first;
        for ( int y : xy.second)
        {
            std::cerr << "    [" << std::right << std::setw(2) << x << "]-->[" << std::right << std::setw(2) << y << "] " << std::endl;
            n++;
        }   // end foreach
    }   // end foreach
    std::cerr << "    " << n << " xymap" << std::endl;
}   // end listSaddlePoints
*/


void Remesher::_init()
{
    assert( _fm.mesh().hasSequentialIds());
    _resetMaxDistanceHeap();
    _vTimes.clear();
    _saddlePoints.clear();
    _outmesh = Mesh::create();
    const size_t N = _fm.mesh().numVtxs();
    _nearestSources = VecXi::Ones(N) * -1;  // Denote no source mapping initially
}   // end _init


void Remesher::_resetMaxDistanceHeap()
{
    while ( !_maxHeap.empty())
    {
        r3d::MaxVertex* v = _maxHeap.top();
        _maxHeap.pop();
        delete v;
    }   // end while
    _heapMap.clear();
}   // end _resetMaxDistanceHeap


int Remesher::_popMaxDistanceHeap( float& t)
{
    if ( _maxHeap.empty())
        return -1;
    r3d::MaxVertex* v = _maxHeap.top();
    _maxHeap.pop();
    const int vidx = v->vidx;
    t = v->time;
    _heapMap.erase( vidx);
    delete v;
    return vidx;
}   // end _popMaxDistanceHeap


void Remesher::_updateMaxHeap( int A, float t)
{
    if ( !_heapMap.count(A))
    {
        r3d::MaxVertex* v = _heapMap[A] = new r3d::MaxVertex( A, t);
        v->maxHeapHandle = _maxHeap.push(v);   // O(log(N))
    }   // end if
    else
    {
        r3d::MaxVertex* v = _heapMap.at(A);
        v->time = t;
        _maxHeap.decrease( v->maxHeapHandle);  // O(log(N))
    }   // end else
}   // end _updateMaxHeap


void Remesher::_updateNarrowBand( int A, float t)
{
    if ( _inarrowBand.count(A) == 0)
    {
        r3d::MinVertex* v = _inarrowBand[A] = new r3d::MinVertex( A, t);
        v->minHeapHandle = _iminHeap.push(v);    // O(log(N))
    }   // end if
    else
    {
        r3d::MinVertex* v = _inarrowBand.at(A);
        v->time = t;
        _iminHeap.increase( v->minHeapHandle);  // O(log(N))
    }   // end else
}   // end _updateNarrowBand


Remesher::Remesher( const FastMarcher &fm) : _fm(fm) {}


void Remesher::createSaddleEdges( unordered_map<int,IntSet>& xymap) const
{
    for ( int vidx : _saddlePoints)
    {
        const int X = _nearestSources[vidx];
        const IntSet& cvs = _fm.mesh().cvtxs(vidx);
        for ( int c : cvs)
        {
            const int Y = _nearestSources[c];
            if ( Y < X)
                xymap[X].insert(Y);
        }   // end for
    }   // end for
}   // end createSaddleEdges


void Remesher::_setVertexNearestSource( int A, int ns)
{
    _nearestSources[A] = ns; // Record that this input vertex has been reached by the new source
    const float t = _vTimes.at(A).at(ns);
    _saddlePoints.erase(A);
    _updateNarrowBand( A, t);
    _updateMaxHeap( A, t);
}   // end _setVertexNearestSource


float Remesher::_calcTimeFromSource( int A, const Vec3f& vS) const
{
    const Vec3f& vA = _fm.mesh().uvtx( A);
    return _fm.speedFn()(A) * (vA - vS).norm();  // Update crossing time at A
}   // end _calcTimeFromSource


int Remesher::sample( int A, size_t npoints) { return _sample( A, npoints, false);}


int Remesher::sampleInterpolated( int A, size_t npoints) { return _sample( A, npoints, true);}


int Remesher::_sample( int A, size_t npoints, bool interpolate)
{
    const Mesh &mesh = _fm.mesh();
    if ( npoints < 3 || npoints > mesh.numVtxs())
    {
        std::cerr << "[WARNING] r3d::Remesher::remesh() : called with < 3 or > N"
                  << " points (where N is the number of unique vertices in the input mesh)." << std::endl;
        return -1;
    }   // end if

    _init();

    FaceInterpolator *interpolator = nullptr;
    if ( interpolate)
        interpolator = new FaceInterpolator( mesh, *_outmesh, _nearestSources, _vTimes);

    int ns = 0;
    float oldt = FLT_MAX;
    while ( _outmesh->numVtxs() < npoints && A >= 0)
    {
        const Vec3f vS = interpolate ? interpolator->interpolate( A) : mesh.uvtx(A);
        // The newly iterpolated point should be closer to A. If it's not, it means that the
        // underlying (input) mesh has lower than required density in this region and that the
        // newly interpolated point should not be added as it cannot be referenced from any input vertex.
        const float newt = _calcTimeFromSource( A, vS);
        if ( newt < oldt)
        {
            ns = _outmesh->addVertex( vS);
            _vTimes[A][ns] = newt;
            _setVertexNearestSource( A, ns);
        }   // end if

        _ifixedSet.clear();
        while ( !_inarrowBand.empty())
            _expandInputFront( ns);

        const int oldA = A;
        do {
            A = _popMaxDistanceHeap( oldt);
        } while ( oldA == A);
    }   // end while

    if ( interpolator)
        delete interpolator;

    return _outmesh->numVtxs();
}   // end _sample


int Remesher::_popNarrowBand()
{
    r3d::MinVertex* v = _iminHeap.top();
    _iminHeap.pop();
    const int A = v->vidx;
    _ifixedSet.insert(A);
    _inarrowBand.erase(A);
    delete v;
    return A;
}   // end _popNarrowBand


void Remesher::_expandInputFront( int Y)
{
    const int A = _popNarrowBand();  // A fixed as closer to _nearestSources[A] == Y than any other source
    VertexCrossingTimeCalculator vtcalc( _fm.angles(), _vTimes, Y);

    const IntSet& cvs = _fm.mesh().cvtxs( A);
    for ( int C : cvs)
    {
        const float t = vtcalc( C, _fm.speedFn()(C));
        _vTimes[C][Y] = t;

        if ( _ifixedSet.count(C) > 0)  // Only update non-fixed
            continue;

        float tC = FLT_MAX;
        const int X = _nearestSources[C];// Current closest source to C
        if ( X >= 0)
            tC = _vTimes.at(C).at(X);    // Current best crossing time at C is from X

        if ( t < tC) // Y is at least as near to C as X is to C - NB sometimes Y == X
            _setVertexNearestSource( C, Y);  // Replace X with Y as nearest interpolated source vertex to C and set C in narrow band for expansion
        else
            _saddlePoints.insert(A);    // Create a saddle record at input vertex A
    }   // end for
}   // end _expandInputFront
