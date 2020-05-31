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

#include <Smoother.h>
#include <boost/heap/fibonacci_heap.hpp>
using r3d::Curvature;
using r3d::Smoother;
using r3d::Mesh;
using r3d::Vec3f;

namespace {

// Keep a heap of vertices ordered on curvature
struct VtxCurv;
struct VtxCurvComparator { bool operator()( const VtxCurv *v0, const VtxCurv *v1) const;};
using MaxHeap = boost::heap::fibonacci_heap<VtxCurv*, boost::heap::compare<VtxCurvComparator> >;

struct VtxCurv
{
    VtxCurv( const Curvature &c, int vi, const MaxHeap *ph) : cm(c), vidx(vi), pheap(ph) {}

    const Curvature &cm;
    int vidx;   // Vertex ID
    const MaxHeap *pheap; // Pointer to heap for comparing max curvature vertex to this one
    MaxHeap::handle_type handle;

    static float calcCurvature( const Curvature &cmap, int vi)
    {
        float kp1, kp2;
        cmap.vertexPC1( vi, kp1);
        cmap.vertexPC2( vi, kp2);
        return 0.5f * (fabsf(kp1) + fabsf(kp2));
    }   // end calcCurvature

    float curvature() const { return calcCurvature( cm, vidx);}
};  // end struct


bool VtxCurvComparator::operator()( const VtxCurv *v0, const VtxCurv *v1) const
{
    const float c0 = v0->curvature();
    const float c1 = v1->curvature();
    if ( c0 < c1)
        return true;
    if ( c0 > c1)
        return false;

    // In case curvature is equal, prefer vertex that's further from the vertex having maximum curvature.
    const Mesh &m = v0->cm.mesh();
    assert( !v0->pheap->empty());
    const int vidx = v0->pheap->top()->vidx;
    assert( m.vtxIds().count(vidx) > 0);
    const Vec3f &mpos = m.uvtx( vidx);  // Position of max curvature vertex
    return (mpos - m.uvtx( v0->vidx)).norm() >= (mpos - m.uvtx( v1->vidx)).norm();
}   // end operator()


int popHeap( MaxHeap &heap, std::unordered_map<int, VtxCurv*> &vcmap, IntSet &hcset)
{
    VtxCurv *vc = heap.top();
    heap.pop();
    const int vidx = vc->vidx;
    hcset.insert(vidx);
    vcmap.erase(vidx);
    delete vc;
    return vidx;
}   // end popHeap


void pushHeap( MaxHeap &heap, std::unordered_map<int, VtxCurv*> &vcmap, int vidx, const Curvature &cm)
{
    if ( vcmap.count(vidx) == 0)
    {
        VtxCurv *vc = new VtxCurv( cm, vidx, &heap);
        vc->handle = heap.push(vc);    // O(log(N))
        vcmap[vidx] = vc;
    }   // end if
    else
    {
        VtxCurv *vc = vcmap.at(vidx);
        heap.increase( vc->handle); // O(log(N))
    }   // end else
}   // end pushHeap


// Get all initial vertices having curvature greater than maxc
void createHeap( IntSet &hcset, MaxHeap &heap, std::unordered_map<int, VtxCurv*> &vcmap, const Curvature &cm, float maxc)
{
    for ( int vidx : hcset) // for all vertices in the high curvature set
        if ( VtxCurv::calcCurvature( cm, vidx) > maxc)
            pushHeap( heap, vcmap, vidx, cm);
    hcset.clear();
}   // end createHeap

}   // end namespace


Smoother::Smoother( float maxc, size_t mnits) : _maxc(maxc), _maxIts(mnits) {}


namespace {
Vec3f calcAdjustedVertex( const Mesh &m, int vidx)
{
    Vec3f nv = Vec3f::Zero();    // Interpolate new position as mean of connected vertices
    const IntSet& cvtxs = m.cvtxs(vidx);
    for ( int cv : cvtxs)
        nv += m.uvtx(cv);
    return nv / cvtxs.size();
}   // end calcAdjustedVertex
}   // end namespace


void Smoother::operator()( Mesh &mesh, Curvature& cmap, const IntSet *subset)
{
    if ( !subset)   // If subset not given, look at all vertices on the mesh.
        subset = &mesh.vtxIds();

    cmap.setMesh(mesh);

    IntSet hcset;   // Vertices with curvature more than the allowed max
    for ( int vid : *subset)
        if ( VtxCurv::calcCurvature( cmap, vid) > _maxc) // Get initial vertices as those only with curvature > allowed max.
            hcset.insert(vid);

    MaxHeap heap;
    std::unordered_map<int, VtxCurv*> vcmap; // Map vertex IDs to corresponding curvature MaxHeap nodes

    size_t i = 0;
    while ( i < _maxIts && !hcset.empty())
    {
        i++;    // Create a max heap of the vertices with maximum curvature which will be a subset of hcset.
        createHeap( hcset, heap, vcmap, cmap, _maxc);  // hcset empty on return

        while ( !heap.empty())
        {
            const int vidx = popHeap( heap, vcmap, hcset);  // vidx inserted into hcset and removed from vcmap.
            const Vec3f npos = calcAdjustedVertex( mesh, vidx);   // Repositioned as mean of connected vertices.
            cmap.adjustRawVertex( mesh, vidx, npos);              // Update curvature at the vertex (and vertices connected to it).

            // Parse the connected vertices of the newly adjusted vertex since their curvature will have changed.
            for ( int cv : mesh.cvtxs(vidx))
            {
                // Don't add vertices parsed while heap not empty; only add previously identified surface vertices,
                // and don't add any vertices with curvature <= _maxc.
                if ( hcset.count(cv) == 0 && subset->count(cv) > 0 && VtxCurv::calcCurvature( cmap, cv) > _maxc)
                    pushHeap( heap, vcmap, cv, cmap);
            }   // end for
        }   // end while
    }   // end while

    assert( heap.empty());
    assert( vcmap.empty());
}   // end operator()
