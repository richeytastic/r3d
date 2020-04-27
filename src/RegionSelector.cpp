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

#include <RegionSelector.h>
#include <algorithm>
#include <cassert>
using r3d::RegionSelector;
using r3d::Mesh;
using r3d::Vec3f;


// public static
RegionSelector::Ptr RegionSelector::create( const Mesh& mesh)
{
    assert( mesh.numFaces() > 0);
    return Ptr( new RegionSelector( mesh), [](RegionSelector* d){delete d;});
}   // end create


// private
RegionSelector::RegionSelector( const Mesh& mesh) : _mesh(mesh), _cval(0,0,0), _front( new IntSet) {}


// private
RegionSelector::~RegionSelector() { delete _front;}


Vec3f RegionSelector::centre() const
{
    const int* fvidxs = _mesh.fvidxs(_cf);
    const Vec3f &a = _mesh.vtx(fvidxs[0]);
    const Vec3f &b = _mesh.vtx(fvidxs[1]);
    const Vec3f &c = _mesh.vtx(fvidxs[2]);
    return _cval[0] * a + _cval[1] * b + _cval[2] * c;
    //return _mesh.toAbsFromProp( _cf, _cval);
}   // end centre


namespace {

// vflag denotes whether vidx should:
// -1) Be neither on the front or in the body since it is outside the radius threshold (default assumption).
//  0) Stay on the front which requires that vidx is inside the radius threshold but that at least one of its
//     connected vertices are outside the radius threshold, or that vidx is an edge vertex.
//  1) Be in the body because all of its connected vertices are *not outside* the radius threshold.
int testMembership( int vidx, const Mesh& m, const Vec3f& ov, float R)
{
    const float rval = (m.vtx(vidx) - ov).squaredNorm();
    if ( rval > R)
        return -1;

    // vidx in body unless a connected vertex is outside radius threshold,
    // or vidx and a connected vertex makes a boundary edge.
    for ( int cv : m.cvtxs(vidx))
    {
        if ( (( m.vtx(cv) - ov).squaredNorm() > R) || (m.nsfaces( vidx, cv) == 1))
            return 0;   // vidx found to be on the front.
    }   // end for
    return 1;
}   // end testMembership

}   // end namespace


// public
size_t RegionSelector::update( int svtx, const Vec3f& c, float nrad)
{
    assert( svtx >= 0);
    assert( !_mesh.faces(svtx).empty());
    _cf = *_mesh.faces(svtx).begin();
    const int* fvidxs = _mesh.fvidxs(_cf);
    _cval = calcBarycentric( _mesh.vtx(fvidxs[0]), _mesh.vtx(fvidxs[1]), _mesh.vtx(fvidxs[2]), c);
    _body.clear();
    _front->clear();
    _front->insert( svtx);

    nrad = std::max(0.0f, nrad);
    const float R = nrad < sqrtf( FLT_MAX) ? nrad*nrad : nrad;
    const Vec3f ov = centre();

    IntSet* nfront = new IntSet;
    IntSet cfront = *_front; // Front vertices changed in the last iteration
    while ( !cfront.empty())
    {
        int fvidx = *cfront.begin();    // Get the next vertex from the front.
        cfront.erase(fvidx);
        const int vflag = testMembership( fvidx, _mesh, ov, R);

        if ( vflag == -1)
        {
            // fvidx is outside the radius threshold so all of its connected vertices
            // *** that are marked as being in the body ***
            // now need to be considered in subsequent loop iterations as potential front vertices.
            nfront->erase(fvidx);
            for ( int cv : _mesh.cvtxs(fvidx))
            {
                if ( _body.count(cv) > 0)
                {
                    _body.erase(cv);
                    cfront.insert(cv);
                    nfront->insert(cv);
                }   // end if
            }   // end for

            // If the new front is now empty, the set radius was too small to retain even a single
            // seed vertex (which is necessary).
            if ( nfront->empty() && cfront.empty())
            {
                nrad = (_mesh.vtx(fvidx) - ov).norm();
                //std::cerr << "Final radius = " << nrad << std::endl;
                nfront->insert(fvidx);
            }   // end if
        }   // end if
        else
        {
            if ( vflag == 0)
            {
                nfront->insert(fvidx);  // fvidx will continue to be a front vertex.
                // Adjustments in the boundary that do not cause fvidx to stop being a front vertex, but that may
                // affect the state of its connected vertices (that are currently marked as front vertices) are
                // addressed when the loop gets around to those front vertices.
            }   // end if
            else if ( vflag == 1)
            {
                // fvidx is in the body now and so all of its connected vertices
                // *** that aren't in the body now ***
                // need to be considered as potential front vertices.
                nfront->erase(fvidx);
                _body.insert(fvidx);
            }   // end else if

            for ( int cv : _mesh.cvtxs(fvidx))
            {
                if ( _body.count(cv) == 0 && nfront->count(cv) == 0)
                {
                    cfront.insert(cv);
                    nfront->insert(cv);
                }   // end if
            }   // end for
        }   // end else
    }   // end while

    delete _front;
    _front = nfront;
    _rad = nrad;

    return _front->size() + _body.size();
}   // end update


namespace {

// Gets the next vertex in the set that's connected to v AND is most distant from ov.
int getNextVertexInSet( const Mesh& cmesh, const Vec3f& ov, const IntSet& fvidxs, int v)
{
    const IntSet& cvs = cmesh.cvtxs(v);
    v = -1;
    float maxd = 0;
    for ( int cv : cvs)
    {
        if ( fvidxs.count(cv) > 0)
        {
            const float rval = (cmesh.vtx(cv) - ov).squaredNorm();
            if ( rval > maxd)
            {
                v = cv;
                maxd = rval;
            }   // end if
        }   // end if
    }   // end for
    return v;
}   // end getNextVertexInSet

}   // end namespace


size_t RegionSelector::selectedFaces( IntSet& cfids, const IntSet *onlyIn) const
{
    cfids.clear();

    if ( onlyIn)
    {
        for ( int cv : _body)
            for ( int fid : _mesh.faces(cv))
                if ( onlyIn->count(fid) > 0)
                    cfids.insert(fid);
    }   // end if
    else
    {
        for ( int cv : _body)
        {
            const IntSet& fcs = _mesh.faces(cv);
            std::for_each(std::begin(fcs), std::end(fcs), [&](int x){cfids.insert(x);});
        }   // end for
    }   // end else

    return cfids.size();
}   // end selectedFaces
