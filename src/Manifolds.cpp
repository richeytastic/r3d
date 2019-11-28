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

#include <Manifolds.h>
#include <FaceParser.h>
#include <algorithm>
#include <iomanip>
#include <cassert>
using r3d::Manifolds;
using r3d::FaceParser;
using r3d::Boundaries;
using r3d::Manifold;
using r3d::Mesh;
using r3d::Face;
using r3d::Vec2i;
using r3d::Vec3f;


namespace {

void createEdgeSet( const IntSet& mset, IntSet& eset, const Mesh* mesh)
{
#ifndef NDEBUG
    std::unordered_map<int, IntSet> ecounts;    // For checking that mset actually is a manifold
#endif
    for ( int fid : mset)
    {
        const Face& f = mesh->face(fid);
        const int e0 = mesh->edgeId( f[0], f[1]);
        const int e1 = mesh->edgeId( f[1], f[2]);
        const int e2 = mesh->edgeId( f[2], f[0]);

#ifndef NDEBUG
        ecounts[e0].insert(fid);
        ecounts[e1].insert(fid);
        ecounts[e2].insert(fid);
#endif

        if ( eset.count(e0) == 0)
            eset.insert(e0);
        else
            eset.erase(e0);

        if ( eset.count(e1) == 0)
            eset.insert(e1);
        else
            eset.erase(e1);

        if ( eset.count(e2) == 0)
            eset.insert(e2);
        else
            eset.erase(e2);
    }   // end for

#ifndef NDEBUG
    int errId = -1;
    for ( const auto& p : ecounts)
    {
        if ( p.second.size() > 2)
        {
            errId = p.first;
            std::cerr << "[ERROR] More than two faces attached to manifold edge " << p.first << ":" << std::endl;
            for ( int fid : p.second)
            {
                const Face& f = mesh->face(fid);
                std::cerr << "    " << std::setw(6) << std::right << f << std::endl;
            }   // end for
        }   // end if
    }   // end for
    assert( errId < 0);
#endif
}   // end createEdgeSet


struct BoundaryEdger2 : r3d::BoundaryParser
{
    explicit BoundaryEdger2( const IntSet& faces) : _faces(faces) {}

    bool parseEdge( int fid, const Vec2i& e, int& pnfid) override
    {
        const int v0 = e[0];
        const int v1 = e[1];
        bool crossEdge = false;
        // Only cross if ns == 2 and the face on the opposite edge is in the provisional
        // set and one of its opposite edges is shared by no other faces in the provisional manifold set.
        if ( mesh->nsfaces( v0, v1) == 2)
        {
            // Work out whether we should enter pnfid next (cross v0-->v1) based on how
            // edge connected pnfid is to the set of provisional faces. If face 
            // pnfid is connected on both its other edges to faces in _faces then we
            // don't cross to it meaning that the final set of faces parsed for
            // removal from the provisional set will not include pnfid (but will include fid).
            pnfid = mesh->oppositeFace( fid, v0, v1);
            if ( _faces.count( pnfid) > 0)
            {
                assert( pnfid >= 0);
                const Face& pnfc = mesh->face(pnfid);
                const int v2 = pnfc.opposite( v0, v1);
                crossEdge = !_testFarEdgeMembership( pnfid, v2, v0) || !_testFarEdgeMembership( pnfid, v2, v1);
            }   // end if
        }   // end if
        assert( !crossEdge || pnfid >= 0);
        return crossEdge;
    }   // end parseEdge

private:
    const IntSet& _faces;

    bool _testFarEdgeMembership( int fc, int v0, int v1) const
    {
        for ( int f : mesh->sfaces( v0, v1))  // Get shared faces from one edge
        {
            if ( f != fc && _faces.count(f) > 0) // Edge connects to a face in the provisional set?
                return true;
        }   // end for
        return false;
    }   // end _testFarEdgeMembership
};  // end struct


struct BoundaryEdger : r3d::BoundaryParser
{
    BoundaryEdger() : /*_debug(false),*/ _faces(nullptr) {}

    //void setDebug( bool d) { _debug = d;}
    size_t numFacesRemain() const { return _rfaces.size();}
    int nextSeedFace() const { return *_rfaces.begin();}
    void reset() override { _rfaces = mesh->faces();}


    bool parseEdge( int fid, const Vec2i& e, int& pnfid) override   // Called multiple times for fid
    {
        const int eid = mesh->edgeId( e[0], e[1]);
#ifndef NDEBUG
        /*
        if ( _debug)
            std::cerr << "BoundaryEdger::parseEdge( FaceId=" << fid << ", EdgeId=" << eid << ")" << std::endl;
        */
        if ( _rfaces.count(fid) == 0)  // Shouldn't happen
            std::cerr << " *** Face " << fid << " not found in remaining set!" << std::endl;
#endif
        assert( _rfaces.count(fid) > 0);
        // Set this face (fid) in the provisional manifold set. It will only be removed from _rfaces after cleaning.
        _faces->insert(fid);

        // Get only the faces shared on this edge that still remain to be set in a manifold (excluding fid).
        IntSet sfids = mesh->sfaces(e[0], e[1]);    // Copy out
        auto it = sfids.begin();
        while ( it != sfids.end())
        {
            if ( _rfaces.count(*it) == 0 || _faces->count(*it) > 0) // Erase if already in this manifold, or seen previously.
                it = sfids.erase(it);
            else
                ++it;
        }   // end while

        pnfid = -1;

        // sfids now only contains face ids not previously found on other manifolds
        // Cross edge only if it's shared by exactly one other face.
        const size_t ns = sfids.size();
        if ( ns == 1)
        {
            pnfid = *sfids.begin();
            assert( _rfaces.count( pnfid) > 0);
/*
#ifndef NDEBUG
            if ( _debug)
                std::cerr << " Specified face to be parsed next = " << pnfid << std::endl;
#endif
*/
        }   // end if

        if ( mesh->nsfaces( eid) > 2)
        {
            // Once parsing is complete, faces not in the parsed set that
            // are attached to the problem edges are discounted from the faces attached
            // to those edges to reassess if the edge should be treated as genuine.
            if ( _pedges[eid] == nullptr)
                _pedges[eid] = new IntSet;
            _pedges[eid]->insert(fid);   // Will only need to be looked at further if # of faces attached to edge eid > 2.
/*
#ifndef NDEBUG
            if ( _debug)
                std::cerr << " Added face " << fid << " to edge " << eid << std::endl;
#endif
*/
        }   // end if

        return pnfid >= 0;
    }   // end parseEdge


    void setNewManifold( IntSet* faces)//, IntSet* edges)
    {
        _faces = faces;
        assert( _faces);
        assert( _faces->empty());
        assert( _pedges.empty());
    }   // end setNewManifold


    // Call after parsing complete with the provisional set of manifold faces.
    // Returns the number of faces in the final manifold.
    size_t cleanProvisionalManifold()
    {
        IntSet srfids;  // Seed faces to remove from the provisional manifold

        // Go through the problem edges to check the number of manifold faces attached.
        // Only edges with more than 2 manifold faces present potential problems.
        while ( !_pedges.empty())
        {
            auto it = _pedges.begin();
            IntSet* efids = it->second;
            // If only two faces in the manifold, then no problem (and not an edge).
            if ( efids->size() > 2)
            {
                // The edge has more than 2 faces in the provisional manifold, but we can only have 2.
                // For each face attached to the edge, score its flatness with respect to the other
                // triangles it's connected to on its adjacent sides. Only the two triangles with the
                // best flatness metrics are kept. The others form a second set of seed faces which
                // will be removed from the first.
                const r3d::Edge& edge = mesh->edge( it->first);
                _setFlattestFacePair( edge[0], edge[1], *efids);
                for ( int f : *efids)
                    srfids.insert( f);
            }   // end if
            delete efids;
            _pedges.erase(it);
        }   // end while

        // Remove selected faces from the parsed manifold.
        while ( !srfids.empty())
        {
            int sfid = *srfids.begin();
            srfids.erase(sfid);
            if ( _faces->count(sfid) > 0)
                _removeNonManifoldFaces( sfid);
        }   // end while

        for ( int f : *_faces)
            _rfaces.erase(f);

        return _faces->size();
    }   // end cleanProvisionalManifold

private:
    //bool _debug;
    IntSet* _faces;  // Manifold faces
    IntSet _rfaces;  // Remaining faces on mesh to find
    std::unordered_map<int, IntSet*> _pedges;   // Problem edges mapped to provisional manifold faces


    void _removeNonManifoldFaces( int f)
    {
        FaceParser m2parser(*mesh);
        BoundaryEdger2 edger( *_faces);
        m2parser.setBoundaryParser( &edger);
        m2parser.parse( f);
        const IntSet& remset = m2parser.parsed();
        // Remove all the parsed faces from _faces
        for ( int fid : remset)
            _faces->erase( fid);
    }   // end _removeNonManifoldFaces


    void _setFlattestFacePair( int v0, int v1, IntSet& efids)
    {
        double bmetric0 = -DBL_MAX;    // First best flatness score
        double bmetric1 = -DBL_MAX;    // Second best flatness score
        int bi0 = -1;  // The most flat triangle
        int bi1 = -1;  // The second most flat triangle

        Vec3f u10 = mesh->uvtx(v1) - mesh->uvtx(v0);    // The edge vector
        u10.normalize();

        for ( int f : efids)
        {
            double fmetric = 0;
            int cnt = 0;

            const int vi = mesh->face(f).opposite(v0, v1);    // Vertex opposite edge v0-->v1 on triangle fid.

            Vec3f ui0 = mesh->uvtx(vi) - mesh->uvtx(v0);
            ui0.normalize();

            const Vec3f ufidi = ui0.cross(u10); // Normal vector for triangle fid weighted by area

            // If triangle fid has adjacent triangles on either of its two opposite edges (i.e. v0-->vi, or v1-->vi),
            // then calculate the norms of these triangles to compare against the norm of triangle fid for a flatness
            // metric being the average dot product of the target triangle's norm with these two neighbours' norms.
            // Target triangles without neighbours on the opposite edges have a flatness score of zero since they can't
            // be reached except through the problem edge and therefore should definitely not remain in the manifold.
            
            if ( mesh->nsfaces( v0, vi) == 2)
            {
                const int vj = mesh->face( mesh->oppositeFace( f, v0, vi)).opposite(v0, vi);
                Vec3f uj0 = mesh->uvtx(vj) - mesh->uvtx(v0);
                uj0.normalize();
                const Vec3f ufidj = uj0.cross(ui0);
                fmetric = ufidj.dot(ufidi);
                cnt++;
            }   // end if

            if ( mesh->nsfaces( v1, vi) == 2)
            {
                const int vk = mesh->face( mesh->oppositeFace( f, v1, vi)).opposite(v1, vi);
                Vec3f uki = mesh->uvtx(vk) - mesh->uvtx(vi);
                uki.normalize();
                Vec3f u1i = mesh->uvtx(v1) - mesh->uvtx(vi);
                u1i.normalize();
                const Vec3f ufidk = uki.cross(u1i);
                fmetric += ufidk.dot(ufidi);
                cnt++;
            }   // end if

            if ( cnt > 1)
                fmetric /= cnt;

            if ( fmetric > bmetric0)
            {
                // Make the first flattest the second flattest
                bi1 = bi0;
                bmetric1 = bmetric0;
                // Set new flattest
                bmetric0 = fmetric;
                bi0 = f;
            }   // end if
            else if ( fmetric > bmetric1)
            {
                bmetric1 = fmetric;
                bi1 = f;
            }   // end else if
        }   // end for

        // Finally, ensure the chosen two flattest triangles for the manifold are removed from efids.
        assert( bi0);
        assert( bi1);
        efids.erase(bi0);
        efids.erase(bi1);
    }   // end _setFlattestFacePair
};  // end struct

}   // end namespace


Manifolds::Manifolds( const Mesh& mesh) : _mesh( &mesh)
{
    FaceParser mparser( mesh); // For parsing the mesh manifolds
    BoundaryEdger boundaryEdger;
    mparser.setBoundaryParser( &boundaryEdger);

    while ( boundaryEdger.numFacesRemain() > 0)
    {
        _manfs.push_back( new Manifold);
        Manifold& m = *_manfs.back();
        m._mesh = &mesh;
        boundaryEdger.setNewManifold( &m._faces);
        mparser.parse( boundaryEdger.nextSeedFace());
        boundaryEdger.cleanProvisionalManifold();
        createEdgeSet( m._faces, m._edges, &mesh);
    }   // end while

    // Sort manifolds in descending order of number of faces.
    std::sort( std::begin(_manfs), std::end(_manfs),
            []( const Manifold* m0, const Manifold* m1){return m1->faces().size() < m0->faces().size();});

    // Set reverse lookup for face IDs to manifold ID
    const int n = static_cast<int>(_manfs.size());
    for ( int i = 0; i < n; ++i)
        for ( int f : _manfs.at(size_t(i))->faces())
            _face2manf[f] = i;
}   // end ctor


Manifolds::Ptr Manifolds::create( const Mesh& m)
{
    return Ptr( new Manifolds(m), [](Manifolds* d){ delete d;});
}   // end create


Manifolds::Ptr Manifolds::deepCopy() const
{
    Ptr omc( new Manifolds, [](Manifolds* d){ delete d;});
    *omc = *this;
    return omc;
}   // end deepCopy

// private
Manifolds::Manifolds() {}

// private
Manifolds::Manifolds( const Manifolds& m) { *this = m;}

// private
Manifolds& Manifolds::operator=( const Manifolds& m)
{
    _mesh = m._mesh;
    const int n = int(m.count());
    _manfs.resize(n);
    for ( int i = 0; i < n; ++i)
        _manfs[i] = new Manifold( *m.manifold(i));
    _face2manf = m._face2manf;
    return *this;
}   // end operator=


// private
Manifolds::~Manifolds()
{
    for ( Manifold* m : _manfs)
        delete m;
}   // end dtor


const Manifold* Manifolds::manifold( int i) const
{
    if ( i < 0 || i >= int(_manfs.size()))
        return nullptr;
    return _manfs.at(size_t(i));
}   // end manifold


// private
Manifold::Manifold() : _mesh(nullptr) {}


// private
Manifold::Manifold( const Manifold& m) { *this = m;}


// private
Manifold& Manifold::operator=( const Manifold& m)
{
    _mesh = m._mesh;
    _faces = m._faces;
    _edges = m._edges;
    _bnds = m._bnds;
    return *this;
}   // end operator=


const IntSet& Manifold::vertices() const
{
    if ( _verts.empty())
    {
        for ( int fid : _faces)
        {
            const int *f = _mesh->fvidxs(fid);
            _verts.insert(f[0]);
            _verts.insert(f[1]);
            _verts.insert(f[2]);
        }   // end for
    }   // end if
    return _verts;
}   // end vertices


const Boundaries& Manifold::boundaries() const
{
    if ( _bnds.count() == 0 && !_edges.empty())
    {// Find the boundaries in this manifold
#ifndef NDEBUG
        int nbs = _bnds.sort( _mesh, _edges);
        assert( nbs >= 0);
#else
        _bnds.sort( _mesh, _edges);
#endif
    }   // end if
    return _bnds;
}   // end boundaries


int Manifolds::manifoldId( int f) const
{
    return _face2manf.count(f) > 0 ? _face2manf.at(f) : -1;
}   // end manifoldId


Mesh::Ptr Manifolds::reduceManifolds( int n) const
{
    Mesh::Ptr nmod = Mesh::create();
    nmod->copyInMaterials( *_mesh);

    int v0, v1, v2, nfid, mid;
    n = std::max( 1, std::min( n, int(count())));
    for ( int i = 0; i < n; ++i)
    {
        const Manifold* sman = manifold(i); // Source manifold on this object

        // Add vertex remapped faces
        std::unordered_map<int,int> vvmap;  // Old to new vertices
        for ( int fid : sman->_faces)
        {
            const Face& f = _mesh->face(fid);
            v0 = nmod->addVertex(_mesh->uvtx(f[0]));
            v1 = nmod->addVertex(_mesh->uvtx(f[1]));
            v2 = nmod->addVertex(_mesh->uvtx(f[2]));

            vvmap[f[0]] = v0;
            vvmap[f[1]] = v1;
            vvmap[f[2]] = v2;

            nfid = nmod->addFace( v0, v1, v2);
            mid = _mesh->faceMaterialId(fid);
            if ( mid >= 0)
            {
                const Vec2f& uva = _mesh->faceUV( fid, 0);
                const Vec2f& uvb = _mesh->faceUV( fid, 1);
                const Vec2f& uvc = _mesh->faceUV( fid, 2);
                nmod->setOrderedFaceUVs( mid, nfid, uva, uvb, uvc);
            }   // end if
        }   // end for - faces
    }   // end for - manifolds

    nmod->setTransformMatrix( _mesh->transformMatrix());
    return nmod;
}   // end reduceManifolds
