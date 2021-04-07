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

#include <Mesh.h>
#include <Image.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <cassert>
#include <algorithm>
using r3d::Mesh;
using r3d::Face;
using r3d::Edge;
using r3d::Material;
using r3d::Vec2i;
using r3d::Vec2f;
using r3d::Vec3f;
using r3d::Vec3d;
using r3d::Mat4f;
using r3d::MatX3f;
using r3d::MatX6f;
using r3d::FaceMat;


namespace {
static const IntSet EMPTY_INT_SET;
static const size_t HASH_NDP = 4;   // KEEP THIS AS 4!!!!
}   // end namespace


size_t Mesh::numTextureEdges( int va, int vb) const
{
    const IntSet &sfids = sfaces( va, vb);
    if ( sfids.empty())
        return 0;

    size_t ntex = 0;
    IntSet tset;    // Records the texture vertices
    for ( int fid : sfids)
    {
        const int *uvs = faceUVs( fid);
        if ( !uvs)
            continue;

        const Face &fc = face(fid);

        int a = 0;  // First edge vertex index
        int b = 1;  // Second edge vertex index
        if ( fc[0] == va)
        {
            a = 0;
            b = fc[1] == vb ? 1 : 2;
        }   // end if
        else if ( fc[1] == va)
        {
            a = 1;
            b = fc[0] == vb ? 0 : 2;
        }   // end else if
        else
        {
            a = 2;
            b = fc[0] == vb ? 0 : 1;
        }   // end else if

        assert( fc[a] == va);
        assert( fc[b] == vb);

        // fc[a] == va and fc[b] == vb, so uvs[a] and uvs[b] is the corresponding texture edge.
        // If either of the texture vertices are new, it counts as a new texture edge.
        if ( tset.count(uvs[a]) == 0 || tset.count(uvs[b]) == 0)
            ntex++;
        tset.insert(uvs[a]);
        tset.insert(uvs[b]);
    }   // end for

    return ntex;
}   // end numTextureEdges


Mesh::Ptr Mesh::deepCopy() const { return Ptr( new Mesh(*this), [](Mesh *x){delete x;});}


Mesh::Ptr Mesh::repackedCopy() const
{
    Mesh *m = new Mesh;
    m->_tmat = _tmat;
    m->_imat = _imat;

    std::unordered_map<int,int> vvmap;  // Old to new vertex IDs
    for ( int vid : _vids)
        vvmap[vid] = m->addVertex( _vtxs.at(vid));

    std::unordered_map<int,int> ffmap;  // Old to new face IDs
    for ( int fid : _fids)
    {
        const int *f = fvidxs(fid);
        ffmap[fid] = m->addFace( vvmap[f[0]], vvmap[f[1]], vvmap[f[2]]);
    }   // end for

    m->copyInMaterials( *this);   // Note that material IDs don't need to be sequential
    for ( const auto &p : _f2m)
    {
        const Vec2f &uv0 = faceUV(p.first, 0);
        const Vec2f &uv1 = faceUV(p.first, 1);
        const Vec2f &uv2 = faceUV(p.first, 2);
        m->setOrderedFaceUVs( p.second, ffmap[p.first], uv0, uv1, uv2);
    }   // end for

    return Ptr( m, [](Mesh *x){delete x;});
}   // end repackedCopy


// public static
Mesh::Ptr Mesh::create()
{
    return Ptr( new Mesh, [](Mesh *x){delete x;});
}   // end create


// public static
Mesh::Ptr Mesh::fromVertices( const MatX3f &vrows)
{
    Mesh *m = new Mesh;
    const int n = int(vrows.rows());
    for ( int i = 0; i < n; ++i)
        m->addVertex( vrows.row(i));
    return Ptr( m, [](Mesh *x){delete x;});
}   // end fromVertices


// public static
Mesh::Ptr Mesh::fromVertexSubset( const Mesh &src, const IntSet &vidxs)
{
    Mesh *m = new Mesh;
    for ( int vid : vidxs)
        m->addVertex( src.uvtx(vid));
    m->setTransformMatrix( src.transformMatrix());
    return Ptr( m, [](Mesh *x){delete x;});
}   // end fromVertices


Mesh::Mesh() : _vCounter(0), _fCounter(0), _eCounter(0), _mCounter(0), _tmat(Mat4f::Identity()), _imat(Mat4f::Identity()) {}


Mesh::~Mesh() {}


void Mesh::setTransformMatrix( const Mat4f &tmat)
{
    if ( tmat != _tmat)
    {
        _tvtxs.clear(); // Will force recalculations of vertex positions
        _tmat = tmat;
        _imat = tmat.inverse();
    }   // end if
}   // end setTransformMatrix


void Mesh::addTransformMatrix( const Mat4f &tmat)
{
    setTransformMatrix( tmat * _tmat);
}   // end addTransformMatrix


void Mesh::fixTransformMatrix()
{
    for ( int vidx : _vids)
    {
        Vec3f &vec = _vtxs[vidx];   // The vertex to modify
        const size_t h = hash(vec, HASH_NDP);
        assert( _v2id.count(h) > 0);
        _v2id.erase( h);    // Remove original vertex hash value
        vec = vtx(vidx); // Ensures transform against existing matrix is done.
        _v2id[ hash( vec, HASH_NDP)] = vidx;  // Hash back with new vertices
    }   // end for

    _imat = _tmat = Mat4f::Identity();  // NB not necessary to clear vertex cache _tvtxs
}   // end fixTransformMatrix


bool Mesh::hasFixedTransform( float precision) const
{
    return _tmat.isIdentity( precision);
}   // end hasFixedTransform


const Vec3f &Mesh::vtx( int vidx) const
{
    assert( _vtxs.count(vidx) > 0);
    if ( _tvtxs.count(vidx) == 0)   // Refresh transform?
    {
        Vec4f hvec;
        hvec << _vtxs.at(vidx), 1;
        _tvtxs[vidx] = (_tmat * hvec).segment<3>(0);    // _tvtxs is mutable
    }   // end if
    return _tvtxs.at(vidx);
}   // end vtx


int Mesh::faceMaterialId( int fid) const
{
    assert( _fids.count(fid) > 0);
    if ( _f2m.count(fid) == 0) // No material for this face
        return -1;
    return _f2m.at(fid);
}   // end faceMaterialId


const IntSet &Mesh::materialFaceIds( int mid) const
{
    if ( _mats.count(mid) > 0)
        return _mats.at(mid)._fids;
    return EMPTY_INT_SET;
}   // end materialFaceIds


int Mesh::addMaterial( const cv::Mat &m, size_t maxd)
{
    if ( m.empty())
        return -1;
    const int mid = (int)_mCounter++;
    _addMaterial( mid, m, maxd);
    return mid;
}   // end addMaterial


void Mesh::_addMaterial( int mid, const cv::Mat &m, size_t maxd)
{
    _mids.insert( mid);
    _mats[mid]._tx = shrink2Max( m, maxd);
}   // end _addMaterial


void Mesh::removeMaterial( int mid)
{
    _mids.erase(mid);
    _mats.erase(mid);
}   // end removeMaterial


void Mesh::removeAllMaterials()
{
    while ( !_mats.empty())
        removeMaterial( _mats.begin()->first);
    _mCounter = 0;
}   // end removeAllMaterials


void Mesh::copyInMaterials( const Mesh &omc)
{
    const IntSet &matIds = omc.materialIds();
    for ( int mid : matIds)
    {
        const cv::Mat tx = omc.texture(mid);
        _addMaterial( mid, tx, size_t(std::max(tx.rows, tx.cols)));
    }   // end for
    _mCounter = omc._mCounter;
}   // end copyInMaterials


namespace {
// Calculate a new UV coordinate from an old UV.
void calcNewUV( Vec2f &uv, int nrows, int ncols, const std::vector<int> &scols, int i)
{
    // v is unchanged, only u affected
    const float oldWidth = static_cast<float>((i == (int)(scols.size())-1) ? ncols - scols[i] : scols[i+1] - scols[i]);
    uv[0] = (scols[i] + (uv[0] * oldWidth))/ncols;
}   // end calcNewUV
}   // end namespace


size_t Mesh::mergeMaterials()
{
    if ( _mats.size() <= 1)
        return _mats.size();

    const IntSet mids = materialIds();   // Copied out because changing
    const int nmats = (int)mids.size();

    // Concatenate images across all of the materials into a single texture image.
    std::vector<cv::Mat> txs; // Textures from all materials
    std::vector<int> midSeq;  // Repeatable sequence of material IDs
    for ( int mid : mids)
    {
        midSeq.push_back(mid);
        cv::Mat tx  = texture(mid);
        if ( !tx.empty())
            txs.push_back(tx);
    }   // end for

    std::vector<int> scols; // The starting columns for the concatenated texture images
    const cv::Mat tx = concatHorizontalMax( txs, &scols);
    const int nrows = tx.rows;
    const int ncols = tx.cols;

    const int mmid = addMaterial( tx); // Create the new "merge" material

    for ( int i = 0; i < nmats; ++i)
    {
        const int mid = midSeq[i];
        // Map all the faces from material m to mmat
        const IntSet &fids = materialFaceIds( mid);
        assert( !fids.empty());
        for ( int fid : fids)
        {
            // Get and set the new texture offsets for the face based on
            // the horizontal concatentation of the texture images.
            const int *uvidxs = faceUVs(fid);

            Vec2f uv0 = uv(mid, uvidxs[0]);
            Vec2f uv1 = uv(mid, uvidxs[1]);
            Vec2f uv2 = uv(mid, uvidxs[2]);

            // Calculate new offsets
            calcNewUV( uv0, nrows, ncols, scols, i);
            calcNewUV( uv1, nrows, ncols, scols, i);
            calcNewUV( uv2, nrows, ncols, scols, i);

            // Set in the merged material
            setOrderedFaceUVs( mmid, fid, uv0, uv1, uv2);
        }   // end for

        removeMaterial( mid);   // Remove the old material
    }   // end for

    return nmats;   // The number of materials that were merged.
}   // end mergeMaterials


const cv::Mat &Mesh::texture( int mid) const
{
    assert( _mats.count(mid) > 0);
    return _mats.at(mid)._tx;
}   // end texture


void Mesh::_removeFaceUVs( int mid, int fid)
{
    assert( materialIds().count( mid) > 0);
    _mats.at(mid).removeFaceUVs(fid);
    _f2m.erase(fid);
}   // end _removeFaceUVs


int Mesh::addVertex( float x, float y, float z) { return addVertex( Vec3f(x,y,z));}


int Mesh::_addCheckedVertex( const Vec3f &v, size_t hkey)
{
    if ( _v2id.count( hkey) > 0)
        return _v2id.at(hkey);

    const int vi = (int)_vCounter++;

    _v2id[hkey] = vi;
    _vids.insert(vi);
    _vtxs[vi] = v;

    return vi;
}   // end _addCheckedVertex


bool Mesh::_checkNewVertex( const Vec3f &V, Vec3f &v) const
{
    if ( V.array().isNaN().any())
    {
#ifndef NDEBUG
        std::cerr << "[ERROR] r3d::Mesh::_checkNewVertex: vertex is NaN!" << std::endl;
#endif
        return false;
    }   // end if

    if ( hasFixedTransform())
        v = V;
    else
    {
        v = transform( _imat, V);
#ifndef NDEBUG
        std::cerr << "[WARNING] r3d::Mesh::_checkNewVertex: vertex added to transformed mesh!" << std::endl;
#endif
    }   // end if
    return true;
}   // end _checkNewVertex


int Mesh::addVertex( const Vec3f &V)
{
    Vec3f v;
    if ( !_checkNewVertex( V, v))
        return -1;
    return _addCheckedVertex( v, hash( v, HASH_NDP));
}   // end addVertex


bool Mesh::removeVertex( int vi)
{
    assert( _vids.count(vi) > 0);
    assert(_v2v.count(vi) == 0);
    assert(_v2e.count(vi) == 0);
    assert(_v2f.count(vi) == 0);

    // Check that this vertex is no longer connected to others by edges/faces.
    const IntSet &cvs = cvtxs(vi);
    if ( !cvs.empty())
        return false;

    const size_t key = hash( _vtxs.at(vi), HASH_NDP);
    assert(_v2id.count(key) > 0);
    _v2id.erase(key);
    _vids.erase(vi);
    _vtxs.erase(vi);
    _tvtxs.erase(vi);

    return true;
}   // end removeVertex


Vec2f Mesh::calcTextureCoords( int fid, const Vec3f &p) const
{
    const int matId = faceMaterialId( fid);
    assert( matId >= 0);
    if ( matId < 0)
        return Vec2f(-1,-1);

    const int *vidxs = fvidxs(fid);
    const Vec3f &v0 = uvtx(vidxs[0]);
    const Vec3f &v1 = uvtx(vidxs[1]);
    const Vec3f &v2 = uvtx(vidxs[2]);

    // Calculate and return the barycentric coordinates for the point
    Vec3f bcds = calcBarycentric( v0, v1, v2, p);

    // Get the UV coordinates corresponding to the triangle's vertices
    const int *uvs = faceUVs( fid);
    const Vec2f &t0 = uv( matId, uvs[0]);
    const Vec2f &t1 = uv( matId, uvs[1]);
    const Vec2f &t2 = uv( matId, uvs[2]);

    return bcds[0]*t0 + bcds[1]*t1 + bcds[2]*t2;
}   // end calcTextureCoords


bool Mesh::adjustRawVertex( int vidx, float x, float y, float z) { return adjustRawVertex( vidx, Vec3f(x,y,z));}


bool Mesh::adjustRawVertex( int vidx, const Vec3f &v)
{
    assert( _vids.count(vidx) > 0);
    if ( v.array().isNaN().any())
        return false;

    Vec3f &vec = _vtxs[vidx];          // The vertex to modify
    size_t h = hash(vec, HASH_NDP);    // Existing hash
    assert( _v2id.count(h) > 0);
    _v2id.erase( h); // Remove original vertex hash value

    vec = v; // Update with new position of vertex
    h = hash(vec, HASH_NDP);    // The new hash value

    _v2id[h] = vidx;  // Hash back with new vertices
    _tvtxs.erase(vidx); // Force recalculation of cached vertex

    return true;
}   // end adjustRawVertex


void Mesh::swapVertexPositions( int v0, int v1)
{
    assert( _vids.count(v0) > 0);
    assert( _vids.count(v1) > 0);
    Vec3f t0 = _vtxs.at(v0);
    Vec3f t1 = _vtxs.at(v1);
    adjustRawVertex( v0, t1);
    adjustRawVertex( v1, t0);
}   // end swapVertexPositions


bool Mesh::scaleVertex( int vidx, float sf)
{
    assert( _vids.count(vidx) > 0);
    if ( cvIsNaN( sf))
        return false;

    if ( !hasFixedTransform())
        std::cerr << "[WARNING] r3d::Mesh::scaleVertex: transform is not Identity!" << std::endl;

    Vec3f &vec = _vtxs[vidx];   // The vertex to modify
    const size_t h = hash(vec, HASH_NDP);
    assert( _v2id.count(h) > 0);
    _v2id.erase( h);    // Remove original vertex hash value
    vec = sf*vec; // Update with new position of vertex
    _v2id[ hash( vec, HASH_NDP)] = vidx;  // Hash back with new vertices
    _tvtxs.erase(vidx); // Force recalculation of cached vertex

    return true;
}   // end scaleVertex


bool Mesh::hasEdge( int v0, int v1) const
{
    return _v2v.count(v0) > 0 ? _v2v.at(v0).count(v1) > 0 : false;
}   // end hasEdge


int Mesh::edgeId( int v0, int v1) const
{
    const Edge ledge(v0,v1); // Lookup edge
    if ( _e2id.count(ledge) == 0)
        return -1;
    return _e2id.at(ledge);
}   // end edgeId


int Mesh::edgeId( const Vec2i &e) const { return edgeId( e[0], e[1]);}


int Mesh::_connectEdge( int v0, int v1)
{
    assert( v0 != v1);
    if ( hasEdge(v0,v1))
        return edgeId(v0,v1);
    const int eidx = (int)_eCounter++;
    _connectEdge( eidx, v0, v1);
    return eidx;
}   // end _connectEdge


void Mesh::_connectEdge( int ei, int v0, int v1)
{
    const Edge &e = _edges[ei] = Edge( v0, v1);  // e[0] < e[1]
    _e2id[e] = ei;  // Reverse lookup
    _eids.insert(ei);
    _v2v[v0].insert(v1);
    _v2v[v1].insert(v0);
    _v2e[v0].insert(ei);
    _v2e[v1].insert(ei);
}   // end _connectEdge


void Mesh::_removeEdge( int ei)
{
    const Edge &e = _edges[ei];

    _v2v[e[0]].erase(e[1]);
    if ( _v2v[e[0]].empty())
        _v2v.erase(e[0]);

    _v2v[e[1]].erase(e[0]);
    if ( _v2v[e[1]].empty())
        _v2v.erase(e[1]);

    _v2e[e[0]].erase(ei);
    if ( _v2e[e[0]].empty())
        _v2e.erase(e[0]);

    _v2e[e[1]].erase(ei);
    if ( _v2e[e[1]].empty())
        _v2e.erase(e[1]);

    if ( _e2f[ei].empty())
        _e2f.erase(ei);

    _e2id.erase(e);
    _eids.erase(ei);
    _edges.erase(ei);
}   // end _removeEdge


int Mesh::addEdge( int v0, int v1)
{
    if ( hasEdge(v0,v1))
        return edgeId(v0,v1);

    // If there are vertices T currently connected to v0 that are also connected to v1,
    // then connecting v0 and v1 directly will create new faces between the
    // currently connected vertices T and v0 and v1.
    const IntSet &u0Conn = cvtxs( v0);
    const IntSet &u1Conn = cvtxs( v1);
    std::vector<int> ucs;

    // Iterate over smaller set
    const IntSet *uset = &u0Conn;
    const IntSet *sset = &u1Conn;
    if ( u1Conn.size() < u0Conn.size())
    {
        uset = &u1Conn;
        sset = &u0Conn;
    }   // end if
    for ( int ui : *uset)
    {
        if ( sset->count(ui) > 0)
            ucs.push_back(ui);
    }   // end foreach

    const int ei = _connectEdge( v0, v1);
    // Create the face(s)
    for ( int v : ucs)
        addFace( v, v0, v1);
    return ei;
}   // end addEdge


int Mesh::face( int v0, int v1, int v2) const
{
    if ( hasEdge(v0,v1) && hasEdge(v1,v2) && hasEdge(v2,v0))
    {
        const int e01 = edgeId(v0,v1);
        const int e02 = edgeId(v0,v2);
        const IntSet &fs0 = _e2f.at(e01);
        const IntSet &fs1 = _e2f.at(e02);

        // Look for the common face by iterating over the smaller set
        if ( fs0.size() < fs1.size())
        {
            for ( int fc : fs0)
            {
                if ( fs1.count(fc) > 0)
                    return fc;
            }   // end for
        }   // end if
        else
        {
            for ( int fc : fs1)
            {
                if ( fs0.count(fc) > 0)
                    return fc;
            }   // end for
        }   // end else
    }   // end if

    return -1;
}   // end face


int Mesh::addFace( const int *vtxs) { return addFace( vtxs[0], vtxs[1], vtxs[2]);}


int Mesh::addFace( int v0, int v1, int v2)
{
    // Check that the vertices are present
    if ( _vids.count(v0) == 0 || _vids.count(v1) == 0 || _vids.count(v2) == 0)
    {
        assert(false);
        return -1;
    }   // end if

    // Check that the vertices are all different
    if ( v0 == v1 || v1 == v2 || v0 == v2)
    {
        assert(false);
        return -1;
    }   // end if

    // Return the existing face id if already present
    const Face fc( v0, v1, v2);
    if ( _f2id.count(fc) > 0)
        return _f2id.at(fc);

    // Check for linear independence of the proposed edges.
    const Vec3f &a = _vtxs.at(v0);
    const Vec3f &b = _vtxs.at(v1);
    const Vec3f &c = _vtxs.at(v2);
    if ( fabsf( (a-b).dot(c-b)) == 1)
        return -1;

    const int fid = (int)_fCounter++;
    _faces[fid] = fc;
    _fids.insert(fid);
    _f2id[fc] = fid;

    _v2f[v0].insert(fid);
    _v2f[v1].insert(fid);
    _v2f[v2].insert(fid);

    int e01 = _connectEdge( v0, v1);
    int e12 = _connectEdge( v1, v2);
    int e20 = _connectEdge( v2, v0);

    _e2f[e01].insert(fid);
    _e2f[e12].insert(fid);
    _e2f[e20].insert(fid);

    return fid;
}   // end addFace


int Mesh::addFace( const Vec3f &V0, const Vec3f &V1, const Vec3f &V2)
{
    Vec3f v0;
    if ( !_checkNewVertex( V0, v0))
        return -1;
    Vec3f v1;
    if ( !_checkNewVertex( V1, v1))
        return -1;
    Vec3f v2;
    if ( !_checkNewVertex( V2, v2))
        return -1;

    const size_t k0 = hash( v0, HASH_NDP);
    const size_t k1 = hash( v1, HASH_NDP);
    const size_t k2 = hash( v2, HASH_NDP);
    if ((k0 == k1) || (k0 == k2) || (k1 == k2))
        return -1;

    const int i0 = _addCheckedVertex( v0, k0);
    if ( i0 < 0)
        return -1;
    const int i1 = _addCheckedVertex( v1, k1);
    if ( i1 < 0)
        return -1;
    const int i2 = _addCheckedVertex( v2, k2);
    if ( i2 < 0)
        return -1;

    return addFace( i0, i1, i2);
}   // end addFace


void Mesh::setFaces( const FaceMat &fmat)
{
    assert( numFaces() == 0);   // Cannot already have faces
    const int N = int(fmat.rows());
    for ( int i = 0; i < N; ++i)
    {
        // Vertex IDs must already be present
        assert( _vids.count(fmat(i,0)) > 0);
        assert( _vids.count(fmat(i,1)) > 0);
        assert( _vids.count(fmat(i,2)) > 0);
        addFace( fmat(i,0), fmat(i,1), fmat(i,2));
    }   // end setFaces
}   // nd setFaces


bool Mesh::removeFace( int fid)
{
    assert( _fids.count(fid) > 0);
    if ( _fids.count(fid) == 0)
        return false;

    // Remove from the Material if present
    const int matid = faceMaterialId( fid);
    if ( matid >= 0)
        _removeFaceUVs( matid, fid);

    const int *vidxs = fvidxs(fid);
    const int v0 = vidxs[0];
    const int v1 = vidxs[1];
    const int v2 = vidxs[2];

    _v2f[v0].erase(fid);
    if ( _v2f[v0].empty())
        _v2f.erase(v0);

    _v2f[v1].erase(fid);
    if ( _v2f[v1].empty())
        _v2f.erase(v1);

    _v2f[v2].erase(fid);
    if ( _v2f[v2].empty())
        _v2f.erase(v2);

    const int e01 = edgeId(v0,v1);
    const int e12 = edgeId(v1,v2);
    const int e20 = edgeId(v2,v0);

    // Remove the entries if the connection between vertices shares no more faces.
    _e2f[e01].erase(fid);
    if ( _e2f[e01].empty())
    {
        _e2f[e01].erase(fid);
        _removeEdge(e01);
    }   // end if

    _e2f[e12].erase(fid);
    if ( _e2f[e12].empty())
    {
        _e2f[e12].erase(fid);
        _removeEdge(e12);
    }   // end if

    _e2f[e20].erase(fid);
    if ( _e2f[e20].empty())
    {
        _e2f[e20].erase(fid);
        _removeEdge(e20);
    }   // end if

    _f2id.erase(_faces[fid]);
    _fids.erase(fid);
    _faces.erase(fid);

    return true;
}   // end removeFace


bool Mesh::setOrderedFaceUVs( int mid, int fid, const Vec2f &uv0, const Vec2f &uv1, const Vec2f &uv2)
{
    assert( _fids.count(fid) > 0);
    if ( _fids.count(fid) == 0)
        return false;
    assert( _mids.count(mid) > 0);
    if ( _mats.count(mid) == 0)
        return false;

    assert( !uv0.array().isNaN().any());
    assert( !uv1.array().isNaN().any());
    assert( !uv2.array().isNaN().any());

    Material &mat = _mats[mid];
    assert( mat._f2uv.count(fid) == 0); // Can't have added previously to material

    _f2m[fid] = mid;    // Overwrite material lookup for face.
    mat.mapFaceUVs( fid, uv0, uv1, uv2);

    return true;
}   // end setOrderedFaceUVs


bool Mesh::setOrderedFaceUVs( int mid, int fid, const Vec2f uvs[3])
{
    return setOrderedFaceUVs( mid, fid, uvs[0], uvs[1], uvs[2]);
}   // end setOrderedFaceUVs


const IntSet &Mesh::uvs( int mid) const
{
    assert( _mats.count(mid) > 0);
    return _mats.at(mid)._uvIds;
}   // end uvs


const Vec2f &Mesh::uv( int mid, int uvID) const
{
    assert( _mats.count(mid) > 0);
    const Material &mat = _mats.at(mid);
    assert( mat._uvs.count(uvID) > 0);
    return mat._uvs.at(uvID);
}   // end uv


const Vec2f &Mesh::faceUV( int fid, int i) const
{
    assert( _fids.count(fid) > 0);
    assert( i >= 0 && i < 3);
    const int mid = faceMaterialId(fid);
    assert( mid >= 0);
    const Material &mat = _mats.at(mid);
    return mat._uvs.at( mat._f2uv.at(fid)[i]);
}   // end faceUV


const int* Mesh::faceUVs( int fid) const
{
    assert( _fids.count(fid) > 0);
    const int mid = faceMaterialId(fid);
    if ( mid < 0)
        return nullptr;
    const Material &mat = _mats.at(mid);
    return &mat._f2uv.at(fid)[0];
}   // end faceUVs


const int* Mesh::fvidxs( int fid) const
{
    assert( _fids.count(fid) > 0);
    if ( _fids.count(fid) == 0)
        return nullptr;
    return _faces.at(fid).vertices();
}   // end fvidxs


void Mesh::reverseFaceVertices( int fid)
{
    assert( _fids.count(fid) > 0);
    Face &fc = _faces.at(fid);
    std::swap( fc[0], fc[2]);

    const int mid = faceMaterialId( fid);
    if ( mid >= 0)
    {
        Vec3i &fuvis = _mats.at(mid)._f2uv.at(fid);
        std::swap( fuvis[0], fuvis[2]);
    }   // end if
}   // end reverseFaceVertices


void Mesh::invertNormals()
{
    for ( int fid : _fids)
        reverseFaceVertices(fid);
}   // end invertNormals


Vec3f Mesh::calcFaceNorm( int fid, bool useTransformed) const
{
    Vec3f nrm = calcFaceVector( fid, useTransformed);
    nrm.normalize();
    return nrm;
}   // end calcFaceNorm


Vec3f Mesh::calcVertexNorm( int vidx, bool useTransformed) const
{
    Vec3f nrm = Vec3f::Zero();
    const IntSet &fids = faces(vidx);
    for ( int fid : fids)
        nrm += calcFaceVector( fid, useTransformed);
    nrm.normalize();
    return nrm;
}   // end calcVertexNorm


Vec3f Mesh::calcFaceVector( int fid, bool useTransformed) const
{
    assert( faces().count(fid) > 0);
    const int *vids = fvidxs(fid);
    const Vec3f *vA, *vB, *vC;
    if ( useTransformed)
    {
        vA = &vtx(vids[0]);
        vB = &vtx(vids[1]);
        vC = &vtx(vids[2]);
    }   // end if
    else
    {
        vA = &uvtx(vids[0]);
        vB = &uvtx(vids[1]);
        vC = &uvtx(vids[2]);
    }   // end else
    return (*vB - *vA).cross( *vC - *vB);
}   // end calcFaceVector


float Mesh::calcFaceArea( int fid) const { return calcFaceVector( fid).norm() / 2;}


bool Mesh::removeEdge( int ei)
{
    if ( _edges.count(ei) == 0)
        return false;

    const Edge &e = edge( ei);
    // Removing an edge removes all attached faces
    const IntSet fids = sfaces( e[0], e[1]);    // Copy out
    for ( int fid : fids)
        removeFace( fid);   // May remove edge if shared by a single face
    if ( _edges.count(ei) > 0)
        _removeEdge( ei);
    return true;
}   // end removeEdge


bool Mesh::removeEdge( int vi, int vj) { return removeEdge( edgeId( vi, vj));}


int Mesh::subdivideFace( int fidx, const Vec3f& V)
{
    if ( _faces.count(fidx) == 0)
        return -1;

    const Vec3f v = transform( _imat, V);

    const int nvidx = addVertex(v);   // New vertex added
    const int* vidxs = fvidxs(fidx);
    // These addFace orderings will ensure that the subdivided faces
    // have the same direction normal as the parent face being subdivided.
    const int fid01 = addFace( nvidx, vidxs[0], vidxs[1]);
    const int fid12 = addFace( nvidx, vidxs[1], vidxs[2]);
    const int fid20 = addFace( nvidx, vidxs[2], vidxs[0]);

    // Set material if present
    const int matId = faceMaterialId( fidx);
    if ( matId >= 0)
    {
        const int* uvs = faceUVs( fidx);
        const Vec2f& uv0 = uv( matId, uvs[0]);
        const Vec2f& uv1 = uv( matId, uvs[1]);
        const Vec2f& uv2 = uv( matId, uvs[2]);
        const Vec2f uvn = calcTextureCoords( fidx, v);

        setOrderedFaceUVs( matId, fid01, uv0, uv1, uvn);
        setOrderedFaceUVs( matId, fid12, uv1, uv2, uvn);
        setOrderedFaceUVs( matId, fid20, uv2, uv0, uvn);
    }   // end if

    // Finally, remove the old face.
    removeFace(fidx);
    return nvidx;
}   // end subdivideFace

/*
// public
int Mesh::subdivideFace( int fidx, int *nfidxs)
{
    if ( _faces.count(fidx) == 0)
        return -1;

    const int* vidxs = fvidxs( fidx);

    // Get the three new vertex positions
    const cv::Vec3f v0 = (vtx(vidxs[0]) + vtx(vidxs[1])) * 0.5f;
    const cv::Vec3f v1 = (vtx(vidxs[1]) + vtx(vidxs[2])) * 0.5f;
    const cv::Vec3f v2 = (vtx(vidxs[2]) + vtx(vidxs[0])) * 0.5f;

    // Add the three new vertices...
    const int nv0 = addVertex( v0);
    const int nv1 = addVertex( v1);
    const int nv2 = addVertex( v2);

    const int nf0 = addFace( nv0, nv1, nv2);  // ... add the new centre face ...
    // ... and the new faces adjacent to it.
    const int nf1 = addFace( vidxs[1], nv1, nv0);
    const int nf2 = addFace( vidxs[2], nv2, nv1);
    const int nf3 = addFace( vidxs[0], nv0, nv2);
    // NB the above new faces have vertex orders that maintain the normal directions of the face being subdivided.

    const int matId = faceMaterialId( fidx);
    if ( matId >= 0)
    {
        const cv::Vec2f uv0 = calcTextureCoords( fidx, v0);
        const cv::Vec2f uv1 = calcTextureCoords( fidx, v1);
        const cv::Vec2f uv2 = calcTextureCoords( fidx, v2);
        setOrderedFaceUVs( matId, nf0, uv2, uv0, uv1);

        const int* uvs = faceUVs( fidx);
        setOrderedFaceUVs( matId, nf1, uv0, uv( matId, uvs[1]), uv1);
        setOrderedFaceUVs( matId, nf2, uv1, uv( matId, uvs[2]), uv2);
        setOrderedFaceUVs( matId, nf3, uv2, uv( matId, uvs[0]), uv0);
    }   // end if

    removeFace(fidx);

    if ( nfidxs) // Copy out the newly added face IDs into user provided storage (if given).
    {
        nfidxs[0] = nf0;
        nfidxs[1] = nf1;
        nfidxs[2] = nf2;
        nfidxs[3] = nf3;
    }   // end if

    return nf0;
}   // end subdivideFace


// public
bool Mesh::subdivideEdge( int vi, int vj, int vn)
{
    const IntSet& sfids = sfaces( vi, vj);
    if ( sfids.empty())
        return false;

    assert( vn >= 0 && vn != vi && vn != vj);
    for ( int fid : sfids)
    {
        const int* vidxs = fvidxs(fid);

        // Create two new faces
        // Order of vidxs will be clockwise and match the order of uvs.
        // This order needs to be maintained so that face normals don't flip.
        const int vk = face(fid).opposite(vi,vj);    // Vertex on the shared face that isn't the edge vertex
        // Find the right ordering of vi and vj so that vi is immediately after vk and vj is after vi.
        int k = 0;
        int i = 1;
        int j = 2;
        if ( vidxs[0] == vk)
        {
            vi = vidxs[1];
            vj = vidxs[2];
        }   // end if
        else if ( vidxs[1] == vk)
        {
            k = 1;
            i = 2;
            j = 0;
            vi = vidxs[2];
            vj = vidxs[0];
        }   // end else if
        else
        {
            k = 2;
            i = 0;
            j = 1;
            vi = vidxs[0];
            vj = vidxs[1];
        }   // end else

        const int f0 = addFace( vn, vk, vi);
        const int f1 = addFace( vn, vj, vk);

        // Set material if present
        const int mid = faceMaterialId( fid);
        if ( mid >= 0)
        {
            const int* uvs = faceUVs( fid);
            const cv::Vec2f& uvk = uv(mid, uvs[k]);
            const cv::Vec2f& uvi = uv(mid, uvs[i]);
            const cv::Vec2f& uvj = uv(mid, uvs[j]);
            const cv::Vec2f uvn = calcTextureCoords( fid, vtx(vn));
            setOrderedFaceUVs( mid, f0, uvn, uvk, uvi);
            setOrderedFaceUVs( mid, f1, uvn, uvj, uvk);
        }   // end if
    }   // end foreach

    const int eid = edgeId( vi, vj);
    _removeEdge( eid); // Finally, remove the old faces attached to the edge
    return true;
}   // end subdivideEdge
*/


bool Mesh::edge( int ei, int &v0, int &v1) const
{
    if ( _edges.count(ei) == 0)
        return false;
    const Edge &e = edge(ei);
    v0 = e[0];
    v1 = e[1];
    return true;
}   // end edge


const Edge *Mesh::commonEdge( int fid0, int fid1) const
{
    const int *fvids = fvidxs(fid0);
    if ( sfaces( fvids[0], fvids[1]).count(fid1) > 0)
        return &edge( edgeId( fvids[0], fvids[1]));
    if ( sfaces( fvids[1], fvids[2]).count(fid1) > 0)
        return &edge( edgeId( fvids[1], fvids[2]));
    if ( sfaces( fvids[2], fvids[0]).count(fid1) > 0)
        return &edge( edgeId( fvids[2], fvids[0]));
    return nullptr;
}   // end commonEdge


namespace {

bool otherSmallsInLarge( int notfid, const IntSet &smallSet, const IntSet &largeSet)
{
    for ( int fid : smallSet)
        if ( fid != notfid && largeSet.count(fid) > 0)
            return true;
    return false;
}   // end otherSmallsInLarge

}   // end namespace


IntSet Mesh::pseudoBoundaries( const IntSet &fids) const
{
    IntSet bset;
    for ( int fid : fids)
    {
        const int *fvids = fvidxs(fid);

        if ( !otherSmallsInLarge( fid, sfaces( fvids[0], fvids[1]), fids))
            bset.insert( edgeId( fvids[0], fvids[1]));

        if ( !otherSmallsInLarge( fid, sfaces( fvids[1], fvids[2]), fids))
            bset.insert( edgeId( fvids[1], fvids[2]));

        if ( !otherSmallsInLarge( fid, sfaces( fvids[2], fvids[0]), fids))
            bset.insert( edgeId( fvids[2], fvids[0]));
    }   // end for
    return bset;
}   // end pseudoBoundaries


const IntSet &Mesh::sfaces( int vi, int vj) const
{
    if ( !hasEdge(vi,vj))
        return EMPTY_INT_SET;
    return sfaces( edgeId(vi,vj));
}   // end sfaces


int Mesh::nsfaces( int vi, int vj) const { return static_cast<int>( sfaces( vi, vj).size());}


int Mesh::nsfaces( int eid) const
{
    int v0, v1;
    if ( !edge( eid, v0, v1))
        return 0;
    return nsfaces( v0, v1);
}   // end nsfaces


const IntSet &Mesh::sfaces( int ei) const
{
    assert( _eids.count(ei) > 0);
    if ( _e2f.at(ei).empty())
        return EMPTY_INT_SET;
    return _e2f.at(ei);
}   // end sfaces


bool Mesh::flipFacePair( int vi, int vj)
{
    const IntSet& sfids = sfaces( vi, vj);
    if ( sfids.size() <= 1 || sfids.size() > 2)
        return false;

    int fid0 = *sfids.begin();
    int fid1 = *(++sfids.begin());

    Face &f0 = _faces[fid0];
    Face &f1 = _faces[fid1];
    const int vk = f0.opposite( vi, vj);
    const int vl = f1.opposite( vi, vj);

    // The order of vi,vj is important for ordering vertices on the flipped faces
    // to ensure that normal directions on the flipped faces are the same as those
    // on the original faces.
    f0.opposite( vk, vi, vj);

    // Update vertex to face mappings
    // f0: i,j,k (i-->l)
    // f1: i,l,j (j-->k)

    // vi and vj no longer connected
    _v2f[vi].erase(fid0);
    _v2f[vj].erase(fid1);
    const int eij = edgeId(vi,vj);
    _e2f[eij].erase(fid0);
    _e2f[eij].erase(fid1);
    _removeEdge(eij);

    // vk and vl now connect
    const int ekl = _connectEdge(vk, vl);
    _v2f[vl].insert(fid0);
    _v2f[vk].insert(fid1);
    _e2f[ekl].insert(fid0);
    _e2f[ekl].insert(fid1);

    const int eik = edgeId(vi,vk);
    _e2f[eik].erase(fid0);
    _e2f[eik].insert(fid1);

    const int ejl = edgeId(vj,vl);
    _e2f[ejl].erase(fid1);
    _e2f[ejl].insert(fid0);

    // Reset the vertices in the Faces
    f0[0] = vl;
    f0[1] = vj;
    f0[2] = vk;
    f1[0] = vi;
    f1[1] = vl;
    f1[2] = vk;

    // Remap texture coords if necessary
    const int m0 = faceMaterialId(fid0);
    const int m1 = faceMaterialId(fid1);
    if ( m0 != m1)
    {
        // Remove texture coords if not the same!
        if (m0 >= 0)
            _removeFaceUVs(m0, fid0);
        if (m1 >= 0)
            _removeFaceUVs(m1, fid1);
    }   // end if
    else if ( m0 >= 0)
    {   
        // Faces share the same material ID (m0 or m1)
        Material &mat = _mats.at(m0);

        int* f0vtorder = f0._fvindices;      // Vertex order
        Vec3i &f0uvorder = mat._f2uv.at(fid0);  // UV order
        int* f1vtorder = f1._fvindices;      // Vertex order
        Vec3i &f1uvorder = mat._f2uv.at(fid1);  // UV order
        int f0i, f1j;
        int uvi, uvj, uvk, uvl;
        for ( int i = 0; i < 3; ++i)
        {
            if ( f0vtorder[i] == vi)
            {
                f0vtorder[i] = vl;
                uvi = f0uvorder[i];
                f0i = i;
            }   // end if
            else if ( f0vtorder[i] == vk)
                uvk = f0uvorder[i];

            if ( f1vtorder[i] == vj)
            {
                f1vtorder[i] = vk;
                uvj = f1uvorder[i];
                f1j = i;
            }   // end if
            else if ( f1vtorder[i] == vl)
                uvl = f1uvorder[i];
        }   // end for

        mat._uv2f[uvi].erase(fid0);
        mat._uv2f[uvj].erase(fid1);
        mat._uv2f[uvk].insert(fid1);
        mat._uv2f[uvl].insert(fid0);

        f0uvorder[f0i] = uvl;
        f1uvorder[f1j] = uvk;
    }   // end else

    return true;
}   // end flipFacePair


Vec3f Mesh::projectToFacePlane( int fid, const Vec3f &P) const
{
    const Vec3f p = transform( _imat, P);
    const Vec3f &A = uvtx(fvidxs(fid)[0]);   // Origin vertex (untransformed)
    const Vec3f pa = p - A;
    const Vec3f z = calcFaceNorm( fid);     // Direction doesn't matter
    // If pa is colinear with z (should be extremely rare if P is random), then P projects to A.
    if ( fabsf( pa.dot(z)) == 1)
        return vtx(fvidxs(fid)[0]); // Origin vertex (transformed)

    Vec3f u = z.cross(pa).cross(z);
    u.normalize();  // Make u unit length for projection of pa 
    return transform( _tmat, A + pa.dot(u)*u); // Return projection of pa along u.
}   // end projectToFacePlane


Vec3f Mesh::nearestPositionWithinFace( int fid, const Vec3f &inP) const
{
    Vec3f P = projectToFacePlane( fid, inP); // Don't assume inP is in the plane of the face.
    // If P is now inside the face, then we're done: P is the projected point.
    if ( isVertexInsideFace( fid, P))
        return P;

    const int *vidxs = fvidxs(fid);
    const Vec3f &A = vtx( vidxs[0]);
    const Vec3f &B = vtx( vidxs[1]);
    const Vec3f &C = vtx( vidxs[2]);

    // P is coplanar with the face but outside of it. Find the vertex (X) it's closest to.
    const Vec3f pa = P-A;
    const Vec3f pb = P-B;
    const Vec3f pc = P-C;

    // Assume P is closest to A initially
    const Vec3f *X = &A;
    float best_l2sq = pa.squaredNorm();

    // Is P actually closer to B?
    const float l2sq_pb = pb.squaredNorm();
    if ( l2sq_pb < best_l2sq)
    {
        best_l2sq = l2sq_pb;
        X = &B;
    }   // end if

    // Is P actually closer to C?
    const float l2sq_pc = pc.squaredNorm();
    if ( l2sq_pc < best_l2sq)
        X = &C;

    // Get the other two normalised edge vectors of the triangle rooted at x as ei and ej
    Vec3f ei, ej;
    if ( X == &A)
    {
        ei = B-A;
        ej = C-A;
    }   // end if
    else if ( X == &B)
    {
        ei = C-B;
        ej = A-B;
    }   // end else if
    else
    {
        ei = A-C;
        ej = B-C;
    }   // end else

    ei.normalize();
    ej.normalize();

    // Difference vector of P with the closest vertex on the triangle
    const Vec3f px = P - *X;

    // Project px onto edge vectors ei and ej to find where it intersects on both edges.
    // Note that the intersection point along e{i,j} can NEVER be beyond its length otherwise
    // X would have been chosen as that vertex endpoint of e{i,j}.
    // If the intersection point on both edges is negative, then the closest position on
    // the triangle to P is vertex X.
    // If the intersection point is positive only on one edge, then the closest point is
    // that projected position along the edge.
    // If the intersection point is positive for both edges, then the closest point is
    // the nearest to P of the two projected positions along the edges.
    // These conditions can be simplified into the following expressions which uses the
    // longer of the two projections along the edge vectors to determine which edge P is closer to.
    const float pxi = std::max( 0.0f, px.dot(ei));
    const float pxj = std::max( 0.0f, px.dot(ej));

    // Whichever of {pxi,pxj} is greater indicates the closer edge vector. Of course, both of them
    // might have projection values of zero, but then this just returns *X which is correct.
    if ( pxi > pxj)
        return *X + pxi*ei;

    return *X + pxj*ej;
}   // end nearestPositionWithinFace


namespace {
double calcTwiceArea( const Vec3d &a, const Vec3d &b, const Vec3d &c)
{
    return (a-b).cross(b-c).norm();
}   // end calcTwiceArea
}   // end namespace


bool Mesh::isVertexInsideFace( int fid, const Vec3f &fP) const
{
    const int *vidxs = fvidxs( fid);
    const Vec3d A = vtx( vidxs[0]).cast<double>();
    const Vec3d B = vtx( vidxs[1]).cast<double>();
    const Vec3d C = vtx( vidxs[2]).cast<double>();
    const Vec3d P = fP.cast<double>();

    // All twice the areas of the respective triangles so division by 2 for all not necessary.
    const double areaABC = calcTwiceArea( A,B,C);
    const double areaPAB = calcTwiceArea( P,A,B);
    const double areaPBC = calcTwiceArea( P,B,C);
    const double areaPCA = calcTwiceArea( P,C,A);

    // This only works if P is in the plane of fid.
    return fabs(areaPAB + areaPBC + areaPCA - areaABC) <= 1e-4;
}   // end isVertexInsideFace


Vec3f Mesh::toBarycentric( int fid, const Vec3f &p) const
{
    const int *vidxs = fvidxs(fid);
    assert(vidxs);
    return calcBarycentric( vtx(vidxs[0]), vtx(vidxs[1]), vtx(vidxs[2]), p);
}   // end toBaryCentric


Vec3f Mesh::fromBarycentric( int fid, const Vec3f &b) const
{
    const int *vidxs = fvidxs(fid);
    return b[0] * vtx(vidxs[0]) + b[1] * vtx(vidxs[1]) + b[2] * vtx(vidxs[2]);
}   // end fromBarycentric


bool Mesh::isEdgeVertex( int vidx, bool assume2DManifold) const
{
    bool isEdge = false;

    if ( assume2DManifold)
        isEdge = cvtxs(vidx).size() > faces(vidx).size();
    else
    {   // Not assuming a 2D manifold so need to check every edge.
        for ( int cv : cvtxs(vidx))
        {
            if ( nsfaces( vidx, cv) == 1)
            {
                isEdge = true;
                break;
            }   // end if
        }   // end for
    }   // end else

    return isEdge;
}   // end isEdgeVertex


bool Mesh::isManifoldEdge( int v0, int v1) const
{
    if ( !hasEdge(v0,v1))
        return false;
    return sfaces( v0, v1).size() != 2;
}   // end isManifoldEdge


bool Mesh::isManifoldEdge( int eid) const
{
    int v0, v1;
    if ( !edge( eid, v0, v1))
        return false;
    return isManifoldEdge( v0, v1);
}   // end isManifoldEdge


int Mesh::oppositeFace( int fid, int vi, int vj) const
{
    if ( _faces.count(fid) == 0)
        return -1;
    const IntSet &sfs = sfaces( vi, vj);
    if ( sfs.size() != 2) // Assumes exactly two faces using the specified edge.
        return -1;
    const int bf = *sfs.begin();
    return bf != fid ? bf : *(++sfs.begin());
}   // end oppositeFace


const IntSet &Mesh::faces( int vi) const
{
    if ( _v2f.count(vi) == 0)
        return EMPTY_INT_SET;
    return _v2f.at(vi);
}   // end faces


const IntSet &Mesh::cvtxs( int vid) const
{
    return _v2v.count(vid) == 0 ?  EMPTY_INT_SET : _v2v.at(vid);
}   // end cvtxs


int Mesh::maximallyExtrudedVertex( const std::vector<int>& vidxs) const
{
    const size_t N = vidxs.size();
    assert( N > 2);
    if ( N <= 2)
        return -1;

    if ( N == 3)
        return 1;

    const Vec3f &v0 = uvtx( vidxs[0]);
    const Vec3f &v1 = uvtx( vidxs[N-1]);
    size_t maxidx = 0;
    float maxvdist = 0;
    for ( size_t i = 1; i < (N-1); ++i)
    {
        const Vec3f& v = uvtx(vidxs[i]);
        const float vdist = (v - v0).norm() + (v - v1).norm();
        if ( vdist > maxvdist)
        {
            maxvdist = vdist;
            maxidx = i;
        }   // end if
    }   // end for

    return int(maxidx);
}   // end maximallyExtrudedVertex


void Mesh::showDebug( bool withDetails) const
{
    // Print vertex info
    std::cerr << "===============[ r3d::Mesh ]===============" << std::endl;
    const int nv = int(numVtxs());
    const int nf = int(numFaces());

    if ( withDetails)
    {
        std::cerr << " Vertices:" << std::endl;
        for ( int vid : _vids)
        {
            const Vec3f &v = _vtxs.at( vid);
            std::cerr << "\tVTX_" << vid << ") " << v.transpose() << " [VTX connections:";
            // Show connected vertices
            const IntSet& cvs = cvtxs( vid);
            std::for_each( std::begin(cvs), std::end(cvs), [](int cv){ std::cerr << " " << cv;});
            std::cerr << "]" << std::endl;
        }   // end for
        std::cerr << std::endl;
    }   // end if

    const IntSet &matIds = materialIds();
    std::cerr << " Mesh has " << matIds.size() << " materials" << std::endl;
    for ( int matId : matIds)
    {
        const Material &mat = _mats.at(matId);
        std::cerr << " Material has " << mat._uvs.size() << " UV coordinates referencing it" << std::endl;
    }   // end for

    if ( withDetails)
    {
        std::cerr << " Face vertex & UV indices:" << std::endl;
        for ( int fid : _fids)
        {
            const int *vids = fvidxs(fid);
            std::cerr << "    F_" << fid << ") VTs: " << vids[0] << ", " << vids[1] << ", " << vids[2] << std::endl;
            const int mid = faceMaterialId(fid);
            if ( mid >= 0)
            {
                const int *uvis = faceUVs(fid);
                std::cerr << std::right << std::setw(int(log10(std::max(1,fid))) + 14)
                                        << " UVs: " << uvis[0] << "=" << uv(mid, uvis[0])
                                            << ", " << uvis[1] << "=" << uv(mid, uvis[1])
                                            << ", " << uvis[2] << "=" << uv(mid, uvis[2])
                                            << "  on Mat " << mid << std::endl;
            }   // end if
        }   // end for
        std::cerr << std::endl;
        std::cerr << " --------------------------------------------------- " << std::endl;
    }   // end if

    std::cerr << " --------------------------------------------------- " << std::endl;
    std::cerr << " Transform Matrix:" << std::endl;
    std::cerr << " " << _tmat << std::endl;
    std::cerr << " --------------------------------------------------- " << std::endl;
    std::cerr << " # vertices =      " << std::setw(8) << nv << std::endl;
    std::cerr << " _vids.size() =    " << std::setw(8) << _vids.size() << std::endl;
    std::cerr << " _vtxs.size() =    " << std::setw(8) << _vtxs.size() << std::endl;
    std::cerr << " _v2v.size() =     " << std::setw(8) << _v2v.size() << std::endl;
    std::cerr << " _v2e.size() =     " << std::setw(8) << _v2e.size() << std::endl;
    std::cerr << " _v2f.size() =     " << std::setw(8) << _v2f.size() << std::endl;
    std::cerr << " _tvtxs.size() =   " << std::setw(8) << _tvtxs.size() << std::endl;
    std::cerr << " --------------------------------------------------- " << std::endl;
    std::cerr << " # faces =         " << std::setw(8) << nf << std::endl;
    std::cerr << " _fids.size() =    " << std::setw(8) << _fids.size() << std::endl;
    std::cerr << " _faces.size() =   " << std::setw(8) << _faces.size() << std::endl;
    std::cerr << " _f2id.size() =    " << std::setw(8) << _f2id.size() << std::endl;
    std::cerr << " _f2m.size() =     " << std::setw(8) << _f2m.size() << std::endl;
    std::cerr << " --------------------------------------------------- " << std::endl;
    std::cerr << " # edges =         " << std::setw(8) << _edges.size() << std::endl;
    std::cerr << " _eids.size() =    " << std::setw(8) << _eids.size() << std::endl;
    std::cerr << " _edges.size() =   " << std::setw(8) << _edges.size() << std::endl;
    std::cerr << " _e2id.size() =    " << std::setw(8) << _e2id.size() << std::endl;
    std::cerr << " _e2f.size() =     " << std::setw(8) << _e2f.size() << std::endl;
    std::cerr << "=====================================================" << std::endl;
}   // end showDebug


MatX3f Mesh::vertices2Matrix( bool useTransformed) const
{
    assert( hasSequentialVertexIds());
    const int N = (int)numVtxs();
    MatX3f M(N, 3);
    if ( useTransformed)
    {
        for ( int i = 0; i < N; ++i)
            M.row(i) = vtx(i);
    }   // end if
    else
    {
        for ( int i = 0; i < N; ++i)
            M.row(i) = uvtx(i);
    }   // end else
    return M;
}   // end vertices2Matrix


MatX6f Mesh::toFeatures( bool useTransformed) const
{
    assert( hasSequentialVertexIds());
    assert( hasSequentialFaceIds());

    // Copy in vertex positions and zero out the normal region
    const int N = (int)numVtxs();
    MatX6f F(N, 6);

    if ( useTransformed)
    {
        for ( int i = 0; i < N; ++i)
            F.block<1,3>(i,0) = vtx(i);
    }   // end if
    else
    {
        for ( int i = 0; i < N; ++i)
            F.block<1,3>(i,0) = uvtx(i);
    }   // end else

    for ( int i = 0; i < N; ++i)
        F.block<1,3>(i,3) = Vec3f::Zero();

    // Copy in face vertex IDs in stored order and calc and insert weighted vertex normals
    const int M = (int)numFaces();
    for ( int i = 0; i < M; ++i)
    {
        const int *fvids = fvidxs(i);
        const Vec3f& vA = F.block<1,3>( fvids[0], 0);
        const Vec3f& vB = F.block<1,3>( fvids[1], 0);
        const Vec3f& vC = F.block<1,3>( fvids[2], 0);

        // Calc area weighted triangle norm (magnitude is twice triangle's area)
        const Vec3f fnrm = (vB - vA).cross( vC - vB);

        // Add back to the associated vertices
        F.block<1,3>(fvids[0], 3) += fnrm;
        F.block<1,3>(fvids[1], 3) += fnrm;
        F.block<1,3>(fvids[2], 3) += fnrm;
    }   // end for

    // Finally, normalise the vertex normals
    for ( int i = 0; i < N; ++i)
        F.block<1,3>(i,3).normalize();  // in place

    return F;
}   // end toFeatures


FaceMat Mesh::toFaces() const
{
    assert( hasSequentialFaceIds());
    const int M = (int)numFaces();
    FaceMat H( M, 3);
    for ( int i = 0; i < M; ++i)
        H.row(i) = asEigen(fvidxs(i));
    return H;
}   // end toFaces


bool Mesh::adjustRawVertices( const MatX3f &F)
{
    if ( !hasSequentialVertexIds())
    {
        assert(false);
        return false;
    }   // end if

    const int N = int(numVtxs());
    if ( N != F.rows())
    {
        assert(false);
        return false;
    }   // end if

    for ( int i = 0; i < N; ++i)
        adjustRawVertex( i, F.row(i));

    return true;
}   // end adjustRawVertices


void Mesh::join( const Mesh& mod, bool txs)
{
    assert( mod.hasFixedTransform());
    assert( mod.hasSequentialIds());
    assert( hasSequentialIds());
    assert( hasFixedTransform());

    std::unordered_map<int,int> vvmap;  // mod.vertexId to this->vertexId
    const int N = int(mod.numVtxs());
    for ( int v = 0; v < N; ++v)
        vvmap[v] = addVertex( mod.uvtx(v));

    const int F = int(mod.numFaces());
    for ( int fid = 0; fid < F; ++fid)
    {
        const int *fids = mod.fvidxs(fid);
        const int nv0 = vvmap.at(fids[0]);
        const int nv1 = vvmap.at(fids[1]);
        const int nv2 = vvmap.at(fids[2]);
        const int nfid = addFace( nv0, nv1, nv2);

        if ( txs)
        {
            const int mid = mod.faceMaterialId(fid);
            if ( mid >= 0)
            {
                assert( _mids.count(mid) > 0);  // Material must already be set on this Mesh!
                const int *uvids = mod.faceUVs(fid);
                const Vec2f &uv0 = mod.uv(mid, uvids[0]);
                const Vec2f &uv1 = mod.uv(mid, uvids[1]);
                const Vec2f &uv2 = mod.uv(mid, uvids[2]);
                setOrderedFaceUVs( mid, nfid, uv0, uv1, uv2);
            }   // end if
        }   // end if
    }   // end for

    if ( txs)
        copyInMaterials( mod);
}   // end join


Mesh::Ptr Mesh::extractVerticesSubset( const IntSet& svidxs, size_t nedges, bool withTx) const
{
    IntSet fidxs;

    int v1, v2;
    if ( nedges == 0)
    {
        // If nedges == 0, only want faces where all three vertices are in the given set
        for ( int v0 : svidxs)
        {
            for ( int fid : faces(v0))
            {
                const Face& fc = face(fid);
                fc.opposite( v0, v1, v2);
                if ( svidxs.count(v1) > 0 && svidxs.count(v2) > 0)
                    fidxs.insert(fid);
            }   // end for
        }   // end for
    }   // end if
    else
    {
        // Copy in seed vertices
        IntSet allvidxs = svidxs;
        IntSet avidxs = svidxs;
        IntSet bvidxs;

        IntSet *rvtxs = &avidxs;
        IntSet *nvtxs = &bvidxs;

        for ( size_t j = 0; j < nedges; ++j)
        {
            for ( int v0 : *rvtxs)
            {
                for ( int fid : faces(v0))
                {
                    if ( fidxs.count(fid) > 0)
                        continue;

                    fidxs.insert(fid);
                    const Face& fc = face(fid);
                    fc.opposite( v0, v1, v2);

                    if ( allvidxs.count(v1) == 0)
                    {
                        allvidxs.insert(v1);
                        nvtxs->insert(v1);
                    }   // end if
                    if ( allvidxs.count(v2) == 0)
                    {
                        allvidxs.insert(v2);
                        nvtxs->insert(v2);
                    }   // end if
                }   // end for
            }   // end for

            std::swap( rvtxs, nvtxs);
            nvtxs->clear();
        }   // end for
    }   // end if

    return extractFacesSubset( fidxs, withTx);
}   // extractVerticesSubset


Mesh::Ptr Mesh::extractFacesSubset( const IntSet &fidxs, bool withTx) const
{
    Mesh::Ptr cm = Mesh::create();
    if ( withTx)
        cm->copyInMaterials( *this);

    for ( int f : fidxs)
    {
        const Face& fc = face(f);
        const int j0 = cm->addVertex( uvtx(fc[0]));
        const int j1 = cm->addVertex( uvtx(fc[1]));
        const int j2 = cm->addVertex( uvtx(fc[2]));
        const int nfid = cm->addFace( j0, j1, j2);

        if ( withTx)
        {
            const int mid = faceMaterialId(f);
            if ( mid >= 0)
            {
                const int *uvids = faceUVs(f);
                cm->setOrderedFaceUVs( mid, nfid, uv(mid, uvids[0]), uv(mid, uvids[1]), uv(mid, uvids[2]));
            }   // end if
        }   // end if
    }   // end for

    cm->setTransformMatrix( transformMatrix());
    return cm;
}   // end extractFacesSubset


size_t Mesh::removeDisconnectedVertices()
{
    IntSet rvidxs;
    const IntSet& vidxs = vtxIds();
    for ( int vidx : vidxs)
    {
        if ( cvtxs(vidx).empty())
            rvidxs.insert(vidx);
    }   // end for

    for ( int vidx : rvidxs)
        removeVertex(vidx);
    return rvidxs.size();
}   // end removeDisconnectedVertices
