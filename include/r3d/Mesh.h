/************************************************************************
 * Copyright (C) 2020 Richard Palmer
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

#ifndef R3D_MESH_H
#define R3D_MESH_H

#include "Internal.h"

namespace r3d {

class r3d_EXPORT Mesh
{
public:
    /********************************************************************************************************************/
    /****** Creating / Copying ******************************************************************************************/
    /********************************************************************************************************************/

    using Ptr = std::shared_ptr<Mesh>;

    /**
     * Create and return a new object mesh ready for data population.
     */
    static Ptr create();

    /**
     * In general, the static Mesh::create function would be used unless a temporary
     * Mesh object is needed for whatever reason.
     */
    Mesh();
    Mesh( const Mesh&) = default;
    Mesh& operator=( const Mesh&) = default;
    virtual ~Mesh();

    /**
     * Create and return a new mesh as a simple point cloud (no connectivity).
     * See function setFaces below to set vertex connectivity from a FaceMat.
     */
    static Ptr fromVertices( const MatX3f &vtxRows);

    /**
     * Create and return a new mesh as a simple point cloud (no connectivity) using just the
     * vertices given in the provided unordered set with positions defined by source Mesh s.
     * The returned mesh will have the source mesh's transformation matrix.
     */
    static Ptr fromVertexSubset( const Mesh &s, const IntSet &vidxs);

    /**
     * Copy out and return a subset of faces connected at most N edges from the given initial vertex set.
     * If N == 0, only faces having all three of their vertices in vtxIds will be returned.
     * Note that texture is NOT copied over by default unless withTexture is set true.
     */
    Ptr extractVerticesSubset( const IntSet& vtxIds, size_t N=0, bool withTexture=false) const;

    /**
     * Copy out and return the subset of faces of this mesh with given face IDs.
     * Note that texture is NOT copied over by default unless withTexture is set true.
     */
    Ptr extractFacesSubset( const IntSet&, bool withTexture=false) const;

    /**
     * Create and return a deep copy of this mesh.
     */
    Ptr deepCopy() const;

    /**
     * If this object has non sequential IDs for its vertices, faces, or edges, then this
     * function returns a deep copy of this object where all IDs are indexed sequentially
     * on [0,N) where N is the corresponding number of vertices/faces/edges. If this object
     * already has all its IDs sequentially ordered, this function is equivalent to deepCopy.
     */
    Ptr repackedCopy() const;

    /**
     * Convenience function to check if all of this mesh's IDs are indexed sequentially.
     */
    bool hasSequentialIds() const { return hasSequentialVertexIds() && hasSequentialFaceIds() && hasSequentialEdgeIds();}

    /**
     * Join the vertices and faces from the given mesh to this mesh. Note that the raw (untransformed) vertices
     * from this model are added to the given mesh, and the given mesh should not have its own transform set.
     * Material texture vertices are also mapped over if txvrts == true (as well as the textures).
     */
    void join( const Mesh&, bool txvrts=true);


    /********************************************************************************************************************/
    /****** Transform ***************************************************************************************************/
    /********************************************************************************************************************/

    /**
     * For affine transforms of this object, base vertex positions can be automatically transformed through a matrix
     * set here. Calculated vertex positions are cached after calculation with the given matrix.
     */
    void setTransformMatrix( const Mat4f& m=Mat4f::Identity());

    /**
     * Add matrix U as a transform to be applied after this mesh's existing transform matrix is applied.
     * Effectively just updates this mesh's transform matrix T as T' = U*T.
     */
    void addTransformMatrix( const Mat4f&);

    /**
     * Return this mesh's transform matrix being applied to its vertices.
     */
    inline const Mat4f &transformMatrix() const { return _tmat;}

    /**
     * Return the inverse of this mesh's transform matrix.
     */
    inline const Mat4f &inverseTransformMatrix() const { return _imat;}

    /**
     * Fixing the transform matrix means setting this object's base vertex positions to be their transformed
     * versions and then setting the transform matrix to the identity matrix. Use this to efficiently update
     * the object's vertices when doing an affine transform.
     */
    void fixTransformMatrix();

    /**
     * Returns true iff this mesh's transform matrix has been fixed (is the identity matrix) so that its
     * raw and transformed vertices are the same (i.e. the return value for uvtx(i) == vtx(i) forall i).
     */
    bool hasFixedTransform() const;


    /********************************************************************************************************************/
    /****** Vertices ****************************************************************************************************/
    /********************************************************************************************************************/

    /**
     * Return the number vertices this mesh has.
     */
    inline size_t numVtxs() const { return _vtxs.size();}

    /**
     * Return the vertex IDs.
     */
    inline const IntSet &vtxIds() const { return _vids;}

    /**
     * Returns the position of vertex vidx transformed by the internal transformation matrix.
     */
    const Vec3f &vtx( int vidx) const;

    /**
     * Returns the position of the given vertex without it having been transformed through the internal matrix.
     */
    inline const Vec3f &uvtx( int vidx) const { return _vtxs.at(vidx);}

    /**
     * Some algorithms require sequential vertex IDs so that vertex IDs can be treated as indices.
     * Vertex IDs are stored sequentially until the removeVertex function is called. Use function
     * Mesh::repackedCopy after removing vertices to create a new copy of this object with
     * vertices (and other elements) stored with IDs in sequential order.
     */
    inline bool hasSequentialVertexIds() const { return numVtxs() == _vCounter;}

    /**
     * Add a vertex and return its ID, or return -1 (and don't add) if any of the position coordinates
     * are NaN. A new ID is assigned and returned for new position vertices. However, if the position
     * matches an existing vertex, then no new vertex is added and the ID of that vertex is returned.
     * Note that vertices are added to the untransformed mesh, i.e. the transform matrix is ignored
     * when adding vertices. In particular, the existence of the same vertex is checked via hash which
     * ignores the transform matrix.
     */
    int addVertex( float x, float y, float z);
    int addVertex( const Vec3f&);

    /**
     * Remove a vertex. Client MUST first remove the associated faces (which removes texture offsets).
     * If function cvtxs returns a non-empty set for the vertex, this function will fail. Note that
     * removing a vertex also removes its ID from this object which can affect whether algorithms
     * that require this object to have sequential vertex IDs work. See hasSequentialVertexIds.
     */
    bool removeVertex( int vidx);

    /**
     * Removes all vertices not used in the definition of any faces and returns the number removed.
     */
    size_t removeDisconnectedVertices();

    /**
     * Adjust raw (untransformed) vertex vidx to be in a new position.
     */
    bool adjustRawVertex( int vidx, const Vec3f&);
    bool adjustRawVertex( int vidx, float x, float y, float z);

    /**
     * Multiply the component of each vertex by sfactor.
     */
    bool scaleVertex( int vidx, float sfactor);

    /**
     * Returns the set of vertex indices that are connected to the parameter vertex.
     * Empty set returned if given vertex is not found or if the vertex is unconnected.
     */
    const IntSet &cvtxs( int vid) const;

    /**
     * Given a vector of vertex IDs, return the index of the element of the given vector
     * for the vertex x that maximises (v0 - x).norm() + (v1 - x).norm() where v0 and v1
     * and the beginning and end vertices in the given vector.
     */
    int maximallyExtrudedVertex( const std::vector<int>&) const;


    /********************************************************************************************************************/
    /****** Faces *******************************************************************************************************/
    /********************************************************************************************************************/

    /**
     * Return the number of faces in this mesh.
     */
    inline size_t numFaces() const { return _faces.size();}

    /**
     * Return the face IDs.
     */
    inline const IntSet& faces() const { return _fids;}

    /**
     * Returns true iff the face IDs are stored in sequential order. Always the case unless Mesh::removeFace
     * is used after which it is necessary to use Mesh::repackedCopy to restore sequential ordering of IDs.
     */
    inline bool hasSequentialFaceIds() const { return numFaces() == _fCounter;}

    /**
     * Make a face from already added vertices. Returns the ID of the newly created face (or the ID of face
     * already created with those vertices). Returns -1 if the face could not be created either because the
     * referenced vertices don't yet exist, two or more of the vertex IDs are the same, or because the proposed
     * edges of the face are not linearly independent (the area of a face must be strictly positive).
     * The specified order of vertex IDs on a face matters since this defines its normal vector.
     */
    int addFace( const int* vidxs);
    int addFace( int v0, int v1, int v2);

    /**
     * Add a face even if vertices haven't already been given. If any of the vertices hash to the same location
     * the face is not added and -1 is returned otherwise this is equivalent to calling addVertex three times
     * followed by addFace with the returned vertex IDs as parameters in corresponding order.
     */
    int addFace( const Vec3f& v0, const Vec3f &v1, const Vec3f &v2);

    /**
     * Sets face connectivity on a pure point cloud (vertices only). Usually used after
     * constructing point wise or via Mesh::fromVertices. Obviously, all vertex IDs
     * referenced in the given matrix must already be present. The order of vertex
     * IDs given in the columns of each row defines the normal for the face.
     */
    void setFaces( const FaceMat&);

    /**
     * Remove a face, also removing its ID.
     */
    bool removeFace( int);

    /**
     * Return the specified face.
     */
    inline const Face& face( int id) const { return _faces.at(id);}

    /**
     * Convenience function to return the given face's vertex IDs in the order they are stored or null if ID invalid.
     */
    const int* fvidxs( int id) const;

    /**
     * Returns ID of the face if it exists with the given vertices or -1 if not found.
     */
    int face( int v0, int v1, int v2) const;

    /**
     * Reverse lookup a face ID from a face or -1 if not present.
     */
    inline int rface( const Face& fc) const { return _f2id.count(fc) > 0 ? _f2id.at(fc) : -1;}

    /**
     * Returns the IDs of the faces that use the given vertex.
     */
    const IntSet& faces( int vidx) const;

    /**
     * Reverse the order of the vertices set on the given face. Will have the effect of flipping
     * the direction of the normal returned by calcFaceNorm. Ensures that texture mapping also
     * remains correctly associated (matched to the face's vertices).
     */
    void reverseFaceVertices( int id);

    /**
     * Given the ordering of vertices on the face, calculate and return the face's unit normal.
     * The normal is calculated from the UNTRANSFORMED vertices unless useTransformed is true.
     */
    Vec3f calcFaceNorm( int fid, bool useTransformed=false) const;

    /**
     * Returns the non normalised version of the face normal with direction specified
     * by the ordering of vertices on the face. Note that the magnitude of this returned
     * vector is twice the area of the face it was calculated from. The UNTRANSFORMED
     * vertices are used to calculate this vector unless useTransformed is true.
     */
    Vec3f calcFaceVector( int fid, bool useTransformed=false) const;

    /**
     * Convenience function to calculate and return the area of the given face.
     * This is simply calcFaceVector(fid).norm() / 2.
     */
    float calcFaceArea( int fid) const;

    /**
     * Given two vertex indices vx and vy, return the number of faces they share.
     * If this number is zero, then the vertices are not directly connected.
     * If this number is one, then the vertices share an edge on the mesh's boundary.
     * If this number is greater than two then neither of the vertices are on a "flat"
     * section of the mesh.
     * Vertex x is defined as flat IFF for all vertices y \in Y where Y is the set of
     * of all vertices directly connected to x, nsfaces(x,y) <= 2.
     */
    int nsfaces( int vx, int vy) const;
    int nsfaces( int edgeId) const;

    /**
     * Return the IDs of the faces shared between the given vertices.
     */
    const IntSet& sfaces( int vi, int vj) const;
    const IntSet& sfaces( int edgeId) const;

    /**
     * If edge i,j (vertex IDs) shares exactly two triangles (T0 = i,j,k and T1=i,j,l),
     * this function flips the edge between the two triangles to be k,l instead of i,j.
     * This changes the shape of the surface between the triangles, and effectively creates
     * a replacement pair of triangles using the same four vertices but with different
     * vertex membership. This function does NOT add or remove face IDs so existing
     * references to the set of face IDs from function faces() will not be corrupted.
     * NB: Concerning material mappings, if the two faces are mapped to the same material,
     * the texture coordinates will be updated to reflect the change in geometry of the
     * two triangles. HOWEVER, if the triangles are mapped to different materials, the
     * material mappings on the newly adjusted triangles will be removed!
     * Returns true iff two triangles are successfully flipped in this manner.
     * Returns false in all other cases (and the mesh is unchanged).
     */
    bool flipFacePair( int vi, int vj);

    /**
     * Turn one face into three by creating three new faces with the given point (added as a new vertex)
     * being the shared corner of the three faces with the sides of the face being the corresponding bases
     * of the new face triangles. Returns the ID of the new vertex that was added. The original face
     * is removed. This function will cause hasSequentialFaceIds() to return false.
     */
    int subdivideFace( int fid, const Vec3f&);


    /********************************************************************************************************************/
    /****** Edges *******************************************************************************************************/
    /********************************************************************************************************************/

    /**
     * Return the number of edges in this mesh.
     */
    inline size_t numEdges() const { return _edges.size();}

    /**
     * Return the edge IDs.
     */
    inline const IntSet& edgeIds() const { return _eids;}

    /**
     * Returns true iff this mesh has its edge IDs stored in sequential order. Calling removeEdge breaks
     * sequential ordering of edge IDs and repackedCopy must be used afterwards to restore sequential order.
     */
    inline bool hasSequentialEdgeIds() const { return numEdges() == _eCounter;}

    /**
     * Returns true iff an edge exists connecting the two given vertices.
     */
    bool hasEdge( int vi, int vj) const;

    /**
     * Return the specified edge.
     */
    inline const Edge& edge( int edgeId) const { return _edges.at(edgeId);}

    /**
     * Given two face IDs, return their common edge or null if no edge is shared by the given faces.
     */
    const Edge *commonEdge( int fid0, int fid1) const;

    /**
     * Given a set of face IDs, return the "psuedo" boundary edge ID set which is comprised of the edges
     * of faces in the given set where the adjacent faces of the edge are NOT in the given set. The returned
     * set of edges should be sorted into different boundary lists before use (see r3d::Boundaries).
     */
    IntSet pseudoBoundaries( const IntSet &fids) const;

    /**
     * Return the IDs of all edges with the given vertex in common.
     */
    inline const IntSet& edgeIds( int vid) const { return _v2e.at(vid);}

    /**
     * Returns true iff the edge with given ID exists and on return sets out parameters v0 and v1
     * to the indices of the edge vertices.
     */
    bool edge( int edgeId, int& v0, int& v1) const;

    /**
     * Returns the ID of the edge comprised of vertex vi and vj, or -1 if no edge connecting those vertices exists.
     */
    int edgeId( int vi, int vj) const;
    int edgeId( const Vec2i&) const;

    /**
     * Connect up vertices vi and vj and return the edge ID or the existing edge ID if already connected.
     * Checks to see if connecting vi and vj makes a triangle; if so, creates one if not already present.
     * NB the order of vi and vj is important if the caller cares about consistent face normals!
     * When setting this edge causes one or more new faces to be created, the faces are set with
     * vertex ordering vs,vi,vj where vs \in VS and VS is the set of vertices already connected by an
     * edge to both vi and vj.
     */
    int addEdge( int vi, int vj);

    /**
     * Removes the given edge and any adjacent faces. Does NOT remove vertices! Calling these functions will
     * break sequential edge ID ordering (see hasSequentialEdgeIds). Returns true if the edge existed and was removed.
     */
    bool removeEdge( int edgeId);
    bool removeEdge( int vi, int vj);


    /********************************************************************************************************************/
    /****** Materials ***************************************************************************************************/
    /********************************************************************************************************************/

    /**
     * Returns the number of materials this mesh uses.
     */
    inline size_t numMats() const { return _mids.size();}

    /**
     * Test if this mesh has one or more materials defined.
     */
    inline bool hasMaterials() const { return numMats() > 0;}

    /**
     * Returns the IDs of the materials set on this mesh.
     * Note that material IDs are not necessarily in sequential order even for a repacked object.
     */
    const IntSet& materialIds() const { return _mids;}

    /**
     * Set texture for a new material (resized so it's no wider/higher than maxDim cols/rows)
     * and return the new material's ID. Fails and returns -1 if the texture is empty.
     */
    int addMaterial( const cv::Mat&, size_t maxDim=4096);

    /**
     * Return the texture image for the given material ID. Returns an empty cv::Mat if material doesn't exist.
     */
    const cv::Mat &texture( int mid) const;

    /**
     * Remove the given material and texture mapping coordinates.
     */
    void removeMaterial( int mid);

    /**
     * Remove all materials and texture mapping coordinates from this mesh.
     */
    void removeAllMaterials();

    /**
     * Copy in the materials from the parameter Mesh to this one.
     */
    void copyInMaterials( const Mesh&);

    /**
     * Merge the materials on this object into a single material. Returns the number of materials that
     * were merged (== numMats() prior to calling). This creates a single large texture image from
     * the individual texture across the materials. This also changes face material membership.
     * This function is available because many rendering schemes have problems when it comes to mapping
     * more than one texture to an object. In particular, multi-texturing support in VTK (as of version 7.1)
     * is unreliable, and rendering of 3D objects in PDFs (as embedded U3D scenes) can result in unwelcome
     * lighting issues. This function circumvents such issues by mapping just a single texture.
     */
    size_t mergeMaterials();

    /**
     * Set the ordering of texture offsets (uvs) that correspond with the order of vertices specified in addFace.
     */
    bool setOrderedFaceUVs( int mid, int fid, const Vec2f uvs[3]);
    bool setOrderedFaceUVs( int mid, int fid, const Vec2f &uv0, const Vec2f &uv1, const Vec2f &uv2);

    /**
     * Get the material ID for the given face (not set until setOrderedFaceUVs() called).
     * Returns -1 if no material set for the given face.
     */
    int faceMaterialId( int fid) const;

    /**
     * For assumed geometry edge v0-->v1, returns the number of associated texture edges from all of the materials mapped to
     * this edge. This primarily depends upon the number of materials mapped to the adjacent faces, and the number of faces
     * shared by this edge, but it also depends upon the nature of the material/texture mapping itself. It is possible that
     * a single geometric (3D) edge shared between two (or more) faces - all of which are mapped to the same material - can
     * be mapped to two (or more) texture (2D) edges. Such edges can present difficulties with remeshing operations
     * (e.g. flipFacePair) because modifying the edge in 3D necessitates a more complicated modification of the "texture"
     * edges so that the texture mapping remains visually consistent with the underlying geometry. This function can be
     * used to test for this situation and so avoid carrying out geometric modifications on these kind of edges.
     */
    size_t numTextureEdges( int v0, int v1) const;

    /**
     * Return the texture UV IDs from the given face or null if this face has no UV mappings.
     * The specific material these UV IDs relate to is found with faceMaterialId( fid).
     */
    const int* faceUVs( int fid) const;

    /**
     * Return a specific UV.
     */
    const Vec2f &uv( int materialID, int uvID) const;

    /**
     * Return a specific UV from a face vertex i on [0,2]. Only valid if the face has a material!
     */
    const Vec2f &faceUV( int fid, int i) const;

    /**
     * Get the set of faces that map just to the given material.
     */
    const IntSet& materialFaceIds( int mid) const;

    /**
     * Get all texture UV identifiers from the given material.
     */
    const IntSet& uvs( int mid) const;

    /**
     * For a face with an existing texture, return the texture coord for the vertex in the plane of that face
     * (vertex doesn't need to be inside the face). Returns Vec2f(-1,-1) if fid has no texture coords.
     * The given point is considered transformed in accordance with this mesh's transform matrix.
     */
    Vec2f calcTextureCoords( int fid, const Vec3f&) const;


    /********************************************************************************************************************/
    /****** Utilities / Misc ********************************************************************************************/
    /********************************************************************************************************************/

    /**
     * Return the position coplanar to face fid that the point projects to (not necessarily within the face's bounds).
     * Given point is considered transformed in accordance with this mesh's transform matrix.
     */
    Vec3f projectToFacePlane( int fid, const Vec3f&) const;

    /**
     * Return the position within the bounds of face fid that the given point is closest to.
     * Given point is considered transformed in accordance with this mesh's transform matrix.
     */
    Vec3f nearestPositionWithinFace( int fid, const Vec3f&) const;

    /**
     * Calculate and return the barycentric coordinates of the given actual point within the given face.
     */
    Vec3f toBarycentric( int fid, const Vec3f&) const;

    /**
     * Convert the barycentric coordinates with respect to the given face's vertices to an actual point.
     */
    Vec3f fromBarycentric( int fid, const Vec3f&) const;

    /**
     * Returns true iff given point *which must already be coplanar with the given triangle* is within it.
     * The given point is considered transformed in accordance with this mesh's transform matrix.
     */
    bool isVertexInsideFace( int fid, const Vec3f&) const;

    /**
     * Returns true iff vidx is paired with some other vertex forming an edge used just by a single face.
     * By default, this function checks other vertices connected to vidx until it finds an edge pair used
     * only by a single triangle making the function linear in the number of connected vertices.
     * However, if assume2DManifold is set to true, it is taken that the vertex belongs to a local 2D manifold
     * (i.e. its set of adjacent faces are topologically homeomorphic to a 2D plane) which allows a constant
     * time test for edge membership to be employed which simply entails checking if the number of
     * connected vertices is greater than the number of adjacent faces.
     */
    bool isEdgeVertex( int vidx, bool assume2DManifold=false) const;

    /**
     * Returns true iff the edge v0-->v1 exists, and it is shared by 1 or 3 or more triangles.
     */
    bool isManifoldEdge( int v0, int v1) const;
    bool isManifoldEdge( int edgeId) const;

    /**
     * Given edge v0-->v1 on face fid, return the other shared face on edge v0-->v1 that isn't fid.
     * If the edge does not share exactly one other face, return -1.
     */
    int oppositeFace( int fid, int v0, int v1) const;

    /**
     * Create the Eigen feature and face matrices from this object. Note that if using r3d::Curvature,
     * it will be more efficient to use the normals from there rather than recalculating them again here.
     * The vertex positions copied over are the UNTRANSFORMED positions unless useTransformed is true.
     */
    FeatMat toFeatures( FaceMat&, bool useTransformed=false) const;

    /**
     * Create and return the face matrix from this mesh's face topology.
     */
    FaceMat toFaces() const;

    /**
     * Simply create and return the 3 column matrix of vertex positions with one row per vertex.
     * This mesh must have sequential vertex IDs!
     * The vertex positions copied over are the UNTRANSFORMED positions unless useTransformed is true.
     */
    MatX3f vertices2Matrix( bool useTransformed=false) const;

    /**
     * Adjust this mesh's vertex positions with those in the columns of the given matrix.
     * Returns false if this mesh's vertices are not in sequential order, or if the number
     * of rows in the given matrix does not match the number of vertices in this mesh.
     * The untransformed positions are adjusted and so the values in the matrix should be
     * the raw (non-transformed) vertex positions.
     */
    bool adjustRawVertices( const MatX3f&);

    /**
     * Show info about this mesh for debugging purposes. Set showDetail=true to show the
     * values set for individual elements (only use when testing small meshs).
     */
    void showDebug( bool showDetail=false) const;

private:
    size_t _vCounter;    // Vertex counter
    size_t _fCounter;    // Face counter
    size_t _eCounter;    // Edge counter
    size_t _mCounter;    // Material counter
    Mat4f _tmat, _imat;  // The transform matrix and its inverse.

    IntSet _vids;                                   // Vertex IDs
    std::unordered_map<int, Vec3f> _vtxs;           // Vertex positions (untransformed)
    std::unordered_map<size_t, int> _v2id;          // Reverse lookup vertex IDs
    mutable std::unordered_map<int, Vec3f> _tvtxs;  // Cached vertex positions (transformed)

    IntSet _fids;                                   // Face IDs
    std::unordered_map<int, Face> _faces;           // Faces
    std::unordered_map<Face, int, HashFace> _f2id;  // Reverse lookup face IDs

    IntSet _eids;                                   // Edge IDs
    std::unordered_map<int, Edge> _edges;           // Edges
    std::unordered_map<Edge, int, HashEdge> _e2id;  // Reverse lookup edge IDs

    std::unordered_map<int, IntSet> _v2v;           // Vertex to vertex connections (symmetric)
    std::unordered_map<int, IntSet> _v2e;           // Vertex ID to edge ID lookup
    std::unordered_map<int, IntSet> _v2f;           // Vertex ID to face ID lookup
    std::unordered_map<int, IntSet> _e2f;           // Edge ID to face ID lookup

    IntSet _mids;                                   // Material IDs
    std::unordered_map<int, Material> _mats;        // Materials mapped by ID
    std::unordered_map<int, int> _f2m;              // Face IDs to material IDs

    int _connectEdge( int, int);
    void _connectEdge( int, int, int);
    void _removeEdge( int);
    void _removeFaceUVs( int, int);
    void _addMaterial( int, const cv::Mat&, size_t);
    int _addCheckedVertex( const Vec3f&, size_t);
    bool _checkNewVertex( const Vec3f&, Vec3f&) const;
};  // end class

}   // end namespace

#endif
