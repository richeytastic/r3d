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

#ifndef R3D_MANIFOLDS_H
#define R3D_MANIFOLDS_H

#include "Boundaries.h"

/**
 * Find separate manifolds of a mesh as the distinct subsets
 * of triangles forming 2D triangulated manifolds. Some faces can
 * not be included in triangulated manifolds and these are returned via
 * nonManifoldFacegons (typically these are faces with an edge
 * that is shared by two or more other faces).
 */
namespace r3d {

class Manifolds;

class r3d_EXPORT Manifold
{
public:
    inline const Mesh &mesh() const { return *_mesh;}

    // Return the set of faces that define this manifold.
    inline const IntSet& faces() const { return _faces;}

    // Returns the (unordered) set of boundary edge ids. All edges are shared with a single face.
    // The Boundaries object (returned from boundaries) orders these edges into boundary lists.
    inline const IntSet& edges() const { return _edges;}

    // Vertices are created on demand from the faces.
    const IntSet& vertices() const;

    // Return the boundary lists of this manifold comprised of the ordered edges (returned from edges).
    // Note that these boundary lists are never computed from the edges if this function is never called.
    const Boundaries& boundaries() const;

private:
    const Mesh *_mesh;
    IntSet _faces;
    IntSet _edges;
    mutable IntSet _verts;
    mutable Boundaries _bnds;

    friend class Manifolds;
    Manifold();
    Manifold( const Manifold&);
    Manifold& operator=( const Manifold&);
};  // end class


class r3d_EXPORT Manifolds
{
public:
    using Ptr = std::shared_ptr<Manifolds>;
    static Ptr create( const Mesh&);

    Ptr deepCopy() const;   // Deep copies will still refer to the same internal mesh

    inline size_t count() const { return _manfs.size();}

    inline const Mesh &mesh() const { return *_mesh;}

    // Replace internal mesh with the given one. Does not reparse anything so the
    // provided mesh should be a deep copy of the original (in terms of connectivity).
    void setMesh( const Mesh&);

    // Returns the id of the manifold for the given face or -1 if not found.
    int fromFaceId( int fid) const;

    // Returns manifold at position i. Manifolds are stored in descending order of # faces.
    const Manifold& at( int i) const;
    const Manifold& operator[]( int i) const;

    // Create and return a new mesh having only the top n manifolds (largest in terms of face count).
    // The returned mesh will only contain connected vertices that are part of the top n manifolds.
    // Materials are shared with the constructor mesh.
    Mesh::Ptr reduceManifolds( int n) const;

private:
    const Mesh *_mesh;
    std::vector<Manifold*> _manfs;
    std::unordered_map<int, int> _face2manf;    // Face IDs to manifold IDs.

    Manifolds();
    explicit Manifolds( const Mesh&);
    virtual ~Manifolds();
    Manifolds( const Manifolds&);
    Manifolds& operator=( const Manifolds&);
};  // end class

}   // end namespace

#endif
