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

#ifndef R3D_KD_TREE_H
#define R3D_KD_TREE_H

#include "Mesh.h"

namespace r3d {

class r3d_EXPORT KDTree
{
public:
    using Ptr = std::shared_ptr<KDTree>;

    static Ptr create( const Mesh&);

    inline const Mesh &mesh() const { return _mesh;}

    /**
     * Find the ID of the mesh vertex closest to the given point. Returned vertex ID is
     * valid for the mesh used to construct this KDTree. If not null,
     * set sqdis on return to be the squared distance to the vertex.
     */
    int find( const Vec3f&, float *sqdis=nullptr) const;

    /**
     * Find the n vertices on the mesh that are closest to the given point.
     * Arrays nvidxs and sqdis must both be of length n (sqdis is optional).
     * Returns actual number of points found which may be less than n.
     */
    size_t findn( const Vec3f&, size_t n, size_t *nvidxs, float* sqdis=nullptr) const;

    /**
     * Find vertices within r on the mesh to the query point.
     * Returns the number of points found and sets the given vector of matches with
     * each pair giving the vertex index and squared distance.
     */
    size_t findr( const Vec3f&, float r, std::vector<std::pair<size_t, float> > &matches) const;

private:
    const Mesh &_mesh;
    class Impl;
    Impl *_impl;

    explicit KDTree( const Mesh&);
    ~KDTree();
    KDTree( const KDTree&) = delete;
    KDTree& operator=( const KDTree&) = delete;
};  // end class


template <typename T>
class MeshPoints
{
public:
    MeshPoints( const Mesh& m) : _m(m) {}
    inline size_t kdtree_get_point_count() const { return _m.numVtxs();}
    inline T kdtree_get_pt( const size_t idx, int dim) const { return _m.uvtx((int)idx)[dim];}
    template <class BBOX>
    inline bool kdtree_get_bbox( BBOX&) const { return false;}
private:
    const Mesh& _m;
};  // end class

}   // end namespace

#endif
