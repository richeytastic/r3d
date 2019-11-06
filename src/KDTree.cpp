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

#include <KDTree.h>
#include <nanoflann.hpp>
using r3d::Vec3f;
using r3d::Mesh;
using r3d::KDTree;
using r3d::MeshPoints;
using NanoKDTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, MeshPoints<float> >, MeshPoints<float>, 3>;


class KDTree::Impl
{
public:
    explicit Impl( const Mesh& mesh) : _pcloud(mesh)
    {
        _kdtree = new NanoKDTree( 3, _pcloud, nanoflann::KDTreeSingleIndexAdaptorParams(10)); // What's the difference with 15?
        _kdtree->buildIndex();
    }   // end ctor


    ~Impl() { delete _kdtree;}


    size_t size() const { return _pcloud.kdtree_get_point_count();}


    int find( const Vec3f &q, float* sqdis=nullptr) const
    {
        size_t fvid;
        if ( sqdis)
            _kdtree->knnSearch( &q[0], 1, &fvid, sqdis);
        else
        {
            float sd;
            _kdtree->knnSearch( &q[0], 1, &fvid, &sd);
        }   // end else

        return static_cast<int>(fvid);
    }   // end find


    size_t findn( const Vec3f &q, size_t n, size_t *nidxs, float *sqdis=nullptr) const
    {
        if ( sqdis)
            n = _kdtree->knnSearch( &q[0], n, nidxs, sqdis);
        else
        {
            std::vector<float> lsqdis(n);
            n = _kdtree->knnSearch( &q[0], n, nidxs, &lsqdis[0]);
        }   // end 
        return n;
    }   // end findn


    size_t findr( const Vec3f &q, float radius, std::vector<std::pair<size_t, float> > &matches) const
    {
        nanoflann::SearchParams params;
        return _kdtree->radiusSearch( &q[0], radius, matches, params);
    }   // end findr


private:
    const MeshPoints<float> _pcloud;
    NanoKDTree *_kdtree;
};  // end class


// public
KDTree::Ptr KDTree::create( const Mesh& mesh) { return Ptr( new KDTree( mesh), [](KDTree* d){delete d;});}

// private
KDTree::KDTree( const Mesh& mesh) : _mesh(mesh), _impl( new Impl(mesh)) {}

// private
KDTree::~KDTree() { delete _impl;}

int KDTree::find( const Vec3f &p, float* sqdis) const
{
    const Vec3f q = transform( _mesh.inverseTransformMatrix(), p);
    return _impl->find( q, sqdis);
}   // end find

size_t KDTree::findn( const Vec3f &p, size_t n, size_t *nv, float *sqdis) const
{
    const Vec3f q = transform( _mesh.inverseTransformMatrix(), p);
    return _impl->findn( q, n, nv, sqdis);
}   // end findn

size_t KDTree::findr( const Vec3f &p, float r, std::vector<std::pair<size_t, float> > &m) const
{
    const Vec3f q = transform( _mesh.inverseTransformMatrix(), p);
    return _impl->findr( q, r, m);
}   // end findr
