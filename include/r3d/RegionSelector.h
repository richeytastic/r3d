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

#ifndef R3D_REGION_SELECTOR_H
#define R3D_REGION_SELECTOR_H

#include "Mesh.h"

namespace r3d {

class r3d_EXPORT RegionSelector
{
public:
    // Set the source vertex from which Euclidean distances are measured, and a seed vertex on the mesh from which
    // the selected component will be grown or shrunk as a single connected component. Initially, the selected component
    // will be the whole mesh (or at least that component connected to seedVtx - which is set to a random vertex
    // if left as -1). The initial component is parsed across all edges connecting triangles and across all 1 dimensional
    // vertices - i.e. vertices common to two or more triangles not otherwise connected via an edge. This can be
    // problematic for some algorithms that require a triangulated manifold where all faces connect via edges
    // (so that triangle normals can be propagated). If this is the case, ensure that the mesh supplied to this
    // constructor first has none of these kinds of vertices so that the component is always constructed from
    // triangles sharing edges.
    using Ptr = std::shared_ptr<RegionSelector>;
    static Ptr create( const Mesh&, int vtx=-1);

    // Return the mesh this region selector is for.
    inline const Mesh& mesh() const { return _mesh;}

    // Adjust the radius of the selected region to grow or shrink in size maintaining the old centre.
    // Returns the number of vertices within the new region.
    size_t setRadius( float newRadiusThreshold);
    inline float radius() const { return _rad;}   // Get the current radius value

    // Adjust the centre of the radial region to position c. Vertex cvtx must be the nearest vertex
    // on the mesh to c. The old radius value is maintained. Returns the number of vertices within the
    // newly selected region or 0 if the new centre is not within the existing radius.
    size_t setCentre( int cvtx, const Vec3f& c);
    Vec3f centre() const;  // Get the current centre

    // Get the boundary vertices.
    inline const IntSet* boundary() const { return _front;}

    // Get the boundary vertices as an ordered list of vertices returning the number of vertices.
    // The provided list is cleared before being populated.
    size_t boundary( std::list<int>& vidxs) const;

    // Sets the provided set to the face indices of the input mesh that are within the selected region.
    // Returns the number of faces inside this region.
    size_t selectedFaces( IntSet& cfids) const;

private:
    const Mesh &_mesh;
    int _cf;    // The face attached to _cv being used as the local coordinate frame
    Vec3f _cval;
    IntSet *_front;
    float _rad;
    IntSet _body;

    void _calcBasisVectors( Vec3f&, Vec3f&, Vec3f&) const;
    RegionSelector( const Mesh&, int);
    virtual ~RegionSelector();
    RegionSelector( const RegionSelector&) = delete;
    void operator=( const RegionSelector&) = delete;
};  // end class

}   // end namespace

#endif
