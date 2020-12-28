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

#ifndef R3D_REGION_SELECTOR_H
#define R3D_REGION_SELECTOR_H

#include "Mesh.h"

namespace r3d {

class r3d_EXPORT RegionSelector
{
public:
    using Ptr = std::shared_ptr<RegionSelector>;
    static Ptr create( const Mesh&);

    // Return the mesh this region selector is for.
    inline const Mesh& mesh() const { return _mesh;}

    // Adjust the centre of the radial region to position c. Face Id cfidx must be face ID on
    // the mesh under c. Use rad as the radius value. Returns the number of vertices within the
    // newly selected region or 0 if the new centre is not within the existing radius.
    size_t update( int cfidx, const Vec3f& c, float rad);

    Vec3f centre() const;  // Get the current centre
    inline float radius() const { return _rad;}   // Get the current radius value

    // Get the boundary vertices.
    inline const IntSet* boundary() const { return _front;}

    // Sets the provided set to the face indices of the input mesh that are within the selected region.
    // If onlyin is not null, only face IDs that are within this set are set in cfids.
    // Returns the number of faces inside this region.
    size_t selectedFaces( IntSet& cfids, const IntSet *onlyin=nullptr) const;

private:
    const Mesh &_mesh;
    int _cf;    // The face attached to _cv being used as the local coordinate frame
    Vec3f _cval;
    IntSet *_front;
    float _rad;
    IntSet _body;

    void _calcBasisVectors( Vec3f&, Vec3f&, Vec3f&) const;
    explicit RegionSelector( const Mesh&);
    virtual ~RegionSelector();
    RegionSelector( const RegionSelector&) = delete;
    void operator=( const RegionSelector&) = delete;
};  // end class

}   // end namespace

#endif
