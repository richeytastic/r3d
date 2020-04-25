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

#ifndef R3D_BOUNDARIES_H
#define R3D_BOUNDARIES_H

#include "Mesh.h"

/**
 * Set boundaries of into lists of vertices in connected order.
 */
namespace r3d {

class r3d_EXPORT Boundaries
{
public:
    Boundaries() = default;
    Boundaries( const Boundaries&);
    Boundaries& operator=( const Boundaries&);
    virtual ~Boundaries();

    void reset();

    // Pass all edge pairs to be sorted into separate boundary lists.
    int sort( const Mesh&, const IntSet& edgeIds);

    size_t count() const { return _bnds.size();}

    // Boundaries are stored in descending order of vertex count.
    const std::list<int>& boundary( int i) const { return *_bnds.at(size_t(i));}

private:
    std::vector<std::list<int>*> _bnds;
};  // end class

}   // end namespace

#endif
