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

#ifndef R3D_PATCH_BENDING_ENERGY_H
#define R3D_PATCH_BENDING_ENERGY_H

#include "Mesh.h"

namespace r3d {

class r3d_EXPORT PatchBendingEnergy
{
public:
    PatchBendingEnergy( const Mesh &m, const Mesh &n);

    // Calculate the bending energy required to deform patch mp (taken from m) to
    // patch np (taken from n) using the 2D thin-plate spline model.
    // Invalid (negative) value returned if mp.size() != np.size().
    float operator()( const IntSet& mp, const IntSet& np) const;

    // As above, but with vertices given as row vectors where p.rows() == q.rows().
    static float calc( const MatX3f &mp, const MatX3f &np);

private:
    const Mesh &_m;
    const Mesh &_n;
};  // end class

}   // end namespace

#endif
