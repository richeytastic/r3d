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

#ifndef R3D_DIRECTED_SURFACE_POINT_FINDER_H
#define R3D_DIRECTED_SURFACE_POINT_FINDER_H

#include "KDTree.h"

namespace r3d {

class r3d_EXPORT DirectedSurfacePointFinder
{
public:
    explicit DirectedSurfacePointFinder( const KDTree& kdt) : _kdt(kdt) {}

    /**
     * Given point x, and a normalised direction vector, find point y that is a
     * multiple of u from x and intersects with the mesh surface. Returns the
     * id of the face iff a valid y was found, otherwise -1.
     **/
    int find( Vec3f x, const Vec3f &u, Vec3f &y) const;

private:
    const KDTree &_kdt;
};  // end class

}   // end namespace

#endif
