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

#ifndef R3D_PLANAR_SLICING_PATH_H
#define R3D_PLANAR_SLICING_PATH_H

#include "Mesh.h"
#include <functional>

namespace r3d {

class r3d_EXPORT PlanarSlicingPath
{
public:
    // Starting point (v0) must be inside facet fidv0, and
    // ending point (v1) must be inside facet fidv1.
    // Function facetSlicingPlane must define the slicing plane for the given facet.
    PlanarSlicingPath( const Mesh&, int fidv0, const Vec3f &v0,
                                    int fidv1, const Vec3f &v1,
                                    const std::function<Vec3f(int)>& facetSlicingPlane);

    // Finds and returns the shortest path on the surface or an
    // empty vector if no path was found. On return, plen (if given)
    // is set to the length of the path (FLT_MAX if no path found).
    std::vector<Vec3f> shortestPath( float *plen=nullptr);

private:
    const Mesh &_mesh;
    const int _ifid;
    const Vec3f _v0;
    const int _efid;
    const Vec3f _v1;
    const std::function<Vec3f(int)> &_facetSlicePlaneFn;
    std::vector<Vec3f> _vtxs;
    std::vector<int> _fids;
    IntSet _pfids;
    int _setEdgeCrossing(int);
    bool _extend();
};  // end class

}   // end namespace

#endif
