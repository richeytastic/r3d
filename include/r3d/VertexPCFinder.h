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

#ifndef R3D_VERTEX_PC_FINDER_H
#define R3D_VERTEX_PC_FINDER_H

/**
 * Find the principal components of vertex variation with the supplied mesh.
 */

#include "r3dTypes.h"

#ifdef _WIN32
#pragma warning( disable : 4251)
#endif

namespace r3d {

class r3d_EXPORT VertexPCFinder
{
public:
    explicit VertexPCFinder( const MatX3f& verts);

    // Return the eigen vectors of the mesh's vertex distribution.
    // Client should premultiply the returned matrix by each point to orient.
    inline const Mat3f &operator()() const { return _evs;}

private:
    Mat3f _evs;
};  // end class

}   // end namespace

#endif
