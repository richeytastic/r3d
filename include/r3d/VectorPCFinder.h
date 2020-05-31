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

#ifndef R3D_VECTOR_PC_FINDER_H
#define R3D_VECTOR_PC_FINDER_H

#include "Mesh.h"

#ifdef _WIN32
#pragma warning( disable : 4251)
#endif

namespace r3d {

class r3d_EXPORT VectorPCFinder
{
public:
    /**
     * Solved on instantiation.
     */
    explicit VectorPCFinder( const MatX3f& verts);

    /**
     * Return's eigen vectors of the vertex distribution as column vectors.
     * The eigenvectors are sorted in descending order by eigenvalue magnitude.
     */
    inline const Mat3f &eigenVectors() const { return _evecs;}
    inline const Vec3f &eigenValues() const { return _evals;}

    /**
     * Take eigen vectors and create a rotation matrix by reordering
     * them to be stored with the eigen vector having the largest
     * coefficient in the first position in the first column, the
     * one with the largest coefficient in the second position to
     * be in the second column and the other one in the third column.
     * In addition, the vector directions are swapped if dot products
     * are negative with the canonical X,Y,and Z axes.
     */
    static Mat3f eigenVectors2RotationMatrix( const Mat3f&);

    /**
     * Estimate and return the rotation matrix for the given model
     * using only the given subset of vertices.
     */
    static Mat3f estimateRotationMatrix( const Mesh&, const IntSet &vids);

private:
    Mat3f _evecs;
    Vec3f _evals;
};  // end class

}   // end namespace

#endif
