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

#ifndef R3D_CURVATURE_H
#define R3D_CURVATURE_H

/**
 * Implements:
 * "Estimating the Tensor of Curvature of a Surface from a Facehedral Approximation"
 * by Gabriel Taubin (1995).
 */

#include "Mesh.h"

namespace r3d {

class r3d_EXPORT Curvature
{
public:
    using Ptr = std::shared_ptr<Curvature>;
    static Ptr create( Mesh&);
    Curvature( Mesh&);

    inline const Mesh &mesh() const { return _mesh;}

    // Get the principal curvature vectors tangent to the surface at vertex vi.
    // On return, kp1 and kp2 are set to the corresponding curvature metrics.
    Vec3f vertexPC1( int vi, float &kp1) const;   // First principal component of curvature
    Vec3f vertexPC2( int vi, float &kp2) const;   // Second principal component of curvature

    // Get the normal for the given vertex weighted by the areas of its adjacent polygons
    // Larger polygons weight the normal more in that polygon's direction. Note that the
    // returned normal itself is unit length - only its orientation is weighted.
    // Vertex normals are calculated from the raw vertex positions (i.e. without the
    // mesh's transform matrix having been applied) so the mesh's transform matrix
    // should be separately applied if necessary (e.g. for visualising).
    inline Vec3f vertexNormal( int vi) const { return _vtxNormals.row(vi);}

    // Return the full matrix of vertex normals (untransformed vertices).
    inline const MatX3f &vertexNormals() const { return _vtxNormals;}

    // Adjust the position of a single vertex on the mesh and also update its curvature
    // (along with the curvature of all connected vertices as well).
    // This function is provided only for making iterative adjustments to vertex positions
    // where the resulting modified curvature needs to be reparsed (e.g. for smoothing).
    // If iterative curvature changes on single vertices are not needed, it is more
    // efficient to create this object again from scratch after making mesh wide changes.
    void adjustVertex( int vidx, const Vec3f&);

    /*****************************************************************************************/

    // Calculate the normal for the given vertex weighted by the areas of its adjacent polygons
    // Larger polygons weight the normal more in the direction of that polygon. The returned
    // normal is always unit length.
    static Vec3f calcVertexNormal( const Mesh&, int vi);

    // Given scalars a and b, compute c = cos(t) and s = sin(t) for some angle t so:
    // | c  s|t  |a|  =  |r|
    // |-s  c|   |b|     |0|
    // Returns r (which is always positive).
    static float calcGivensRotation( float a, float b, float& c, float& s);

private:
    Mesh &_mesh;       // Mesh reference
    MatX3f _vtxNormals;     // Normals at vertices
    VecXf _edgeFaceSums;    // Sum of face areas with rows as edge ID
    VecXf _vtxAdjFacesSum;  // Sum of face areas with rows as vertex ID
    Eigen::Matrix<float, Eigen::Dynamic, 8> _vtxCurvature;  // Per vertex (rows) curvature

    void _updateFace( int, float);
    void _setVertexCurvature( int);
    void _addEdgeCurvature( int, int, Mat3f&) const;
};  // end class

}   // end namespace

#endif
