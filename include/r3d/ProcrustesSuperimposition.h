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

#ifndef R3D_PROCRUSTES_SUPERIMPOSITION_H
#define R3D_PROCRUSTES_SUPERIMPOSITION_H

#include "r3dTypes.h"

namespace r3d {

// Use Procrustes to fit a set of vertices to a target. There must be a one-to-one correspondence
// of vertex indices between the target (constructor) vertex set and the operator() argument.
class r3d_EXPORT ProcrustesSuperimposition
{
public:
    // Provide target vertices as row vectors to superimpose to and per vertex weights.
    // If scaleUp true, calcTransform will return transformation matrices that scale the
    // argument vertices to match the target vertice's dimensions.
    ProcrustesSuperimposition( const MatX3f& tgtVertexRows, const VecXf& W, bool scaleUp=false);

    // Calculate the transform to map the given set of vertices to the constructor target
    // vertices with corresponding vertex indices and one step Procrustes superimposition.
    Mat4f operator()( const MatX3f&) const;

private:
    const VecXf &_W;        // Vertex weights.
    bool _scaleUp;          // Whether or not to scale transformed mesh up to the target dimensions.
    Vec3f _vbar;            // The centroid of A.
    MatX3f _A;              // Mesh vertices as row vectors.
    float _s;               // Target's initial scale.

    ProcrustesSuperimposition( const ProcrustesSuperimposition&) = delete;
    void operator=( const ProcrustesSuperimposition&) = delete;
};  // end class

}   // end namespace

#endif
