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

#ifndef R3D_SMOOTHER_H
#define R3D_SMOOTHER_H

#include "Curvature.h"

namespace r3d {

class r3d_EXPORT Smoother
{
public:
    // Reinterpolate vertices having curvature greater than maxc. Prioritises highest curvature
    // vertices first. Smoothing may not result in all vertices having curvature <= maxc on return
    // since interpolation is local. Finishes when no vertices have curvature > maxc or when max
    // iterations reached.
    explicit Smoother( float maxc, size_t maxIterations=10);
 
    // If vidxs is given, only vertices in the given set are looked at.
    void operator()( Curvature&, const IntSet *vidxs=nullptr);

private:
    const float _maxc;
    const size_t _maxIts;
    Smoother( const Smoother&) = delete;
    void operator=( const Smoother&) = delete;
};  // end class

}   // end namespace

#endif
