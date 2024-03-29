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

#ifndef R3D_PPF_MATCHER_H
#define R3D_PPF_MATCHER_H

/**
 * A variant of ICP using Point Pair Features for model/scene registration.
 * Must provide normals as well as point locations.
 */

#include "r3dTypes.h"

namespace r3d {

class r3d_EXPORT PPFMatcher
{
public:
    explicit PPFMatcher( const MatX6f&);

    // Calculate and return transform to map given vertices to the constructor target object using ICP.
    Mat4f operator()( const MatX6f&) const;

private:
    const MatX6f &_tgt;
};  // end class

}   // end namespace

#endif
