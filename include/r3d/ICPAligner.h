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

#ifndef R3D_ICP_ALIGNER_H
#define R3D_ICP_ALIGNER_H

#include "r3dTypes.h"

namespace r3d {

class r3d_EXPORT ICPAligner
{
public:
    explicit ICPAligner( const FeatMat&);

    // Calculate and return the transform to map the given vertices to the constructor target object using ICP.
    Mat4f operator()( const FeatMat&) const;

private:
    const FeatMat &_tgt;
};  // end class

}   // end namespace

#endif
