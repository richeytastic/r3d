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

#include <LocalPlaneSlicingPath.h>
using r3d::LocalPlaneSlicingPath;
using r3d::Vec3f;


Vec3f LocalPlaneSlicingPath::faceSlicingPlane( int fid, const Vec3f& v) const
{
    assert(_endPath);
    const Vec3f fn = mesh().calcFaceNorm(fid);
    const Vec3f& q = _endPath->lastVertexAdded();
    Vec3f u = (q-v).cross(fn);
    u.normalize();
    return u;
}   // end faceSlicingPlane
