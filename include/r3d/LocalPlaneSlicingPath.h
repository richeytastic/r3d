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

#ifndef R3D_LOCAL_PLANE_SLICING_PATH_H
#define R3D_LOCAL_PLANE_SLICING_PATH_H

#include "PlaneSlicingPath.h"

namespace r3d {

class r3d_EXPORT LocalPlaneSlicingPath : public PlaneSlicingPath
{
public:
    // Point p must be in the plane of face fid. Point q is the endpoint of the path from p.
    LocalPlaneSlicingPath( const Mesh& m, int fid, const Vec3f& p)
        : PlaneSlicingPath( m, fid, p), _endPath(nullptr) {}

    void setEndPath( const LocalPlaneSlicingPath& spath) { _endPath = &spath;}

protected:
    Vec3f faceSlicingPlane( int, const Vec3f&) const override;

private:
    const LocalPlaneSlicingPath *_endPath;
};  // end class

}   // end namespace

#endif
