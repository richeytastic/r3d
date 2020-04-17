/************************************************************************
 * Copyright (C) 2020 Richard Palmer
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

#include <DirectedSurfacePointFinder.h>
using r3d::DirectedSurfacePointFinder;
using r3d::KDTree;
using r3d::Mesh;
using r3d::Vec3f;


// u must be a unit vector in the direction we want to move x
int DirectedSurfacePointFinder::find( Vec3f x, const Vec3f &u, Vec3f &y) const
{
    static const size_t MAX_LOOPS = 10;
    const Mesh &mesh = _kdt.mesh();
    int lvidx = 0;
    int vidx = -1;
    float maxd = 0.0f;

    while ( true)
    {
        lvidx = vidx;
        Vec3f ix = x;
        size_t loopCount = 0;
        while ( vidx == lvidx && loopCount < MAX_LOOPS)
        {
            x = ix + loopCount*maxd * u;
            vidx = _kdt.find( x);
            loopCount++;
        }   // end while

        if ( vidx == lvidx) // Fail?
            break;

        // Try to project into one of the connected faces of vidx. If we can, then we're done.
        // If not, then the next delta to move x along u by is found as the maximum over the
        // distance to vidx and the magnitudes of the vector projections from all of vertices
        // connected to vidx with position x.
        for ( int fid : mesh.faces( vidx))
        {
            const Vec3f p = mesh.projectToFacePlane( fid, x);
            const Vec3f v = p - x; // Calculate intersection in the plane defined by face normal
            const float f = v.squaredNorm() / v.dot(u);
            y = x + f*u;
            if ( mesh.isVertexInsideFace( fid, y))
                return fid;
        }   // end for

        maxd = (mesh.vtx(vidx) - x).norm();
        for ( int cvidx : mesh.cvtxs(vidx))
            maxd = std::max( (mesh.vtx(cvidx) - x).dot(u), maxd);
    }   // end while

    return -1;
}   // end find
