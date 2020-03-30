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
    /*
    std::cerr << "-------------------------------------------------" << std::endl;
    std::cerr << "x = " << x.transpose() << std::endl;
    std::cerr << "u = " << u.transpose() << std::endl;
    */

    const Mesh &mesh = _kdt.mesh();
    int lvidx = 0;
    int vidx = -1;

    static const int MAXLOOPS = 7;
    int loopCount = 0;
    while ( lvidx != vidx && loopCount++ < MAXLOOPS)
    {
        float minSqDis = FLT_MAX;
        while ( true)
        {
            lvidx = vidx;
            float sqdis;
            vidx = _kdt.find( x, &sqdis);

            /*
            std::cerr << x.transpose() << " near " << vidx << " sqdis = " << sqdis;
            if ( sqdis > 20.0f)
                std::cerr << " WHAT!!";
            std::cerr << std::endl;
            */

            if ( vidx == lvidx || sqdis > minSqDis)
                break;
            else
            {
                minSqDis = sqdis;
                x += sqrt(sqdis) * u;
            }   // end else
        }   // end while

        // At loop exit, point x has been found to be closest to vertex vidx after
        // two consecutive iterations. Now find the face that x projects into.
        Vec3f nearY = x;
        float minM = FLT_MAX;
        for ( int fid : mesh.faces( vidx))
        {
            const Vec3f p = mesh.projectToFacePlane( fid, x);
            const Vec3f v = p - x; // Calculate intersection in the plane defined by face normal
            const float f = v.squaredNorm() / v.dot(u);
            y = x + f*u;
            if ( mesh.isVertexInsideFace( fid, y))
                return fid;

            const float m = fabsf(f);
            if ( m <= minM)
            {
                minM = m;
                nearY = y;
            }   // end if
        }   // end for

        // The intersection point may not be inside the face. If not, just keep going!
        /*
        std::cerr << loopCount << ") Intersection not found on any face connected to "
                  << vidx << " (" << mesh.vtx(vidx).transpose() << ")" << std::endl;
        std::cerr << "x = " << x.transpose() << " (mag = " << minM << ")" << std::endl;
        */

        x = nearY;
        vidx = -1;
    }   // end while

    return -1;
}   // end find
