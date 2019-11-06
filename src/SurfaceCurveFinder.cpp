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

#include <SurfaceCurveFinder.h>
#include <SurfacePointFinder.h>
#include <cassert>
using r3d::SurfaceCurveFinder;
using r3d::SurfacePathFinder;
using r3d::KDTree;
using r3d::Mesh;
using r3d::Vec3f;


namespace {

bool calcCrossing( const Mesh& mesh, int a, int b, const Vec3f& rv, const Vec3f& av, Vec3f& cp)
{
    assert( a != b);
    const Vec3f& v0 = mesh.vtx(a);
    const Vec3f& v1 = mesh.vtx(b);
    cp = r3d::intersection( v0, v1, av, rv);
    return r3d::isPointOnBothLineSegments( v0, v1, av, rv, cp);
}   // end calcCrossing


Vec3f calcPlanePosition( Vec3f n, const Vec3f& sv, const Vec3f& fv)
{
    const Vec3f d = fv - sv;
     // Calculate projection vector
    Vec3f p = n.cross(d.cross(n));
    p.normalize();
    Vec3f pp = sv + d.norm()*p;  // The projected point
    assert( r3d::cosi( sv, pp, fv) >= 0);
    return pp;
}   // end calcPlanePosition


Vec3f calcNorm( const Mesh& mesh, int c, int h, int j)
{
    Vec3f u = mesh.vtx(h) - mesh.vtx(c);
    Vec3f v = mesh.vtx(j) - mesh.vtx(c);
    return u.cross(v);
}   // end calcNorm

}   // end namespace


// Return the vertex on f that is nearest to being on the line segment sv,fv.
int SurfaceCurveFinder::_getOppositeEdge( const Vec3f &sv, const Vec3f& fv, int f) const
{
    const int* vidxs = _kdt.mesh().fvidxs(f);
    const Vec3f& v0 = _kdt.mesh().vtx(vidxs[0]);
    const Vec3f& v1 = _kdt.mesh().vtx(vidxs[1]);
    const Vec3f& v2 = _kdt.mesh().vtx(vidxs[2]);

    float dmin = cosi( v0, sv, fv);
    const float d1 = cosi( v1, sv, fv);
    const float d2 = cosi( v2, sv, fv);

    int c = vidxs[0];
    if ( d1 < dmin)
    {
        c = vidxs[1];
        dmin = d1;
    }   // end if
    if ( d2 < dmin)
        c = vidxs[2];

    return c;
}   // end _getOppositeEdge



SurfaceCurveFinder::SurfaceCurveFinder( const KDTree& k) : SurfacePathFinder(k) {}


float SurfaceCurveFinder::findPath( const Vec3f& fsv, const Vec3f& ffv)
{
    static const float EPS = 1e-6f;
    _lpath.clear(); // protected

    SurfacePointFinder spf(_kdt.mesh());
    int sfid, lfid;
    int p0 = _kdt.find(fsv);
    int p1 = _kdt.find(ffv);
    Vec3f cp = spf.find( fsv, p0, &sfid);
    const Vec3f dfv = spf.find( ffv, p1, &lfid);

    _lpath.push_back(cp);

    //std::cerr << " Path start --> finish vertices:  " << fsv << " --> " << ffv << std::endl;

    IntSet faces;

    Vec3f fv, sv;

    int h, j, c, s;
    s = c = _getOppositeEdge( cp, dfv, sfid);
    _kdt.mesh().face(sfid).opposite(c, h, j);
    int f = sfid;

    while ( f >= 0 && f != lfid && (sv - dfv).squaredNorm() > EPS)
    {
        c = _kdt.mesh().face(f).opposite(h,j);
        faces.insert(f);
        /*
        std::cerr << "  F=" << std::setw(6) << std::right << f
                  << "; c=" << std::setw(6) << c
                  << "; h=" << std::setw(6) << h
                  << "; j=" << std::setw(6) << j << std::endl;
        */
        sv = cp;
        const Vec3f tn = calcNorm( _kdt.mesh(), c, h, j);    // Calculate this face normal
        fv = calcPlanePosition( tn, sv, dfv);

        // Find the next edge h,j on face f to unfold
        if ( calcCrossing( _kdt.mesh(), c, j, sv, fv, cp))
        {
            s = h;  // Record where h was.
            h = c;  // Move h to c.
        }   // end if
        else if ( calcCrossing( _kdt.mesh(), h, c, sv, fv, cp))
        {
            s = j;  // Record where j was.
            j = c;  // Move j to c.
        }   // end else if
        else
        {
            //std::cerr << " Using pseudo-triangle" << std::endl;

            // If neither edge h-c or j-c is crossed, then the algorithm is trying to
            // re-cross edge h-j. To progress, a pseudo triangle is constructed from vertices c,s,t
            // where t is h or j (whichever is closer to fv) and s is the previous position of either h or j.

            // First ensure that vertex h is always closer than vertex j to the endpoint.
            if ( (_kdt.mesh().vtx(h) - ffv).squaredNorm() > (_kdt.mesh().vtx(j) - ffv).squaredNorm())
                std::swap(h,j);

            const Vec3f tn = calcNorm( _kdt.mesh(), h, c, s);

            const Vec3f hv = _kdt.mesh().vtx(h);
            // Project sv into the plane of the pseudo triangle or the crossing calculation won't work.
            sv = calcPlanePosition( tn, hv, sv);
            fv = calcPlanePosition( tn, sv, dfv);

            // It now must be the case that a crossing exists on either edge h-c, or h-s.
            int u = j;  // (remember where j was for making sure we obtain the correct real triangle after)
            if ( (sv - fv).squaredNorm() > EPS)
            {
                if ( calcCrossing( _kdt.mesh(), h, c, sv, fv, cp))
                    j = c;
                else if ( calcCrossing( _kdt.mesh(), h, s, sv, fv, cp))
                    j = s;
                else
                    f = -1;
            }   // end if
            else
            {
                //std::cerr << " Path discovery ended with sv == fv" << std::endl;
                f = -1;
            }   // end else

            if ( f >= 0)
            {
                // Since the edge was calculated using a pseudo-triangle, need to get the real
                // triangle on this edge which will be comprised of vertices h,j,u.

                // Set f to the face that shares edge h-j and has u as its opposite vertex
                const IntSet& sfs = _kdt.mesh().sfaces( h, j);
                f = *sfs.begin();
                if ( sfs.size() > 1 && _kdt.mesh().face(f).opposite(h,j) != u)
                    f = *(++sfs.begin());
                if ( _kdt.mesh().face(f).opposite(h,j) != u)    // Stop - unable to match triangle up.
                {
                    //std::cerr << " Non-matched face - ending path discovery prematurely!" << std::endl;
                    f = -1;
                }   // end if
            }   // end if
        }   // end else

        if ( f >= 0)
        {
            _lpath.push_back( cp);
            f = _kdt.mesh().oppositeFace( f, h, j);   // Next face to unfold (could be -1)
            if ( faces.count(f) > 0)
            {
                //std::cerr << " Loop encountered - ending path discovery prematurely!" << std::endl;
                f = -1;
            }   // end if
        }   // end if
    }   // end while

    _lpath.push_back(ffv);

    float psum = -1;
    if ( f == lfid)
       psum = calcPathLength(_lpath); 
    return psum;
}   // end findPath
