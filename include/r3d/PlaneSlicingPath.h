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

#ifndef R3D_PLANE_SLICING_PATH_H
#define R3D_PLANE_SLICING_PATH_H

#include "Mesh.h"
#include <deque>

namespace r3d {

class r3d_EXPORT PlaneSlicingPath
{
public:
    // The starting point must be inside face infid.
    PlaneSlicingPath( const Mesh&, int infid, const Vec3f&);
    virtual ~PlaneSlicingPath(){}

    void reset();

    // There are two directions to go in - specify the next face in the path
    // by specifying the face that SHOULDN'T be next.
    void init( int notMeFaceId=-1);

    inline int initFace() const { return _ifid;}
    inline int firstFace() const { return _ffid;}
    inline int nextFace() const { return _nfid;}

    bool canSplice( const PlaneSlicingPath&) const;

    void splice( const PlaneSlicingPath&, std::vector<Vec3f>&) const;

    // Returns true iff this path can be extended at one or both of its ends.
    inline bool canExtend() const { return _nfid >= 0;}

    // Returns true if can keep extending the front or back of the path.
    bool extend();

protected:
    virtual Vec3f faceSlicingPlane( int thisFid) const = 0;

    inline const Mesh& mesh() const { return _mesh;}

    inline const std::deque<Vec3f> &edgeCrossings() const { return _evtxs;}

    inline const int lastParsedFace() const { return _lastParsedFace;}

private:
    const Mesh &_mesh;
    const Vec3f _ip;    // The initial vertex (inside the initial face)
    const int _ifid;    // Initial face 
    int _ffid;          // The first face after the initial (might be -1)
    int _nfid;          // Next face id
    IntSet _pfids;
    int _lastParsedFace;
    std::deque<Vec3f> _evtxs;    // Edge crossing vertices

    void _pushOnInitialToBack( std::vector<Vec3f>& path) const;
    void _pushOnBackToInitial( std::vector<Vec3f>& path) const;
    int _findNextFaceEdgeVertex( Vec3f&);
};  // end class

}   // end namespace

#endif
