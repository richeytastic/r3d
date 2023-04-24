/************************************************************************
 * Copyright (C) 2022 Richard Palmer
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

#ifndef R3D_INTERNAL_H
#define R3D_INTERNAL_H

#include "r3dTypes.h"
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <cassert>
#include <memory>
#include <cmath>

#ifdef _WIN32
// Disable warnings about standard template library specialisations not being exported in the DLL interface
#pragma warning( disable : 4251)
#pragma warning( disable : 4275)
#endif

using IntSet = std::unordered_set<int>;

namespace r3d {

class Mesh;

// A triangular face of the mesh with vertices stored in order of setting and defining the triangle's normal orientation.
struct r3d_EXPORT Face
{
    Face();
    Face( const Face&) = default;
    Face& operator=( const Face&) = default;
    Face( int v0, int v1, int v2);

    // Returns true if the parameter poly shares an edge with this one.
    bool isAdjacent( const Face&) const;
  
    // Get vertices that aren't v0. Returns false iff not found.
    bool opposite( int v0, int& v1, int& v2) const;

    // Returns the vertex that isn't v0 or v1 (or -1 if not found).
    int opposite( int v0, int v1) const;

    // Returns the index of vidx (0,1, or 2) as stored in this poly or -1 if not found.
    int index( int vidx) const;

    // Returns true if this face has the given vertex.
    bool has( int vidx) const;

    // Face are the same if they share the same vertices even if they're differently ordered.
    bool operator==( const Face&) const;

    inline const int* vertices() const { return _fvindices;}

    inline int at( int i) const { return _fvindices[i];}
    inline int operator[]( int i) const { return at(i);}
    inline int& operator[]( int i) { return _fvindices[i];}

private:
    // Vertex indices describing the triangle. Order defines normal orientation.
    int _fvindices[3];
    friend class Mesh;
};  // end struct


// Define an edge as the end point vertex IDs
struct r3d_EXPORT Edge
{
    Edge();
    Edge( const Edge&) = default;
    Edge& operator=( const Edge&) = default;
    Edge( int, int);    // Supply two different vertex IDs

    bool operator==( const Edge& e) const;

    inline int at( int i) const { assert(i == 0 || i == 1); return i == 0 ? v0 : v1;}
    inline int operator[]( int i) const { return at(i);}
    inline int& operator[]( int i) { return i == 0 ? v0 : v1;}

private:
    int v0, v1; // Vertex indices (v0 always < v1 if via constructor)
};  // end struct


struct r3d_EXPORT Material
{
    Material();
    Material( const Material&) = default;
    Material& operator=( const Material&) = default;

private:
    int _uvCounter;
    cv::Mat _tx;
    IntSet _uvIds;
    std::unordered_map<int, Vec2f> _uvs;    // UV IDs to UVs
    std::unordered_map<size_t, int> _uv2id; // Reverse mapping of UVs to IDs
    std::unordered_map<int, IntSet> _uv2f;  // UV IDs to polys
    std::unordered_map<int, Vec3i> _f2uv;   // Face to UV IDs in vertex matching order
    IntSet _fids;                           // Face IDs that map to this material

    void updateUV( int uvId, const Vec2f&);
    void mapFaceUVs( int fid, const Vec2f&, const Vec2f&, const Vec2f&);
    int _uvKey( const Vec2f&);
    void removeFaceUVs( int);

    friend class Mesh;
};  // end struct


/**
 * Float values are rounded to ndp decimal places, then hash combined and returned.
 * An existing hash value can be provided which is combined with the new value before returning
 * so that long trains of Vec2/3fs can be hashed together.
 */
r3d_EXPORT size_t hash( const Vec3f&, size_t ndp, size_t h=0);
r3d_EXPORT size_t hash( const Vec2f&, size_t ndp, size_t h=0);

struct HashFace { size_t operator()( const Face&) const;};
struct HashEdge { size_t operator()( const Edge&) const;};


}   // end namespace

std::ostream& operator<<( std::ostream&, const r3d::Face&);
std::istream& operator>>( std::istream&, r3d::Face&);

#endif
