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

#ifndef R3D_FACE_PARSER_H
#define R3D_FACE_PARSER_H

/**
 * Parses an object by traversing its edges using a consistent ordering of vertices so that adjacent
 * triangles *normally* have their "outward" faces defined in a similar direction (meaning that for
 * two triangles lying in the same plane, the inner product of their normal vectors is 1).
 * HOWEVER in cases where the triangulated mesh is twisted (e.g. like a mobius strip), this rule will
 * not hold and there will be two triangles lying in the same plane where the order of their vertices
 * results in surface normals being defined where their inner product is -1. If such a situation is
 * encountered, function twisted() will return true after the parse() function returns.
 * Parsing is started using the given polygon id (or the first one in the mesh if not given). Facegons
 * that can't be reached by parsing edges are not parsed. After all triangles that can be reached in
 * the parsing operation have been parsed, the set of parsed triangles is given by the set returned
 * from the parsed() function.
 */
#include "Mesh.h"

namespace r3d {

// Parse the given triangle with a consistent vertex edge ordering.
// The vertices are always the vertex IDs in the associated mesh.
struct r3d_EXPORT TriangleParser
{
    TriangleParser() : mesh(nullptr) {}
    virtual ~TriangleParser(){}
    virtual void parseTriangle( int fid, int vroot, int va, int vb) = 0;
    virtual void finishedParsing(){}   // Be informed when parsing all triangles finished.
    virtual void reset(){}  // Called in FaceParser::setTriangleParser AFTER mesh set
    const Mesh* mesh;
};  // end struct

// Given a triangle edge in the direction e[0]-->e[1], specify that parsing should progress beyond this
// edge by returning true from parseEdge(). Optionally, specify exactly which polygon parsing should
// go to next by setting out parameter pnfid. Returning false causes the mesh parser to treat this
// edge as a boundary (whether or not it is an actual boundary of the mesh). It's safe to always return
// true from this function in which case traversal will continue up to the actual bounds of the mesh.
// Parameter fid is the id of the triangle just parsed, and the vertices are the edge (either the top or
// the left edge of that face).
struct r3d_EXPORT BoundaryParser
{
    BoundaryParser() : mesh(nullptr) {}
    virtual ~BoundaryParser(){}
    virtual bool parseEdge( int fid, const Vec2i&, int& pnfid) = 0;
    virtual void finishedParsing(){}   // Be informed when parsing all triangles finished.
    virtual void reset(){}  // Called in FaceParser::setBoundaryParser AFTER mesh set
    const Mesh* mesh;
};  // end struct


class r3d_EXPORT FaceParser
{
public:
    explicit FaceParser( const Mesh&);
    virtual ~FaceParser();

    // Only triangles connected via a shared edge are included in the parsing.
    // Returns the number of triangles parsed.
    // startFace: The ID of the mesh polygon to start parsing with.
    // planev:    The half of coordinate space to use as the "positive"
    //            half regarding the direction of the starting polygon's normal.
    //            This defines the consistent parse ordering of polygon vertices.
    //            If left as default (zero vector), an undefined ordering is used.
    int parse( int startFace=0, const Vec3f &planev=Vec3f::Zero(), bool clearParsed=true);

    // If this function returns true after a call to parse, then the set of parsed
    // polygons involves an odd number of "twists" where edge vertices have their order
    // reversed and a consistent ordering of surface normals is not possible.
    bool twisted() const { return _twisted;}

    // Returns the set of polygons parsed from the last call to parse().
    // (the set of parsed polygons is cleared for every new call to parse()
    // by default, but may be held by setting clearParsed to false).
    const IntSet& parsed() const { return _parsed;}

    // Add processing delegates
    bool addTriangleParser( TriangleParser*);   // Returns true if added or already set
    void setBoundaryParser( BoundaryParser*);

    const Mesh& mesh() const { return _mesh;}

private:
    const Mesh& _mesh;
    bool _twisted;
    IntSet _parsed;
    BoundaryParser* _bparser;
    std::unordered_set<TriangleParser*> _tparsers;

    void _processTriangleParsers( int, const Vec3i&);
    void _informFinishedParsing();
    bool _parseEdge( int, const Vec2i&, int&);
    struct Triangle;
};  // end class

}   // end namespace

#endif
