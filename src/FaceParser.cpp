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

#include <FaceParser.h>
#include <algorithm>
#include <cassert>
#include <stack>
using r3d::FaceParser;
using r3d::BoundaryParser;
using r3d::TriangleParser;
using r3d::Mesh;
using r3d::Face;
using r3d::Vec3f;
using r3d::Vec3i;
using r3d::Vec2i;

FaceParser::FaceParser( const Mesh& m) : _mesh(m), _twisted(false), _bparser(nullptr) {}

FaceParser::~FaceParser() {}


void FaceParser::setBoundaryParser( BoundaryParser* bp)
{
    _bparser = bp;
    assert(_bparser != nullptr);
    _bparser->mesh = &_mesh;
    _bparser->reset();
}   // end setBoundaryParser


bool FaceParser::addTriangleParser( TriangleParser* tp)
{
    bool added = false;
    if ( tp != nullptr)
    {
        if ( !_tparsers.count(tp))
        {
            tp->mesh = &_mesh;
            tp->reset();
            _tparsers.insert(tp);
        }   // end if
        added = true;
    }   // end if
    return added;
}   // end addTriangleParser


struct FaceParser::Triangle
{
    Triangle( FaceParser* parser, int f, Vec2i e)
        : _parser(parser), fid(f), nfid(-1), vtxs( e[0], e[1], parser->mesh().face(f).opposite( e[0], e[1]))
    {
        parser->_parsed.insert(fid);
        parser->_processTriangleParsers( fid, vtxs);
    }   // end ctor

    int id() const { return fid;}

    // Triangle on edge opposite root vertex
    bool canTop() { return findNext( topEdge());}
    Triangle* goTop() { return new Triangle( _parser, nfid, topEdge());}

    // Triangle adjacent to edge opposite vertex a
    bool canLeft() { return findNext( leftEdge());}
    Triangle* goLeft() { return new Triangle( _parser, nfid, leftEdge());}

    // Triangle adjacent to edge opposite vertex b
    bool canRight() { return findNext( rightEdge());}
    Triangle* goRight() { return new Triangle( _parser, nfid, rightEdge());}

    bool isTwisted( const std::unordered_map<int, Triangle*>* alltgl) const
    {
        return isEdgeTwisted( alltgl, topEdge()) ||
               isEdgeTwisted( alltgl, leftEdge()) ||
               isEdgeTwisted( alltgl, rightEdge());
    }   // end isTwisted

private:
    bool findNext( Vec2i e)
    {
        const Mesh& mod = _parser->mesh();

        nfid = -1;
        int pnfid = -1;  // Provisional directed fid to parse next (if specified)
        const IntSet& sfs = mod.sfaces( e[0], e[1]);

        if ( _parser->_parseEdge( fid, e, pnfid) && sfs.size() > 1)
        {
            if ( pnfid >= 0)
            {
                // Check that pnfid actually is a member of the shared edge and that it hasn't yet been parsed.
                if ( _parser->_parsed.count(pnfid) == 0 && sfs.count( pnfid) > 0)
                    nfid = pnfid;
            }   // end if
            else // pnfid < 0: Next polygon to parse wasn't specified.
            {
                for ( int fd : sfs)    // Get the next possible face to parse from all connected to the edge.
                {
                    if ( _parser->_parsed.count(fd) == 0)    // Found a candidate that's not yet been parsed?
                    {
                        nfid = fd;
                        break;
                    }   // end if
                }   // end for
            }   // end if
        }   // end if

        return nfid >= 0;
    }   // end findNext

    bool isEdgeTwisted( const std::unordered_map<int, Triangle*>* alltgl, const Vec2i& e) const
    {
        // Check if any of the shared face ids (not tgl->id()) are in alltgl, and if so,
        // that their vertex ordering is correct. Note that whatever edge from tgl that
        // we're checking, the correct matching order of vertices on the adjacent triangle
        // (if present in alltgl) should always be its right edge (first two vertices).
        for ( int fd : _parser->mesh().sfaces(e[0], e[1]))
        {
            if ( fd != fid && alltgl->count(fd) > 0)
            {
                if ( !alltgl->at(fd)->joinMatch(e))
                    return true;
            }   // end if
        }   // end for
        return false;
    }   // end isEdgeTwisted

    // Returns true iff the given edge matches the joining edge on this triangle.
    bool joinMatch( const Vec2i& e) const { return e[0] == vtxs[0] && e[1] == vtxs[1];}

    Vec2i topEdge() const { return Vec2i( vtxs[2], vtxs[1]);}
    Vec2i leftEdge() const { return Vec2i( vtxs[0], vtxs[2]);}
    Vec2i rightEdge() const { return Vec2i( vtxs[1], vtxs[0]);}

    FaceParser *_parser;
    int fid, nfid;  // id of this triangle and of the next triangle to parse.
    Vec3i vtxs; // Vertex order of this triangle
};  // end struct


int FaceParser::parse( int fid, const Vec3f &planev, bool clearParsed)
{
    if ( clearParsed)
        _parsed.clear();
    _twisted = false;

    const int* vindices = _mesh.fvidxs(fid);
    int vroot = vindices[0];
    int va = vindices[1];

    // Decide parse ordering of face vertices?
    // If the face normal calculated is not in the direction of planev, swap starting vertices.
    if ( planev.norm() > 0.0f && planev.dot(_mesh.calcFaceNorm(fid)) < 0)
        std::swap( vroot, va);

    std::unordered_map<int, Triangle*> *alltgl = new std::unordered_map<int, Triangle*>; // All triangles parsed (need to record vertex ordering)
    std::stack<Triangle*> *stack = new std::stack<Triangle*>;
    Triangle* tgl = new Triangle( this, fid, Vec2i( vroot, va));
    (*alltgl)[tgl->id()] = tgl;
    stack->push( tgl);

    while ( !stack->empty())
    {
        tgl = stack->top();
        stack->pop();

        while ( true)
        {
            if ( tgl->canTop())   // Try going to the next triangle via the top edge first.
            {
                Triangle* tnew = tgl->goTop();
                if ( tgl->canLeft() || tgl->canRight())
                    stack->push(tgl);
                tgl = (*alltgl)[tnew->id()] = tnew;
            }   // end if
            else if ( tgl->canLeft()) // Blocked via the top edge, so try getting the next triangle via the left edge.
            {
                Triangle* tnew = tgl->goLeft();
                if ( tgl->canRight())
                    stack->push(tgl);
                tgl = (*alltgl)[tnew->id()] = tnew;
            }   // end else if
            else if ( tgl->canRight()) // Blocked via the left edge too, so try getting the next triangle via the right edge.
            {
                tgl = tgl->goRight();
                (*alltgl)[tgl->id()] = tgl;
            }   // end else if
            else // All options blocked, so need to pop another triangle from the stack.
                break;
        }   // end while

        // If tgl passage to an adjacent triangle was blocked because the adjacent triangle has already been
        // parsed, we check if the already parsed adjacent triangle has vertices in the expected order.
        // If it doesn't then twisting of the mesh has occurred and consistent surface normal ordering across
        // all parsed polygons will not be possible and function twisted() should return true.
        if ( !_twisted)  // Don't need to check if already found to be twisted
            _twisted = tgl->isTwisted( alltgl);
    }   // end while

    const int nparsed = static_cast<int>(alltgl->size());
    for ( const auto& p : *alltgl)
        delete p.second;
    delete alltgl;
    delete stack;

    _informFinishedParsing();
    return nparsed;
}   // end parse


void FaceParser::_processTriangleParsers( int fid, const Vec3i& vs)
{
    std::for_each( std::begin(_tparsers), std::end(_tparsers), [=]( TriangleParser* t){ t->parseTriangle( fid, vs[0], vs[1], vs[2]);});
}   // end _processTriangleParsers


void FaceParser::_informFinishedParsing()
{
    std::for_each( std::begin(_tparsers), std::end(_tparsers), []( TriangleParser* t){ t->finishedParsing();});
    if ( _bparser)
        _bparser->finishedParsing();
}   // end _informFinishedParsing


bool FaceParser::_parseEdge( int fid, const Vec2i& e, int& pnfid)
{
    bool v = true;
    if ( _bparser)
        v = _bparser->parseEdge( fid, e, pnfid);
    return v;
}   // end _parseEdge
