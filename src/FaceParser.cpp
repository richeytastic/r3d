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

#include <FaceParser.h>
#include <algorithm>
#include <cassert>
#include <stack>
using r3d::FaceParser;
using r3d::BoundaryParser;
using r3d::TriangleParser;
using r3d::Vec3i;
using r3d::Vec2i;

FaceParser::FaceParser( const r3d::Mesh& m) : _mesh(m), _twisted(false), _bparser(nullptr) {}

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
    Triangle( FaceParser* parser, int f, const Vec3i& v) : _parser(parser), _fid(f), _nfid(-1), _vtxs(v)
    {
        parser->_parsed.insert( _fid);
        parser->_processTriangleParsers( _fid, _vtxs);
    }   // end ctor

    int id() const { return _fid;}

    // Triangle on edge opposite root vertex
    bool canTop() { return _findNext( Vec2i( _vtxs[2], _vtxs[1]));}
    Triangle* goTop() const
    {
        return new Triangle( _parser, _nfid, Vec3i( _parser->mesh().face(_nfid).opposite( _vtxs[2], _vtxs[1]), _vtxs[2], _vtxs[1]));
    }   // end goTop

    // Triangle adjacent to edge opposite vertex a
    bool canLeft() { return _findNext( Vec2i( _vtxs[2], _vtxs[0]));}
    Triangle* goLeft() const
    {
        return new Triangle( _parser, _nfid, Vec3i( _vtxs[2], _parser->mesh().face(_nfid).opposite( _vtxs[2], _vtxs[0]), _vtxs[0]));
    }   // end goLeft

    // Triangle adjacent to edge opposite vertex b
    bool canRight() { return _findNext( Vec2i( _vtxs[1], _vtxs[0]));}
    Triangle* goRight() const
    {
        return new Triangle( _parser, _nfid, Vec3i( _vtxs[1], _vtxs[0], _parser->mesh().face(_nfid).opposite( _vtxs[1], _vtxs[0])));
    }   // end goRight

    bool isTwisted( const std::unordered_map<int, Triangle*> &alltgl) const
    {
        return !_isEdgeMatch( alltgl, 2, 1) ||   // Top edge vertices
               !_isEdgeMatch( alltgl, 2, 0) ||   // Left edge vertices
               !_isEdgeMatch( alltgl, 1, 0);     // Right edge vertices
    }   // end isTwisted

private:
    bool _findNext( const Vec2i &e)
    {
        _nfid = -1;
        int pnfid = -1;  // Provisional directed _fid to parse next (if specified)
        const IntSet& sfs = _parser->mesh().sfaces( e[0], e[1]);

        if ( _parser->_parseEdge( _fid, e, pnfid) && sfs.size() > 1)
        {
            if ( pnfid >= 0)
            {
                // Check that pnfid actually is a member of the shared edge and that it hasn't yet been parsed.
                if ( _parser->_parsed.count(pnfid) == 0 && sfs.count( pnfid) > 0)
                    _nfid = pnfid;
            }   // end if
            else // pnfid < 0: Next polygon to parse wasn't specified.
            {
                for ( int fd : sfs)    // Get the next possible face to parse from all connected to the edge.
                {
                    if ( _parser->_parsed.count(fd) == 0)    // Found a candidate that's not yet been parsed?
                    {
                        _nfid = fd;
                        break;
                    }   // end if
                }   // end for
            }   // end if
        }   // end if

        return _nfid >= 0;
    }   // end _findNext

    bool _isEdgeMatch( const std::unordered_map<int, Triangle*>& alltgl, int i, int j) const
    {
        const IntSet &sfids = _parser->mesh().sfaces( _vtxs[i], _vtxs[j]);
        for ( int fd : sfids)
        {
            if ( fd != _fid && alltgl.count(fd) > 0)
            {
                const Triangle &toth = *alltgl.at(fd);
                if ( toth._vtxs[i] != _vtxs[j] || toth._vtxs[j] != _vtxs[i])
                    return false;
            }   // end if
        }   // end for
        return true;
    }   // end isEdgeMatch

    FaceParser *_parser;
    int _fid, _nfid;  // id of this triangle and of the next triangle to parse.
    Vec3i _vtxs; // Vertex order of this triangle (r,a,b) where r is bottom corner, a is top right and b is top left.
};  // end struct


int FaceParser::parse( int fid, const r3d::Vec3f &planev, bool clearParsed)
{
    if ( clearParsed)
        _parsed.clear();
    _twisted = false;

    const int* vindices = _mesh.fvidxs(fid);
    int vroot = vindices[0];
    int va = vindices[1];
    const int vb = vindices[2];

    // Decide parse ordering of face vertices?
    // If the face normal calculated is not in the direction of planev, swap starting vertices.
    if ( planev.norm() > 0.0f && planev.dot(_mesh.calcFaceNorm(fid)) < 0)
        std::swap( vroot, va);

    // All triangles parsed (need to record vertex ordering)
    std::unordered_map<int, Triangle*> *alltgl = new std::unordered_map<int, Triangle*>;
    std::stack<Triangle*> *stack = new std::stack<Triangle*>;
    Triangle* tgl = new Triangle( this, fid, Vec3i( vroot, va, vb));
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
            else // All options blocked, so pop another triangle from the stack.
                break;
        }   // end while

        // If tgl passage to an adjacent triangle was blocked because the adjacent triangle has already been
        // parsed, we check if the already parsed adjacent triangle has vertices in the expected order.
        // If it doesn't then twisting of the mesh has occurred and consistent surface normal ordering across
        // all parsed polygons will not be possible and function twisted() should return true.
        if ( !_twisted)  // Don't need to check if already found to be twisted
            _twisted = tgl->isTwisted( *alltgl);

        //std::cerr << "alltgl->size() = " << alltgl->size() << ": twisted = " << std::boolalpha << _twisted << std::endl;
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
    std::for_each( std::begin(_tparsers), std::end(_tparsers),
            [&]( TriangleParser* t){ t->parseTriangle( fid, vs[0], vs[1], vs[2]);});
}   // end _processTriangleParsers


void FaceParser::_informFinishedParsing()
{
    std::for_each( std::begin(_tparsers), std::end(_tparsers),
            []( TriangleParser* t){ t->finishedParsing();});
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
