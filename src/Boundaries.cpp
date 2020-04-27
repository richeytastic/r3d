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

#include <Boundaries.h>
#include <algorithm>
#include <iomanip>
#include <cassert>
using r3d::Boundaries;
using r3d::Mesh;
using r3d::Face;


Boundaries::Boundaries( const Boundaries& ombb)
{
    *this = ombb;
}   // end ctor


Boundaries& Boundaries::operator=( const Boundaries& ombb)
{
    const size_t nb = ombb.count();
    _bnds.resize( nb);
    for ( size_t i = 0; i < nb; ++i)
        _bnds[i] = new std::list<int>( ombb.boundary(int(i)));
    return *this;
}   // end operator=


Boundaries::~Boundaries()
{
    reset();
}   // end dtor


void Boundaries::reset()
{
    for ( std::list<int>* d : _bnds)
        delete d;
    _bnds.clear();
}   // end reset


namespace {

struct VtxCounts
{
    // Collect the number of times each vertex is entered or exited (counts always obviously even).
    VtxCounts( const Mesh &cmesh, const IntSet& edgeIds) : _cmesh(cmesh), _edgeIds(edgeIds)
    {
        assert( edgeIds.size() > 2);   // Need at least one boundary on one triangle!
        int v0, v1;
        for ( int eid : edgeIds)
        {
            cmesh.edge( eid, v0, v1);
            add(v0);
            add(v1);
        }   // end for

/*
#ifndef NDEBUG
        std::vector<int> mlist;
        for ( const auto& p : _vcounts)
            if ( p.second > 2)
                mlist.push_back(p.first);
        std::sort( mlist.begin(), mlist.end());
        for ( int vm : mlist)
            std::cerr << "Mutli: " << vm << " {" << _vcounts.at(vm) << "}" << std::endl;
#endif
*/
    }   // end ctor

    // Call when vertex v is entered OR exited.
    void pass( int v)
    {
        _vcounts[v]--;
        if ( _vcounts[v] <= 0)
        {
            _vcounts.erase(v);
            _pset.erase(v);
        }   // end if
    }   // end pass

    // Get the next vertex to start a boundary with - preferencing _pset vertices.
    int start() const
    {
        int v = -1;
        if ( !_pset.empty())
            v = *_pset.begin();
        else if ( !_vcounts.empty())
            v = _vcounts.begin()->first;
        return v;
    }   // end start

    // Return the vertex that forms the next edge with hv to append to the current boundary.
    int next( int rv, int bhv, int hv)
    {
        int v = -1;

        const IntSet& cvtxs = _cmesh.cvtxs( hv);
        for ( int cv : cvtxs)
        {
            const int eid = _cmesh.edgeId( hv, cv);
            if ( cv != bhv && cv != rv && _vcounts.count(cv) > 0 && _edgeIds.count(eid) > 0 && _usedEdgeIds.count(eid) == 0)
            {
                v = cv;
                if ( _pset.count(v) == 0)   // Prefer vertices that aren't junctions of multiple edges.
                    break;
            }   // end if
        }   // end for

        if ( v < 0)
        {
            // Done if head-->root is a valid edge of sufficient length and hasn't already been used
            const int eid = _cmesh.edgeId( rv, hv);
            if ( bhv != rv && _edgeIds.count(eid) > 0 && _usedEdgeIds.count(eid) == 0)
                v = rv;
        }   // end if

        if ( v >= 0)
            _usedEdgeIds.insert( _cmesh.edgeId( v, hv));   // Prevent this edge from being used again

        return v;
    }   // end next

    // Returns true iff v is part of the "point" set of vertices (having more than 2 entry/exit edges).
    // Note that this is only ever called AFTER the vertex in question has been appended so if the
    // vertex count is still 2, then it was 3 prior to being appended!
    bool isMulti( int v) const { return _vcounts.count(v) > 0 && _vcounts.at(v) >= 2;}
    bool empty() const { return _vcounts.empty();}
//#ifndef NDEBUG
    int useCount( int v) const { return _vcounts.count(v) == 0 ? 0 : _vcounts.at(v);}

    void printConnected( int bv, int v) const
    {
        const IntSet& cvtxs = _cmesh.cvtxs( v);
        for (int cv : cvtxs)
        {
            if ( cv == bv)  // Skip the vertex connected before v.
                continue;

            const int eid = _cmesh.edgeId( v, cv);
            if ( _edgeIds.count( eid) > 0)
            {
                int cnt = 0;
                if ( _vcounts.count(cv) > 0)
                    cnt = _vcounts.at(cv);
                std::cerr << std::setw(6) << cv << " shares " << _cmesh.nsfaces(v, cv) << " polys [" << cnt << "]" << std::endl;
            }   // end if
        }   // end for
    }   // end printConnected
//#endif


private:
    const Mesh &_cmesh;
    const IntSet& _edgeIds;
    IntSet _usedEdgeIds;
    std::unordered_map<int,int> _vcounts;
    IntSet _pset;   // Special set of vertices with 4 or more entry/exit edges

    void add( int v)
    {
        _vcounts[v]++;
        if ( _vcounts[v] > 2)
            _pset.insert(v);
    }   // end add

    VtxCounts( const VtxCounts&) = delete;
    void operator=( const VtxCounts&) = delete;
};  // end struct - VtxCounts


struct BoundaryPartials;


struct Boundary
{
    explicit Boundary( VtxCounts& vtxs)
        : _vtxs(vtxs), _complete(false), _blist( new std::list<int>)
    {
        append( vtxs.start()); // New boundary starting vertex
    }   // end ctor

    ~Boundary()
    {
        if ( _blist)
            delete _blist;
    }   // end dtor

    bool complete() const { return _complete;}
    size_t length() const { return _blist->size();}


    int extendBoundary()
    {
        const int v = _vtxs.next( root(), beforeHead(), head());
        if ( v >= 0)
            append(v);
        return v;
    }   // end extendBoundary


    // Boundary b1's list ends can be joined with this boundary (b0) if endpoints are shared where b0 is A-->B and:
    // b1: A-->B or,
    // b1: B-->A.
    // If two boundaries can be joined to form a closed loop then they should be.
    bool canJoinLoop( const Boundary* b1) const
    {
        return (head() == b1->head() && root() == b1->root()) || (root() == b1->head() && head() == b1->root());
    }   // end canJoinLoop


    // On return, the parameter boundary is empty and should be discarded.
    void joinLoop( Boundary* b)
    {
        assert( length() > 1);
        assert( b->length() > 1);
        assert( !complete());
        assert( !b->complete());
        assert( canJoinLoop(b));
        assert( !canSplice(b));

        // Don't want duplicate vertices at ends
        if ( head() == b->head())
            reverse();
        popRoot();
        popHead();
        _blist->splice( _blist->end(), *b->_blist);
        _complete = true;
        assert( b->length() == 0);
    }   // end joinLoop


    // Boundary b1's list can be spliced onto this boundary (b0) if either the root, *xor* the head of b0 and b1 is shared.
    // If *both* the root and the head are shared, then joinLoop should be called instead.
    // That is, where b0 is A-->B, splicing is only possible if b1 is any of:
    // b1: C-->B or B-->C or
    // b1: A-->C or C-->A.
    bool canSplice( const Boundary* b1) const
    {
        return (head() == b1->head() && root() != b1->root())
            || (head() == b1->root() && root() != b1->head())
            || (root() == b1->head() && head() != b1->root())
            || (root() == b1->root() && head() != b1->head());
    }   // end canSplice


    // On return, the parameter boundary is empty and should be discarded.
    void splice( Boundary* b)
    {
        assert( length() > 1);
        assert( b->length() > 1);
        assert( !complete());
        assert( !b->complete());
        assert( !canJoinLoop(b));
        assert( canSplice(b));

        // Assume this boundary is A-->B
        if ( head() == b->head() || root() == b->root())    // b: C-->B or A-->C?
            b->reverse();  // b: B-->C or C-->A

        // Case1: (A-->B) + (B-->C)    | splice b on to this boundary at this boundary's end
        if ( head() == b->root())
        {
            b->popRoot();
            _blist->splice( _blist->end(), *b->_blist);
        }   // end if
        else // Case2: (C-->A) + (A-->B)    | splice b on to this boundary at this boundary's start
        {
            b->popHead();
            _blist->splice( _blist->begin(), *b->_blist);
        }   // end else

        assert( b->length() == 0);
    }   // end splice


    int root() const
    {
        assert( _blist);
        if ( _blist->empty())
            return -1;
        return _blist->front();
    }   // end root

    int beforeRoot() const
    {
        assert( _blist);
        if ( _blist->size() < 2)
            return -1;
        return *(++_blist->begin());
    }   // end beforeRoot


    int head() const
    {
        assert( _blist);
        if ( _blist->empty())
            return -1;
        return _blist->back();
    }   // end head

    int beforeHead() const
    {
        assert( _blist);
        if ( _blist->size() < 2)
            return -1;
        return *(++_blist->rbegin());
    }   // end beforeHead


private:
    VtxCounts &_vtxs;
    bool _complete;
    std::list<int> *_blist;
    IntSet _vchk;   // Only used while appending vertices to check for duplicates

    void popRoot() { _blist->pop_front();}
    void popHead() { _blist->pop_back();}
    void reverse() { _blist->reverse();}

    void append( int v)
    {
        if ( length() > 1 && head() >= 0)
            _vtxs.pass(head());   // Exit the current head

        assert( _blist);
        assert( v >= 0);
        assert( !complete());

        if ( v == root())
            _complete = true;
        else
        {
            _blist->push_back(v);
            _vchk.insert(v);
            assert( _vchk.size() == _blist->size());
        }   // end else

        _vtxs.pass(v);  // Entering the new head
/*
#ifndef NDEBUG
        if ( complete() || _vtxs.isMulti(v) || length() == 1)
        {
            std::cerr << "(" << std::setw(6) << v << ") {" << _vtxs.useCount(v) << "}";
            if ( length() == 1)
                std::cerr << " --> ";
        }   // end if
#endif
*/
    }   // end append

    // Transfer out the internal list without deleting - invalidates this Boundary from further actions!
    std::list<int>* transferOut()
    {
        std::list<int>* bout = _blist;
        _blist = nullptr;   // NOT DELETED!
        _vchk.clear();
        return bout;
    }   // end transferOut

    friend struct BoundaryPartials;
};  // end struct - Boundary


using BoundarySet = std::unordered_set<Boundary*>;


struct BoundaryPartials
{
    explicit BoundaryPartials( std::vector<std::list<int>*>& bnds) : _bnds(bnds) {}


    // Consolidate the partial boundaries in polynomial time (can probably be improved
    // by better understanding how the initial set of boundary vertices should be parsed).
    void consolidatePartials()
    {
        splicePairs();
        while ( !_bset.empty())
        {
            auto it = _bset.begin();
            Boundary *b0 = *it;
            Boundary *b1 = nullptr;

            // Find the other partial boundary that joins to b (there must be one after splicePairs).
            auto jt = it;
            jt++;
            for ( ; jt != _bset.end(); ++jt)
            {
                b1 = *jt;
                if ( b0->canJoinLoop(b1))
                {
/*
#ifndef NDEBUG
                    std::cerr << " - Closing loop from partials "
                        << "(" << std::setw(6) << b0->root() << " --> " << std::setw(6) << b0->head() << ") and "
                        << "(" << std::setw(6) << b1->root() << " --> " << std::setw(6) << b1->head() << ")" << std::endl;
#endif
*/
                    b0->joinLoop(b1);
                    break;
                }   // end if
                b1 = nullptr;
            }   // end for

            assert( b1 != nullptr);
            assert( b0->complete());
            transferToFinal( b0);

            _bset.erase(b1);
            delete b1;
        }   // end while
    }   // end consolidatePartials


    // Returns true iff boundary complete.
    bool add( Boundary* b)
    {
        bool complete = false;

/*
#ifndef NDEBUG
        std::cerr << " | " << std::hex << std::setw(10) << b << std::dec;
        if ( b->complete())
            std::cerr << " - Loop";
        else
            std::cerr << " - Partial [" << _bset.size() << "]";
        std::cerr << " #" << b->length() << std::endl;
#endif
*/

        if ( b->complete())
        {
            complete = true;
            transferToFinal( b);
        }   // end if
        else
        {
            _bset.insert(b);
            _partials[b->head()].insert(b);
            _partials[b->root()].insert(b);
        }   // end else

        return complete;
    }   // end add

private:
    std::vector<std::list<int>*>& _bnds;
    BoundarySet _bset;
    std::unordered_map<int, BoundarySet> _partials;

    // If end vertices of partial boundaries only appear in the complete list of partial boundaries
    // twice, these partial boundaries can be spliced together (which *may* cause them to become completed).
    void splicePairs()
    {
        for ( auto it = _partials.begin(); it != _partials.end(); ++it)
        {
            //const int jvtx = it->first; // Vertex attaching the partials
            const BoundarySet& pset = it->second;
            assert( static_cast<int>(pset.size()) % 2 == 0);

            if ( pset.size() == 2)
            {
                Boundary *b0 = *pset.begin();
                Boundary *b1 = *(++pset.begin());
                assert( b0 != b1);

                _partials[b0->head()].erase(b0);
                _partials[b0->root()].erase(b0);
                _partials[b1->head()].erase(b1);
                _partials[b1->root()].erase(b1);

                if ( b0->canJoinLoop(b1))   // Are the endpoints shared? Can a complete loop be made?
                {
/*
#ifndef NDEBUG
                    std::cerr << " - Closing loop from partials "
                        << "(" << std::setw(6) << b0->root() << " --> " << std::setw(6) << b0->head() << ") and "
                        << "(" << std::setw(6) << b1->root() << " --> " << std::setw(6) << b1->head() << ")" << std::endl;
#endif
*/
                    b0->joinLoop(b1);       // b1 now zero length and b0 complete
                    transferToFinal( b0);
                }   // end if
                else
                {
/*
#ifndef NDEBUG
                    std::cerr << " - Splicing partials "
                        << "(" << std::setw(6) << b0->root() << " --> " << std::setw(6) << b0->head() << ") and "
                        << "(" << std::setw(6) << b1->root() << " --> " << std::setw(6) << b1->head() << ")";
#endif
*/
                    b0->splice(b1);
/*
#ifndef NDEBUG
                    std::cerr << " = (" << std::setw(6) << b0->root() << " --> " << std::setw(6) << b0->head() << ")" << std::endl;
#endif
*/
                    _partials[b0->head()].insert(b0);
                    _partials[b0->root()].insert(b0);
                }   // end else

                _bset.erase(b1);
                delete b1;
            }   // end if
        }   // end for
    }   // end splicePairs


    void transferToFinal( Boundary* b)
    {
        _bnds.push_back( b->transferOut());
        _bset.erase(b);
        delete b;
    }   // end transferToFinal
};  // end struct - BoundaryPartials

}   // end namespace


// public
int Boundaries::sort( const Mesh &cmesh, const IntSet& eids)
{
    reset();
    if ( eids.empty())
        return 0;

    // Consolidate all edge vertices, noting those involved in multiple boundaries.
    VtxCounts vtxCounts( cmesh, eids);
    BoundaryPartials boundaries( _bnds);  // Partial (incomplete) boundaries

    // Append all vertices to either partial or full boundaries.
    Boundary* bnd = nullptr;
    while ( !vtxCounts.empty())
    {
        bnd = new Boundary( vtxCounts);

        int bv = -1;
        while ( !bnd->complete() && !vtxCounts.isMulti(bv))
        {
            bv = bnd->extendBoundary();
//#ifndef NDEBUG
            if ( bv < 0)
            {
                std::cerr << "[ERROR] r3d::Boundaries::sort:" << std::endl;
                std::cerr << " - Available vertex connections to head " << bnd->head() << " in edge set:" << std::endl;
                vtxCounts.printConnected( bnd->beforeHead(), bnd->head());
                std::cerr << " - Available vertex connections to root " << bnd->root() << " in edge set:" << std::endl;
                vtxCounts.printConnected( bnd->beforeRoot(), bnd->root());
                return -1;
            }   // end if
//#endif
            assert( bv != -1);
        }   // end while

        boundaries.add(bnd);
        bnd = nullptr;
    }   // end while

    boundaries.consolidatePartials();

    // Sort boundaries in descending order of vertex count.
    std::sort( std::begin(_bnds), std::end(_bnds), []( std::list<int>* p0, std::list<int>* p1){return p1->size() < p0->size();});

    return static_cast<int>(_bnds.size());
}   // end sort
