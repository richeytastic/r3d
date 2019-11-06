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

#include <EdgeFaceAdder.h>
#include <iostream>
#include <cassert>
using r3d::EdgeFaceAdder;
using r3d::Mesh;


void EdgeFaceAdder::addFaces( const std::unordered_map<int,IntSet>& xyset)
{
    _edgeUse.clear();
    using XYMap = std::pair<int,IntSet>;
    for ( const XYMap& xys : xyset)
    {
        const int X = xys.first;
        const IntSet& yset = xys.second;   // Vertices connected to X
        for ( int Y : yset)
        {
            if ( xyset.count(Y))
            {
                const IntSet& zset = xyset.at(Y);
                for ( int Z : zset)
                {
                    if ( yset.count(Z))
                        _setFace(X,Y,Z);
                }   // end for
            }   // end if
        }   // end for
    }   // end for
}   // end addFaces


bool EdgeFaceAdder::_setFace( int x, int y, int z)
{
    _init(x,y);
    _init(x,z);
    _init(y,z);
    const int xy = _edgeUse[x][y];
    const int xz = _edgeUse[x][z];
    const int yz = _edgeUse[y][z];

    // Reject faces that cannot be added because an edge is already shared by 2 faces.
    if ( xy == 2 || xz == 2 || yz == 2)
        return false;

    const int edgeSum = xy + xz + yz;

    // If only a single edge of the proposed triangle is shared by another face, just add the new triangle.
    // Alternatively, add triangle if no triangles already shared by the given edges themselves share a common edge.
    if ( edgeSum <= 1 || ( edgeSum >= 2 && !_areSharedFacesJoined( x, y, z)))
    {
        _addTriangle( x, y, z);
        return true;
    }   // end if

    return false;
}   // end _setFace


bool EdgeFaceAdder::_areSharedFacesJoined( int x, int y, int z) const
{
    int fxy = -1;
    if ( _mesh.hasEdge(x,y))
        fxy = *_mesh.sfaces( x,y).begin();

    int fxz = -1;
    if ( _mesh.hasEdge(x,z))
        fxz = *_mesh.sfaces( x,z).begin();

    int fyz = -1;
    if ( _mesh.hasEdge(y,z))
        fyz = *_mesh.sfaces( y,z).begin();

    const Face* pxy = fxy >= 0 ? &_mesh.face(fxy) : nullptr;
    const Face* pyz = fyz >= 0 ? &_mesh.face(fyz) : nullptr;
    const Face* pxz = fxz >= 0 ? &_mesh.face(fxz) : nullptr;

    return (pxy && pxz && pxy->isAdjacent( *pxz))
        || (pxz && pyz && pxz->isAdjacent( *pyz))
        || (pyz && pxy && pyz->isAdjacent( *pxy));
}   // end _areSharedFacesJoined


void EdgeFaceAdder::_init( int x, int y)
{
    if ( !_edgeUse.count(x) || !_edgeUse[x].count(y))
        _edgeUse[x][y] = 0;
}   // end _init


void EdgeFaceAdder::_addTriangle( int x, int y, int z)
{
    assert( _mesh.face( x, y, z) == -1);
    _mesh.addFace( x,y,z);
    _edgeUse[x][y]++;
    _edgeUse[x][z]++;
    _edgeUse[y][z]++;
}   // end addTriangle
