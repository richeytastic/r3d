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

#include <Orientation.h>
#include <boost/property_tree/xml_parser.hpp>
using r3d::Orientation;
using r3d::PTree;
using r3d::Vec3f;
using r3d::Mat4f;
using r3d::Mat3f;

Orientation::Orientation() : _nvec(0,0,1), _uvec(0,1,0) { }

Orientation::Orientation( const PTree& ptree) { ptree >> *this;}

Orientation::Orientation( const Vec3f& n, const Vec3f& u)
{
    setN(n);
    setU(u);
}   // end ctor

Orientation::Orientation( const Mat4f& m)
{
    setN( m.block<3,1>(0,2));
    setU( m.block<3,1>(0,1));
}   // end ctor


void Orientation::rotate( const Mat3f& T)
{
    _nvec = T * _nvec;
    _uvec = T * _uvec;
}   // end rotate


Mat4f Orientation::asMatrix( const Vec3f &t) const
{
    Vec3f xvec = _uvec.cross(_nvec);
    xvec.normalize();
    Mat4f T;
    T.col(0) << xvec, 0.0f;
    T.col(1) << _uvec, 0.0f;
    T.col(2) << _nvec, 0.0f;
    T.col(3) << t, 1.0f;
    return T;
}   // end asMatrix


bool Orientation::operator==( const Orientation& o) const
{
    return _nvec == o._nvec && _uvec == o._uvec;
}   // end operator==


void r3d::putVertex( PTree& node, const Vec3f& v)
{
    node.put( "x", v[0]);
    node.put( "y", v[1]);
    node.put( "z", v[2]);
}   // end putVertex

void r3d::putNamedVertex( PTree& node, const std::string& label, const Vec3f& v)
{
    putVertex( node.put( label, ""), v);
}   // end putNamedVertex


Vec3f r3d::getVertex( const PTree& node)
{
    return Vec3f( node.get<float>("x"), node.get<float>("y"), node.get<float>("z"));
}   // end getVertex


bool r3d::getNamedVertex( const PTree& n0, const std::string& label, Vec3f& v)
{
    if ( n0.count(label) == 0)
        return false;
    v = getVertex( n0.get_child(label));
    return true;
}   // end getNamedVertex


// public
PTree& r3d::operator<<( PTree& orientation, const Orientation& v)
{
    PTree& normal = orientation.add( "normal","");
    putVertex( normal, v.nvec());
    PTree& upnode = orientation.add( "up","");
    putVertex( upnode, v.uvec());
    return orientation;
}   // end operator<<


// public
const PTree& r3d::operator>>( const PTree& orientation, Orientation& v)
{
    v.setN( getVertex( orientation.get_child("normal")));
    v.setU( getVertex( orientation.get_child("up")));
    return orientation;
}   // end operator>>


// public
std::ostream& r3d::operator<<( std::ostream& os, const Orientation& v)
{
    os << v.nvec() << v.uvec();
    return os;
}   // end operator<<
