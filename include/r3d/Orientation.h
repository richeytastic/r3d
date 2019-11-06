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

#ifndef R3D_ORIENTATION_H
#define R3D_ORIENTATION_H

#ifdef _WIN32
#pragma warning( disable : 4251)
#endif

#include "r3dTypes.h"
#include <boost/property_tree/ptree.hpp>

namespace r3d {

using PTree = boost::property_tree::ptree;

class r3d_EXPORT Orientation
{
public:
    /**
     * Create a default orientation object with normal +Z (i.e. [0,0,1]) and up vector as +Y (i.e. [0,1,0]).
     */
    Orientation();

    /**
     * Instantiate from a property tree.
     */
    explicit Orientation( const PTree&);
    Orientation( const Vec3f& nvec, const Vec3f& uvec);

    /**
     * Get the normal vector as the normalised first 3 elements as the 3rd column,
     * and the up vector as the normalised first 3 elements of the 2nd column.
     */
    Orientation( const Mat4f&);

    /**
     * Set the normal and up vectors.
     */
    void setN( const Vec3f& n) { _nvec = n; _nvec.normalize();}
    void setU( const Vec3f& u) { _uvec = u; _uvec.normalize();}

    const Vec3f& nvec() const { return _nvec;}
    const Vec3f& uvec() const { return _uvec;}

    /**
     * Rotate this orientation by the given rotation matrix.
     */
    void rotate( const Mat3f&);

    /**
     * Return this orientation object as a homogeneous matrix with optional translation.
     */
    Mat4f asMatrix( const Vec3f& t=Vec3f::Zero()) const;

    bool operator==( const Orientation&) const;
    bool operator!=( const Orientation& o) const { return !operator==(o);}

private:
    Vec3f _nvec, _uvec;     // normal and up vector
};  // end class

r3d_EXPORT PTree& operator<<( PTree&, const Orientation&);        // Orientation writer to PTree
r3d_EXPORT const PTree& operator>>( const PTree&, Orientation&);  // Orientation reader from PTree

r3d_EXPORT void putVertex( PTree&, const Vec3f&); // Creates keyed values "x", "y", and "z" in the provided record.

r3d_EXPORT void putNamedVertex( PTree&, const std::string&, const Vec3f&); // Sets child node with given label to vertex.

r3d_EXPORT Vec3f getVertex( const PTree&);        // Uses keyed values "x", "y", and "z" to create the returned vertex.

// Sets v to the named vertex if found in the given PTree and returns true if found (false if not).
r3d_EXPORT bool getNamedVertex( const PTree&, const std::string&, Vec3f& v);

r3d_EXPORT std::ostream& operator<<( std::ostream&, const Orientation&);

}   // end namespace

#endif
