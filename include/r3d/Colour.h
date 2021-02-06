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

#ifndef R3D_COLOUR_H
#define R3D_COLOUR_H

#include "r3d_Export.h"
#include <cstdlib>

namespace r3d {

class r3d_EXPORT Colour
{
public:
    Colour();
    Colour( const Colour&) = default;
    Colour &operator=( const Colour&) = default;

    Colour( int, int, int);             // Values in [0,255]
    Colour( size_t, size_t, size_t);    // Values in [0,255]

    Colour( double, double, double);    // Values in [0,1]
    explicit Colour( const float*);
    explicit Colour( const double*);

    double& operator[]( int);
    const double& operator[]( int) const;

    static Colour hsv2rgb( const Colour&);
    static Colour rgb2hsv( const Colour&);

    static Colour white();
    static Colour black();
    static Colour red();
    static Colour green();
    static Colour blue();

private:
    double _vals[3];
};  // end class

}   // end namespace

#endif

