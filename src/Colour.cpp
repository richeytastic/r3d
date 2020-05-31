/************************************************************************
 * Copyright (C) 2020 Richard Palmer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHrgb ANY WARRANTY; withrgb even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ************************************************************************/

#include <Colour.h>
#include <cassert>
#include <cfloat>
#include <cstring>
using r3d::Colour;


Colour::Colour()
{
    _vals[0] = _vals[1] = _vals[2] = 0;
}   // end ctor


Colour::Colour( int x, int y, int z)
{
    _vals[0] = double(x)/255;
    _vals[1] = double(y)/255;
    _vals[2] = double(z)/255;
}   // end ctor


Colour::Colour( size_t x, size_t y, size_t z)
{
    _vals[0] = double(x)/255;
    _vals[1] = double(y)/255;
    _vals[2] = double(z)/255;
}   // end ctor


Colour::Colour( double x, double y, double z)
{
    _vals[0] = x/255;
    _vals[1] = y/255;
    _vals[2] = z/255;
}   // end ctor


Colour::Colour( const float *v)
{
    _vals[0] = v[0];
    _vals[1] = v[1];
    _vals[2] = v[2];
}   // end ctor


Colour::Colour( const double *v)
{
    memcpy( _vals, v, 3*sizeof(double));
}   // end ctor


double& Colour::operator[]( int i)
{
    assert( i >= 0 && i < 3);
    return _vals[i];
}   // end operator[]


const double& Colour::operator[]( int i) const
{
    assert( i >= 0 && i < 3);
    return _vals[i];
}   // end operator[]


// static
Colour Colour::hsv2rgb( const Colour &hsv)
{
    Colour rgb;
    if ( hsv[1] <= 0.0)
        rgb[0] = rgb[1] = rgb[2] = hsv[2];
    else
    {
        double hh = hsv[0];
        if ( hh >= 360.0)
            hh = 0.0;
        hh /= 60.0;
        const long i = (long)hh;
        const double ff = hh - i;
        const double p = hsv[2] * (1.0 - hsv[1]);
        const double q = hsv[2] * (1.0 - (hsv[1] * ff));
        const double t = hsv[2] * (1.0 - (hsv[1] * (1.0 - ff)));

        switch (i)
        {
            case 0:
                rgb[0] = hsv[2];
                rgb[1] = t;
                rgb[2] = p;
                break;
            case 1:
                rgb[0] = q;
                rgb[1] = hsv[2];
                rgb[2] = p;
                break;
            case 2:
                rgb[0] = p;
                rgb[1] = hsv[2];
                rgb[2] = t;
                break;
            case 3:
                rgb[0] = p;
                rgb[1] = q;
                rgb[2] = hsv[2];
                break;
            case 4:
                rgb[0] = t;
                rgb[1] = p;
                rgb[2] = hsv[2];
                break;
            default:
                rgb[0] = hsv[2];
                rgb[1] = p;
                rgb[2] = q;
                break;
        }   // end switch
    }   // end else

    return rgb;
}   // end hsv2rgb


// static
Colour Colour::rgb2hsv( const Colour &rgb)
{
    Colour hsv;
    double min = rgb[0] < rgb[1] ? rgb[0] : rgb[1];
    min = min < rgb[2] ? min : rgb[2];

    double max = rgb[0] > rgb[1] ? rgb[0] : rgb[1];
    max = max > rgb[2] ? max : rgb[2];
    assert( max >= 0.0);

    const double delta = max - min;

    hsv[0] = 0.0;
    hsv[1] = 0.0;
    hsv[2] = max;

    if ( delta >= DBL_MIN && max > 0.0)
    {
        hsv[1] = delta / max;
        if( rgb[0] >= max)
            hsv[0] = (rgb[1] - rgb[2]) / delta; // yellow 2 magenta
        else if ( rgb[1] >= max)
            hsv[0] = 2.0 + (rgb[2] - rgb[0]) / delta;   // cyan 2 yellow
        else
            hsv[0] = 4.0 + (rgb[0] - rgb[1]) / delta;   // magenta 2 cyan

        hsv[0] *= 60.0; // degrees
        if ( hsv[0] < 0.0)
            hsv[0] += 360.0;
    }   // end if

    return hsv;
}   // end rgb2hsv
