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
#include <algorithm>
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


Colour Colour::hsv2rgb( const Colour &hsv)
{
    const double h = hsv[0];
    const double s = hsv[1];
    const double v = hsv[2];

    const int hi = (int)(h / 60.0) % 6;
    const double f  = (h / 60.0) - hi;
    const double p  = v * (1.0 - s);
    const double q  = v * (1.0 - s * f);
    const double t  = v * (1.0 - s * (1.0 - f));

    double r = 0;
    double g = 0;
    double b = 0;
    switch (hi)
    {
        case 0: r = v, g = t, b = p; break;
        case 1: r = q, g = v, b = p; break;
        case 2: r = p, g = v, b = t; break;
        case 3: r = p, g = q, b = v; break;
        case 4: r = t, g = p, b = v; break;
        case 5: r = v, g = p, b = q; break;
    }   // end switch

    Colour rgb;
    rgb[0] = r;
    rgb[1] = g;
    rgb[2] = b;
    return rgb;
}   // end hsv2rgb


Colour Colour::rgb2hsv( const Colour &rgb)
{
    const double r = rgb[0];
    const double g = rgb[1];
    const double b = rgb[2];
    const double max = std::max( r, std::max(g, b));
    const double min = std::min( r, std::min(g, b));

    Colour hsv;
    hsv[2] = max;

    if (max == 0.0)
    {
        hsv[0] = 0;
        hsv[1] = 0;
    }   // end if
    else if (max - min == 0.0)
    {
        hsv[1] = 0;
        hsv[0] = 0;
    }   // end else if
    else
    {
        hsv[1] = (max - min) / max;
        if ( max == r)
            hsv[0] = 60 * ((g - b) / (max - min)) + 0;
        else if (max == g)
            hsv[0] = 60 * ((b - r) / (max - min)) + 120;
        else
            hsv[0] = 60 * ((r - g) / (max - min)) + 240;
    }   // end else

    if (hsv[0] < 0)
        hsv[0] += 360.0;

    return hsv;
}   // end rgb2hsv
