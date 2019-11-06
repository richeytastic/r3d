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

#include <FunctionMapper.h>
using r3d::FunctionMapper;
using r3d::Mesh;
#include <iostream>
#include <iomanip>


// public
FunctionMapper::FunctionMapper( int rows, int cols, float xo, float yo, float xs, float ys)
    : _zvals( rows, cols), _useLiteral(false), _vidxs( rows, cols), _xoffset(xo), _yoffset(yo), _xscale(xs), _yscale(ys) {}


// public
FunctionMapper::FunctionMapper( const cv::Mat_<float>& litZ)
    : _zvals(litZ), _useLiteral(true), _vidxs( litZ.rows, litZ.cols), _xoffset(0), _yoffset(0), _xscale(0), _yscale(0) {}


// public
FunctionMapper::~FunctionMapper() {}


// public
bool FunctionMapper::textureMap( const std::string& txfile)
{
    const cv::Mat m = cv::imread( txfile);
    return textureMap( m);
}   // end textureMap


// public
bool FunctionMapper::textureMap( const cv::Mat& m)
{
    if ( m.empty() || !_mesh)
        return false;

    _mesh->removeAllMaterials();
    const int mid = _mesh->addMaterial( m);

    const int nrows = _vidxs.rows;
    const int ncols = _vidxs.cols;

    const float txstep = 1.0f/ncols;
    const float tystep = 1.0f/nrows;

    int fid = 0;
    for ( int i = 1; i < nrows; ++i)
    {
        for ( int j = 1; j < ncols; ++j)
        {
            const Vec2f uv0( tystep*i, txstep*j);
            const Vec2f uv1( uv0[0], txstep*(j-1));
            const Vec2f uv2( tystep*(i-1), uv1[1]);
            const Vec2f uv3( uv2[0], uv0[1]);
            _mesh->setOrderedFaceUVs( mid, fid++, uv0, uv1, uv2);
            _mesh->setOrderedFaceUVs( mid, fid++, uv0, uv3, uv2);
        }   // end for
    }   // end for

    return true;
}   // end textureMap


// public
Mesh::Ptr FunctionMapper::map()
{
    _mesh = Mesh::create();
    const int nrows = _zvals.rows;
    const int ncols = _zvals.cols;

    const bool mapLiteralZ = _useLiteral;
    float x, y, z;
    const float xoffset = _xoffset;
    const float yoffset = _yoffset;
    const float xstep = _xscale/ncols;
    const float ystep = _yscale/nrows;
    const int m = -nrows/2;
    const int n = -ncols/2;

    _vidxs = cv::Mat_<int>( nrows, ncols);

    for ( int i = 0; i < nrows; ++i)
    {
        y = ystep*(m+i) + yoffset;  // Only used if mapping literal z vals

        for ( int j = 0; j < ncols; ++j)
        {
            if ( mapLiteralZ)
                _vidxs(i,j) = _mesh->addVertex( (float)j, (float)i, (float)_zvals.at<float>(i,j));
            else
            {
                x = xstep*(n+j) + xoffset;
                z = _zvals.at<float>(i,j) = calcZ( x, y);
                _vidxs(i,j) = _mesh->addVertex( (float)x, (float)y, (float)z);
                //std::cerr << "fn( " << std::setw(6) << std::fixed << std::setprecision(4) << x
                //            << ", " << std::setw(6) << std::fixed << std::setprecision(4) << y
                //            << ") = " << std::fixed << std::setprecision(4) << z << std::endl;
            }   // end else

            if ( i > 0 && j > 0)
            {
                _mesh->addFace( _vidxs(i,j), _vidxs(i,j-1), _vidxs(i-1,j-1));
                _mesh->addFace( _vidxs(i,j), _vidxs(i-1,j), _vidxs(i-1,j-1));
            }   // end if
        }   // end for
    }   // end for

    return _mesh;
}   // end map


// protected virtual
float FunctionMapper::calcZ( float x, float y) const
{
    return 0.0;
}   // end calcZ
