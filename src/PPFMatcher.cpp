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

#include <PPFMatcher.h>
#include <opencv2/surface_matching/icp.hpp>
#include <opencv2/core/eigen.hpp>
using r3d::PPFMatcher;
using r3d::FeatMat;
using r3d::Mat4f;


PPFMatcher::PPFMatcher( const FeatMat& m) : _tgt( m)
{
}   // end ctor


Mat4f PPFMatcher::operator()( const FeatMat &s) const
{
    assert( s.cols() == _tgt.cols());
    cv::Mat src, tgt;
    cv::eigen2cv<float>(   s, src);
    cv::eigen2cv<float>(_tgt, tgt);

    double residual;
    cv::Matx44d pose;
    cv::ppf_match_3d::ICP icp;
    icp.registerModelToScene( src, tgt, residual, pose);

    Mat4d epose;
    cv::cv2eigen<double, 4, 4>( pose, epose);

    return epose.cast<float>();
}   // end operator()
