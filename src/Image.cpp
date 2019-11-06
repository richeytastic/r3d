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

#include <Image.h>
using namespace r3d;


cv::Mat r3d::shrink2Max( const cv::Mat img, size_t md)
{
    if ( img.rows < (int)md && img.cols < (int)md)
        return img.clone();
    const double sf = img.rows > img.cols ? double(md)/img.rows : double(md)/img.cols;
    cv::Mat outimg;
    cv::resize( img, outimg, outimg.size(), sf, sf, cv::INTER_AREA);
    return outimg;
}   // end shrink2Max


cv::Mat r3d::concatHorizontalMax( const std::vector<cv::Mat>& imgs, std::vector<int>* cols)
{
    if ( imgs.empty())
        return cv::Mat();

    // Get the max height of the input images and check that they're all the same type.
    const int mtype = imgs[0].type();
    const int n = (int)imgs.size();
    int nrows = 0;
    for ( int i = 0; i < n; ++i)
    {
        nrows = std::max(nrows, imgs[i].rows);
        if ( imgs[i].type() != mtype)
            return cv::Mat();
    }   // end for

    int ncols = 0;  // Will be total columns
    std::vector<int> resizedWidths(n);  // Get the new width of every image
    for ( int i = 0; i < n; ++i)
    {
        const double sf = double(nrows) / imgs[i].rows; // Scale factor to keep aspect ratio on images intact
        resizedWidths[i] = (int)sf*imgs[i].cols;
        ncols += resizedWidths[i];
    }   // end for

    cv::Mat cmat( nrows, ncols, mtype); // Create the final concatenated image to return.
    const cv::Range rowRange(0,nrows);
    int cleft = 0;
    for ( int i = 0; i < n; ++i)
    {
        const cv::Size reqSize( resizedWidths[i], nrows);
        const cv::Range colRange( cleft, cleft + reqSize.width);
        cleft += reqSize.width;
        // Resize each image into the right location of cmat
        // cv::INTER_LINEAR "faster but still looks okay" (use cv::INTER_CUBIC for slower but better looking)
        cv::resize( imgs[i], cmat( rowRange, colRange), reqSize, 0, 0, cv::INTER_CUBIC);
    }   // end for

    // Copy starting indices on the new image
    if ( cols != nullptr)
    {
        cols->resize(imgs.size());
        (*cols)[0] = 0;
        for ( int i = 1; i < n; ++i)
            (*cols)[i] = (*cols)[i-1] + resizedWidths[i-1];
    }   // end if

    return cmat;
}   // end concatHorizontalMax
