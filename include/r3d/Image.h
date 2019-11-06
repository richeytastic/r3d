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

#ifndef R3D_IMAGE_H
#define R3D_IMAGE_H

#include "r3d_Export.h"
#include <opencv2/opencv.hpp>
#include <vector>

namespace r3d {

// Shrink and return a copy of img to be no larger than md rows/cols.
r3d_EXPORT cv::Mat shrink2Max( const cv::Mat img, size_t md);


// Horizontally concatenate a bunch of cv::Mat (which must all be of the same type)
// into a single returned image where the height of the returned image is the maximum
// height (#rows) from all of the input images. Any input image having a smaller height
// is resized up. If not null, cols will be set with the column index that each
// concatentated (and possibily resized) image starts at in the returned image (cols
// is resized to be imgs.size()). This is useful so that position references to the
// input images can be recalculated to account for the new image sizes and positions.
// If the input images are not of the same type, an empty cv::Mat is returned.
r3d_EXPORT cv::Mat concatHorizontalMax( const std::vector<cv::Mat> &imgs, std::vector<int> *cols=nullptr);

}   // end namespace

#endif
