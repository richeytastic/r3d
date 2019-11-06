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

#ifndef RFEATURES_OBJ_MODEL_CLEANER_H
#define RFEATURES_OBJ_MODEL_CLEANER_H

#include "ObjModel.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelCleaner
{
public:
    explicit ObjModelCleaner( ObjModel::Ptr);

    // Modify the model's vertex connectivity to make it homeomorphic
    // to a 2D manifold according to the following steps:
    // 1) Identify all edges shared by 3 or more triangles.
    // 2) Identify the triangles on these edges that should have their vertices
    //    replaced.
    // 3) Create new replacement vertices for these triangles having slightly
    //    different positions (being fractionally further out from the "touching"
    //    surface of the other triangles).
    // 4) Identify single vertex joins by finding groups of triangles that
    //    connect via their edges.
    // 5) Create new "tip" vertices for these positions - a new one for each group
    //    of triangles locally connected to the vertex.
    // 6) Set each group of triangles to share a unique "tip" vertex.
    // The function returns the number of new vertices added.
    int makeSurface();

private:
    ObjModel::Ptr _model;

    ObjModelCleaner( const ObjModelCleaner&) = delete;
    void operator=( const ObjModelCleaner&) = delete;
};  // end class

}   // end namespace

#endif
