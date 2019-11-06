/************************************************************************
 * Copyright (C) 2017 Richard Palmer
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

/**
 * Collapse polygons in models that are triangulated meshes
 * without leaving holes - maintaining the model's triangulation.
 */

#ifndef RFEATURES_OBJ_MODEL_POLYGON_COLLAPSER_H
#define RFEATURES_OBJ_MODEL_POLYGON_COLLAPSER_H

#include "ObjModel.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelPolygonCollapser
{
public:
    explicit ObjModelPolygonCollapser( ObjModel::Ptr);

    ObjModel::Ptr model() const { return _model;}

    // Collapse the given polygon to a point (removing it in the process).
    // Maintains the model's triangulation. Returns the number of other polygons
    // shared by the collapsed polygon. Returns -1 if fid is not a valid polygon
    // and -2 if there is not a valid triangulation around the polygon.
    int collapse( int fid);

private:
    ObjModel::Ptr _model;
};  // end class

}   // end namespace

#endif
