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

#ifndef R3D_FUNCTION_MAPPER_H
#define R3D_FUNCTION_MAPPER_H

#include "Mesh.h"

namespace r3d {

class r3d_EXPORT FunctionMapper
{
public:
    FunctionMapper( int rows, int cols,                     // Mapping resolution
                    float xoffset=0.0f, float yoffset=0.0f, // Function origin
                    float xscale=1.0f, float yscale=1.0f);  // Function scale parameters

    // Provide literal z values at x,y column,row indices instead of function(x,y) calculated.
    explicit FunctionMapper( const cv::Mat_<float>& literalZ);

    virtual ~FunctionMapper();

    Mesh::Ptr map();    // Map the function and return the mesh.
    Mesh::Ptr mesh() const { return _mesh;}   // Returns the last mesh made from map().

    // Textures the mesh. Returns false if mesh not yet created or texture empty.
    bool textureMap( const std::string& txfile);
    bool textureMap( const cv::Mat& m);

    // Returned matrix contains function mapped values after call to map().
    // If constructed using second constructor, this simply returns the parameter
    // to the second constructor.
    inline const cv::Mat_<float>& getFunctionMap() const { return _zvals;}

protected:
    virtual float calcZ( float x, float y) const;    // Default is to return 0 for every input.

private:
    cv::Mat_<float> _zvals;
    const bool _useLiteral;
    cv::Mat_<int> _vidxs;
    float _xoffset, _yoffset;
    float _xscale, _yscale;
    Mesh::Ptr _mesh;
};  // end class

}   // end namespace

#endif
