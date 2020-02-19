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

#ifndef R3D_FRONT_FINDER_H
#define R3D_FRONT_FINDER_H

#include "Mesh.h"

namespace r3d {

class r3d_EXPORT FrontFinder
{
public:
    explicit FrontFinder( const Mesh&);

    const Vec3f& operator()();

    inline const Vec3f &meanPos() const { return _mpos;}
    inline const Vec3f &meanNrm() const { return _mnrm;}

private:
    std::unordered_map<int, Vec3f> _fnrm;
    std::unordered_map<int, Vec3f> _fpos;
    std::vector<int> _ifids;
    Vec3f _mnrm, _mpos;

    bool _updatePosAndNorm();
    FrontFinder( const FrontFinder&) = delete;
    void operator=( const FrontFinder&) = delete;
};  // end class

}   // end namespace

#endif
