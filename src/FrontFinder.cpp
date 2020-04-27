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

#include <FrontFinder.h>
#include <cassert>
using r3d::FrontFinder;


FrontFinder::FrontFinder( const r3d::Mesh &mesh)
{
    assert( mesh.hasSequentialFaceIds());
    for ( int fid : mesh.faces())
    {
        _fnrm[fid] = mesh.calcFaceNorm( fid, true);
        const int *fvidxs = mesh.fvidxs(fid);
        _fpos[fid] = (mesh.vtx(fvidxs[0]) + mesh.vtx(fvidxs[1]) + mesh.vtx(fvidxs[2]))/3;
    }   // end for
}   // end ctor


bool FrontFinder::_updatePosAndNorm()
{
    const size_t n = _ifids.size();
    if ( n > 0)
    {
        _mnrm = _mpos = Vec3f::Zero();
        for ( int fid : _ifids)
        {
            _mpos += _fpos.at( fid);
            _mnrm += _fnrm.at( fid);
        }   // end for
        _mpos /= float(n);
        _mnrm /= float(n);
    }   // end if

    return n > 0;
}   // end _updatePosAndNorm


const r3d::Vec3f& FrontFinder::operator()()
{
    _ifids.clear();
    for ( const auto& p : _fnrm)
        _ifids.push_back( p.first);

    size_t oldSize = UINT_MAX;
    while ( _ifids.size() < oldSize && _ifids.size() > 1)
    {
        _updatePosAndNorm();
        //std::cout << "pos: " << _mpos.transpose() << ", nrm: " << _mnrm.transpose();

        // Calculate variance of each face normal from current inlier normal.
        std::unordered_map<int, float> fvars;
        float var = 0.0f;
        for ( int fid : _ifids)
        {
            const Vec3f &fnrm = _fnrm.at(fid);
            fvars[fid] = (_mnrm - fnrm).squaredNorm();
            var += fvars[fid];
        }   // end for
        var /= fvars.size();

        // Use only faces with normals in same general direction as current mean normal.
        static const float INLIER_SIGMA_SQ = 4.0f;
        const float VAR_THRESH = INLIER_SIGMA_SQ * var;
        oldSize = _ifids.size();
        _ifids.clear();
        for ( const auto& p : fvars)
        {
            if ( p.second <= VAR_THRESH)
            {
                const Vec3f dv = _fpos.at( p.first) - _mpos;
                if ( dv.dot(_mnrm) > 0) // Front half faces only
                    _ifids.push_back( p.first);
            }   // end if
        }   // end for

        //std::cout << ", inliers reduced by " << 100*(1.0f - float(_ifids.size())/oldSize) << std::endl;
    }   // end while

    return _mpos;
}   // end operator()
