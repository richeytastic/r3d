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

#ifndef R3D_BOUNDS_H
#define R3D_BOUNDS_H

/**
 * Oriented bounding cuboid around a mesh or a subset of a mesh.
 */

#include "Mesh.h"

namespace r3d {

class r3d_EXPORT Bounds
{
public:
    using Ptr = std::shared_ptr<Bounds>;
    static Ptr create( const Vec3f& minCorner, const Vec3f& maxCorner, const Mat4f& m=Mat4f::Identity());
    static Ptr create( const Mesh&, const Mat4f& m=Mat4f::Identity(), const IntSet *vset=nullptr);
    static Ptr create( const Mesh&, const Mat4f&, const IntSet& fids);

    /**
     * Set the bounding corners of a cuboid directly. The corners represent the cuboid
     * in its transformed position due to the provided matrix. That is, applying the inverse
     * of m to the corner vertices should give the cuboid region in standard position.
     */
    Bounds( const Vec3f& minCorner, const Vec3f& maxCorner, const Mat4f& m=Mat4f::Identity());

    /**
     * Find the bounds of the given mesh over the given set of vertices (or all vertices if null).
     * The matrix gives the orientation of the mesh and the bounds are computed with respect to this.
     * Note that this is a semantically different transformation matrix than the mesh's own transform
     * which is used only for its efficient transformation without updating vertices. Calculation of the
     * bounds proceeds by finding the upright bounds by checking each vertex of the mesh transformed by
     * the inverse of the provided matrix, then setting the transform for these bounds to the provided matrix.
     */
    Bounds( const Mesh&, const Mat4f& m=Mat4f::Identity(), const IntSet *vset=nullptr);

    /**
     * Find the bounds of the given mesh over the given set of faces.
     */
    Bounds( const Mesh&, const Mat4f&, const IntSet&);

    /**
     * Copy constructors.
     */
    Bounds( const Bounds&) = default;
    Bounds& operator=( const Bounds&) = default;
    Ptr deepCopy() const;

    /**
     * Set (overwriting) the existing transform to specify the position and orientation of these bounds.
     */
    void setTransformMatrix( const Mat4f&);

    /**
     * Set the transform by adding a transform after the existing one. If the existing
     * transform is the identity matrix then this is equivalent to setTransform.
     */
    void addTransformMatrix( const Mat4f&);

    /**
     * Return the transform matrix being applied to the bounds.
     */
    const Mat4f& transformMatrix() const { return _tmat;}

    /**
     * Rebaseline the corners of the cuboid according to the transformed corners (i.e. fix in place
     * the transformed corners), and reset the internal transform matrix to the identity matrix.
     */
    void fixTransformMatrix();

    /**
     * Make this Bounds a union with the given bounds so that on return this object
     * has bounds that encompass the parameter bounds (note that the transform matrix on the
     * parameter bounds may be different to this one's).
     */
    void encompass( const Bounds&);

    /**
     * Returns true iff the parameter bounds intersects with this one.
     */
    bool intersects( const Bounds&) const;

    /**
     * Return the transformed min and max corners of this bounding region.
     */
    Vec3f minCorner() const;
    Vec3f maxCorner() const;

    /**
     * Return the min and max corners representing the cuboid region containing the transformed
     * bounds. These corners change as the transform matrix is updated.
     */
    inline const Vec3f& minExtent() const { return _mine;}
    inline const Vec3f& maxExtent() const { return _maxe;}

    /**
     * Return the centre of the bounding box.
     */
    Vec3f centre() const;

    /**
     * Return the lengths of the three sides of the cuboid.
     */
    float xlen() const;
    float ylen() const;
    float zlen() const;

    /**
     * Return the diagonal distance.
     */
    float diagonal() const;
        
    /**
     * Return the UNTRANSFORMED corners as: X_min, X_max, Y_min, Y_max, Z_min, Z_max.
     */
    Vec6f cornersAs6f() const;

    /**
     * Return the extents as: X_min, X_max, Y_min, Y_max, Z_min, Z_max.
     */
    Vec6f extentsAs6f() const;

    /**
     * Return the two corners as a single vector with elements: X_min, X_max, Y_min, Y_max, Z_min, Z_max.
     */
    static Vec6f as6f( const Vec3f& minc, const Vec3f& maxc);

private:
    Vec3f _minc, _maxc; // Min/max corners
    Vec3f _mine, _maxe; // Min/max extents
    Mat4f _tmat, _imat;
    void _init( const Mesh&, const Mat4f&, const IntSet&);
    void _calcExtents();
};  // end class

}   // end namespace

#endif
