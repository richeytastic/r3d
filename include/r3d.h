/************************************************************************
 * Copyright (C) 2021 Richard Palmer
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

#ifndef R3D_H
#define R3D_H

#include "r3d/r3dTypes.h"
#include "r3d/AStarSearch.h"
#include "r3d/Boundaries.h"
#include "r3d/Bounds.h"
#include "r3d/CameraParams.h"
#include "r3d/Copier.h"
#include "r3d/Curvature.h"
#include "r3d/CurvatureMetrics.h"
#include "r3d/EdgeFaceAdder.h"
#include "r3d/FaceAngles.h"
#include "r3d/FaceInterpolator.h"
#include "r3d/FaceParser.h"
#include "r3d/FacePlane.h"
#include "r3d/FaceUnfolder.h"
#include "r3d/FaceUnfoldingVertexSearcher.h"
#include "r3d/FastMarcher.h"
#include "r3d/FrontFinder.h"
#include "r3d/FunctionMapper.h"
#include "r3d/GlobalPlaneSlicingPath.h"
#include "r3d/HoleFiller.h"
#include "r3d/Image.h"
#include "r3d/Internal.h"
#include "r3d/LocalPlaneSlicingPath.h"
#include "r3d/Manifolds.h"
#include "r3d/Orientation.h"
#include "r3d/PatchBendingEnergy.h"
#include "r3d/PlaneSlicingPath.h"
#include "r3d/ProcrustesSuperimposition.h"
#include "r3d/Reflector.h"
#include "r3d/RegionSelector.h"
#include "r3d/Remesher.h"
#include "r3d/Slicer.h"
#include "r3d/Smoother.h"
#include "r3d/SurfaceCurveFinder.h"
#include "r3d/SurfaceGlobalPlanePathFinder.h"
#include "r3d/SurfaceLocalPlanePathFinder.h"
#include "r3d/SurfacePathFinder.h"
#include "r3d/SurfacePointFinder.h"
#include "r3d/TetrahedronReplacer.h"
#include "r3d/Transformer.h"
#include "r3d/VertexCrossingTimeCalculator.h"
#include "r3d/VectorPCFinder.h"

#endif
