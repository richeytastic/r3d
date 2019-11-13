cmake_minimum_required(VERSION 3.12.2 FATAL_ERROR)

project( r3d)

set(WITH_BOOST TRUE)
set(WITH_EIGEN TRUE)
set(WITH_OPENCV TRUE)
set(WITH_NANOFLANN TRUE)
include( "cmake/FindLibs.cmake")

set( INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
set( INCLUDE_F "${INCLUDE_DIR}/${PROJECT_NAME}")
set( SRC_DIR "${PROJECT_SOURCE_DIR}/src")
include_directories( ${INCLUDE_DIR})
include_directories( ${INCLUDE_F})

set( INCLUDE_FILES
    "${INCLUDE_F}.h"
    "${INCLUDE_F}/r3dTypes.h"
    "${INCLUDE_F}/AStarSearch.h"
    "${INCLUDE_F}/Boundaries.h"
    "${INCLUDE_F}/Bounds.h"
    "${INCLUDE_F}/CameraParams.h"
    #"${INCLUDE_F}/Cleaner.h"
    "${INCLUDE_F}/Copier.h"
    "${INCLUDE_F}/Curvature.h"
    "${INCLUDE_F}/CurvatureMetrics.h"
    "${INCLUDE_F}/EdgeFaceAdder.h"
    "${INCLUDE_F}/FaceAngles.h"
    "${INCLUDE_F}/FaceInterpolator.h"
    "${INCLUDE_F}/FaceParser.h"
    "${INCLUDE_F}/FacePlane.h"
    #"${INCLUDE_F}/FaceRemover.h"
    "${INCLUDE_F}/FaceUnfolder.h"
    "${INCLUDE_F}/FaceUnfoldingVertexSearcher.h"
    "${INCLUDE_F}/FastMarcher.h"
    "${INCLUDE_F}/FunctionMapper.h"
    "${INCLUDE_F}/GlobalPlaneSlicingPath.h"
    "${INCLUDE_F}/HoleFiller.h"
    "${INCLUDE_F}/Image.h"
    "${INCLUDE_F}/Internal.h"
    "${INCLUDE_F}/KDTree.h"
    "${INCLUDE_F}/LocalPlaneSlicingPath.h"
    "${INCLUDE_F}/Manifolds.h"
    "${INCLUDE_F}/Mesh.h"
    "${INCLUDE_F}/Orientation.h"
    "${INCLUDE_F}/PatchBendingEnergy.h"
    "${INCLUDE_F}/PlaneSlicingPath.h"
    "${INCLUDE_F}/ProcrustesSuperimposition.h"
    "${INCLUDE_F}/PPFMatcher.h"
    "${INCLUDE_F}/Reflector.h"
    "${INCLUDE_F}/RegionSelector.h"
    "${INCLUDE_F}/Remesher.h"
    "${INCLUDE_F}/Slicer.h"
    "${INCLUDE_F}/Smoother.h"
    "${INCLUDE_F}/SurfaceCurveFinder.h"
    "${INCLUDE_F}/SurfaceGlobalPlanePathFinder.h"
    "${INCLUDE_F}/SurfaceLocalPlanePathFinder.h"
    "${INCLUDE_F}/SurfacePathFinder.h"
    "${INCLUDE_F}/SurfacePointFinder.h"
    "${INCLUDE_F}/TetrahedronReplacer.h"
    "${INCLUDE_F}/Transformer.h"
    #"${INCLUDE_F}/VertexAdder.h"
    "${INCLUDE_F}/VertexCrossingTimeCalculator.h"
    "${INCLUDE_F}/VertexPCFinder.h"
    )

set( SRC_FILES
    ${SRC_DIR}/r3dTypes
    ${SRC_DIR}/AStarSearch
    ${SRC_DIR}/Boundaries
    ${SRC_DIR}/Bounds
    ${SRC_DIR}/CameraParams
    #${SRC_DIR}/Cleaner
    ${SRC_DIR}/Copier
    ${SRC_DIR}/Curvature
    ${SRC_DIR}/CurvatureMetrics
    ${SRC_DIR}/EdgeFaceAdder
    ${SRC_DIR}/FaceAngles
    ${SRC_DIR}/FaceInterpolator
    ${SRC_DIR}/FaceParser
    ${SRC_DIR}/FacePlane
    #${SRC_DIR}/FaceRemover
    ${SRC_DIR}/FaceUnfolder
    ${SRC_DIR}/FaceUnfoldingVertexSearcher
    ${SRC_DIR}/FastMarcher
    ${SRC_DIR}/FunctionMapper
    ${SRC_DIR}/HoleFiller
    ${SRC_DIR}/Image
    ${SRC_DIR}/Internal
    ${SRC_DIR}/KDTree
    ${SRC_DIR}/LocalPlaneSlicingPath
    ${SRC_DIR}/Manifolds
    ${SRC_DIR}/Mesh
    ${SRC_DIR}/Orientation
    ${SRC_DIR}/PatchBendingEnergy
    ${SRC_DIR}/PlaneSlicingPath
    ${SRC_DIR}/ProcrustesSuperimposition
    ${SRC_DIR}/PPFMatcher
    ${SRC_DIR}/Reflector
    ${SRC_DIR}/RegionSelector
    ${SRC_DIR}/Remesher
    ${SRC_DIR}/Slicer
    ${SRC_DIR}/Smoother
    ${SRC_DIR}/SurfaceCurveFinder
    ${SRC_DIR}/SurfaceGlobalPlanePathFinder
    ${SRC_DIR}/SurfaceLocalPlanePathFinder
    ${SRC_DIR}/SurfacePathFinder
    ${SRC_DIR}/SurfacePointFinder
    ${SRC_DIR}/TetrahedronReplacer
    ${SRC_DIR}/Transformer
    #${SRC_DIR}/VertexAdder
    ${SRC_DIR}/VertexCrossingTimeCalculator
    ${SRC_DIR}/VertexPCFinder
    )

add_library( ${PROJECT_NAME} ${SRC_FILES} ${INCLUDE_FILES})
include( "cmake/LinkLibs.cmake")
