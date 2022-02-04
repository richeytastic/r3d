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
    "${INCLUDE_F}/DirectedSurfacePointFinder.h"
    "${INCLUDE_F}/EdgeFaceAdder.h"
    "${INCLUDE_F}/FaceAngles.h"
    "${INCLUDE_F}/FaceInterpolator.h"
    "${INCLUDE_F}/FaceParser.h"
    "${INCLUDE_F}/FacetCut.h"
    #"${INCLUDE_F}/FaceRemover.h"
    "${INCLUDE_F}/FaceUnfolder.h"
    "${INCLUDE_F}/FaceUnfoldingVertexSearcher.h"
    "${INCLUDE_F}/FastMarcher.h"
    "${INCLUDE_F}/FrontFinder.h"
    "${INCLUDE_F}/FunctionMapper.h"
    "${INCLUDE_F}/HoleFiller.h"
    "${INCLUDE_F}/Image.h"
    "${INCLUDE_F}/Internal.h"
    "${INCLUDE_F}/KDTree.h"
    "${INCLUDE_F}/Manifolds.h"
    "${INCLUDE_F}/Mesh.h"
    "${INCLUDE_F}/Orientation.h"
    "${INCLUDE_F}/PatchBendingEnergy.h"
    "${INCLUDE_F}/PlanarSlicingPath.h"
    "${INCLUDE_F}/ProcrustesSuperimposition.h"
    "${INCLUDE_F}/PPFMatcher.h"
    "${INCLUDE_F}/Reflector.h"
    "${INCLUDE_F}/RegionSelector.h"
    "${INCLUDE_F}/Remesher.h"
    "${INCLUDE_F}/Slicer.h"
    "${INCLUDE_F}/Smoother.h"
    "${INCLUDE_F}/SurfaceCurveFinder.h"
    "${INCLUDE_F}/SurfacePlanarPathFinder.h"
    "${INCLUDE_F}/SurfacePathFinder.h"
    "${INCLUDE_F}/SurfacePointFinder.h"
    "${INCLUDE_F}/TetrahedronReplacer.h"
    "${INCLUDE_F}/Transformer.h"
    #"${INCLUDE_F}/VertexAdder.h"
    "${INCLUDE_F}/VertexCrossingTimeCalculator.h"
    "${INCLUDE_F}/VectorPCFinder.h"
    )

set( SRC_FILES
    "${SRC_DIR}/r3dTypes.cpp"
    "${SRC_DIR}/AStarSearch.cpp"
    "${SRC_DIR}/Boundaries.cpp"
    "${SRC_DIR}/Bounds.cpp"
    "${SRC_DIR}/CameraParams.cpp"
    #"${SRC_DIR}/Cleaner.cpp"
    "${SRC_DIR}/Copier.cpp"
    "${SRC_DIR}/Curvature.cpp"
    "${SRC_DIR}/CurvatureMetrics.cpp"
    "${SRC_DIR}/DirectedSurfacePointFinder.cpp"
    "${SRC_DIR}/EdgeFaceAdder.cpp"
    "${SRC_DIR}/FaceAngles.cpp"
    "${SRC_DIR}/FaceInterpolator.cpp"
    "${SRC_DIR}/FaceParser.cpp"
    "${SRC_DIR}/FacetCut.cpp"
    #"${SRC_DIR}/FaceRemover.cpp"
    "${SRC_DIR}/FaceUnfolder.cpp"
    "${SRC_DIR}/FaceUnfoldingVertexSearcher.cpp"
    "${SRC_DIR}/FastMarcher.cpp"
    "${SRC_DIR}/FrontFinder.cpp"
    "${SRC_DIR}/FunctionMapper.cpp"
    "${SRC_DIR}/HoleFiller.cpp"
    "${SRC_DIR}/Image.cpp"
    "${SRC_DIR}/Internal.cpp"
    "${SRC_DIR}/KDTree.cpp"
    "${SRC_DIR}/Manifolds.cpp"
    "${SRC_DIR}/Mesh.cpp"
    "${SRC_DIR}/Orientation.cpp"
    "${SRC_DIR}/PatchBendingEnergy.cpp"
    "${SRC_DIR}/PlanarSlicingPath.cpp"
    "${SRC_DIR}/ProcrustesSuperimposition.cpp"
    "${SRC_DIR}/PPFMatcher.cpp"
    "${SRC_DIR}/Reflector.cpp"
    "${SRC_DIR}/RegionSelector.cpp"
    "${SRC_DIR}/Remesher.cpp"
    "${SRC_DIR}/Slicer.cpp"
    "${SRC_DIR}/Smoother.cpp"
    "${SRC_DIR}/SurfaceCurveFinder.cpp"
    "${SRC_DIR}/SurfacePlanarPathFinder.cpp"
    "${SRC_DIR}/SurfacePathFinder.cpp"
    "${SRC_DIR}/SurfacePointFinder.cpp"
    "${SRC_DIR}/TetrahedronReplacer.cpp"
    "${SRC_DIR}/Transformer.cpp"
    #"${SRC_DIR}/VertexAdder.cpp"
    "${SRC_DIR}/VertexCrossingTimeCalculator.cpp"
    "${SRC_DIR}/VectorPCFinder.cpp"
    )

add_library( ${PROJECT_NAME} ${SRC_FILES} ${INCLUDE_FILES})
include( "cmake/LinkLibs.cmake")
