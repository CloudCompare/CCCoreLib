# SPDX-License-Identifier: MIT
# Copyright © Andy Maloney <asmaloney@gmail.com>

target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/AutoSegmentationTools.cpp
		${CMAKE_CURRENT_LIST_DIR}/CCMiscTools.cpp
		${CMAKE_CURRENT_LIST_DIR}/CCShareable.cpp
		${CMAKE_CURRENT_LIST_DIR}/ChamferDistanceTransform.cpp
		${CMAKE_CURRENT_LIST_DIR}/Chi2Helper.h
		${CMAKE_CURRENT_LIST_DIR}/CloudSamplingTools.cpp
		${CMAKE_CURRENT_LIST_DIR}/Delaunay2dMesh.cpp
		${CMAKE_CURRENT_LIST_DIR}/DgmOctree.cpp
		${CMAKE_CURRENT_LIST_DIR}/DgmOctreeReferenceCloud.cpp
		${CMAKE_CURRENT_LIST_DIR}/DistanceComputationTools.cpp
		${CMAKE_CURRENT_LIST_DIR}/ErrorFunction.cpp
		${CMAKE_CURRENT_LIST_DIR}/FastMarching.cpp
		${CMAKE_CURRENT_LIST_DIR}/FastMarchingForPropagation.cpp
		${CMAKE_CURRENT_LIST_DIR}/GeometricalAnalysisTools.cpp
		${CMAKE_CURRENT_LIST_DIR}/GridAndMeshIntersection.cpp
		${CMAKE_CURRENT_LIST_DIR}/KdTree.cpp
		${CMAKE_CURRENT_LIST_DIR}/LocalModel.cpp
		${CMAKE_CURRENT_LIST_DIR}/ManualSegmentationTools.cpp
		${CMAKE_CURRENT_LIST_DIR}/MeshSamplingTools.cpp
		${CMAKE_CURRENT_LIST_DIR}/Neighbourhood.cpp
		${CMAKE_CURRENT_LIST_DIR}/NormalDistribution.cpp
		${CMAKE_CURRENT_LIST_DIR}/NormalizedProgress.cpp
		${CMAKE_CURRENT_LIST_DIR}/PointProjectionTools.cpp
		${CMAKE_CURRENT_LIST_DIR}/Polyline.cpp
		${CMAKE_CURRENT_LIST_DIR}/ReferenceCloud.cpp
		${CMAKE_CURRENT_LIST_DIR}/RegistrationTools.cpp
		${CMAKE_CURRENT_LIST_DIR}/SaitoSquaredDistanceTransform.cpp
		${CMAKE_CURRENT_LIST_DIR}/ScalarField.cpp
		${CMAKE_CURRENT_LIST_DIR}/ScalarFieldTools.cpp
		${CMAKE_CURRENT_LIST_DIR}/SimpleMesh.cpp
		${CMAKE_CURRENT_LIST_DIR}/StatisticalTestingTools.cpp
		${CMAKE_CURRENT_LIST_DIR}/TrueKdTree.cpp
		${CMAKE_CURRENT_LIST_DIR}/WeibullDistribution.cpp
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/src
)
