// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include <DistanceComputationTools.h>

//local
#include <DgmOctreeReferenceCloud.h>
#include <FastMarchingForPropagation.h>
#include <LocalModel.h>
#include <PointCloud.h>
#include <Polyline.h>
#include <ReferenceCloud.h>
#include <SaitoSquaredDistanceTransform.h>
#include <ScalarField.h>
#include <ScalarFieldTools.h>
#include <SimpleTriangle.h>
#include <SquareMatrix.h>

//system
#include <algorithm>
#include <cassert>

#ifndef CC_DEBUG
#if defined(CC_CORE_LIB_USES_QT_CONCURRENT)
#define ENABLE_CLOUD2MESH_DIST_MT
#include <QtConcurrentMap>
#include <QtCore>
#elif defined(CC_CORE_LIB_USES_TBB)
//enables multi-threading handling with TBB
#define ENABLE_CLOUD2MESH_DIST_MT
#include <mutex>
#include <tbb/parallel_for.h>
#else
//Note that there is the case CC_DEBUG=OFF and neither TBB nor Qt
#undef ENABLE_CLOUD2MESH_DIST_MT
#endif
#endif // not CC_DEBUG

namespace CCCoreLib
{

	//! List of triangles (indexes)
	struct TriangleList
	{
		//! Triangles indexes
		std::vector<unsigned> indexes;

		//! Adds a triangle index
		/** \return success
		**/
		inline bool push(unsigned index)
		{
			try
			{
				indexes.push_back(index);
			}
			catch (const std::bad_alloc&)
			{
				return false;
			}
			return true;
		}
	};
}

using namespace CCCoreLib;

bool DistanceComputationTools::MultiThreadSupport()
{
#ifdef ENABLE_CLOUD2MESH_DIST_MT
	return true;
#else
	return false;
#endif
}

int DistanceComputationTools::computeCloud2CloudDistances(	GenericIndexedCloudPersist* comparedCloud,
															GenericIndexedCloudPersist* referenceCloud,
															Cloud2CloudDistancesComputationParams& params,
															GenericProgressCallback* progressCb/*=nullptr*/,
															DgmOctree* compOctree/*=nullptr*/,
															DgmOctree* refOctree/*=nullptr*/)
{
	assert(comparedCloud && referenceCloud);
	if (!comparedCloud)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}

	if (comparedCloud->size() == 0)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}

	if (!referenceCloud)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_REFERENCECLOUD;
	}

	if (referenceCloud->size() == 0)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_REFERENCECLOUD;
	}

	if (params.CPSet && params.maxSearchDist > 0)
	{
		//we can't use a 'max search distance' criterion if the "Closest Point Set" is requested
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_CANT_USE_MAX_SEARCH_DIST_AND_CLOSEST_POINT_SET;
	}

	//we spatially 'synchronize' the octrees
	DgmOctree *comparedOctree = compOctree;
	DgmOctree *referenceOctree = refOctree;
	SOReturnCode soCode = synchronizeOctrees(	comparedCloud,
												referenceCloud,
												comparedOctree,
												referenceOctree,
												static_cast<PointCoordinateType>(params.maxSearchDist),
												progressCb);

	if (soCode != SYNCHRONIZED && soCode != DISJOINT)
	{
		//not enough memory (or invalid input)
		return DISTANCE_COMPUTATION_RESULTS::ERROR_SYNCHRONIZE_OCTREES_FAILURE;
	}

	//we 'enable' a scalar field  (if it is not already done) to store resulting distances
	if (!comparedCloud->enableScalarField())
	{
		//not enough memory
		return DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY;
	}

	//internally we don't use the maxSearchDist parameters as is, but the square of it
	double maxSearchSquareDistd = params.maxSearchDist <= 0 ? 0 : static_cast<double>(params.maxSearchDist) * params.maxSearchDist;

	//closest point set
	if (params.CPSet)
	{
		assert(maxSearchSquareDistd <= 0);

		if (!params.CPSet->resize(comparedCloud->size()))
		{
			//not enough memory
			if (comparedOctree && !compOctree)
				delete comparedOctree;
			if (referenceOctree && !refOctree)
				delete referenceOctree;
			return DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY;
		}
	}

	//by default we reset any former value stored in the 'enabled' scalar field
	const ScalarType resetValue = maxSearchSquareDistd <= 0 ? NAN_VALUE : params.maxSearchDist;
	if (params.resetFormerDistances)
	{
		for (unsigned i = 0; i < comparedCloud->size(); ++i)
		{
			comparedCloud->setPointScalarValue(i, resetValue);
		}
	}

	//specific case: a max search distance has been defined and octrees are totally disjoint
	if (maxSearchSquareDistd > 0 && soCode == DISJOINT)
	{
		//nothing to do! (all points are farther than 'maxSearchDist'
		return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
	}

	//if necessary we try to guess the best octree level for distances computation
	if (params.octreeLevel == 0 && referenceOctree) //DGM: referenceOctree can be 0 if the input entities bounding-boxes are disjoint!
	{
		params.octreeLevel = comparedOctree->findBestLevelForComparisonWithOctree(referenceOctree);
	}

	//whether to compute split distances or not
	bool computeSplitDistances = false;
	{
		for (int i = 0; i < 3; ++i)
		{
			if (params.splitDistances[i] && params.splitDistances[i]->currentSize() == comparedCloud->size())
			{
				computeSplitDistances = true;
				params.splitDistances[i]->fill(NAN_VALUE);
			}
		}
	}

	//additional parameters
	void* additionalParameters[] = {	reinterpret_cast<void*>(referenceCloud),
										reinterpret_cast<void*>(referenceOctree),
										reinterpret_cast<void*>(&params),
										reinterpret_cast<void*>(&maxSearchSquareDistd),
										reinterpret_cast<void*>(&computeSplitDistances)
								   };

	int result = DISTANCE_COMPUTATION_RESULTS::SUCCESS;

	if (!comparedOctree)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDOCTREE;
	}

	result = comparedOctree->executeFunctionForAllCellsAtLevel(params.octreeLevel,
															   params.localModel == NO_MODEL ? computeCellHausdorffDistance : computeCellHausdorffDistanceWithLocalModel,
															   additionalParameters,
															   params.multiThread,
															   progressCb,
															   "Cloud-Cloud Distance",
															   params.maxThreadCount);
	if(result == 0) //executeFunctionForAllCellsAtLevel returns zero if error or canceled
	{
		//something went wrong
		result = DISTANCE_COMPUTATION_RESULTS::ERROR_EXECUTE_FUNCTION_FOR_ALL_CELLS_AT_LEVEL_FAILURE;
	}


	if (comparedOctree && !compOctree)
	{
		delete comparedOctree;
		comparedOctree = nullptr;
	}
	if (referenceOctree && !refOctree)
	{
		delete referenceOctree;
		referenceOctree = nullptr;
	}

	return result;
}

DistanceComputationTools::SOReturnCode
DistanceComputationTools::synchronizeOctrees(	GenericIndexedCloudPersist* comparedCloud,
												GenericIndexedCloudPersist* referenceCloud,
												DgmOctree* &comparedOctree,
												DgmOctree* &referenceOctree,
												PointCoordinateType maxDist,
												GenericProgressCallback* progressCb/*=nullptr*/)
{
	assert(comparedCloud && referenceCloud);
	if (!comparedCloud || !referenceCloud)
	{
		return EMPTY_CLOUD;
	}
	unsigned nA = comparedCloud->size();
	unsigned nB = referenceCloud->size();

	if (nA == 0 || nB == 0)
	{
		return EMPTY_CLOUD;
	}

	//we compute the bounding box of BOTH clouds
	CCVector3 minsA;
	CCVector3 minsB;
	CCVector3 maxsA;
	CCVector3 maxsB;
	comparedCloud->getBoundingBox(minsA, maxsA);
	referenceCloud->getBoundingBox(minsB, maxsB);

	//we compute the union of both bounding-boxes
	CCVector3 maxD;
	CCVector3 minD;
	{
		for (unsigned char k = 0; k < 3; k++)
		{
			minD.u[k] = std::min(minsA.u[k], minsB.u[k]);
			maxD.u[k] = std::max(maxsA.u[k], maxsB.u[k]);
		}
	}

	if (maxDist > 0)
	{
		//we reduce the bounding box to the intersection of both bounding-boxes enlarged by 'maxDist'
		for (unsigned char k = 0; k < 3; k++)
		{
			minD.u[k] = std::max(minD.u[k], std::max(minsA.u[k], minsB.u[k]) - maxDist);
			maxD.u[k] = std::min(maxD.u[k], std::min(maxsA.u[k], maxsB.u[k]) + maxDist);
			if (minD.u[k] > maxD.u[k])
			{
				return DISJOINT;
			}
		}
	}

	CCVector3 minPoints = minD;
	CCVector3 maxPoints = maxD;

	//we make this bounding-box cubical (+0.1% growth to avoid round-off issues)
	CCMiscTools::MakeMinAndMaxCubical(minD, maxD, 0.001);

	//then we (re)compute octree A if necessary
	bool needToRecalculateOctreeA = true;
	if (comparedOctree && comparedOctree->getNumberOfProjectedPoints() != 0)
	{
		needToRecalculateOctreeA = false;
		for (unsigned char k = 0; k < 3; k++)
		{
			if (	maxD.u[k] != comparedOctree->getOctreeMaxs().u[k]
					||	minD.u[k] != comparedOctree->getOctreeMins().u[k] )
			{
				needToRecalculateOctreeA = true;
				break;
			}
		}
	}

	bool octreeACreated = false;
	if (needToRecalculateOctreeA)
	{
		if (comparedOctree)
		{
			comparedOctree->clear();
		}
		else
		{
			comparedOctree = new DgmOctree(comparedCloud);
			octreeACreated = true;
		}

		if (comparedOctree->build(minD, maxD, &minPoints, &maxPoints, progressCb) < 1)
		{
			if (octreeACreated)
			{
				delete comparedOctree;
				comparedOctree = nullptr;
			}
			return OUT_OF_MEMORY;
		}
	}

	//and we (re)compute octree B as well if necessary
	bool needToRecalculateOctreeB = true;
	if (referenceOctree && referenceOctree->getNumberOfProjectedPoints() != 0)
	{
		needToRecalculateOctreeB = false;
		for (unsigned char k = 0; k < 3; k++)
		{
			if (	maxD.u[k] != referenceOctree->getOctreeMaxs().u[k]
					||	minD.u[k] != referenceOctree->getOctreeMins().u[k] )
			{
				needToRecalculateOctreeB = true;
				break;
			}
		}
	}

	if (needToRecalculateOctreeB)
	{
		bool octreeBCreated = false;
		if (referenceOctree)
		{
			referenceOctree->clear();
		}
		else
		{
			referenceOctree = new DgmOctree(referenceCloud);
			octreeBCreated = true;
		}

		if (referenceOctree->build(minD, maxD, &minPoints, &maxPoints, progressCb) < 1)
		{
			if (octreeACreated)
			{
				delete comparedOctree;
				comparedOctree = nullptr;
			}
			if (octreeBCreated)
			{
				delete referenceOctree;
				referenceOctree = nullptr;
			}
			return OUT_OF_MEMORY;
		}
	}

	//we check that both octrees are ok
	assert(comparedOctree && referenceOctree);
	assert(comparedOctree->getNumberOfProjectedPoints() != 0 && referenceOctree->getNumberOfProjectedPoints() != 0);
	return SYNCHRONIZED;
}

//Description of expected 'additionalParameters'
// [0] -> (GenericIndexedCloudPersist*) reference cloud
// [1] -> (Octree*): reference cloud octree
// [2] -> (Cloud2CloudDistancesComputationParams*): parameters
// [3] -> (ScalarType*): max search distance (squared)
bool DistanceComputationTools::computeCellHausdorffDistance(const DgmOctree::octreeCell& cell,
															void** additionalParameters,
															NormalizedProgress* nProgress/*=nullptr*/)
{
	//additional parameters
	const GenericIndexedCloudPersist* referenceCloud	= reinterpret_cast<GenericIndexedCloudPersist*>(additionalParameters[0]);
	const DgmOctree* referenceOctree					= reinterpret_cast<DgmOctree*>(additionalParameters[1]);
	Cloud2CloudDistancesComputationParams* params		= reinterpret_cast<Cloud2CloudDistancesComputationParams*>(additionalParameters[2]);
	const double* maxSearchSquareDistd					= reinterpret_cast<double*>(additionalParameters[3]);
	bool computeSplitDistances							= *reinterpret_cast<bool*>(additionalParameters[4]);

	//structure for the nearest neighbor search
	DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level								= cell.level;
	nNSS.alreadyVisitedNeighbourhoodSize	= 0;
	nNSS.theNearestPointIndex				= 0;
	nNSS.maxSearchSquareDistd				= *maxSearchSquareDistd;

	//we can already compute the position of the 'equivalent' cell in the reference octree
	referenceOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	//and we deduce its center
	referenceOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	//for each point of the current cell (compared octree) we look for its nearest neighbour in the reference cloud
	unsigned pointCount = cell.points->size();
	for (unsigned i = 0; i < pointCount; i++)
	{
		cell.points->getPoint(i, nNSS.queryPoint);

		if (params->CPSet || referenceCloud->testVisibility(nNSS.queryPoint) == POINT_VISIBLE) //to build the closest point set up we must process the point whatever its visibility is!
		{
			double squareDist = referenceOctree->findTheNearestNeighborStartingFromCell(nNSS);
			if (!std::isfinite(squareDist))
			{
				//not enough memory
				return false;
			}
			else if (squareDist >= 0)
			{
				ScalarType dist = static_cast<ScalarType>(sqrt(squareDist));
				cell.points->setPointScalarValue(i, dist);

				if (params->CPSet)
				{
					params->CPSet->setPointIndex(cell.points->getPointGlobalIndex(i), nNSS.theNearestPointIndex);
				}

				if (computeSplitDistances)
				{
					CCVector3 P;
					referenceCloud->getPoint(nNSS.theNearestPointIndex, P);

					unsigned index = cell.points->getPointGlobalIndex(i);
					if (params->splitDistances[0])
						params->splitDistances[0]->setValue(index, static_cast<ScalarType>(nNSS.queryPoint.x - P.x));
					if (params->splitDistances[1])
						params->splitDistances[1]->setValue(index, static_cast<ScalarType>(nNSS.queryPoint.y - P.y));
					if (params->splitDistances[2])
						params->splitDistances[2]->setValue(index, static_cast<ScalarType>(nNSS.queryPoint.z - P.z));
				}
			}
			else
			{
				assert(!params->CPSet);
			}
		}
		else
		{
			cell.points->setPointScalarValue(i, NAN_VALUE);
		}

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	return true;
}

//Description of expected 'additionalParameters'
// [0] -> (GenericIndexedCloudPersist*) reference cloud
// [1] -> (Octree*): reference cloud octree
// [2] -> (Cloud2CloudDistancesComputationParams*): parameters
// [3] -> (ScalarType*): max search distance (squared)
bool DistanceComputationTools::computeCellHausdorffDistanceWithLocalModel(	const DgmOctree::octreeCell& cell,
																			void** additionalParameters,
																			NormalizedProgress* nProgress/*=nullptr*/)
{
	//additional parameters
	GenericIndexedCloudPersist* referenceCloud		= reinterpret_cast<GenericIndexedCloudPersist*>(additionalParameters[0]);
	const DgmOctree* referenceOctree				= reinterpret_cast<DgmOctree*>(additionalParameters[1]);
	Cloud2CloudDistancesComputationParams* params	= reinterpret_cast<Cloud2CloudDistancesComputationParams*>(additionalParameters[2]);
	const double* maxSearchSquareDistd				= reinterpret_cast<double*>(additionalParameters[3]);
	bool computeSplitDistances						= *reinterpret_cast<bool*>(additionalParameters[4]);

	assert(params && params->localModel != NO_MODEL);

	//structure for the nearest neighbor search
	DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level								= cell.level;
	nNSS.alreadyVisitedNeighbourhoodSize	= 0;
	nNSS.theNearestPointIndex				= 0;
	nNSS.maxSearchSquareDistd				= *maxSearchSquareDistd;
	//we already compute the position of the 'equivalent' cell in the reference octree
	referenceOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	//and we deduce its center
	referenceOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	//structures for determining the nearest neighbours of the 'nearest neighbour' (to compute the local model)
	//either inside a sphere or the k nearest
	DgmOctree::NearestNeighboursSearchStruct nNSS_Model;
	nNSS_Model.level = cell.level;
	if (!params->useSphericalSearchForLocalModel)
	{
		nNSS_Model.minNumberOfNeighbors = params->kNNForLocalModel;
	}

	//already computed models
	std::vector<const LocalModel*> models;

	//for each point of the current cell (compared octree) we look for its nearest neighbour in the reference cloud
	unsigned pointCount = cell.points->size();
	for (unsigned i = 0; i < pointCount; ++i)
	{
		//distance of the current point
		ScalarType distPt = NAN_VALUE;

		cell.points->getPoint(i,nNSS.queryPoint);
		if (params->CPSet || referenceCloud->testVisibility(nNSS.queryPoint) == POINT_VISIBLE) //to build the closest point set up we must process the point whatever its visibility is!
		{
			//first, we look for the nearest point to "_queryPoint" in the reference cloud
			double squareDistToNearestPoint = referenceOctree->findTheNearestNeighborStartingFromCell(nNSS);
			if (!std::isfinite(squareDistToNearestPoint))
			{
				//not enough memory
				return false;
			}

			//if it exists
			if (squareDistToNearestPoint >= 0)
			{
				ScalarType distToNearestPoint = static_cast<ScalarType>(sqrt(squareDistToNearestPoint));

				CCVector3 nearestPoint;
				referenceCloud->getPoint(nNSS.theNearestPointIndex, nearestPoint);

				//local model for the 'nearest point'
				const LocalModel* lm = nullptr;

				if (params->reuseExistingLocalModels)
				{
					//we look if the nearest point is close to existing models
					for (std::vector<const LocalModel*>::const_iterator it = models.begin(); it != models.end(); ++it)
					{
						//we take the first model that 'includes' the nearest point
						if ( ((*it)->getCenter() - nearestPoint).norm2() <= (*it)->getSquareSize())
						{
							lm = *it;
							break;
						}
					}
				}

				//create new local model
				if (!lm)
				{
					nNSS_Model.queryPoint = nearestPoint;

					//update cell pos information (as the nearestPoint may not be inside the same cell as the actual query point!)
					{
						bool inbounds = false;
						Tuple3i cellPos;
						referenceOctree->getTheCellPosWhichIncludesThePoint(&nearestPoint, cellPos, cell.level, inbounds);
						//if the cell is different or the structure has not yet been initialized, we reset it!
						if (	cellPos.x != nNSS_Model.cellPos.x
								||	cellPos.y != nNSS_Model.cellPos.y
								||	cellPos.z != nNSS_Model.cellPos.z)
						{
							nNSS_Model.cellPos = cellPos;
							referenceOctree->computeCellCenter(nNSS_Model.cellPos, nNSS_Model.level, nNSS_Model.cellCenter);
							assert(inbounds);
							nNSS_Model.minimalCellsSetToVisit.clear();
							nNSS_Model.pointsInNeighbourhood.clear();
							nNSS_Model.alreadyVisitedNeighbourhoodSize = inbounds ? 0 : 1;
							//nNSS_Model.theNearestPointIndex = 0;
						}
					}
					//let's grab the nearest neighbours of the 'nearest point'
					unsigned kNN = 0;
					if (params->useSphericalSearchForLocalModel)
					{
						//we only need to sort neighbours if we want to use the 'reuseExistingLocalModels' optimization
						//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (kNN)!
						kNN = referenceOctree->findNeighborsInASphereStartingFromCell(	nNSS_Model,
																						static_cast<PointCoordinateType>(params->radiusForLocalModel),
																						params->reuseExistingLocalModels);
					}
					else
					{
						kNN = referenceOctree->findNearestNeighborsStartingFromCell(nNSS_Model);
						kNN = std::min(kNN, params->kNNForLocalModel);
					}

					//if there's enough neighbours
					if (kNN >= CC_LOCAL_MODEL_MIN_SIZE[params->localModel])
					{
						DgmOctreeReferenceCloud neighboursCloud(&nNSS_Model.pointsInNeighbourhood,kNN);
						Neighbourhood Z(&neighboursCloud);

						//Neighbours are sorted, so the farthest is at the end. It also gives us
						//an approximation of the model 'size'
						const double& maxSquareDist = nNSS_Model.pointsInNeighbourhood[kNN-1].squareDistd;
						if (maxSquareDist > 0) //DGM: with duplicate points, all neighbors can be at the same place :(
						{
							lm = LocalModel::New(params->localModel, Z, nearestPoint, static_cast<PointCoordinateType>(maxSquareDist));
							if (lm && params->reuseExistingLocalModels)
							{
								//we add the model to the 'existing models' list
								try
								{
									models.push_back(lm);
								}
								catch (const std::bad_alloc&)
								{
									//not enough memory!
									while (!models.empty())
									{
										delete models.back();
										models.pop_back();
									}
									return false;
								}
							}
						}
						//neighbours->clear();
					}
				}

				//if we have a local model
				if (lm)
				{
					CCVector3 nearestModelPoint;
					ScalarType distToModel = lm->computeDistanceFromModelToPoint(&nNSS.queryPoint, computeSplitDistances ? &nearestModelPoint : nullptr);

					//we take the best estimation between the nearest neighbor and the model!
					//this way we only reduce any potential noise (that would be due to sampling)
					//instead of 'adding' noise if the model is badly shaped
					if (distToNearestPoint <= distToModel)
					{
						distPt = distToNearestPoint;
					}
					else
					{
						distPt = distToModel;
						nearestPoint = nearestModelPoint;
					}

					if (!params->reuseExistingLocalModels)
					{
						//we don't need the local model anymore!
						delete lm;
						lm = nullptr;
					}

					if (computeSplitDistances)
					{
						unsigned index = cell.points->getPointGlobalIndex(i);
						if (params->splitDistances[0])
							params->splitDistances[0]->setValue(index, static_cast<ScalarType>(nNSS.queryPoint.x - nearestPoint.x));
						if (params->splitDistances[1])
							params->splitDistances[1]->setValue(index, static_cast<ScalarType>(nNSS.queryPoint.y - nearestPoint.y));
						if (params->splitDistances[2])
							params->splitDistances[2]->setValue(index, static_cast<ScalarType>(nNSS.queryPoint.z - nearestPoint.z));
					}
				}
				else
				{
					distPt = distToNearestPoint;
				}
			}
			else if (nNSS.maxSearchSquareDistd > 0)
			{
				distPt = static_cast<ScalarType>(sqrt(nNSS.maxSearchSquareDistd));
			}

			if (params->CPSet)
			{
				params->CPSet->setPointIndex(cell.points->getPointGlobalIndex(i), nNSS.theNearestPointIndex);
			}
		}

		cell.points->setPointScalarValue(i, distPt);

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	//clear all models for this cell
	while (!models.empty())
	{
		delete models.back();
		models.pop_back();
	}

	return true;
}

//! Method used by computeCloud2MeshDistancesWithOctree
static void ComparePointsAndTriangles(	ReferenceCloud& Yk,
										unsigned& remainingPoints,
										const GenericIndexedMesh* mesh,
										std::vector<unsigned>& trianglesToTest,
										std::size_t& trianglesToTestCount,
										std::vector<ScalarType>& minDists,
										ScalarType maxRadius,
										DistanceComputationTools::Cloud2MeshDistancesComputationParams& params)
{
	assert(mesh);
	assert(remainingPoints <= Yk.size());
	assert(trianglesToTestCount <= trianglesToTest.size());

	bool firstComparisonDone = (trianglesToTestCount != 0);

	CCVector3 nearestPoint;
	CCVector3* _nearestPoint = params.CPSet ? &nearestPoint : nullptr;

	//for each triangle
	while (trianglesToTestCount != 0)
	{
		//we query the vertex coordinates
		SimpleTriangle tri;
		unsigned triIndex = trianglesToTest[--trianglesToTestCount];
		mesh->getTriangleVertices(triIndex, tri.A, tri.B, tri.C);

		//for each point inside the current cell
		if (params.signedDistances)
		{
			//we have to use absolute distances
			for (unsigned j = 0; j < remainingPoints; ++j)
			{
				//compute the distance to the triangle
				ScalarType dPTri = DistanceComputationTools::computePoint2TriangleDistance(Yk.getPoint(j), &tri, true, _nearestPoint);
				//keep it if it's smaller
				ScalarType min_d = Yk.getPointScalarValue(j);
				if (!ScalarField::ValidValue(min_d) || min_d * min_d > dPTri*dPTri)
				{
					Yk.setPointScalarValue(j, params.flipNormals ? -dPTri : dPTri);
					if (params.CPSet)
					{
						//Closest Point Set: save the nearest point and nearest triangle as well
						unsigned pointIndex = Yk.getPointGlobalIndex(j);
						assert(_nearestPoint);
						*const_cast<CCVector3*>(params.CPSet->getPoint(pointIndex)) = *_nearestPoint;
						params.CPSet->setPointScalarValue(pointIndex, static_cast<ScalarType>(triIndex));
					}
				}
			}
		}
		else //squared distances
		{
			for (unsigned j = 0; j < remainingPoints; ++j)
			{
				//compute the (SQUARED) distance to the triangle
				ScalarType dPTri = DistanceComputationTools::computePoint2TriangleDistance(Yk.getPoint(j), &tri, false, _nearestPoint);
				//keep it if it's smaller
				ScalarType min_d = Yk.getPointScalarValue(j);
				if (!ScalarField::ValidValue(min_d) || dPTri < min_d)
				{
					Yk.setPointScalarValue(j, dPTri);
					if (params.CPSet)
					{
						//Closest Point Set: save the nearest point and nearest triangle as well
						assert(_nearestPoint);
						unsigned pointIndex = Yk.getPointGlobalIndex(j);
						*const_cast<CCVector3*>(params.CPSet->getPoint(pointIndex)) = *_nearestPoint;
						params.CPSet->setPointScalarValue(pointIndex, static_cast<ScalarType>(triIndex));
					}
				}
			}
		}
	}

	//we can 'remove' all the eligible points at the current neighborhood radius
	if (firstComparisonDone)
	{
		Yk.placeIteratorAtBeginning();
		for (unsigned j = 0; j < remainingPoints; )
		{
			//eligibility distance
			ScalarType eligibleDist = minDists[j] + maxRadius;
			ScalarType dPTri = Yk.getCurrentPointScalarValue();
			if (params.signedDistances)
			{
				//need to get the square distance in all cases
				dPTri *= dPTri;
			}
			if (dPTri <= eligibleDist*eligibleDist)
			{
				//remove this point
				Yk.removePointGlobalIndex(j);
				//and do the same for the 'minDists' array! (see ReferenceCloud::removePointGlobalIndex)
				assert(remainingPoints != 0);
				minDists[j] = minDists[--remainingPoints];
				//minDists.pop_back();
			}
			else
			{
				Yk.forwardIterator();
				++j;
			}
		}
	}
}


int ComputeMaxNeighborhoodLength(ScalarType maxSearchDist, PointCoordinateType cellSize)
{
	return static_cast<int>(ceil(maxSearchDist / cellSize + static_cast<ScalarType>((sqrt(2.0) - 1.0) / 2)));
}

#ifdef ENABLE_CLOUD2MESH_DIST_MT

/*** MULTI THREADING WRAPPER ***/
static const DgmOctree* s_octree_MT = nullptr;
static NormalizedProgress* s_normProgressCb_MT = nullptr;
static const GridAndMeshIntersection* s_intersection_MT = nullptr;
static bool s_cellFunc_MT_success = true;
static int s_cellFunc_MT_results = DistanceComputationTools::DISTANCE_COMPUTATION_RESULTS::SUCCESS;
static DistanceComputationTools::Cloud2MeshDistancesComputationParams s_params_MT;

//'processTriangles' mechanism (based on bit mask)
static std::vector<std::vector<bool>*> s_bitArrayPool_MT;
static bool s_useBitArrays_MT = true;
#ifdef CC_CORE_LIB_USES_QT_CONCURRENT
static QMutex s_currentBitMaskMutex;
#elif defined(CC_CORE_LIB_USES_TBB)
static std::mutex s_currentBitMaskMutex;
#endif

void cloudMeshDistCellFunc_MT(const DgmOctree::IndexAndCode& desc)
{
	if (!s_cellFunc_MT_success)
	{
		//skip this cell if the process is aborted / has failed
		return;
	}

	if (s_normProgressCb_MT && !s_normProgressCb_MT->oneStep())
	{
		s_cellFunc_MT_success = false;
		s_cellFunc_MT_results = DistanceComputationTools::DISTANCE_COMPUTATION_RESULTS::CANCELED_BY_USER;
		return;
	}

	ReferenceCloud Yk(s_octree_MT->associatedCloud());
	s_octree_MT->getPointsInCellByCellIndex(&Yk, desc.theIndex, s_params_MT.octreeLevel);

	//min distance array
	unsigned remainingPoints = Yk.size();

	std::vector<ScalarType> minDists;
	try
	{
		minDists.resize(remainingPoints);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		s_cellFunc_MT_success = false;
		s_cellFunc_MT_results = DistanceComputationTools::DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY;
		return;
	}

	//get cell pos
	Tuple3i cellPos;
	s_octree_MT->getCellPos(desc.theCode, s_params_MT.octreeLevel, cellPos, true);

	//get the distance to the nearest and farthest boundaries
	Tuple3i signedDistToLowerBorder, signedDistToUpperBorder;
	s_intersection_MT->computeSignedDistToBoundaries(cellPos, signedDistToLowerBorder, signedDistToUpperBorder);

	Tuple3i minDistToGridBoundaries, maxDistToGridBoundaries;
	for (unsigned char k = 0; k < 3; ++k)
	{
		minDistToGridBoundaries.u[k] = std::max(std::max(-signedDistToLowerBorder.u[k], 0), std::max(0, -signedDistToUpperBorder.u[k]));
		maxDistToGridBoundaries.u[k] = std::max(std::max(signedDistToLowerBorder.u[k], 0), std::max(0, signedDistToUpperBorder.u[k]));
	}
	int minDistToBoundaries = std::max(minDistToGridBoundaries.x, std::max(minDistToGridBoundaries.y, minDistToGridBoundaries.z));
	int maxDistToBoundaries = std::max(maxDistToGridBoundaries.x, std::max(maxDistToGridBoundaries.y, maxDistToGridBoundaries.z));


	if (s_params_MT.maxSearchDist > 0)
	{
		//no need to look farther than 'maxNeighbourhoodLength'
		int maxNeighbourhoodLength = ComputeMaxNeighborhoodLength(s_params_MT.maxSearchDist, s_octree_MT->getCellSize(s_params_MT.octreeLevel));
		if (maxNeighbourhoodLength < maxDistToBoundaries)
			maxDistToBoundaries = maxNeighbourhoodLength;

		ScalarType maxDistance = s_params_MT.maxSearchDist;
		if (!s_params_MT.signedDistances)
		{
			//we compute squared distances when not in 'signed' mode!
			maxDistance = s_params_MT.maxSearchDist*s_params_MT.maxSearchDist;
		}

		for (unsigned j = 0; j < remainingPoints; ++j)
			Yk.setPointScalarValue(j, maxDistance);
	}

	//determine the cell center
	CCVector3 cellCenter;
	s_octree_MT->computeCellCenter(cellPos, s_params_MT.octreeLevel, cellCenter);

	//express 'startPos' relatively to the grid borders
	Tuple3i startPos = s_intersection_MT->toLocal(cellPos);

	//octree cell size
	const PointCoordinateType& cellLength = s_octree_MT->getCellSize(s_params_MT.octreeLevel);

	//useful variables
	std::vector<unsigned> trianglesToTest;
	std::size_t trianglesToTestCount = 0;
	std::size_t trianglesToTestCapacity = 0;

	//bit mask for efficient comparisons
	std::vector<bool>* bitArray = nullptr;
	if (s_useBitArrays_MT)
	{
		const unsigned numTri = s_intersection_MT->mesh()->size();
		s_currentBitMaskMutex.lock();
		if (s_bitArrayPool_MT.empty())
		{
			bitArray = new std::vector<bool>(numTri);
		}
		else
		{
			bitArray = s_bitArrayPool_MT.back();
			s_bitArrayPool_MT.pop_back();
		}
		s_currentBitMaskMutex.unlock();
		bitArray->assign(numTri, false);
	}

	//for each point, we pre-compute its distance to the nearest cell border
	//(will be handy later)
	Yk.placeIteratorAtBeginning();
	for (unsigned j = 0; j < remainingPoints; ++j)
	{
		//coordinates of the current point
		const CCVector3 *tempPt = Yk.getCurrentPointCoordinates();
		//distance du bord le plus proche = taille de la cellule - distance la plus grande par rapport au centre de la cellule
		minDists[j] = DgmOctree::ComputeMinDistanceToCellBorder(*tempPt, cellLength, cellCenter);
		Yk.forwardIterator();
	}

	//let's find the nearest triangles for each point in the neighborhood 'Yk'
	ScalarType maxRadius = 0;
	for (int dist = minDistToBoundaries; remainingPoints != 0 && dist <= maxDistToBoundaries; ++dist, maxRadius += static_cast<ScalarType>(cellLength))
	{
		//test the neighbor cells at distance = 'dist'
		//a,b,c,d,e,f are the extents of this neighborhood
		//for the 6 main directions -X,+X,-Y,+Y,-Z,+Z
		int a = std::min(dist, signedDistToLowerBorder.x);
		int b = std::min(dist, signedDistToUpperBorder.x);
		int c = std::min(dist, signedDistToLowerBorder.y);
		int d = std::min(dist, signedDistToUpperBorder.y);
		int e = std::min(dist, signedDistToLowerBorder.z);
		int f = std::min(dist, signedDistToUpperBorder.z);

		for (int i = -a; i <= b; ++i)
		{
			bool imax = (std::abs(i) == dist);
			Tuple3i localCellPos(startPos.x + i, 0, 0);

			for (int j = -c; j <= d; j++)
			{
				localCellPos.y = startPos.y + j;

				//if i or j is 'maximal'
				if (imax || std::abs(j) == dist)
				{
					//we must be on the border of the neighborhood
					for (int k = -e; k <= f; k++)
					{
						//are there any triangles near this cell?
						localCellPos.z = startPos.z + k;
						const TriangleList* triList = s_intersection_MT->trianglesInCell(localCellPos, true);
						if (triList)
						{
							if (trianglesToTestCount + triList->indexes.size() > trianglesToTestCapacity)
							{
								trianglesToTestCapacity = std::max(trianglesToTestCount + triList->indexes.size(), 2 * trianglesToTestCount);
								trianglesToTest.resize(trianglesToTestCapacity);
							}
							//let's test all the triangles that intersect this cell
							for (unsigned p = 0; p<triList->indexes.size(); ++p)
							{
								if (bitArray)
								{
									const unsigned indexTri = triList->indexes[p];
									//if the triangles has not been processed yet
									if (!(*bitArray)[indexTri])
									{
										trianglesToTest[trianglesToTestCount++] = indexTri;
										(*bitArray)[indexTri] = true;
									}
								}
								else
								{
									trianglesToTest[trianglesToTestCount++] = triList->indexes[p];
								}
							}
						}
					}
				}
				else //we must go the cube border
				{
					if (e == dist) //'negative' side
					{
						//are there any triangles near this cell?
						localCellPos.z = startPos.z - e;
						const TriangleList* triList = s_intersection_MT->trianglesInCell(localCellPos, true);
						if (triList)
						{
							if (trianglesToTestCount + triList->indexes.size() > trianglesToTestCapacity)
							{
								trianglesToTestCapacity = std::max(trianglesToTestCount + triList->indexes.size(), 2 * trianglesToTestCount);
								trianglesToTest.resize(trianglesToTestCapacity);
							}
							//let's test all the triangles that intersect this cell
							for (unsigned p = 0; p<triList->indexes.size(); ++p)
							{
								if (bitArray)
								{
									const unsigned indexTri = triList->indexes[p];
									//if the triangles has not been processed yet
									if (!(*bitArray)[indexTri])
									{
										trianglesToTest[trianglesToTestCount++] = indexTri;
										(*bitArray)[indexTri] = true;
									}
								}
								else
								{
									trianglesToTest[trianglesToTestCount++] = triList->indexes[p];
								}
							}
						}
					}

					if (f == dist && dist>0) //'positive' side
					{
						//are there any triangles near this cell?
						localCellPos.z = startPos.z + f;
						const TriangleList* triList = s_intersection_MT->trianglesInCell(localCellPos, true);
						if (triList)
						{
							if (trianglesToTestCount + triList->indexes.size() > trianglesToTestCapacity)
							{
								trianglesToTestCapacity = std::max(trianglesToTestCount + triList->indexes.size(), 2 * trianglesToTestCount);
								trianglesToTest.resize(trianglesToTestCapacity);
							}
							//let's test all the triangles that intersect this cell
							for (unsigned p = 0; p<triList->indexes.size(); ++p)
							{
								if (bitArray)
								{
									const unsigned indexTri = triList->indexes[p];
									//if the triangles has not been processed yet
									if (!(*bitArray)[indexTri])
									{
										trianglesToTest[trianglesToTestCount++] = indexTri;
										(*bitArray)[indexTri] = true;
									}
								}
								else
								{
									trianglesToTest[trianglesToTestCount++] = triList->indexes[p];
								}
							}
						}
					}
				}
			}
		}

		ComparePointsAndTriangles(Yk, remainingPoints, s_intersection_MT->mesh(), trianglesToTest, trianglesToTestCount, minDists, maxRadius, s_params_MT);
	}

	//Save the bit mask
	if (bitArray)
	{
		s_currentBitMaskMutex.lock();
		s_bitArrayPool_MT.push_back(bitArray);
		s_currentBitMaskMutex.unlock();
	}
}

#endif

struct BoundedSearchParams
{
	int maxNeighbourhoodLength = 0;
};

struct TrianglesToTest
{
	TrianglesToTest(const GenericIndexedMesh& mesh)
		: trianglesToTestCount(0)
		, trianglesToTestCapacity(0)
		, numberOfTriangles(0)
	{
		numberOfTriangles = mesh.size();
		try
		{
			processTriangles.resize(numberOfTriangles, 0);
		}
		catch (const std::bad_alloc&)
		{
			//otherwise, no big deal, we can do without it!
		}
	}

	// variables
	std::vector<unsigned> trianglesToTest;
	std::size_t trianglesToTestCount;
	std::size_t trianglesToTestCapacity;
	unsigned numberOfTriangles;

	// optional acceleration structure
	std::vector<unsigned> processTriangles;
};

static int ComputeNeighborhood2MeshDistancesWithOctree(	const GridAndMeshIntersection& intersection,
														DistanceComputationTools::Cloud2MeshDistancesComputationParams& params,
														ReferenceCloud& Yk,
														unsigned cellIndex,
														const Tuple3i& cellPos,
														TrianglesToTest& ttt,
														bool boundedSearch,
														int maxNeighbourhoodLength
	)
{
	//get the distance to the nearest and farthest boundaries
	Tuple3i signedDistToLowerBorder, signedDistToUpperBorder;
	intersection.computeSignedDistToBoundaries(cellPos, signedDistToLowerBorder, signedDistToUpperBorder);

	Tuple3i minDistToGridBoundaries, maxDistToGridBoundaries;
	for (unsigned char k = 0; k < 3; ++k)
	{
		minDistToGridBoundaries.u[k] = std::max(std::max(-signedDistToLowerBorder.u[k], 0), std::max(0, -signedDistToUpperBorder.u[k]));
		maxDistToGridBoundaries.u[k] = std::max(std::max(signedDistToLowerBorder.u[k], 0), std::max(0, signedDistToUpperBorder.u[k]));
	}
	int minDistToBoundaries = std::max(minDistToGridBoundaries.x, std::max(minDistToGridBoundaries.y, minDistToGridBoundaries.z));
	int maxDistToBoundaries = std::max(maxDistToGridBoundaries.x, std::max(maxDistToGridBoundaries.y, maxDistToGridBoundaries.z));

	//determine the cell center
	CCVector3 cellCenter;
	intersection.computeCellCenter(cellPos, cellCenter);

	//express 'startPos' relatively to the inner grid borders
	Tuple3i startPos = intersection.toLocal(cellPos);

	//min distance array ('persistent' version to save some memory)
	std::vector<ScalarType> minDists;
	unsigned remainingPoints = Yk.size();

	try
	{
		minDists.resize(remainingPoints);
	}
	catch (const std::bad_alloc&) //out of memory
	{
		//not enough memory
		return DistanceComputationTools::DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY;
	}

	PointCoordinateType cellSize = intersection.cellSize();

	//for each point, we pre-compute its distance to the nearest cell border
	//(will be handy later)
	for (unsigned j = 0; j < remainingPoints; ++j)
	{
		const CCVector3 *tempPt = Yk.getPointPersistentPtr(j);
		minDists[j] = static_cast<ScalarType>(DgmOctree::ComputeMinDistanceToCellBorder(*tempPt, cellSize, cellCenter));
	}

	if (boundedSearch)
	{
		//no need to look farther than 'maxNeighbourhoodLength'
		if (maxNeighbourhoodLength < maxDistToBoundaries)
			maxDistToBoundaries = maxNeighbourhoodLength;

		//we compute squared distances when not in 'signed' mode!
		ScalarType maxDistance = params.maxSearchDist;
		if (!params.signedDistances)
		{
			//we compute squared distances when not in 'signed' mode!
			maxDistance = params.maxSearchDist * params.maxSearchDist;
		}

		for (unsigned j = 0; j < remainingPoints; ++j)
		{
			Yk.setPointScalarValue(j, maxDistance);
		}
	}

	//let's find the nearest triangles for each point in the neighborhood 'Yk'
	ScalarType maxRadius = 0;
	for (int dist = minDistToBoundaries; dist <= maxDistToBoundaries && remainingPoints != 0; ++dist, maxRadius += static_cast<ScalarType>(cellSize))
	{
		//test the neighbor cells at distance = 'dist'
		//a,b,c,d,e,f are the extents of this neighborhood
		//for the 6 main directions -X,+X,-Y,+Y,-Z,+Z
		int a = std::min(dist, signedDistToLowerBorder.x);
		int b = std::min(dist, signedDistToUpperBorder.x);
		int c = std::min(dist, signedDistToLowerBorder.y);
		int d = std::min(dist, signedDistToUpperBorder.y);
		int e = std::min(dist, signedDistToLowerBorder.z);
		int f = std::min(dist, signedDistToUpperBorder.z);

		for (int i = -a; i <= b; i++)
		{
			bool imax = (std::abs(i) == dist);
			Tuple3i localCellPos(startPos.x + i, 0, 0);

			for (int j = -c; j <= d; j++)
			{
				localCellPos.y = startPos.y + j;

				//if i or j is 'maximal'
				if (imax || std::abs(j) == dist)
				{
					//we must be on the border of the neighborhood
					for (int k = -e; k <= f; k++)
					{
						//are there any triangles near this cell?
						localCellPos.z = startPos.z + k;
						const TriangleList* triList = intersection.trianglesInCell(localCellPos, true);
						if (triList)
						{
							if (ttt.trianglesToTestCount + triList->indexes.size() > ttt.trianglesToTestCapacity)
							{
								ttt.trianglesToTestCapacity = std::max(ttt.trianglesToTestCount + triList->indexes.size(), 2 * ttt.trianglesToTestCount);
								ttt.trianglesToTest.resize(ttt.trianglesToTestCapacity);
							}
							//let's test all the triangles that intersect this cell
							for (unsigned indexTri : triList->indexes)
							{
								if (!ttt.processTriangles.empty())
								{
									//if the triangles has not been processed yet
									if (ttt.processTriangles[indexTri] != cellIndex)
									{
										ttt.trianglesToTest[ttt.trianglesToTestCount++] = indexTri;
										ttt.processTriangles[indexTri] = cellIndex;
									}
								}
								else
								{
									ttt.trianglesToTest[ttt.trianglesToTestCount++] = indexTri;
								}
							}
						}
					}
				}
				else //we must go the cube border
				{
					if (e == dist) //'negative' side
					{
						//are there any triangles near this cell?
						localCellPos.z = startPos.z - e;
						const TriangleList* triList = intersection.trianglesInCell(localCellPos, true);
						if (triList)
						{
							if (ttt.trianglesToTestCount + triList->indexes.size() > ttt.trianglesToTestCapacity)
							{
								ttt.trianglesToTestCapacity = std::max(ttt.trianglesToTestCount + triList->indexes.size(), 2 * ttt.trianglesToTestCount);
								ttt.trianglesToTest.resize(ttt.trianglesToTestCapacity);
							}
							//let's test all the triangles that intersect this cell
							for (unsigned triIndex : triList->indexes)
							{
								if (!ttt.processTriangles.empty())
								{
									//if the triangles has not been processed yet
									if (ttt.processTriangles[triIndex] != cellIndex)
									{
										ttt.trianglesToTest[ttt.trianglesToTestCount++] = triIndex;
										ttt.processTriangles[triIndex] = cellIndex;
									}
								}
								else
								{
									ttt.trianglesToTest[ttt.trianglesToTestCount++] = triIndex;
								}
							}
						}
					}

					if (f == dist && dist > 0) //'positive' side
					{
						//are there any triangles near this cell?
						localCellPos.z = startPos.z + f;
						const TriangleList* triList = intersection.trianglesInCell(localCellPos, true);
						if (triList)
						{
							if (ttt.trianglesToTestCount + triList->indexes.size() > ttt.trianglesToTestCapacity)
							{
								ttt.trianglesToTestCapacity = std::max(ttt.trianglesToTestCount + triList->indexes.size(), 2 * ttt.trianglesToTestCount);
								ttt.trianglesToTest.resize(ttt.trianglesToTestCapacity);
							}
							//let's test all the triangles that intersect this cell
							for (unsigned triIndex : triList->indexes)
							{
								if (!ttt.processTriangles.empty())
								{
									//if the triangles has not been processed yet
									if (ttt.processTriangles[triIndex] != cellIndex)
									{
										ttt.trianglesToTest[ttt.trianglesToTestCount++] = triIndex;
										ttt.processTriangles[triIndex] = cellIndex;
									}
								}
								else
								{
									ttt.trianglesToTest[ttt.trianglesToTestCount++] = triIndex;
								}
							}
						}
					}
				}
			}
		}

		ComparePointsAndTriangles(	Yk,
									remainingPoints,
									intersection.mesh(),
									ttt.trianglesToTest,
									ttt.trianglesToTestCount,
									minDists,
									maxRadius,
									params);
	}

	return DistanceComputationTools::DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

int DistanceComputationTools::computePoint2MeshDistancesWithOctree(	const CCVector3& P,
																	ScalarType& distance,
																	const GridAndMeshIntersection& intersection,
																	Cloud2MeshDistancesComputationParams& params)
{
	distance = NAN_VALUE;

	if (!intersection.isInitialized() || intersection.distanceTransform())
	{
		// Distance Transform acceleration is not supported
		return DistanceComputationTools::DISTANCE_COMPUTATION_RESULTS::ERROR_INVALID_OCTREE_AND_MESH_INTERSECTION;
	}

	const GenericIndexedMesh* mesh = intersection.mesh();
	if (!mesh)
	{
		//invalid input
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_REFERENCEMESH;
	}

	bool boundedSearch = (params.maxSearchDist > 0);

	PointCloud cloud;
	if (!cloud.reserve(1))
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY;
	}
	cloud.addPoint(P);
	if (!cloud.enableScalarField())
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY;
	}
	cloud.setPointScalarValue(0, NAN_VALUE);
	ReferenceCloud Yk(&cloud);
	if (!Yk.reserve(1))
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY;
	}
	Yk.addPointIndex(0);

	TrianglesToTest ttt(*mesh);

	//maximal neighbors search distance (if maxSearchDist is defined)
	int maxNeighbourhoodLength = 0;
	if (boundedSearch)
	{
		maxNeighbourhoodLength = ComputeMaxNeighborhoodLength(params.maxSearchDist, intersection.cellSize());
	}

	//get cell pos
	Tuple3i cellPos = intersection.computeCellPos(P);

	int result = ComputeNeighborhood2MeshDistancesWithOctree(	intersection,
																params,
																Yk,
																1,
																cellPos,
																ttt,
																boundedSearch,
																maxNeighbourhoodLength);

	if (result != DISTANCE_COMPUTATION_RESULTS::SUCCESS)
	{
		// an error occurred
		return result;
	}

	distance = cloud.getPointScalarValue(0);

	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

int DistanceComputationTools::computeCloud2MeshDistancesWithOctree(	const DgmOctree* octree, 
																	const GridAndMeshIntersection& intersection,
																	Cloud2MeshDistancesComputationParams& params,
																	GenericProgressCallback* progressCb/*=nullptr*/)
{
	assert(!params.multiThread || params.maxSearchDist <= 0); //maxSearchDist is not compatible with parallel processing
	assert(!params.signedDistances || !intersection.distanceTransform()); //signed distances are not compatible with Distance Transform acceleration

	if (params.useDistanceMap != (intersection.distanceTransform() != nullptr))
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_OCTREE_AND_MESH_INTERSECTION_MISMATCH;
	}

	if (!octree)
	{
		//invalid input
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_OCTREE;
	}

	if (!intersection.isInitialized())
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_INVALID_OCTREE_AND_MESH_INTERSECTION;
	}

	//Closest Point Set
	if (params.CPSet)
	{
		assert(params.maxSearchDist <= 0);
		assert(params.useDistanceMap == false);

		//reserve memory for the Closest Point Set
		if (!params.CPSet->resize(octree->associatedCloud()->size()))
		{
			//not enough memory
			return DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY;
		}
		//reserve memory for an associated scalar field (the nearest triangle index)
		if (!params.CPSet->enableScalarField())
		{
			//not enough memory
			return DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY;
		}
		assert(params.CPSet->getCurrentInScalarField());
		params.CPSet->getCurrentInScalarField()->fill(0);
	}

#ifdef ENABLE_CLOUD2MESH_DIST_MT
	if (!params.multiThread)
#endif
	{
		const GenericIndexedMesh* mesh = intersection.mesh();
		if (!mesh)
		{
			//invalid input
			assert(false);
			return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_REFERENCEMESH;
		}

		//dimension of an octree cell
		PointCoordinateType cellSize = octree->getCellSize(params.octreeLevel);
		if (cellSize != intersection.cellSize())
		{
			return DISTANCE_COMPUTATION_RESULTS::ERROR_OCTREE_AND_MESH_INTERSECTION_MISMATCH;
		}

		//get the cell indexes at level "octreeLevel"
		DgmOctree::cellsContainer cellCodesAndIndexes;
		if (!octree->getCellCodesAndIndexes(params.octreeLevel, cellCodesAndIndexes, true))
		{
			//not enough memory
			return DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY;
		}

		unsigned numberOfCells = static_cast<unsigned>(cellCodesAndIndexes.size());
		bool boundedSearch = (params.maxSearchDist > 0);

		DgmOctree::cellsContainer::const_iterator pCodeAndIndex = cellCodesAndIndexes.begin();
		ReferenceCloud Yk(octree->associatedCloud());

		//if we only need approximate distances
		if (intersection.distanceTransform())
		{
			//for each cell
			for (unsigned i = 0; i < numberOfCells; ++i, ++pCodeAndIndex)
			{
				octree->getPointsInCellByCellIndex(&Yk, pCodeAndIndex->theIndex, params.octreeLevel);

				//get the cell pos
				Tuple3i cellPos;
				octree->getCellPos(pCodeAndIndex->theCode, params.octreeLevel, cellPos, true);

				//get the Distance Transform distance
				unsigned squareDist = intersection.distanceTransformValue(cellPos, false);

				//assign the distance to all points inside this cell
				ScalarType maxRadius = sqrt(static_cast<ScalarType>(squareDist)) * cellSize;

				if (boundedSearch && maxRadius > params.maxSearchDist)
				{
					maxRadius = params.maxSearchDist;
				}

				unsigned count = Yk.size();
				for (unsigned j = 0; j < count; ++j)
				{
					Yk.setPointScalarValue(j, maxRadius);
				}

				//Yk.clear(); //useless
			}

			return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
		}

		//otherwise we have to compute the distance from each point to its nearest triangle

		//Progress callback
		NormalizedProgress nProgress(progressCb, numberOfCells);
		if (progressCb)
		{
			if (progressCb->textCanBeEdited())
			{
				char buffer[256];
				sprintf(buffer, "Cells: %u", numberOfCells);
				progressCb->setInfo(buffer);
				progressCb->setMethodTitle(params.signedDistances ? "Compute signed distances" : "Compute distances");
			}
			progressCb->update(0);
			progressCb->start();
		}

		TrianglesToTest ttt(*mesh);

		//min distance array ('persistent' version to save some memory)
		std::vector<ScalarType> minDists;

		//maximal neighbors search distance (if maxSearchDist is defined)
		int maxNeighbourhoodLength = 0;
		if (boundedSearch)
		{
			maxNeighbourhoodLength = ComputeMaxNeighborhoodLength(params.maxSearchDist, cellSize);
		}

		//for each cell
		for (unsigned cellIndex = 1; cellIndex <= numberOfCells; ++cellIndex, ++pCodeAndIndex) //cellIndex = unique ID for the current cell
		{
			if (!octree->getPointsInCellByCellIndex(&Yk, pCodeAndIndex->theIndex, params.octreeLevel))
			{
				return DISTANCE_COMPUTATION_RESULTS::ERROR_EXECUTE_GET_POINTS_IN_CELL_BY_INDEX_FAILURE;
			}

			//get cell pos
			Tuple3i cellPos;
			octree->getCellPos(pCodeAndIndex->theCode, params.octreeLevel, cellPos, true);

			int result = ComputeNeighborhood2MeshDistancesWithOctree(	intersection,
																		params,
																		Yk,
																		cellIndex,
																		cellPos,
																		ttt,
																		boundedSearch,
																		maxNeighbourhoodLength);

			if (result != DISTANCE_COMPUTATION_RESULTS::SUCCESS)
			{
				// an error occurred
				return result;
			}

			//Yk.clear(); //not necessary

			if (progressCb && !nProgress.oneStep())
			{
				//process cancelled by the user
				return DISTANCE_COMPUTATION_RESULTS::CANCELED_BY_USER;
			}
		}

		return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
	}
#ifdef ENABLE_CLOUD2MESH_DIST_MT
	else
	{
		//extraction des indexes et codes des cellules du niveau "octreeLevel"
		DgmOctree::cellsContainer cellsDescs;
		if (!octree->getCellCodesAndIndexes(params.octreeLevel, cellsDescs, true))
		{
			return DISTANCE_COMPUTATION_RESULTS::ERROR_GET_CELL_CODES_AND_INDEXES_FAILURE;
		}

		unsigned numberOfCells = static_cast<unsigned>(cellsDescs.size());

		//Progress callback
		NormalizedProgress nProgress(progressCb,numberOfCells);
		if (progressCb)
		{
			if (progressCb->textCanBeEdited())
			{
				char buffer[256];
				sprintf(buffer, "Cells: %u", numberOfCells);
				progressCb->setInfo(buffer);
				progressCb->setMethodTitle("Compute signed distances");
			}
			progressCb->update(0);
			progressCb->start();
		}

		s_octree_MT = octree;
		s_normProgressCb_MT = &nProgress;
		s_cellFunc_MT_success = true;
		s_cellFunc_MT_results = DISTANCE_COMPUTATION_RESULTS::SUCCESS;
		s_params_MT = params;
		s_intersection_MT = &intersection;
		//acceleration structure
		s_useBitArrays_MT = true;

		//Single thread emulation
		//for (unsigned i = 0; i < numberOfCells; ++i)
		//	cloudMeshDistCellFunc_MT(cellsDescs[i]);

#ifdef CC_CORE_LIB_USES_QT_CONCURRENT
		int maxThreadCount = params.maxThreadCount;
		if (maxThreadCount == 0)
		{
			maxThreadCount = QThread::idealThreadCount();
		}
		QThreadPool::globalInstance()->setMaxThreadCount(maxThreadCount);
		QtConcurrent::blockingMap(cellsDescs, cloudMeshDistCellFunc_MT);
#elif defined(CC_CORE_LIB_USES_TBB)
		tbb::parallel_for(tbb::blocked_range<int>(0,cellsDescs.size()),
			[&](tbb::blocked_range<int> r) {
				for (auto i = r.begin(); r.end(); ++i) { cloudMeshDistCellFunc_MT(cellsDescs[i]); }
			}
		);
#endif

		s_octree_MT = nullptr;
		s_normProgressCb_MT = nullptr;
		s_intersection_MT = nullptr;

		//clean acceleration structure
		while (!s_bitArrayPool_MT.empty())
		{
			delete s_bitArrayPool_MT.back();
			s_bitArrayPool_MT.pop_back();
		}
		if (!s_cellFunc_MT_success && s_cellFunc_MT_results == DISTANCE_COMPUTATION_RESULTS::SUCCESS)
		{
			s_cellFunc_MT_results = DISTANCE_COMPUTATION_RESULTS::ERROR_EXECUTE_CLOUD_MESH_DIST_CELL_FUNC_MT_FAILURE;
		}
		return (s_cellFunc_MT_success ? DISTANCE_COMPUTATION_RESULTS::SUCCESS : s_cellFunc_MT_results);
	}
#endif
}

//convert all 'distances' (squared in fact) to their square root
inline void applySqrtToPointDist(const CCVector3 &aPoint, ScalarType& aScalarValue)
{
	if (ScalarField::ValidValue(aScalarValue))
		aScalarValue = sqrt(aScalarValue);
}

int DistanceComputationTools::computeCloud2MeshDistances(	GenericIndexedCloudPersist* pointCloud,
															GenericIndexedMesh* mesh,
															Cloud2MeshDistancesComputationParams& params,
															GenericProgressCallback* progressCb/*=nullptr*/,
															DgmOctree* cloudOctree/*=nullptr*/)
{
	//check the input
	if (!pointCloud) 
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}

	if (pointCloud->size() == 0)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}

	if (!mesh)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_REFERENCEMESH;
	}

	if (mesh->size() == 0)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_REFERENCEMESH;
	}


	if (params.signedDistances)
	{
		//signed distances are incompatible with approximate distances (with Distance Transform)
		params.useDistanceMap = false;
	}
	if (params.CPSet)
	{
		//Closest Point Set determination is incompatible with distance map approximation and max search distance
		params.useDistanceMap = false;
		params.maxSearchDist = 0;
	}


	//compute the bounding box that contains both the cloud and the mesh BBs
	CCVector3 cubicalMinBB;
	CCVector3 cubicalMaxBB;
	//max (non-cubical) bounding-box
	CCVector3 minBB;
	CCVector3 maxBB;
	{
		CCVector3 cloudMinBB;
		CCVector3 cloudMaxBB;
		pointCloud->getBoundingBox(cloudMinBB, cloudMaxBB);

		CCVector3 meshMinBB;
		CCVector3 meshMaxBB;
		mesh->getBoundingBox(meshMinBB, meshMaxBB);

		for (unsigned char k = 0; k < 3; ++k)
		{
			minBB.u[k] = std::min(meshMinBB.u[k], cloudMinBB.u[k]);
			maxBB.u[k] = std::max(meshMaxBB.u[k], cloudMaxBB.u[k]);
		}

		//max cubical bounding-box 
		cubicalMinBB = minBB;
		cubicalMaxBB = maxBB;

		//we make this bounding-box cubical (+0.1% growth to avoid round-off issues when projecting points or triangles in the grid)
		CCMiscTools::MakeMinAndMaxCubical(cubicalMinBB, cubicalMaxBB, 0.001);
	}

	//compute the octree if necessary
	DgmOctree tempOctree(pointCloud);
	DgmOctree* octree = cloudOctree;
	bool rebuildTheOctree = false;
	if (!octree)
	{
		octree = &tempOctree;
		rebuildTheOctree = true;
	}
	else
	{
		//check the input octree dimensions
		const CCVector3& theOctreeMins = octree->getOctreeMins();
		const CCVector3& theOctreeMaxs = octree->getOctreeMaxs();
		for (unsigned char k = 0; k < 3; ++k)
		{
			if (	theOctreeMins.u[k] != cubicalMinBB.u[k]
				||	theOctreeMaxs.u[k] != cubicalMaxBB.u[k] )
			{
				rebuildTheOctree = true;
				break;
			}
		}
	}

	if (rebuildTheOctree)
	{
		//build the octree
		if (octree->build(cubicalMinBB, cubicalMaxBB, nullptr, nullptr, progressCb) <= 0)
		{
			return DISTANCE_COMPUTATION_RESULTS::ERROR_BUILD_OCTREE_FAILURE;
		}
	}

	PointCoordinateType cellSize = octree->getCellSize(params.octreeLevel);

	//Intersect the octree (grid) with the mesh
	GridAndMeshIntersection intersection;
	if (params.useDistanceMap)
	{
		if (false == intersection.initDistanceTransformWithMesh(mesh, cubicalMinBB, cubicalMaxBB, minBB, maxBB, cellSize, progressCb))
		{
			return DISTANCE_COMPUTATION_RESULTS::ERROR_INTERSECT_MESH_WITH_OCTREE_FAILURE;
		}
	}
	else
	{
		if (false == intersection.computeMeshIntersection(mesh, cubicalMinBB, cubicalMaxBB, cellSize, progressCb))
		{
			return DISTANCE_COMPUTATION_RESULTS::ERROR_INTERSECT_MESH_WITH_OCTREE_FAILURE;
		}
	}

	//reset the output distances
	pointCloud->enableScalarField();
	pointCloud->forEach(ScalarFieldTools::SetScalarValueToNaN);

	//WE CAN EVENTUALLY COMPUTE THE DISTANCES!
	int result = computeCloud2MeshDistancesWithOctree(octree, intersection, params, progressCb);

	//don't forget to compute the square root of the (squared) unsigned distances
	if (result == DISTANCE_COMPUTATION_RESULTS::SUCCESS &&
			!params.signedDistances &&
			!params.useDistanceMap)
	{
		pointCloud->forEach(applySqrtToPointDist);
	}

	if (result < DISTANCE_COMPUTATION_RESULTS::SUCCESS)
	{
		if (!(result == DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY ||
			  result == DISTANCE_COMPUTATION_RESULTS::CANCELED_BY_USER))
		{
			return result;
		}
		return DISTANCE_COMPUTATION_RESULTS::ERROR_COMPUTE_CLOUD2_MESH_DISTANCE_WITH_OCTREE_FAILURE;
	}

	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

// Inspired from documents and code by:
// David Eberly
// Geometric Tools, LLC
// http://www.geometrictools.com/
ScalarType DistanceComputationTools::computePoint2TriangleDistance(	const CCVector3* P,
																	const GenericTriangle* theTriangle,
																	bool signedDistances,
																	CCVector3* nearestP/*=nullptr*/)
{
	assert(P && theTriangle);

	const CCVector3* A = theTriangle->_getA();
	const CCVector3* B = theTriangle->_getB();
	const CCVector3* C = theTriangle->_getC();

	//we do all computations with double precision, otherwise
	//some triangles with sharp angles will give very poor results.
	//slight precision improvement by casting to double prior to subtraction
	CCVector3d AP(static_cast<double>(P->x) - A->x, static_cast<double>(P->y) - A->y, static_cast<double>(P->z) - A->z);
	CCVector3d AB(static_cast<double>(B->x) - A->x, static_cast<double>(B->y) - A->y, static_cast<double>(B->z) - A->z);
	CCVector3d AC(static_cast<double>(C->x) - A->x, static_cast<double>(C->y) - A->y, static_cast<double>(C->z) - A->z);

	double a00 =  AB.dot(AB);
	double a01 =  AB.dot(AC);
	double a11 =  AC.dot(AC);
	double b0  = -AP.dot(AB);
	double b1  = -AP.dot(AC);
	double det = a00 * a11 - a01 * a01;
	double t0  = a01 * b1 - a11 * b0;
	double t1  = a01 * b0 - a00 * b1;

	if (t0 + t1 <= det)
	{
		if (t0 < 0)
		{
			if (t1 < 0)	 // region 4
			{
				if (b0 < 0)
				{
					t1 = 0;
					if (-b0 >= a00)	 // V0
					{
						t0 = 1.0;
					}
					else  // E01
					{
						t0 = -b0 / a00;
					}
				}
				else
				{
					t0 = 0;
					if (b1 >= 0)  // V0
					{
						t1 = 0;
					}
					else if (-b1 >= a11)  // V2
					{
						t1 = 1.0;
					}
					else  // E20
					{
						t1 = -b1 / a11;
					}
				}
			}
			else  // region 3
			{
				t0 = 0;
				if (b1 >= 0)  // V0
				{
					t1 = 0;
				}
				else if (-b1 >= a11)  // V2
				{
					t1 = 1.0;
				}
				else  // E20
				{
					t1 = -b1 / a11;
				}
			}
		}
		else if (t1 < 0)  // region 5
		{
			t1 = 0;
			if (b0 >= 0)  // V0
			{
				t0 = 0;
			}
			else if (-b0 >= a00)  // V1
			{
				t0 = 1.0;
			}
			else  // E01
			{
				t0 = -b0 / a00;
			}
		}
		else  // region 0, interior
		{
			t0 /= det;
			t1 /= det;
		}
	}
	else
	{
		double tmp0;
		double tmp1;
		double numer;
		double denom;

		if (t0 < 0)	 // region 2
		{
			tmp0 = a01 + b0;
			tmp1 = a11 + b1;
			if (tmp1 > tmp0)
			{
				numer = tmp1 - tmp0;
				denom = a00 - 2*a01 + a11;
				if (numer >= denom)	 // V1
				{
					t0 = 1.0;
					t1 = 0;
				}
				else  // E12
				{
					t0 = numer / denom;
					t1 = 1.0 - t0;
				}
			}
			else
			{
				t0 = 0;
				if (tmp1 <= 0)	// V2
				{
					t1 = 1.0;
				}
				else if (b1 >= 0)  // V0
				{
					t1 = 0;
				}
				else  // E20
				{
					t1 = -b1 / a11;
				}
			}
		}
		else if (t1 < 0)  // region 6
		{
			tmp0 = a01 + b1;
			tmp1 = a00 + b0;
			if (tmp1 > tmp0)
			{
				numer = tmp1 - tmp0;
				denom = a00 - 2*a01 + a11;
				if (numer >= denom)	 // V2
				{
					t1 = 1.0;
					t0 = 0;
				}
				else  // E12
				{
					t1 = numer / denom;
					t0 = 1.0 - t1;
				}
			}
			else
			{
				t1 = 0;
				if (tmp1 <= 0)	// V1
				{
					t0 = 1.0;
				}
				else if (b0 >= 0)  // V0
				{
					t0 = 0;
				}
				else  // E01
				{
					t0 = -b0 / a00;
				}
			}
		}
		else  // region 1
		{
			numer = a11 + b1 - a01 - b0;
			if (numer <= 0)	 // V2
			{
				t0 = 0;
				t1 = 1.0;
			}
			else
			{
				denom = a00 - 2*a01 + a11;
				if (numer >= denom)	 // V1
				{
					t0 = 1.0;
					t1 = 0;
				}
				else  // 12
				{
					t0 = numer / denom;
					t1 = 1.0 - t0;
				}
			}
		}
	}

	//point on the plane (relative to A)
	CCVector3d Q = t0 * AB + t1 * AC;

	if (nearestP)
	{
		//if requested we also output the nearest point
		*nearestP = *A + Q.toPC();
	}

	double squareDist = (Q - AP).norm2();
	if (signedDistances)
	{
		ScalarType d = static_cast<ScalarType>(sqrt(squareDist));

		//triangle normal
		CCVector3d N = AB.cross(AC);

		//we test the sign of the cross product of the triangle normal and the vector AP
		return (AP.dot(N) < 0 ? -d : d);
	}
	else
	{
		return static_cast<ScalarType>(squareDist);
	}
}

ScalarType DistanceComputationTools::computePoint2PlaneDistance(const CCVector3* P,
																const PointCoordinateType* planeEquation)
{
	//point to plane distance: d = (a0*x+a1*y+a2*z - a3) / sqrt(a0^2+a1^2+a2^2)
	assert(std::abs(CCVector3::vnorm(planeEquation) - PC_ONE) <= std::numeric_limits<PointCoordinateType>::epsilon());

	return static_cast<ScalarType>((CCVector3::vdot(P->u, planeEquation) - planeEquation[3])/*/CCVector3::vnorm(planeEquation)*/); //norm == 1.0!
}

ScalarType DistanceComputationTools::computePoint2LineSegmentDistSquared(const CCVector3* p, const CCVector3* start, const CCVector3* end)
{
	assert(p && start && end);
	CCVector3 line = *end - *start;
	CCVector3 vec;
	ScalarType distSq;
	PointCoordinateType t = line.dot(*p - *start);
	PointCoordinateType normSq = line.norm2();
	if (normSq != 0)
	{
		t /= normSq;
	}
	if (t < 0)
	{
		vec = *p - *start;
	}
	else if (t > 1)
	{
		vec = *p - *end;
	}
	else
	{
		CCVector3 pProjectedOnLine = *start + t * line;
		vec = *p - pProjectedOnLine;
	}

	distSq = vec.norm2();
	return distSq;
}

// This algorithm is a modification of the distance computation between a point and a cone from
// Barbier & Galin's Fast Distance Computation Between a Point and Cylinders, Cones, Line Swept Spheres and Cone-Spheres.
// The modifications from the paper are to compute the closest distance when the point is interior to the cone.
// http://liris.cnrs.fr/Documents/Liris-1297.pdf
int DistanceComputationTools::computeCloud2ConeEquation(GenericIndexedCloudPersist* cloud, const CCVector3& coneP1, const CCVector3& coneP2, const PointCoordinateType coneR1, const PointCoordinateType coneR2, bool signedDistances/*=true*/, bool solutionType/*=false*/, double* rms/*=nullptr*/)
{
	if (!cloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}
	unsigned count = cloud->size();
	if (count == 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}
	if (!cloud->enableScalarField())
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_ENABLE_SCALAR_FIELD_FAILURE;
	}
	if (coneR1 < coneR2)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_CONE_R1_LT_CONE_R2;
	}
	double dSumSq = 0.0;

	CCVector3 coneAxis = coneP2 - coneP1;
	double axisLength = coneAxis.normd();
	coneAxis.normalize();
	double delta = static_cast<double>(coneR2) - coneR1;
	double rr1 = static_cast<double>(coneR1) * coneR1;
	double rr2 = static_cast<double>(coneR2) * coneR2;
	double coneLength = sqrt((axisLength * axisLength) + (delta * delta));
	CCVector3d side{ axisLength, delta, 0.0};
	side /= coneLength;

	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3* P = cloud->getPoint(i);
		CCVector3 n = *P - coneP1;
		double x = n.dot(coneAxis);
		double xx = x * x;
		double yy = (n.norm2d()) - xx;
		double y = 0;
		double d = 0;
		double rx = 0;
		double ry = 0;
		if (yy < 0)
		{
			yy = 0;
		}
		if(x <= 0) //Below the bottom point
		{
			if (yy < rr1)
			{
				if (!solutionType)
				{
					d = -x; //Below the bottom point within larger disk radius
				}
				else
				{
					d = 1; //Works
				}
			}
			else
			{
				if (!solutionType)
				{
					y = sqrt(yy) - coneR1;
					d = sqrt((y * y) + xx); //Below the bottom point not within larger disk radius
				}
				else
				{
					d = 2; //Works
				}
			}

		}
		else //Above the bottom point
		{
			if (yy < rr2) // within smaller disk radius
			{
				if (x > axisLength) //outside cone within smaller disk
				{
					if (!solutionType)
					{
						d = x - axisLength; // Above the top point within top disk radius
					}
					else
					{
						d = 3;
					}
				}
				else //inside cone within smaller disk
				{
					if (!solutionType)
					{
						y = sqrt(yy) - coneR1;
						ry = y * side[0] - x * side[1]; //rotated y value (distance from the coneside axis)
						d = std::min(axisLength - x, x); //determine whether closer to either radii
						d = std::min(d, static_cast<double>(std::abs(ry))); //or to side
						d = -d; //negative inside
					}
					else
					{
						d = 4;
					}
				}
			}
			else // Outside smaller disk radius
			{
				y = sqrt(yy) - coneR1;
				{
					rx = y * side[1] + x * side[0]; //rotated x value (distance along the coneside axis)
					if (rx < 0)
					{
						if (!solutionType)
						{
							d = sqrt(y * y + xx);
						}
						else
						{
							d = 7; // point projects onto the large cap
						}
					}
					else
					{
						ry = y * side[0] - x * side[1]; //rotated y value (distance from the coneside axis)
						if (rx > coneLength)
						{
							if (!solutionType)
							{
								rx -= coneLength;
								d = sqrt(ry * ry + rx * rx);
							}
							else
							{
								d = 8; // point projects onto the small cap
							}
						}
						else
						{
							if (!solutionType)
							{
								d = ry; // point projects onto the side of cone
								if (ry < 0)
								{//point is interior to the cone
									d = std::min(axisLength - x, x); //determine whether closer to either radii
									d = std::min(d, static_cast<double>(std::abs(ry))); //or to side
									d = -d; //negative inside
								}
							}
							else
							{
								d = 9;
							}
						}
					}

				}
			}
		}
		if (signedDistances)
		{
			cloud->setPointScalarValue(i, static_cast<ScalarType>(d));
		}
		else
		{
			cloud->setPointScalarValue(i, static_cast<ScalarType>(std::abs(d)));
		}
		dSumSq += d * d;
	}
	if (rms)
	{
		*rms = sqrt(dSumSq / count);
	}
	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

// This algorithm is a modification of the distance computation between a point and a cylinder from
// Barbier & Galin's Fast Distance Computation Between a Point and Cylinders, Cones, Line Swept Spheres and Cone-Spheres.
// The modifications from the paper are to compute the closest distance when the point is interior to the capped cylinder.
// http://liris.cnrs.fr/Documents/Liris-1297.pdf
// solutionType 1 = exterior to the cylinder and within the bounds of the axis
// solutionType 2 = interior to the cylinder and either closer to an end-cap or the cylinder wall
// solutionType 3 = beyond the bounds of the cylinder's axis and radius
// solutionType 4 = beyond the bounds of the cylinder's axis but within the bounds of it's radius
int DistanceComputationTools::computeCloud2CylinderEquation(GenericIndexedCloudPersist* cloud,
															const CCVector3& cylinderP1,
															const CCVector3& cylinderP2,
															const PointCoordinateType cylinderRadius,
															bool signedDistances/*=true*/,
															bool solutionType/*=false*/,
															double* rms/*=nullptr*/)
{
	if (!cloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}
	unsigned count = cloud->size();
	if (count == 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}
	if (!cloud->enableScalarField())
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_ENABLE_SCALAR_FIELD_FAILURE;
	}
	double dSumSq = 0.0;

	CCVector3 cylinderCenter = (cylinderP1 + cylinderP2) / 2.;

	CCVector3 cylinderAxis = cylinderP2 - cylinderP1;
	double h = cylinderAxis.normd() / 2.;
	cylinderAxis.normalize();
	double cylinderRadius2 = static_cast<double>(cylinderRadius)* cylinderRadius;

	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3* P = cloud->getPoint(i);
		CCVector3 n = *P - cylinderCenter;

		double x = std::abs(n.dot(cylinderAxis));
		double xx = x * x;
		double yy = (n.norm2d()) - xx;
		double y = 0;
		double d = 0;
		if (x <= h)
		{
			if (yy >= cylinderRadius2)
			{
				if (!solutionType)
				{
					d = sqrt(yy) - cylinderRadius; //exterior to the cylinder and within the bounds of the axis
				}
				else
				{
					d = 1;
				}
			}
			else
			{
				if (!solutionType)
				{
					d = -std::min(std::abs(sqrt(yy) - cylinderRadius), std::abs(h - x)); //interior to the cylinder and either closer to an end-cap or the cylinder wall
				}
				else
				{
					d = 2;
				}
			}
		}
		else
		{
			if (yy >= cylinderRadius2)
			{
				if (!solutionType)
				{
					y = sqrt(yy);
					d = sqrt(((y - cylinderRadius) * (y - cylinderRadius)) + ((x - h) * (x - h))); //beyond the bounds of the cylinder's axis and radius
				}
				else
				{
					d = 3;
				}
			}
			else
			{
				if (!solutionType)
				{
					d = x - h; //beyond the bounds of the cylinder's axis but within the bounds of it's radius
				}
				else
				{
					d = 4;
				}
			}

		}

		if (signedDistances)
		{
			cloud->setPointScalarValue(i, static_cast<ScalarType>(d));
		}
		else
		{
			cloud->setPointScalarValue(i, static_cast<ScalarType>(std::abs(d)));
		}
		dSumSq += d * d;
	}
	if (rms)
	{
		*rms = sqrt(dSumSq / count);
	}

	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

int DistanceComputationTools::computeCloud2SphereEquation(	GenericIndexedCloudPersist *cloud,
															const CCVector3& sphereCenter,
															const PointCoordinateType sphereRadius,
															bool signedDistances/*=true*/,
															double* rms/*=nullptr*/)
{
	if (!cloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}
	unsigned count = cloud->size();
	if (count == 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}
	if (!cloud->enableScalarField())
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_ENABLE_SCALAR_FIELD_FAILURE;
	}
	double dSumSq = 0.0;
	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3* P = cloud->getPoint(i);
		double d = (*P - sphereCenter).normd() - sphereRadius;
		if (signedDistances)
		{
			cloud->setPointScalarValue(i, static_cast<ScalarType>(d));
		}
		else
		{
			cloud->setPointScalarValue(i, static_cast<ScalarType>(std::abs(d)));
		}
		dSumSq += d * d;
	}
	if (rms)
	{
		*rms = sqrt(dSumSq / count);
	}

	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

int DistanceComputationTools::computeCloud2PlaneEquation(	GenericIndexedCloudPersist *cloud,
															const PointCoordinateType* planeEquation,
															bool signedDistances/*=true*/,
															double* rms/*=nullptr*/)
{
	assert(cloud && planeEquation);
	if (!cloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}
	if (!planeEquation)
	{
		return DISTANCE_COMPUTATION_RESULTS::NULL_PLANE_EQUATION;
	}
	unsigned count = cloud->size();
	if (count == 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}
	if (!cloud->enableScalarField())
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_ENABLE_SCALAR_FIELD_FAILURE;
	}

	//point to plane distance: d = std::abs(a0*x+a1*y+a2*z-a3) / sqrt(a0^2+a1^2+a2^2) <-- "norm"
	double norm2 = CCVector3::vnorm2d(planeEquation);
	//the norm should always be equal to 1.0!
	if (LessThanSquareEpsilon(norm2))
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_PLANE_NORMAL_LT_ZERO;
	}
	assert(LessThanEpsilon(std::abs(norm2 - 1.0)));

	//compute deviations
	double dSumSq = 0.0;
	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3* P = cloud->getPoint(i);
		double d = CCVector3::vdotd(P->u, planeEquation) - planeEquation[3];
		if (signedDistances)
		{
			cloud->setPointScalarValue(i, static_cast<ScalarType>(d));
		}
		else
		{
			cloud->setPointScalarValue(i, static_cast<ScalarType>(std::abs(d)));
		}
		dSumSq += d * d;
	}
	if (rms)
	{
		*rms = sqrt(dSumSq / count);
	}

	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

int DistanceComputationTools::computeCloud2RectangleEquation(	GenericIndexedCloudPersist *cloud,
																PointCoordinateType widthX,
																PointCoordinateType widthY,
																const SquareMatrix& rotationTransform,
																const CCVector3& center,
																bool signedDistances/*=true*/,
																double* rms/*=nullptr*/)
{
	// p3---------------------p2
	// ^					  |
	// |e1					  |
	// |					  |
	// |		  e0		  |
	// p0-------------------->p1
	//Rect(s,t)=p0 + s*0e + t*e1
	//for s,t in {0,1}
	assert(cloud);
	if (!cloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}
	unsigned count = cloud->size();
	if (count == 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}
	if (!cloud->enableScalarField())
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_ENABLE_SCALAR_FIELD_FAILURE;
	}
	if (widthX <= 0 || widthY <= 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_INVALID_PRIMITIVE_DIMENSIONS;
	}

	CCVector3 widthXVec(widthX, 0, 0);
	CCVector3 widthYVec(0, widthY, 0);
	CCVector3 normalVector(0, 0, 1);

	widthXVec = rotationTransform * widthXVec;
	widthYVec = rotationTransform * widthYVec;
	normalVector = rotationTransform * normalVector;
	PointCoordinateType planeDistance = center.dot(normalVector);
	PointCoordinateType dSumSq = 0;
	CCVector3 rectangleP0 = center - (widthXVec / 2) - (widthYVec / 2);
	CCVector3 rectangleP1 = center + (widthXVec / 2) - (widthYVec / 2);
	CCVector3 rectangleP3 = center - (widthXVec / 2) + (widthYVec / 2);
	CCVector3 e0 = rectangleP1 - rectangleP0;
	CCVector3 e1 = rectangleP3 - rectangleP0;

	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3* pe = cloud->getPoint(i);
		CCVector3 dist = (*pe - rectangleP0);
		PointCoordinateType s = e0.dot(dist);
		if (s > 0)
		{
			PointCoordinateType dot0 = e0.norm2();
			if (s < dot0)
			{
				dist -= (s / dot0)*e0;
			}
			else
			{
				dist -= e0;
			}
		}

		PointCoordinateType t = e1.dot(dist);
		if (t > 0)
		{
			PointCoordinateType dot1 = e1.norm2();
			if (t < dot1)
			{
				dist -= (t / dot1)*e1;
			}
			else
			{
				dist -= e1;
			}
		}
		PointCoordinateType d = static_cast<PointCoordinateType>(dist.normd());
		dSumSq += d * d;
		if (signedDistances && pe->dot(normalVector) - planeDistance < 0)
		{
			d = -d;
		}
		cloud->setPointScalarValue(i, static_cast<ScalarType>(d));
	}
	if (rms)
	{
		*rms = sqrt(dSumSq / count);
	}
	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

int DistanceComputationTools::computeCloud2BoxEquation(	GenericIndexedCloudPersist* cloud,
														const CCVector3& boxDimensions,
														const SquareMatrix& rotationTransform,
														const CCVector3& boxCenter,
														bool signedDistances/*=true*/,
														double* rms/*=nullptr*/)
{
	assert(cloud);
	if (!cloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}
	unsigned count = cloud->size();
	if (count == 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}
	if (!cloud->enableScalarField())
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_ENABLE_SCALAR_FIELD_FAILURE;
	}
	if (boxDimensions.x <= 0 || boxDimensions.y <= 0 || boxDimensions.z <= 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_INVALID_PRIMITIVE_DIMENSIONS;
	}
	// box half lengths hu hv and hw
	const PointCoordinateType hu = boxDimensions.x / 2;
	const PointCoordinateType hv = boxDimensions.y / 2;
	const PointCoordinateType hw = boxDimensions.z / 2;
	// box coordinates unit vectors u,v, and w
	CCVector3 u(1, 0, 0);
	CCVector3 v(0, 1, 0);
	CCVector3 w(0, 0, 1);
	u = rotationTransform * u;
	v = rotationTransform * v;
	w = rotationTransform * w;
	PointCoordinateType dSumSq = 0;
	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3* p = cloud->getPoint(i);
		CCVector3 pointCenterDifference = (*p - boxCenter);
		CCVector3 p_inBoxCoords(pointCenterDifference.dot(u), pointCenterDifference.dot(v), pointCenterDifference.dot(w));
		bool insideBox = false;
		if (p_inBoxCoords.x > -hu && p_inBoxCoords.x < hu && p_inBoxCoords.y > -hv && p_inBoxCoords.y < hv && p_inBoxCoords.z > -hw && p_inBoxCoords.z < hw)
		{
			insideBox = true;
		}

		CCVector3 dist(0, 0, 0);
		if (p_inBoxCoords.x < -hu)
		{
			dist.x = -(p_inBoxCoords.x + hu);
		}
		else if (p_inBoxCoords.x > hu)
		{
			dist.x = p_inBoxCoords.x - hu;
		}
		else if (insideBox)
		{
			dist.x = std::abs(p_inBoxCoords.x) - hu;
		}

		if (p_inBoxCoords.y < -hv)
		{
			dist.y = -(p_inBoxCoords.y + hv);
		}
		else if (p_inBoxCoords.y > hv)
		{
			dist.y = p_inBoxCoords.y - hv;
		}
		else if (insideBox)
		{
			dist.y = std::abs(p_inBoxCoords.y) - hv;
		}

		if (p_inBoxCoords.z < -hw)
		{
			dist.z = -(p_inBoxCoords.z + hw);
		}
		else if (p_inBoxCoords.z > hw)
		{
			dist.z = p_inBoxCoords.z - hw;
		}
		else if (insideBox)
		{
			dist.z = std::abs(p_inBoxCoords.z) - hw;
		}

		if (insideBox)//take min distance inside box
		{
			if (dist.x >= dist.y && dist.x >= dist.z)
			{
				dist.y = 0;
				dist.z = 0;
			}
			else if (dist.y >= dist.x && dist.y >= dist.z)
			{
				dist.x = 0;
				dist.z = 0;
			}
			else if (dist.z >= dist.x && dist.z >= dist.y)
			{
				dist.x = 0;
				dist.y = 0;
			}
		}
		PointCoordinateType d = static_cast<PointCoordinateType>(dist.normd());
		dSumSq += d * d;
		if (signedDistances && insideBox)
		{
			d = -d;
		}
		cloud->setPointScalarValue(i, static_cast<ScalarType>(d));
	}
	if (rms)
	{
		*rms = sqrt(dSumSq / count);
	}
	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

int DistanceComputationTools::computeCloud2PolylineEquation(GenericIndexedCloudPersist* cloud,
															const Polyline* polyline,
															double* rms/*=nullptr*/)
{
	if (!cloud)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}
	unsigned count = cloud->size();
	if (count == 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}
	if (!cloud->enableScalarField())
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_ENABLE_SCALAR_FIELD_FAILURE;
	}
	if (!polyline)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_REFERENCEPOLYLINE;
	}
	if (polyline->size() < 2)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_TOOSMALL_REFERENCEPOLYLINE;
	}
	ScalarType d = 0;
	ScalarType dSumSq = 0;
	for (unsigned i = 0; i < count; ++i)
	{
		ScalarType distSq = NAN_VALUE;
		const CCVector3* p = cloud->getPoint(i);
		for (unsigned j = 0; j < polyline->size() - 1; j++)
		{
			const CCVector3* start = polyline->getPoint(j);
			const CCVector3* end = polyline->getPoint(j + 1);
			CCVector3d startMinusP = (*start - *p);
			CCVector3d endMinusP = (*end - *p);

			//Rejection test
			if (	((startMinusP.x * startMinusP.x >= distSq) && (endMinusP.x * endMinusP.x >= distSq) && GreaterThanSquareEpsilon( startMinusP.x * endMinusP.x )) ||
					((startMinusP.y * startMinusP.y >= distSq) && (endMinusP.y * endMinusP.y >= distSq) && GreaterThanSquareEpsilon( startMinusP.y * endMinusP.y )) ||
					((startMinusP.z * startMinusP.z >= distSq) && (endMinusP.z * endMinusP.z >= distSq) && GreaterThanSquareEpsilon( startMinusP.z * endMinusP.z ))
					)
			{
				continue;
			}
			if (std::isnan(distSq))
			{
				distSq = computePoint2LineSegmentDistSquared(p, start, end);
			}
			else
			{
				distSq = std::min(distSq, computePoint2LineSegmentDistSquared(p, start, end));
			}
		}
		d = sqrt(distSq);
		dSumSq += distSq;
		cloud->setPointScalarValue(i, d);
	}

	if (rms)
	{
		*rms = sqrt(dSumSq / count);
	}
	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

ScalarType DistanceComputationTools::computeCloud2PlaneDistanceRMS( GenericCloud* cloud,
																	const PointCoordinateType* planeEquation)
{
	assert(cloud && planeEquation);
	if (!cloud)
	{
		return 0;
	}
	//point count
	unsigned count = cloud->size();
	if (count == 0)
	{
		return 0;
	}

	//point to plane distance: d = std::abs(a0*x+a1*y+a2*z-a3) / sqrt(a0^2+a1^2+a2^2) <-- "norm"
	double norm2 = CCVector3::vnorm2d(planeEquation);
	//the norm should always be equal to 1.0!
	if (LessThanSquareEpsilon(norm2))
	{
		return NAN_VALUE;
	}
	assert(LessThanEpsilon(std::abs(norm2 - 1.0)));

	//compute deviations
	double dSumSq = 0.0;
	cloud->placeIteratorAtBeginning();
	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3* P = cloud->getNextPoint();
		double d = CCVector3::vdotd(P->u, planeEquation) - planeEquation[3]/*/norm*/; //norm == 1.0

		dSumSq += d*d;
	}

	return static_cast<ScalarType>(sqrt(dSumSq / count));
}

ScalarType DistanceComputationTools::ComputeCloud2PlaneRobustMax(	GenericCloud* cloud,
																	const PointCoordinateType* planeEquation,
																	float percent)
{
	assert(cloud && planeEquation);
	assert(percent < 1.0f);

	if (!cloud)
	{
		return 0;
	}
	//point count
	unsigned count = cloud->size();
	if (count == 0)
	{
		return 0;
	}

	//point to plane distance: d = std::abs(a0*x+a1*y+a2*z-a3) / sqrt(a0^2+a1^2+a2^2) <-- "norm"
	double norm2 = CCVector3::vnorm2d(planeEquation);
	//the norm should always be equal to 1.0!
	if ( LessThanEpsilon( norm2 ) )
	{
		return NAN_VALUE;
	}
	assert(LessThanEpsilon(std::abs(norm2 - 1.0)));

	//we search the max @ 'percent'% (to avoid outliers)
	std::vector<PointCoordinateType> tail;
	std::size_t tailSize = static_cast<std::size_t>(ceil(static_cast<float>(count) * percent));
	tail.resize(tailSize);

	//compute deviations
	cloud->placeIteratorAtBeginning();
	std::size_t pos = 0;
	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3* P = cloud->getNextPoint();
		PointCoordinateType d = std::abs(CCVector3::vdot(P->u, planeEquation) - planeEquation[3])/*/norm*/; //norm == 1.0

		if (pos < tailSize)
		{
			tail[pos++] = d;
		}
		else if (tail.back() < d)
		{
			tail.back() = d;
		}

		//search the max element of the tail
		std::size_t maxPos = pos-1;
		if (maxPos != 0)
		{
			std::size_t maxIndex = maxPos;
			for (std::size_t j = 0; j < maxPos; ++j)
				if (tail[j] < tail[maxIndex])
					maxIndex = j;
			//and put it to the back!
			if (maxPos != maxIndex)
				std::swap(tail[maxIndex], tail[maxPos]);
		}
	}

	return static_cast<ScalarType>(tail.back());
}

ScalarType DistanceComputationTools::ComputeCloud2PlaneMaxDistance( GenericCloud* cloud,
																	const PointCoordinateType* planeEquation)
{
	assert(cloud && planeEquation);
	if (!cloud)
	{
		return 0;
	}
	//point count
	unsigned count = cloud->size();
	if (count == 0)
	{
		return 0;
	}

	//point to plane distance: d = std::abs(a0*x+a1*y+a2*z-a3) / sqrt(a0^2+a1^2+a2^2) <-- "norm"
	double norm2 = CCVector3::vnorm2d(planeEquation);
	//the norm should always be equal to 1.0!
	if ( LessThanEpsilon( norm2 ) )
	{
		return NAN_VALUE;
	}
	assert(LessThanEpsilon(std::abs(norm2 - 1.0)));

	//we search the max distance
	PointCoordinateType maxDist = 0;

	cloud->placeIteratorAtBeginning();
	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3* P = cloud->getNextPoint();
		PointCoordinateType d = std::abs(CCVector3::vdot(P->u,planeEquation) - planeEquation[3])/*/norm*/; //norm == 1.0
		maxDist = std::max(d,maxDist);
	}

	return static_cast<ScalarType>(maxDist);
}

ScalarType DistanceComputationTools::ComputeCloud2PlaneDistance(GenericCloud* cloud,
																const PointCoordinateType* planeEquation,
																ERROR_MEASURES measureType)
{
	switch (measureType)
	{
		case RMS:
			return DistanceComputationTools::computeCloud2PlaneDistanceRMS(cloud,planeEquation);

		case MAX_DIST_68_PERCENT:
			return DistanceComputationTools::ComputeCloud2PlaneRobustMax(cloud,planeEquation,0.32f);
		case MAX_DIST_95_PERCENT:
			return DistanceComputationTools::ComputeCloud2PlaneRobustMax(cloud,planeEquation,0.05f);
		case MAX_DIST_99_PERCENT:
			return DistanceComputationTools::ComputeCloud2PlaneRobustMax(cloud,planeEquation,0.01f);

		case MAX_DIST:
			return DistanceComputationTools::ComputeCloud2PlaneMaxDistance(cloud,planeEquation);

		default:
			assert(false);
			return -1.0;
	}
}

bool DistanceComputationTools::computeGeodesicDistances(GenericIndexedCloudPersist* cloud, unsigned seedPointIndex, unsigned char octreeLevel, GenericProgressCallback* progressCb)
{
	assert(cloud);
	if (!cloud)
	{
		return false;
	}
	unsigned n = cloud->size();
	if (n == 0 || seedPointIndex >= n)
		return false;

	cloud->enableScalarField();
	cloud->forEach(ScalarFieldTools::SetScalarValueToNaN);

	DgmOctree* octree = new DgmOctree(cloud);
	if (octree->build(progressCb) < 1)
	{
		delete octree;
		return false;
	}

	FastMarchingForPropagation fm;
	if (fm.init(cloud,octree,octreeLevel,true) < 0)
	{
		delete octree;
		return false;
	}

	//on cherche la cellule de l'octree qui englobe le "seedPoint"
	Tuple3i cellPos;
	octree->getTheCellPosWhichIncludesThePoint(cloud->getPoint(seedPointIndex), cellPos, octreeLevel);
	fm.setSeedCell(cellPos);

	bool result = false;
	if (fm.propagate() >= 0)
		result = fm.setPropagationTimingsAsDistances();

	delete octree;
	octree = nullptr;

	return result;
}

int DistanceComputationTools::diff( GenericIndexedCloudPersist* comparedCloud,
									GenericIndexedCloudPersist* referenceCloud,
									GenericProgressCallback* progressCb)
{
	if (!comparedCloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}

	if (!referenceCloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_REFERENCECLOUD;
	}

	unsigned nA = comparedCloud->size();
	if (nA == 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}

	if (referenceCloud->size() == 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_REFERENCECLOUD;
	}
	//Reference cloud to store closest point set
	ReferenceCloud A_in_B(referenceCloud);

	Cloud2CloudDistancesComputationParams params;
	params.octreeLevel = DgmOctree::MAX_OCTREE_LEVEL - 1;
	params.CPSet = &A_in_B;

	int result = computeCloud2CloudDistances(comparedCloud, referenceCloud, params, progressCb);
	if (result < DISTANCE_COMPUTATION_RESULTS::SUCCESS)
	{
		if (!(result == DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY ||
			  result == DISTANCE_COMPUTATION_RESULTS::CANCELED_BY_USER))
		{
			return result;
		}
		return DISTANCE_COMPUTATION_RESULTS::ERROR_COMPUTE_CLOUD2_CLOUD_DISTANCE_FAILURE;
	}

	for (unsigned i = 0; i < nA; ++i)
	{
		ScalarType dA = comparedCloud->getPointScalarValue(i);
		ScalarType dB = A_in_B.getPointScalarValue(i);

		//handle invalid values
		comparedCloud->setPointScalarValue(i, ScalarField::ValidValue(dA) && ScalarField::ValidValue(dB) ? dA - dB : NAN_VALUE);
	}

	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

int DistanceComputationTools::computeApproxCloud2CloudDistance( GenericIndexedCloudPersist* comparedCloud,
																GenericIndexedCloudPersist* referenceCloud,
																unsigned char octreeLevel,
																PointCoordinateType maxSearchDist/*=-PC_ONE*/,
																GenericProgressCallback* progressCb/*=nullptr*/,
																DgmOctree* compOctree/*=nullptr*/,
																DgmOctree* refOctree/*=nullptr*/)
{
	if (!comparedCloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}
	if (!referenceCloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_REFERENCECLOUD;
	}
	if (octreeLevel < 1)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_OCTREE_LEVEL_LT_ONE;
	}
	if (octreeLevel > DgmOctree::MAX_OCTREE_LEVEL)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_OCTREE_LEVEL_GT_MAX_OCTREE_LEVEL;
	}
	//compute octrees with the same bounding-box
	DgmOctree *octreeA = compOctree;
	DgmOctree *octreeB = refOctree;
	if (synchronizeOctrees(comparedCloud, referenceCloud, octreeA, octreeB, maxSearchDist, progressCb) != SYNCHRONIZED)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_SYNCHRONIZE_OCTREES_FAILURE;
	}
	const int* minIndexesA = octreeA->getMinFillIndexes(octreeLevel);
	const int* maxIndexesA = octreeA->getMaxFillIndexes(octreeLevel);
	const int* minIndexesB = octreeB->getMinFillIndexes(octreeLevel);
	const int* maxIndexesB = octreeB->getMaxFillIndexes(octreeLevel);

	Tuple3i minIndexes( std::min(minIndexesA[0], minIndexesB[0]),
			std::min(minIndexesA[1], minIndexesB[1]),
			std::min(minIndexesA[2], minIndexesB[2]) );
	Tuple3i maxIndexes( std::max(maxIndexesA[0], maxIndexesB[0]),
			std::max(maxIndexesA[1], maxIndexesB[1]),
			std::max(maxIndexesA[2], maxIndexesB[2]) );

	Tuple3ui boxSize(	static_cast<unsigned>(maxIndexes.x - minIndexes.x + 1),
						static_cast<unsigned>(maxIndexes.y - minIndexes.y + 1),
						static_cast<unsigned>(maxIndexes.z - minIndexes.z + 1) );

	if (!comparedCloud->enableScalarField())
	{
		//not enough memory
		return DISTANCE_COMPUTATION_RESULTS::ERROR_ENABLE_SCALAR_FIELD_FAILURE;
	}
	if (maxSearchDist > 0)
	{
		//if maxSearchDist is defined, we might skip some points
		//so we set a default distance for all of them
		const ScalarType resetValue = static_cast<ScalarType>(maxSearchDist);
		for (unsigned i = 0; i < comparedCloud->size(); ++i)
		{
			comparedCloud->setPointScalarValue(i, resetValue);
		}
	}

	int result = DISTANCE_COMPUTATION_RESULTS::SUCCESS;

	//instantiate the Distance Transform grid
	SaitoSquaredDistanceTransform dtGrid;
	if (dtGrid.initGrid(boxSize))
	{
		//project the (filled) cells of octree B in the DT grid
		{
			DgmOctree::cellCodesContainer theCodes;
			if (!octreeB->getCellCodes(octreeLevel, theCodes, true))
			{
				return DISTANCE_COMPUTATION_RESULTS::ERROR_GET_CELL_CODES_FAILURE;
			}

			while (!theCodes.empty())
			{
				DgmOctree::CellCode theCode = theCodes.back();
				theCodes.pop_back();
				Tuple3i cellPos;
				octreeB->getCellPos(theCode, octreeLevel, cellPos, true);
				cellPos -= minIndexes;
				dtGrid.setValue(cellPos, 1);
			}
		}

		//propagate the Distance Transform over the grid
		if (!dtGrid.propagateDistance(progressCb))
		{
			return DISTANCE_COMPUTATION_RESULTS::ERROR_PROPAGATE_DISTANCE_FAILURE;
		}

		//eventually get the approx. distance for each cell of octree A
		//and assign it to the points inside
		ScalarType cellSize = static_cast<ScalarType>(octreeA->getCellSize(octreeLevel));

		DgmOctree::cellIndexesContainer theIndexes;
		if (!octreeA->getCellIndexes(octreeLevel, theIndexes))
		{
			//not enough memory
			if (!compOctree)
				delete octreeA;
			if (!refOctree)
				delete octreeB;
			return DISTANCE_COMPUTATION_RESULTS::ERROR_GET_CELL_INDEXES_FAILURE;
		}

		ScalarType maxD = 0;
		ReferenceCloud Yk(octreeA->associatedCloud());

		while (!theIndexes.empty())
		{
			unsigned theIndex = theIndexes.back();
			theIndexes.pop_back();

			Tuple3i cellPos;
			octreeA->getCellPos(octreeA->getCellCode(theIndex),octreeLevel,cellPos,false);
			cellPos -= minIndexes;
			unsigned di = dtGrid.getValue(cellPos);
			ScalarType d = sqrt(static_cast<ScalarType>(di)) * cellSize;
			if (d > maxD)
			{
				maxD = d;
			}

			//the maximum distance is 'maxSearchDist' (if defined)
			if (maxSearchDist <= 0 || d < maxSearchDist)
			{
				if (!octreeA->getPointsInCellByCellIndex(&Yk, theIndex, octreeLevel))
				{
					return DISTANCE_COMPUTATION_RESULTS::ERROR_EXECUTE_GET_POINTS_IN_CELL_BY_INDEX_FAILURE;
				}
				for (unsigned j = 0; j < Yk.size(); ++j)
				{
					Yk.setPointScalarValue(j, d);
				}
			}
		}

		result = static_cast<int>(maxD);
	}
	else //DT grid init failed
	{
		result = DISTANCE_COMPUTATION_RESULTS::ERROR_INIT_DISTANCE_TRANSFORM_GRID_FAILURE;
	}

	if (!compOctree)
	{
		delete octreeA;
		octreeA = nullptr;
	}
	if (!refOctree)
	{
		delete octreeB;
		octreeB = nullptr;
	}

	return result;
}

PointCoordinateType DistanceComputationTools::ComputeSquareDistToSegment(const CCVector2& P,
																		 const CCVector2& A,
																		 const CCVector2& B,
																		 bool onlyOrthogonal/*=false*/)
{
	CCVector2 AP = P-A;
	CCVector2 AB = B-A;
	PointCoordinateType dot = AB.dot(AP); // = cos(PAB) * ||AP|| * ||AB||
	if (dot < 0)
	{
		return onlyOrthogonal ? -PC_ONE : AP.norm2();
	}
	else
	{
		PointCoordinateType squareLengthAB = AB.norm2();
		if (dot > squareLengthAB)
		{
			return onlyOrthogonal ? -PC_ONE : (P-B).norm2();
		}
		else
		{
			CCVector2 HP = AP - AB * (dot / squareLengthAB);
			return HP.norm2();
		}
	}
}
