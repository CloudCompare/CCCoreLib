// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include <GeometricalAnalysisTools.h>

//local
#include <CCMath.h>
#include <DgmOctreeReferenceCloud.h>
#include <DistanceComputationTools.h>
#include <GenericProgressCallback.h>
#include <ReferenceCloud.h>
#include <ScalarField.h>
#include <ScalarFieldTools.h>

//system
#include <algorithm>
#include <random>

using namespace CCCoreLib;

//volume of a unit sphere
static double s_UnitSphereVolume = 4.0 * M_PI / 3.0;

GeometricalAnalysisTools::ErrorCode GeometricalAnalysisTools::ComputeCharactersitic(
		GeomCharacteristic c,
		int subOption,
		GenericIndexedCloudPersist* cloud,
		PointCoordinateType kernelRadius,
		const CCVector3* roughnessUpDir/*=nullptr*/,
		GenericProgressCallback* progressCb/*=nullptr*/,
		DgmOctree* inputOctree/*=nullptr*/)
{
	if (!cloud)
	{
		//invalid input cloud
		return InvalidInput;
	}

	unsigned numberOfPoints = cloud->size();

	std::string label;
	switch (c)
	{
		case Feature:
			if (subOption == 0)
				return InvalidInput;
			if (numberOfPoints < 4)
				return NotEnoughPoints;
			label = "Feature computation";
			break;
		case Curvature:
			if (subOption == 0)
				return InvalidInput;
			if (numberOfPoints < 5)
				return NotEnoughPoints;
			label = "Curvature computation";
			break;
		case LocalDensity:
			if (subOption == 0)
				return InvalidInput;
			if (numberOfPoints < 3)
				return NotEnoughPoints;
			label = "Density computation";
			break;
		case ApproxLocalDensity:
			if (subOption == 0)
				return InvalidInput;
			//special case (can't be handled in the same way as the other characteristics)
			return ComputeLocalDensityApprox(cloud, static_cast<Density>(subOption), progressCb, inputOctree);
		case Roughness:
			if (numberOfPoints < 4)
				return NotEnoughPoints;
			label = "Roughness computation";
			break;
		case MomentOrder1:
			if (numberOfPoints < 4)
				return NotEnoughPoints;
			label = "1st order moment computation";
			break;
		default:
			assert(false);
			return UnhandledCharacteristic;
	}

	DgmOctree* octree = inputOctree;
	if (!octree)
	{
		//try to build the octree if none was provided
		octree = new DgmOctree(cloud);
		if (octree->build(progressCb) < 1)
		{
			delete octree;
			return OctreeComputationFailed;
		}
	}

	//enable a scalar field for storing the characteristic values
	cloud->enableScalarField();

	//find the best octree leve to perform the computation
	unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(kernelRadius);

	//parameters
	void* additionalParameters[] =
	{
		static_cast<void*>(&c),
		static_cast<void*>(&subOption),
		static_cast<void*>(&kernelRadius),
		static_cast<void*>(const_cast<CCVector3*>(roughnessUpDir))
	};

	ErrorCode result = NoError;

	if (octree->executeFunctionForAllCellsAtLevel(	level,
													&ComputeGeomCharacteristicAtLevel,
													additionalParameters,
													true,
													progressCb,
													label.c_str()) == 0)
	{
		//something went wrong
		result = ProcessFailed;
	}

	if (octree && !inputOctree)
	{
		delete octree;
		octree = nullptr;
	}

	//we need to finish the work for the Density computation
	if (	result == NoError
			&&	c == LocalDensity
			&&	subOption != DENSITY_KNN //no need to do anything for the 'KNN' mode)
			)
	{
		//compute the right dimensional coef based on the expected output
		ScalarType dimensionalCoef = 1.0f;
		switch (static_cast<Density>(subOption))
		{
			case DENSITY_KNN:
				assert(false);
				dimensionalCoef = 1.0f;
				break;
			case DENSITY_2D:
				dimensionalCoef = static_cast<ScalarType>(M_PI * pow(kernelRadius, 2.0));
				break;
			case DENSITY_3D:
				dimensionalCoef = static_cast<ScalarType>(s_UnitSphereVolume * pow(kernelRadius, 3.0));
				break;
			default:
				assert(false);
				result = InvalidInput;
				break;
		}

		for (unsigned i = 0; i < numberOfPoints; ++i)
		{
			ScalarType s = cloud->getPointScalarValue(i);
			s /= dimensionalCoef;
			cloud->setPointScalarValue(i, s);
		}
	}

	return result;
}

bool GeometricalAnalysisTools::ComputeGeomCharacteristicAtLevel(const DgmOctree::octreeCell& cell,
																void** additionalParameters,
																NormalizedProgress* nProgress/*=nullptr*/)
{
	//parameters
	GeomCharacteristic c            = *static_cast<GeomCharacteristic*>(additionalParameters[0]);
	int subOption                   = *static_cast<Neighbourhood::CurvatureType*>(additionalParameters[1]);
	PointCoordinateType radius      = *static_cast<PointCoordinateType*>(additionalParameters[2]);
	const CCVector3* roughnessUpDir = static_cast<const CCVector3*>(additionalParameters[3]);

	//structure for nearest neighbors search
	DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level = cell.level;
	cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	unsigned n = cell.points->size(); //number of points in the current cell

	//we already know some of the neighbours: the points in the current cell!
	{
		try
		{
			nNSS.pointsInNeighbourhood.resize(n);
		}
		catch (const std::bad_alloc&)
		{
			//out of memory
			return false;
		}

		DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
		for (unsigned i = 0; i < n; ++i, ++it)
		{
			it->point = cell.points->getPointPersistentPtr(i);
			it->pointIndex = cell.points->getPointGlobalIndex(i);
		}
	}
	nNSS.alreadyVisitedNeighbourhoodSize = 1;

	//for each point in the cell
	for (unsigned i = 0; i < n; ++i)
	{
		cell.points->getPoint(i, nNSS.queryPoint);

		//look for neighbors in a sphere
		//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (neighborCount)!
		unsigned neighborCount = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS, radius, false);

		ScalarType value = NAN_VALUE;

		switch (c)
		{
			case Feature:
				if (neighborCount > 3)
				{
					DgmOctreeReferenceCloud neighboursCloud(&nNSS.pointsInNeighbourhood, neighborCount);
					Neighbourhood Z(&neighboursCloud);
					value = static_cast<ScalarType>(Z.computeFeature(static_cast<Neighbourhood::GeomFeature>(subOption)));
				}
				break;

			case Curvature:
				if (neighborCount > 5)
				{
					DgmOctreeReferenceCloud neighboursCloud(&nNSS.pointsInNeighbourhood, neighborCount);
					Neighbourhood Z(&neighboursCloud);
					value = Z.computeCurvature(nNSS.queryPoint, static_cast<Neighbourhood::CurvatureType>(subOption));
				}
				break;

			case LocalDensity:
			{
				value = static_cast<ScalarType>(neighborCount);
			}
				break;

			case Roughness:
				if (neighborCount > 3)
				{
					//find the query point in the nearest neighbors set and place it at the end
					const unsigned globalIndex = cell.points->getPointGlobalIndex(i);
					unsigned localIndex = 0;
					while (localIndex < neighborCount && nNSS.pointsInNeighbourhood[localIndex].pointIndex != globalIndex)
					{
						++localIndex;
					}
					//the query point should be in the nearest neighbors set!
					assert(localIndex < neighborCount);
					if (localIndex + 1 < neighborCount) //no need to swap with another point if it's already at the end!
					{
						std::swap(nNSS.pointsInNeighbourhood[localIndex], nNSS.pointsInNeighbourhood[neighborCount - 1]);
					}

					DgmOctreeReferenceCloud neighboursCloud(&nNSS.pointsInNeighbourhood, neighborCount - 1); //we don't take the query point into account!
					Neighbourhood Z(&neighboursCloud);
					value = Z.computeRoughness(nNSS.queryPoint, roughnessUpDir);

					//swap the points back to their original position (DGM: not necessary in this case)
					//if (localIndex+1 < neighborCount)
					//{
					//	std::swap(nNSS.pointsInNeighbourhood[localIndex],nNSS.pointsInNeighbourhood[neighborCount-1]);
					//}
				}
				break;

			case MomentOrder1:
			{
				DgmOctreeReferenceCloud neighboursCloud(&nNSS.pointsInNeighbourhood, neighborCount);
				Neighbourhood Z(&neighboursCloud);
				value = Z.computeMomentOrder1(nNSS.queryPoint);
			}
				break;

			default:
				assert(false);
				return false;
		}

		cell.points->setPointScalarValue(i, value);

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	return true;
}

GeometricalAnalysisTools::ErrorCode GeometricalAnalysisTools::FlagDuplicatePoints(	GenericIndexedCloudPersist* cloud,
																					double minDistanceBetweenPoints/*=1.0e-12*/,
																					GenericProgressCallback* progressCb/*=nullptr*/,
																					DgmOctree* inputOctree/*=nullptr*/)
{
	if (!cloud)
		return InvalidInput;

	unsigned numberOfPoints = cloud->size();
	if (numberOfPoints <= 1)
		return NotEnoughPoints;

	DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(cloud);
		if (theOctree->build(progressCb) < 1)
		{
			delete theOctree;
			return OctreeComputationFailed;
		}
	}

	if (false == cloud->enableScalarField())
	{
		return NotEnoughMemory;
	}
	
	//set all flags to 0 by default
	cloud->forEach(ScalarFieldTools::SetScalarValueToZero);

	unsigned char level = theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(static_cast<PointCoordinateType>(minDistanceBetweenPoints));

	//parameters
	void* additionalParameters[1] = { static_cast<void*>(&minDistanceBetweenPoints) };

	ErrorCode result = NoError;

	if (theOctree->executeFunctionForAllCellsAtLevel(	level,
														&FlagDuplicatePointsInACellAtLevel,
														additionalParameters,
														false, //doesn't work in parallel!
														progressCb,
														"Flag duplicate points") == 0)
	{
		//something went wrong
		result = ProcessFailed;
	}

	if (!inputOctree)
	{
		delete theOctree;
		theOctree = nullptr;
	}

	return result;
}

//"PER-CELL" METHOD: FLAG DUPLICATE POINTS
//ADDITIONAL PARAMETERS (1):
// [0] -> (double*) maxSquareDistBetweenPoints: max square distance between points
bool GeometricalAnalysisTools::FlagDuplicatePointsInACellAtLevel(	const DgmOctree::octreeCell& cell,
																	void** additionalParameters,
																	NormalizedProgress* nProgress/*=nullptr*/)
{
	//parameter(s)
	double minDistBetweenPoints = *static_cast<double*>(additionalParameters[0]);

	//structure for nearest neighbors search
	DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level = cell.level;
	cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	unsigned n = cell.points->size(); //number of points in the current cell

	//for each point in the cell
	for (unsigned i = 0; i < n; ++i)
	{
		//don't process points already flagged as 'duplicate'
		if (cell.points->getPointScalarValue(i) == 0)
		{
			cell.points->getPoint(i, nNSS.queryPoint);

			//look for neighbors in a sphere
			//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (neighborCount)!
			unsigned neighborCount = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS, minDistBetweenPoints, false);
			if (neighborCount > 1) //the point itself lies in the neighborhood
			{
				unsigned iIndex = cell.points->getPointGlobalIndex(i);
				for (unsigned j = 0; j < neighborCount; ++j)
				{
					if (nNSS.pointsInNeighbourhood[j].pointIndex != iIndex)
					{
						//flag this point as 'duplicate'
						cell.points->getAssociatedCloud()->setPointScalarValue(nNSS.pointsInNeighbourhood[j].pointIndex, static_cast<ScalarType>(1));
					}
				}
			}
		}

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	return true;
}

GeometricalAnalysisTools::ErrorCode GeometricalAnalysisTools::ComputeLocalDensityApprox(
		GenericIndexedCloudPersist* cloud,
		Density densityType,
		GenericProgressCallback* progressCb/*=nullptr*/,
		DgmOctree* inputOctree/*=nullptr*/)
{
	if (!cloud)
		return InvalidInput;

	unsigned numberOfPoints = cloud->size();
	if (numberOfPoints < 3)
		return NotEnoughPoints;

	DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(cloud);
		if (theOctree->build(progressCb) < 1)
		{
			delete theOctree;
			return OctreeComputationFailed;
		}
	}

	cloud->enableScalarField();

	//determine best octree level to perform the computation
	unsigned char level = theOctree->findBestLevelForAGivenPopulationPerCell(3);

	//parameters
	void* additionalParameters[] = { static_cast<void*>(&densityType) };

	ErrorCode result = NoError;

	if (theOctree->executeFunctionForAllCellsAtLevel(	level,
														&ComputeApproxPointsDensityInACellAtLevel,
														additionalParameters,
														true,
														progressCb,
														"Approximate Local Density Computation") == 0)
	{
		//something went wrong
		result = ProcessFailed;
	}

	if (!inputOctree)
	{
		delete theOctree;
		theOctree = nullptr;
	}

	return result;
}

//"PER-CELL" METHOD: APPROXIMATE LOCAL DENSITY
//ADDITIONAL PARAMETERS (0): NONE
bool GeometricalAnalysisTools::ComputeApproxPointsDensityInACellAtLevel(const DgmOctree::octreeCell& cell,
																		void** additionalParameters,
																		NormalizedProgress* nProgress/*=nullptr*/)
{
	//extract additional parameter(s)
	Density densityType = *static_cast<Density*>(additionalParameters[0]);

	DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level								= cell.level;
	nNSS.alreadyVisitedNeighbourhoodSize	= 0;
	nNSS.minNumberOfNeighbors				= 2;
	cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	unsigned n = cell.points->size();
	for (unsigned i = 0; i < n; ++i)
	{
		cell.points->getPoint(i, nNSS.queryPoint);

		//the first point is always the point itself!
		if (cell.parentOctree->findNearestNeighborsStartingFromCell(nNSS) > 1)
		{
			double R2 = nNSS.pointsInNeighbourhood[1].squareDistd;

			ScalarType density = NAN_VALUE;
			if ( GreaterThanSquareEpsilon( R2 ) )
			{
				switch (densityType)
				{
					case DENSITY_KNN:
					{
						//we return in fact the (inverse) distance to the nearest neighbor
						density = static_cast<ScalarType>(1.0 / sqrt(R2));
					}
						break;
					case DENSITY_2D:
					{
						//circle area (2D approximation)
						double circleArea = M_PI * R2;
						density = static_cast<ScalarType>(1.0 / circleArea);
					}
						break;
					case DENSITY_3D:
					{
						double sphereVolume = s_UnitSphereVolume * R2 * sqrt(R2);
						density = static_cast<ScalarType>(1.0 / sphereVolume);
					}
						break;
					default:
						assert(false);
						break;
				}
			}
			cell.points->setPointScalarValue(i, density);
		}
		else
		{
			//shouldn't happen! Apart if the cloud has only one point...
			cell.points->setPointScalarValue(i, NAN_VALUE);
		}

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	return true;
}

CCVector3 GeometricalAnalysisTools::ComputeGravityCenter(GenericCloud* cloud)
{
	assert(cloud);

	unsigned count = cloud->size();
	if (count == 0)
		return CCVector3();

	CCVector3d sum(0, 0, 0);

	cloud->placeIteratorAtBeginning();
	const CCVector3* P = cloud->getNextPoint();
	while (P)
	{
		sum += *P;
		P = cloud->getNextPoint();
	}

	sum /= static_cast<double>(count);
	return sum.toPC();
}

CCVector3 GeometricalAnalysisTools::ComputeWeightedGravityCenter(GenericCloud* cloud, ScalarField* weights)
{
	assert(cloud && weights);

	unsigned count = cloud->size();
	if (count == 0 || !weights || weights->currentSize() < count)
		return CCVector3();

	CCVector3d sum(0, 0, 0);

	cloud->placeIteratorAtBeginning();
	double wSum = 0;
	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3* P = cloud->getNextPoint();
		ScalarType w = weights->getValue(i);
		if (!ScalarField::ValidValue(w))
			continue;
		w = std::abs(w);
		sum += P->toDouble() * w;
		wSum += w;
	}

	if (wSum != 0)
		sum /= wSum;

	return sum.toPC();
}


SquareMatrixd GeometricalAnalysisTools::ComputeCovarianceMatrix(GenericCloud* cloud, const PointCoordinateType* _gravityCenter)
{
	assert(cloud);
	unsigned n = (cloud ? cloud->size() : 0);
	if (n==0)
		return SquareMatrixd();

	SquareMatrixd covMat(3);
	covMat.clear();

	//gravity center
	CCVector3 G = (_gravityCenter ?  CCVector3(_gravityCenter) : ComputeGravityCenter(cloud));

	//cross sums (we use doubles to avoid overflow)
	double mXX = 0;
	double mYY = 0;
	double mZZ = 0;
	double mXY = 0;
	double mXZ = 0;
	double mYZ = 0;

	cloud->placeIteratorAtBeginning();
	for (unsigned i = 0; i < n; ++i)
	{
		const CCVector3* Q = cloud->getNextPoint();

		CCVector3 P = *Q - G;
		mXX += static_cast<double>(P.x*P.x);
		mYY += static_cast<double>(P.y*P.y);
		mZZ += static_cast<double>(P.z*P.z);
		mXY += static_cast<double>(P.x*P.y);
		mXZ += static_cast<double>(P.x*P.z);
		mYZ += static_cast<double>(P.y*P.z);
	}

	covMat.m_values[0][0] = mXX / static_cast<double>(n);
	covMat.m_values[0][0] = mYY / static_cast<double>(n);
	covMat.m_values[0][0] = mZZ / static_cast<double>(n);
	covMat.m_values[1][0] = covMat.m_values[0][1] = mXY / static_cast<double>(n);
	covMat.m_values[2][0] = covMat.m_values[0][2] = mXZ / static_cast<double>(n);
	covMat.m_values[2][1] = covMat.m_values[1][2] = mYZ / static_cast<double>(n);

	return covMat;
}

SquareMatrixd GeometricalAnalysisTools::ComputeCrossCovarianceMatrix(GenericCloud* P,
																			GenericCloud* Q,
																			const CCVector3& Gp,
																			const CCVector3& Gq)
{
	assert(P && Q);
	assert(Q->size() == P->size());

	//shortcuts to output matrix lines
	SquareMatrixd covMat(3);
	double* l1 = covMat.row(0);
	double* l2 = covMat.row(1);
	double* l3 = covMat.row(2);

	P->placeIteratorAtBeginning();
	Q->placeIteratorAtBeginning();

	//sums
	unsigned count = P->size();
	for (unsigned i = 0; i < count; i++)
	{
		CCVector3 Pt = *P->getNextPoint() - Gp;
		CCVector3 Qt = *Q->getNextPoint() - Gq;

		l1[0] += static_cast<double>(Pt.x) * Qt.x;
		l1[1] += static_cast<double>(Pt.x) * Qt.y;
		l1[2] += static_cast<double>(Pt.x) * Qt.z;
		l2[0] += static_cast<double>(Pt.y) * Qt.x;
		l2[1] += static_cast<double>(Pt.y) * Qt.y;
		l2[2] += static_cast<double>(Pt.y) * Qt.z;
		l3[0] += static_cast<double>(Pt.z) * Qt.x;
		l3[1] += static_cast<double>(Pt.z) * Qt.y;
		l3[2] += static_cast<double>(Pt.z) * Qt.z;
	}

	covMat.scale(1.0 / count);

	return covMat;
}

SquareMatrixd GeometricalAnalysisTools::ComputeWeightedCrossCovarianceMatrix(	GenericCloud* P, //data
																				GenericCloud* Q, //model
																				const CCVector3& Gp,
																				const CCVector3& Gq,
																				ScalarField* coupleWeights/*=nullptr*/)
{
	assert(P && Q);
	assert(Q->size() == P->size());
	assert(coupleWeights);
	assert(coupleWeights->currentSize() == P->size());

	//shortcuts to output matrix lines
	SquareMatrixd covMat(3);
	double* r1 = covMat.row(0);
	double* r2 = covMat.row(1);
	double* r3 = covMat.row(2);

	P->placeIteratorAtBeginning();
	Q->placeIteratorAtBeginning();

	//sums
	unsigned count = P->size();
	double wSum = 0.0; //we will normalize by the sum
	for (unsigned i = 0; i<count; i++)
	{
		const CCVector3* Pt = P->getNextPoint();
		const CCVector3* Qt = Q->getNextPoint();

		//Weighting scheme for cross-covariance is inspired from
		//https://en.wikipedia.org/wiki/Weighted_arithmetic_mean#Weighted_sample_covariance
		double wi = 1.0;
		if (coupleWeights)
		{
			ScalarType w = coupleWeights->getValue(i);
			if (!ScalarField::ValidValue(w))
				continue;
			wi = std::abs(w);
		}

		//DGM: we virtually make the P (data) point nearer if it has a lower weight
		CCVector3d Ptw(Pt->x * wi, Pt->y * wi, Pt->z * wi);
		wSum += wi;

		//1st row
		r1[0] += Ptw.x * Qt->x;
		r1[1] += Ptw.x * Qt->y;
		r1[2] += Ptw.x * Qt->z;
		//2nd row
		r2[0] += Ptw.y * Qt->x;
		r2[1] += Ptw.y * Qt->y;
		r2[2] += Ptw.y * Qt->z;
		//3rd row
		r3[0] += Ptw.z * Qt->x;
		r3[1] += Ptw.z * Qt->y;
		r3[2] += Ptw.z * Qt->z;
	}

	if (wSum != 0.0)
	{
		covMat.scale(1.0 / wSum);
	}

	//remove the centers of gravity
	r1[0] -= static_cast<double>(Gp.x) * Gq.x;
	r1[1] -= static_cast<double>(Gp.x) * Gq.y;
	r1[2] -= static_cast<double>(Gp.x) * Gq.z;
	//2nd row
	r2[0] -= static_cast<double>(Gp.y) * Gq.x;
	r2[1] -= static_cast<double>(Gp.y) * Gq.y;
	r2[2] -= static_cast<double>(Gp.y) * Gq.z;
	//3rd row
	r3[0] -= static_cast<double>(Gp.z) * Gq.x;
	r3[1] -= static_cast<double>(Gp.z) * Gq.y;
	r3[2] -= static_cast<double>(Gp.z) * Gq.z;

	return covMat;
}

bool GeometricalAnalysisTools::RefineSphereLS(	GenericIndexedCloudPersist* cloud,
												CCVector3& center,
												PointCoordinateType& radius,
												double minRelativeCenterShift/*=1.0e-3*/)
{
	if (!cloud || cloud->size() < 5)
	{
		//invalid input
		return false;
	}

	CCVector3d c = center;

	unsigned count = cloud->size();

	//compute barycenter
	CCVector3d G(0, 0, 0);
	{
		for (unsigned i = 0; i < count; ++i)
		{
			const CCVector3* P = cloud->getPoint(i);
			G += *P;
		}
		G /= count;
	}

	static const unsigned MAX_ITERATIONS = 100;
	for (unsigned it = 0; it < MAX_ITERATIONS; ++it)
	{
		// Compute average L, dL/da, dL/db, dL/dc.
		double meanNorm = 0.0;
		CCVector3d derivatives(0, 0, 0);
		unsigned realCount = 0;
		for (unsigned i = 0; i < count; ++i)
		{
			const CCVector3* Pi = cloud->getPoint(i);
			CCVector3d Di = Pi->toDouble() - c;
			double norm = Di.norm();
			if (LessThanEpsilon(norm))
			{
				continue;
			}

			meanNorm += norm;
			derivatives += Di / norm;
			++realCount;
		}

		meanNorm /= count;
		derivatives /= count;

		//backup previous center
		CCVector3d c0 = c;
		//deduce new center
		c = G - derivatives * meanNorm;
		radius = static_cast<PointCoordinateType>(meanNorm);

		double shift = (c-c0).norm();
		double relativeShift = shift / radius;
		if (relativeShift < minRelativeCenterShift)
			break;
	}

	return true;
}

GeometricalAnalysisTools::ErrorCode GeometricalAnalysisTools::DetectSphereRobust(
		GenericIndexedCloudPersist* cloud,
		double outliersRatio,
		CCVector3& center,
		PointCoordinateType& radius,
		double& rms,
		GenericProgressCallback* progressCb/*=nullptr*/,
		double confidence/*=0.99*/,
		unsigned seed/*=0*/)
{
	if (!cloud)
	{
		assert(false);
		return InvalidInput;
	}

	unsigned n = cloud->size();
	if (n < 4)
		return NotEnoughPoints;

	assert(confidence < 1.0);
	confidence = std::min(confidence, 1.0 - ZERO_TOLERANCE_D);

	//we'll need an array (sorted) to compute the medians
	std::vector<PointCoordinateType> values;
	try
	{
		values.resize(n);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return NotEnoughMemory;
	}

	//number of samples
	unsigned m = 1;
	const unsigned p = 4;
	if (n > p)
	{
		m = static_cast<unsigned>(log(1.0 - confidence) / log(1.0 - pow(1.0 - outliersRatio, static_cast<double>(p))));
	}

	//for progress notification
	NormalizedProgress nProgress(progressCb, m);
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			char buffer[64];
			snprintf(buffer, 64, "Least Median of Squares samples: %u", m);
			progressCb->setInfo(buffer);
			progressCb->setMethodTitle("Detect sphere");
		}
		progressCb->update(0);
		progressCb->start();
	}

	//now we are going to randomly extract a subset of 4 points and test the resulting sphere each time
	if (seed == 0)
	{
		std::random_device randomGenerator;   // non-deterministic generator
		seed = randomGenerator();
	}
	std::mt19937 gen(seed);  // to seed mersenne twister.
	std::uniform_int_distribution<unsigned> dist(0, n - 1);
	unsigned sampleCount = 0;
	unsigned attempts = 0;
	double minError = -1.0;
	std::vector<unsigned> indexes;
	indexes.resize(p);
	while (sampleCount < m && attempts < 2*m)
	{
		//get 4 random (different) indexes
		for (unsigned j = 0; j < p; ++j)
		{
			bool isOK = false;
			while (!isOK)
			{
				indexes[j] = dist(gen);
				isOK = true;
				for (unsigned k = 0; k < j && isOK; ++k)
					if (indexes[j] == indexes[k])
						isOK = false;
			}
		}

		assert(p == 4);
		const CCVector3* A = cloud->getPoint(indexes[0]);
		const CCVector3* B = cloud->getPoint(indexes[1]);
		const CCVector3* C = cloud->getPoint(indexes[2]);
		const CCVector3* D = cloud->getPoint(indexes[3]);

		++attempts;
		CCVector3 thisCenter;
		PointCoordinateType thisRadius;
		if (ComputeSphereFrom4(*A, *B, *C, *D, thisCenter, thisRadius) != NoError)
			continue;

		//compute residuals
		for (unsigned i = 0; i < n; ++i)
		{
			PointCoordinateType error = (*cloud->getPoint(i) - thisCenter).norm() - thisRadius;
			values[i] = error*error;
		}

		const unsigned int	medianIndex = n / 2;

		std::nth_element(values.begin(), values.begin() + medianIndex, values.end());

		//the error is the median of the squared residuals
		double error = static_cast<double>(values[medianIndex]);

		//we keep track of the solution with the least error
		if (error < minError || minError < 0.0)
		{
			minError = error;
			center = thisCenter;
			radius = thisRadius;
		}

		++sampleCount;

		if (progressCb && !nProgress.oneStep())
		{
			//progress canceled by the user
			return ProcessCancelledByUser;
		}
	}

	//too many failures?!
	if (sampleCount < m)
	{
		return ProcessFailed;
	}

	//last step: robust estimation
	ReferenceCloud candidates(cloud);
	if (n > p)
	{
		//e robust standard deviation estimate (see Zhang's report)
		double sigma = 1.4826 * (1.0 + 5.0 /(n-p)) * sqrt(minError);

		//compute the least-squares best-fitting sphere with the points
		//having residuals below 2.5 sigma
		double maxResidual = 2.5 * sigma;
		if (candidates.reserve(n))
		{
			//compute residuals and select the points
			for (unsigned i = 0; i < n; ++i)
			{
				PointCoordinateType error = (*cloud->getPoint(i) - center).norm() - radius;
				if (error < maxResidual)
					candidates.addPointIndex(i);
			}
			candidates.resize(candidates.size());

			//eventually estimate the robust sphere parameters with least squares (iterative)
			if (RefineSphereLS(&candidates, center, radius))
			{
				//replace input cloud by this subset!
				cloud = &candidates;
				n = cloud->size();
			}
		}
		else
		{
			//not enough memory!
			//we'll keep the rough estimate...
		}
	}

	//update residuals
	{
		double residuals = 0;
		for (unsigned i = 0; i < n; ++i)
		{
			const CCVector3* P = cloud->getPoint(i);
			double e = (*P - center).norm() - radius;
			residuals += e*e;
		}
		rms = sqrt(residuals/n);
	}

	return NoError;
}

static bool Landau_Smith(const std::vector<CCVector2d>& xy, CCVector2d& center, PointCoordinateType& radius)
{
	size_t N = xy.size();
	if (N < 3)
	{
		assert(false);
		return false;
	}

	double p1 = 0.0, p2 = 0.0, p3 = 0.0, p4 = 0.0, p5 = 0.0, p6 = 0.0, p7 = 0.0, p8 = 0.0, p9 = 0.0;

	for (size_t i = 0; i < N; ++i)
	{
		p1 += xy[i].x;
		p2 += xy[i].x * xy[i].x;
		p3 += xy[i].x * xy[i].y;
		p4 += xy[i].y;
		p5 += xy[i].y * xy[i].y;
		p6 += xy[i].x * xy[i].x * xy[i].x;
		p7 += xy[i].x * xy[i].y * xy[i].y;
		p8 += xy[i].y * xy[i].y * xy[i].y;
		p9 += xy[i].x * xy[i].x * xy[i].y;
	}

	double a1 = 2 * (p1 * p1 - N * p2);
	double b1 = 2 * (p1 * p4 - N * p3);
	double a2 = b1;
	double b2 = 2 * (p4 * p4 - N * p5);
	double c1 = p2 * p1 - N * p6 + p1 * p5 - N * p7;
	double c2 = p2 * p4 - N * p8 + p4 * p5 - N * p9;

	center.x = (c1 * b2 - c2 * b1) / (a1 * b2 - a2 * b1); // center along x
	center.y = (a1 * c2 - a2 * c1) / (a1 * b2 - a2 * b1); // center along y
	radius = static_cast<PointCoordinateType>(sqrt(((p2 + p5) - (2 * p1 * center.x) + (N * center.x * center.x) - (2 * p4 * center.y) + (N * center.y * center.y)) / N)); // circle radius

	return true;
}

GeometricalAnalysisTools::ErrorCode GeometricalAnalysisTools::DetectCircle(	GenericIndexedCloudPersist* cloud,
																			CCVector3& center,
																			CCVector3& normal,
																			PointCoordinateType& radius,
																			double& rms,
																			GenericProgressCallback* progressCb/*=nullptr*/)
{
	center = CCVector3(0, 0, 0);
	normal = CCVector3(0, 0, PC_ONE);
	radius = std::numeric_limits<PointCoordinateType>::quiet_NaN();
	rms = std::numeric_limits<double>::quiet_NaN();

	if (!cloud)
	{
		assert(false);
		return InvalidInput;
	}

	unsigned n = cloud->size();
	if (n < 4)
	{
		return NotEnoughPoints;
	}

	//for progress notification
	NormalizedProgress nProgress(progressCb, 7);
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setMethodTitle("Detect circle using Landau Smith algorithm");
		}
		progressCb->update(0);
		progressCb->start();
	}

	//step 1: fit a plane on the cloud to retrieve the eigenvalues and vectors of the covariance matrix
	Neighbourhood Yk(cloud);
	if (!Yk.getLSPlane())
	{
		return ProcessFailed;
	}

	if (progressCb && !nProgress.oneStep())
	{
		//progress canceled by the user
		return ProcessCancelledByUser;
	}
	//step 2: try to fit a circle with each eigenvector and keep the best result
	const CCVector3* eigenvectors[2]
	{
		Yk.getLSPlaneX(), // eigenvector associated to the max eigenvalue (= most elongated dimension)
		Yk.getLSPlaneNormal() // eigenvector associated to the min eigenvalue (= most flat dimension)
	};

	std::vector<CCVector2d> pointsOnPlane;
	try
	{
		pointsOnPlane.resize(cloud->size());
	}
	catch (const std::bad_alloc&)
	{
		return NotEnoughMemory;
	}

	// compute the cloud (gravity) center
	const CCVector3* G = Yk.getGravityCenter();
	assert(G);

#ifdef DEBUG_TRACE
	FILE* fp = fopen("C:\\Temp\\circle_fit.txt", "wt");
#endif

	for (unsigned dim = 0; dim < 2; ++dim)
	{
		const CCVector3* eigenVector = eigenvectors[dim];
		assert(eigenVector);

		// compute a local coordinate system
		CCVector3 x = eigenVector->orthogonal();
		CCVector3 y = eigenVector->cross(x);
#ifdef DEBUG_TRACE
		{
			fprintf(fp, "Dim %i\n", dim);
			fprintf(fp, "X = %f %f %f\n", x.x, x.y, x.z);
			fprintf(fp, "Y = %f %f %f\n", y.x, y.y, y.z);
			fprintf(fp, "Z = %f %f %f\n", eigenVector->x, eigenVector->y, eigenVector->z);
		}
#endif

		//step 3: project the point cloud onto a 2D plane
		for (unsigned i = 0; i < n; ++i)
		{
			CCVector3 Plocal = *cloud->getPoint(i) - *G;
			pointsOnPlane[i] = { Plocal.dot(x), Plocal.dot(y) };
		}

		if (progressCb && !nProgress.oneStep())
		{
			//progress canceled by the user
			return ProcessCancelledByUser;
		}

#ifdef DEBUG_TRACE
		{
			FILE* fpc = nullptr;
			switch (dim)
			{
			case 0:
				fpc = fopen("C:\\Temp\\circle_dim0.asc", "wt");
				break;
			case 1:
				fpc = fopen("C:\\Temp\\circle_dim1.asc", "wt");
				break;
			case 2:
				fpc = fopen("C:\\Temp\\circle_dim2.asc", "wt");
				break;
			}

			for (const CCVector2d& P2D : pointsOnPlane)
			{
				fprintf(fpc, "%f %f 0\n", P2D.x, P2D.y);
			}
			fclose(fpc);
		}
#endif

		//step 4: calculate the circle center and radius on the 2D plane using the Landau Smith algorithm
		CCVector2d thisCenter2D;
		PointCoordinateType thisRadius = 0;
		if (!Landau_Smith(pointsOnPlane, thisCenter2D, thisRadius))
		{
			assert(false);
			return ProcessFailed;
		}

#ifdef DEBUG_TRACE
		fprintf(fp, "Center (%f, %f) - radius = %f\n", thisCenter2D.x, thisCenter2D.y, thisRadius);
#endif

		// estimate the RMS
		{
			double thisRMS = 0.0;
			for (const CCVector2d& P2D : pointsOnPlane)
			{
				double r = (P2D - thisCenter2D).norm();
				double error = thisRadius - r;
				thisRMS += error * error;
			}

			thisRMS = sqrt(thisRMS / n);

#ifdef DEBUG_TRACE
			fprintf(fp, "RMS = %f\n", thisRMS);
			fprintf(fp, "=================\n");
#endif

			if (dim == 0 || thisRMS < rms)
			{
				// reposition the circle center in 3D
				center = *G + static_cast<PointCoordinateType>(thisCenter2D.x) * x + static_cast<PointCoordinateType>(thisCenter2D.y) * y;
				normal = CCVector3::fromArray(eigenVector->u);
				radius = thisRadius;
				rms = thisRMS;
			}
		}

		if (progressCb && !nProgress.oneStep())
		{
			//progress canceled by the user
			return ProcessCancelledByUser;
		}
	}

#ifdef DEBUG_TRACE
	fclose(fp);
#endif

	return NoError;
}

//******************************************************************************
//
//  Purpose:
//
//    DMAT_SOLVE uses Gauss-Jordan elimination to solve an N by N linear system.
//
//  Discussion:
//
//    The doubly dimensioned array A is treated as a one dimensional vector,
//    stored by COLUMNS.  Entry A(I,J) is stored as A[I+J*N]
//
//  Modified:
//
//    29 August 2003
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int N, the order of the matrix.
//
//    Input, int RHS_NUM, the number of right hand sides.  RHS_NUM
//    must be at least 0.
//
//    Input/output, double A[N*(N+RHS_NUM)], contains in rows and columns 1
//    to N the coefficient matrix, and in columns N+1 through
//    N+RHS_NUM, the right hand sides.  On output, the coefficient matrix
//    area has been destroyed, while the right hand sides have
//    been overwritten with the corresponding solutions.
//
//    Output, int DMAT_SOLVE, singularity flag.
//    0, the matrix was not singular, the solutions were computed;
//    J, factorization failed on step J, and the solutions could not
//    be computed.
//
static int DMAT_SOLVE(int n, int rhs_num, double a[])
{
	for (int j = 0; j < n; j++)
	{
		//  Choose a pivot row.
		int ipivot = j;
		double apivot = a[j + j * n];

		for (int i = j; i < n; i++)
		{
			if (std::abs(apivot) < std::abs(a[i + j * n]))
			{
				apivot = a[i + j * n];
				ipivot = i;
			}
		}

		if (apivot == 0.0)
		{
			return j;
		}

		//  Interchange.
		for (int i = 0; i < n + rhs_num; i++)
		{
			std::swap(a[ipivot + i * n], a[j + i * n]);
		}

		//  A(J,J) becomes 1.
		a[j + j * n] = 1.0;
		for (int k = j; k < n + rhs_num; k++)
		{
			a[j + k * n] = a[j + k * n] / apivot;
		}

		//  A(I,J) becomes 0.
		for (int i = 0; i < n; i++)
		{
			if (i != j)
			{
				double factor = a[i + j * n];
				a[i + j * n] = 0.0;
				for (int k = j; k < n + rhs_num; k++)
				{
					a[i + k * n] = a[i + k * n] - factor * a[j + k * n];
				}
			}
		}
	}

	return 0;
}

GeometricalAnalysisTools::ErrorCode GeometricalAnalysisTools::ComputeSphereFrom4(	const CCVector3& A,
																					const CCVector3& B,
																					const CCVector3& C,
																					const CCVector3& D,
																					CCVector3& center,
																					PointCoordinateType& radius)
{
	//inspired from 'tetrahedron_circumsphere_3d' by Adrian Bowyer and John Woodwark

	//Set up the linear system.
	double a[12];
	{
		CCVector3 AB = B - A;
		a[0] = AB.x;
		a[3] = AB.y;
		a[6] = AB.z;
		a[9] = AB.norm2d();
	}
	{
		CCVector3 AC = C - A;
		a[1]  = AC.x;
		a[4]  = AC.y;
		a[7]  = AC.z;
		a[10] = AC.norm2d();
	}
	{
		CCVector3 AD = D - A;
		a[2]  = AD.x;
		a[5]  = AD.y;
		a[8]  = AD.z;
		a[11] = AD.norm2d();
	}

	//  Solve the linear system (with Gauss-Jordan elimination)
	if (DMAT_SOLVE(3, 1, a) != 0)
	{
		//system is singular?
		return ProcessFailed;
	}

	//  Compute the radius and center.
	CCVector3 u = CCVector3(static_cast<PointCoordinateType>(a[0 + 3 * 3]),
							static_cast<PointCoordinateType>(a[1 + 3 * 3]),
							static_cast<PointCoordinateType>(a[2 + 3 * 3])) / 2;
	radius = u.norm();
	center = A + u;

	return NoError;
}
