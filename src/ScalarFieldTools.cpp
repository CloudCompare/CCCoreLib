// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include <ScalarFieldTools.h>

//local
#include <CCMath.h>
#include <GenericProgressCallback.h>
#include <ReferenceCloud.h>
#include <ScalarField.h>

//system
#include <cstdio>

using namespace CCCoreLib;

static const int AVERAGE_NUMBER_OF_POINTS_FOR_GRADIENT_COMPUTATION = 14;

void ScalarFieldTools::SetScalarValueToNaN(const CCVector3& P, ScalarType& scalarValue)
{
	scalarValue = NAN_VALUE;
}

void ScalarFieldTools::SetScalarValueToZero(const CCVector3& P, ScalarType& scalarValue)
{
	scalarValue = 0;
}

void ScalarFieldTools::SetScalarValueInverted(const CCVector3& P, ScalarType& scalarValue)
{
	scalarValue = -scalarValue;
}

int ScalarFieldTools::computeScalarFieldGradient(	GenericIndexedCloudPersist* theCloud,
													PointCoordinateType radius,
													bool euclideanDistances,
													bool sameInAndOutScalarField/*=false*/,
													GenericProgressCallback* progressCb/*=nullptr*/,
													DgmOctree* theCloudOctree/*=nullptr*/)
{
	if (!theCloud)
	{
		return -1;
	}

	DgmOctree* theOctree = theCloudOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb) < 1)
		{
			delete theOctree;
			return -2;
		}
	}

	unsigned char octreeLevel = 0;
	if (radius <= 0)
	{
		octreeLevel = theOctree->findBestLevelForAGivenPopulationPerCell(AVERAGE_NUMBER_OF_POINTS_FOR_GRADIENT_COMPUTATION);
		radius = theOctree->getCellSize(octreeLevel);
	}
	else
	{
		octreeLevel = theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(radius);
	}

	ScalarField* theGradientNorms = new ScalarField("gradient norms");
	ScalarField* _theGradientNorms = nullptr;

	//if the IN and OUT scalar fields are the same
	if (sameInAndOutScalarField)
	{
		if (!theGradientNorms->reserveSafe(theCloud->size())) //not enough memory
		{
			if (!theCloudOctree)
				delete theOctree;
			theGradientNorms->release();
			return -3;
		}
		_theGradientNorms = theGradientNorms;
	}
	else //different IN and OUT scalar fields (default)
	{
		//we init the OUT scalar field
		if (!theCloud->enableScalarField())
		{
			if (!theCloudOctree)
				delete theOctree;
			theGradientNorms->release();
			return -4;
		}
	}

	//additionnal parameters
	void* additionalParameters[3] = {	static_cast<void*>(&euclideanDistances),
										static_cast<void*>(&radius),
										static_cast<void*>(_theGradientNorms)
									};

	int result = 0;

	if (theOctree->executeFunctionForAllCellsAtLevel(	octreeLevel,
														computeMeanGradientOnPatch,
														additionalParameters,
														true,
														progressCb,
														"Gradient Computation") == 0)
	{
		//something went wrong
		result = -5;
	}

	if (!theCloudOctree)
	{
		delete theOctree;
	}
	theGradientNorms->release();

	return result;
}

bool ScalarFieldTools::computeMeanGradientOnPatch(	const DgmOctree::octreeCell& cell,
													void** additionalParameters,
													NormalizedProgress* nProgress/*=nullptr*/)
{
	//additional parameters
	bool euclideanDistances			= *reinterpret_cast<bool*>(additionalParameters[0]);
	PointCoordinateType radius		= *reinterpret_cast<PointCoordinateType*>(additionalParameters[1]);
	ScalarField* theGradientNorms	= reinterpret_cast<ScalarField*>(additionalParameters[2]);

	//number of points inside the current cell
	unsigned n = cell.points->size();

	//spherical neighborhood extraction structure
	DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level = cell.level;
	cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	//we already know the points inside the current cell
	{
		try
		{
			nNSS.pointsInNeighbourhood.resize(n);
		}
		catch (.../*const std::bad_alloc&*/) //out of memory
		{
			return false;
		}
		DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
		for (unsigned j = 0; j < n; ++j, ++it)
		{
			it->point = cell.points->getPointPersistentPtr(j);
			it->pointIndex = cell.points->getPointGlobalIndex(j);
		}
		nNSS.alreadyVisitedNeighbourhoodSize = 1;
	}

	const GenericIndexedCloudPersist* cloud = cell.points->getAssociatedCloud();

	for (unsigned i = 0; i < n; ++i)
	{
		ScalarType gN = NAN_VALUE;
		ScalarType v1 = cell.points->getPointScalarValue(i);

		if (ScalarField::ValidValue(v1))
		{
			cell.points->getPoint(i, nNSS.queryPoint);

			//we extract the point's neighbors
			//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (k)!
			unsigned k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS, radius, true);

			//if more than one neighbour (the query point itself)
			if (k > 1)
			{
				CCVector3d sum(0, 0, 0);
				unsigned counter = 0;

				//j = 1 because the first point is the query point itself --> contribution = 0
				for (unsigned j = 1; j < k; ++j)
				{
					ScalarType v2 = cloud->getPointScalarValue(nNSS.pointsInNeighbourhood[j].pointIndex);
					if (ScalarField::ValidValue(v2))
					{
						CCVector3 deltaPos = *nNSS.pointsInNeighbourhood[j].point - nNSS.queryPoint;
						double norm2 = deltaPos.norm2d();

						if ( GreaterThanSquareEpsilon( norm2 ) )
						{
							double deltaValue = static_cast<double>(v2 - v1);
							if (!euclideanDistances || deltaValue*deltaValue < 1.01 * norm2)
							{
								deltaValue /= norm2; //we divide by 'norm' to get the normalized direction, and by 'norm' again to get the gradient (hence we use the squared norm)
								sum.x += deltaPos.x * deltaValue; //warning: here 'deltaValue'= deltaValue / squaredNorm(deltaPos) ;)
								sum.y += deltaPos.y * deltaValue;
								sum.z += deltaPos.z * deltaValue;
								++counter;
							}
						}
					}
				}

				if (counter != 0)
				{
					gN = static_cast<ScalarType>(sum.norm() / counter);
				}
			}
		}

		if (theGradientNorms)
		{
			//if "IN" and "OUT" SFs are the same
			theGradientNorms->setValue(cell.points->getPointGlobalIndex(i), gN);
		}
		else
		{
			//if "IN" and "OUT" SFs are different
			cell.points->setPointScalarValue(i, gN);
		}

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	return true;
}

bool ScalarFieldTools::applyScalarFieldGaussianFilter(PointCoordinateType sigma,
													  GenericIndexedCloudPersist* theCloud,
													  PointCoordinateType sigmaSF,
													  GenericProgressCallback* progressCb,
													  DgmOctree* theCloudOctree)
{
	if (!theCloud)
		return false;

	unsigned n = theCloud->size();
	if (n == 0)
		return false;

	DgmOctree* theOctree = nullptr;
	if (theCloudOctree)
		theOctree = theCloudOctree;
	else
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb) < 1)
		{
			delete theOctree;
			return false;
		}
	}

	//best octree level
	unsigned char level = theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(3*sigma);

	//output scalar field should be different than input one
	if (!theCloud->enableScalarField())
	{
		if (!theCloudOctree)
			delete theOctree;
		return false;
	}

	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setMethodTitle("Gaussian filter");
			char infos[32];
			snprintf(infos, 32, "Level: %i", level);
			progressCb->setInfo(infos);
		}
		progressCb->update(0);
	}

	void* additionalParameters[2] {	reinterpret_cast<void*>(&sigma),
									reinterpret_cast<void*>(&sigmaSF) };

	bool success = true;

	if (theOctree->executeFunctionForAllCellsAtLevel(	level,
														computeCellGaussianFilter,
														additionalParameters,
														true,
														progressCb,
														"Gaussian Filter computation") == 0)
	{
		//something went wrong
		success = false;
	}

	return success;
}

// Additional parameters:
// [0] -> (PointCoordinateType*) sigma: Gaussian filter sigma
// [1] -> (PointCoordinateType*) sigmaSF: Bilateral filter sigma (if > 0)
bool ScalarFieldTools::computeCellGaussianFilter(	const DgmOctree::octreeCell& cell,
													void** additionalParameters,
													NormalizedProgress* nProgress/*=nullptr*/)
{
	//variables additionnelles
	PointCoordinateType sigma	= *(static_cast<PointCoordinateType*>(additionalParameters[0]));
	PointCoordinateType sigmaSF	= *(static_cast<PointCoordinateType*>(additionalParameters[1]));

	//we use only the squared value of sigma
	PointCoordinateType sigma2 = 2*sigma*sigma;
	PointCoordinateType radius = 3*sigma; //3*sigma > 99.7%

	//we use only the squared value of sigmaSF
	PointCoordinateType sigmaSF2 = 2*sigmaSF*sigmaSF;

	//number of points inside the current cell
	unsigned n = cell.points->size();

	//structures pour la recherche de voisinages SPECIFIQUES
	DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level = cell.level;
	cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	//we already know the points lying in the first cell (this is the one we are treating :)
	try
	{
		nNSS.pointsInNeighbourhood.resize(n);
	}
	catch (.../*const std::bad_alloc&*/) //out of memory
	{
		return false;
	}

	DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
	{
		for (unsigned i = 0; i < n; ++i, ++it)
		{
			it->point = cell.points->getPointPersistentPtr(i);
			it->pointIndex = cell.points->getPointGlobalIndex(i);
		}
	}
	nNSS.alreadyVisitedNeighbourhoodSize = 1;

	const GenericIndexedCloudPersist* cloud = cell.points->getAssociatedCloud();

	bool bilateralFilter = (sigmaSF > 0);

	for (unsigned i = 0; i < n; ++i) //for each point in cell
	{
		ScalarType queryValue = 0;
		if (bilateralFilter)
		{
			queryValue = cell.points->getPointScalarValue(i); //scalar of the query point

			// check that the query SF value is valid, otherwise no need to compute anything
			if (!ScalarField::ValidValue(queryValue))
			{
				cell.points->setPointScalarValue(i, NAN_VALUE);
				continue;
			}
		}

		//we get the points inside a spherical neighbourhood (radius: '3*sigma')
		cell.points->getPoint(i, nNSS.queryPoint);
		//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (k)!
		unsigned k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS, radius, false);

		//each point adds a contribution weighted by its distance to the sphere center
		it = nNSS.pointsInNeighbourhood.begin();
		double meanValue = 0.0;
		double wSum = 0.0;
		for (unsigned j = 0; j < k; ++j, ++it)
		{
			double weight = exp(-(it->squareDistd) / sigma2); //PDF: -exp(-(x-mu)^2/(2*sigma^2))

			ScalarType val = cloud->getPointScalarValue(it->pointIndex);

			//scalar value must be valid
			if (!ScalarField::ValidValue(val))
			{
				continue;
			}

			if (bilateralFilter)
			{
				ScalarType dSF = queryValue - val;
				weight *= exp(-(dSF*dSF) / sigmaSF2);
			}

			meanValue += val * weight;
			wSum += weight;
		}

		cell.points->setPointScalarValue(i, wSum != 0.0 ? static_cast<ScalarType>(meanValue / wSum) : NAN_VALUE);

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	return true;
}

void ScalarFieldTools::multiplyScalarFields(GenericIndexedCloud* firstCloud, GenericIndexedCloud* secondCloud, GenericProgressCallback* progressCb)
{
	if (!firstCloud || !secondCloud)
		return;

	unsigned n1 = firstCloud->size();
	if (n1 != secondCloud->size() || n1 == 0)
		return;

	for (unsigned i = 0; i < n1; ++i)
	{
		ScalarType V1 = firstCloud->getPointScalarValue(i);
		ScalarType V2 = secondCloud->getPointScalarValue(i);

		firstCloud->setPointScalarValue(i, ScalarField::ValidValue(V1) && ScalarField::ValidValue(V2) ? V1*V2 : NAN_VALUE);
	}
}

void ScalarFieldTools::computeScalarFieldExtremas(const GenericCloud* theCloud, ScalarType& minV, ScalarType& maxV)
{
	assert(theCloud);

	minV = maxV = NAN_VALUE;

	unsigned numberOfPoints = theCloud ? theCloud->size() : 0;
	if (numberOfPoints == 0)
		return;

	bool firstValidValue = true;

	for (unsigned i = 0; i < numberOfPoints; ++i)
	{
		ScalarType V = theCloud->getPointScalarValue(i);
		if (ScalarField::ValidValue(V))
		{
			if (!firstValidValue)
			{
				if (V < minV)
					minV = V;
				else if (V > maxV)
					maxV = V;
			}
			else
			{
				minV = maxV = V;
				firstValidValue = false;
			}
		}
	}
}

unsigned ScalarFieldTools::countScalarFieldValidValues(const GenericCloud* theCloud)
{
	assert(theCloud);

	unsigned count = 0;

	if (theCloud)
	{
		unsigned n = theCloud->size();
		for (unsigned i = 0; i < n; ++i)
		{
			ScalarType V = theCloud->getPointScalarValue(i);
			if (ScalarField::ValidValue(V))
				++count;
		}
	}

	return count;
}

void ScalarFieldTools::computeScalarFieldHistogram(const GenericCloud* theCloud, unsigned numberOfClasses, std::vector<int>& histo)
{
	//reset
	histo.clear();

	//valid input?
	if (!theCloud || numberOfClasses == 0)
	{
		assert(false);
		return;
	}
	unsigned pointCount = theCloud->size();

	//specific case: 1 class?!
	if (numberOfClasses == 1)
	{
		histo.push_back(static_cast<int>(pointCount));
		return;
	}

	try
	{
		histo.resize(numberOfClasses, 0);
	}
	catch (const std::bad_alloc)
	{
		//out of memory
		return;
	}

	//compute the min and max sf values
	ScalarType minV;
	ScalarType maxV;
	{
		computeScalarFieldExtremas(theCloud, minV, maxV);

		if (!ScalarField::ValidValue(minV))
		{
			//sf is only composed of NAN values?!
			return;
		}
	}

	//historgram step
	ScalarType invStep = (maxV > minV ? numberOfClasses / (maxV - minV) : 0);

	//histogram computation
	{
		int iNumberOfClasses = static_cast<int>(numberOfClasses);
		for (unsigned i = 0; i < pointCount; ++i)
		{
			ScalarType V = theCloud->getPointScalarValue(i);
			if (ScalarField::ValidValue(V))
			{
				int aimClass = static_cast<int>((V - minV) * invStep);
				if (aimClass == iNumberOfClasses)
					--aimClass; //sepcific case: V == maxV

				++histo[aimClass];
			}
		}
	}
}

bool ScalarFieldTools::computeKmeans(	const GenericCloud* theCloud,
										unsigned char K,
										KMeanClass kmcc[],
										GenericProgressCallback* progressCb)
{
	//valid parameters?
	if (!theCloud || K == 0)
	{
		assert(false);
		return false;
	}

	unsigned n = theCloud->size();
	if (n == 0)
		return false;

	//on a besoin de memoire ici !
	std::vector<ScalarType> theKMeans;		//K clusters centers
	std::vector<unsigned char> belongings;			//index of the cluster the point belongs to
	std::vector<ScalarType> minDistsToMean;	//distance to the nearest cluster center
	std::vector<ScalarType> theKSums;		//sum of distances to the clusters
	std::vector<unsigned> theKNums;			//number of points per clusters
	std::vector<unsigned> theOldKNums;		//number of points per clusters (prior to iteration)

	try
	{
		theKMeans.resize(n);
		belongings.resize(n);
		minDistsToMean.resize(n);
		theKSums.resize(K);
		theKNums.resize(K);
		theOldKNums.resize(K);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	//compute min and max SF values
	ScalarType minV;
	ScalarType maxV;
	{
		computeScalarFieldExtremas(theCloud, minV, maxV);

		if (!ScalarField::ValidValue(minV))
		{
			//sf is only composed of NAN values?!
			return false;
		}
	}

	//init classes centers (regularly sampled)
	{
		ScalarType step = (maxV - minV) / K;
		for (unsigned char j=0; j<K; ++j)
			theKMeans[j] = minV + step * j;
	}

	//for progress notification
	double initialCMD = 0;

	//let's start
	bool meansHaveMoved = false;
	int iteration = 0;
	do
	{
		meansHaveMoved = false;
		++iteration;
		{
			for (unsigned i=0; i<n; ++i)
			{
				unsigned char minK = 0;

				ScalarType V = theCloud->getPointScalarValue(i);
				if (ScalarField::ValidValue(V))
				{
					minDistsToMean[i] = std::abs(theKMeans[minK]-V);

					//we look for the nearest cluster center
					for (unsigned char j = 1; j < K; ++j)
					{
						ScalarType distToMean = std::abs(theKMeans[j] - V);
						if (distToMean<minDistsToMean[i])
						{
							minDistsToMean[i] = distToMean;
							minK = j;
						}
					}
				}

				belongings[i] = minK;
				minDistsToMean[i] = V;
			}
		}

		//compute the clusters centers
		theOldKNums = theKNums;
		std::fill(theKSums.begin(), theKSums.end(), static_cast<ScalarType>(0));
		std::fill(theKNums.begin(), theKNums.end(), static_cast<unsigned>(0));
		{
			for (unsigned i = 0; i < n; ++i)
			{
				if (minDistsToMean[i] >= 0) //must be a valid value!
				{
					theKSums[belongings[i]] += minDistsToMean[i];
					++theKNums[belongings[i]];
				}
			}
		}

		double classMovingDist = 0.0;
		{
			for (unsigned char j = 0; j < K; ++j)
			{
				ScalarType newMean = (theKNums[j] > 0 ? theKSums[j] / theKNums[j] : theKMeans[j]);

				if (theOldKNums[j] != theKNums[j])
					meansHaveMoved = true;

				classMovingDist += std::abs(theKMeans[j] - newMean);

				theKMeans[j] = newMean;
			}
		}

		if (progressCb)
		{
			if (iteration == 1)
			{
				if (progressCb->textCanBeEdited())
				{
					progressCb->setMethodTitle("KMeans");
					char buffer[16];
					snprintf(buffer, 16, "K=%i", K);
					progressCb->setInfo(buffer);
					progressCb->start();
				}
				progressCb->update(0);
				initialCMD = classMovingDist;
			}
			else
			{
				progressCb->update(static_cast<float>((1.0 - classMovingDist / initialCMD) * 100.0));
			}
		}
	} while (meansHaveMoved);

	//update distances
	std::vector<ScalarType> mins;
	std::vector<ScalarType> maxs;
	try
	{
		mins.resize(K, maxV);
		maxs.resize(K, minV);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	//look for min and max values for each cluster
	{
		for (unsigned i = 0; i < n; ++i)
		{
			ScalarType V = theCloud->getPointScalarValue(i);
			if (ScalarField::ValidValue(V))
			{
				if (V < mins[belongings[i]])
					mins[belongings[i]] = V;
				else if (V > maxs[belongings[i]])
					maxs[belongings[i]] = V;
			}
		}
	}

	//last check
	{
		for (unsigned char j = 0; j < K; ++j)
			if (theKNums[j] == 0)
				mins[j] = maxs[j] = -1.0;
	}

	//output
	{
		for (unsigned char j = 0; j < K; ++j)
		{
			kmcc[j].mean = theKMeans[j];
			kmcc[j].minValue = mins[j];
			kmcc[j].maxValue = maxs[j];
		}
	}

	if (progressCb)
		progressCb->stop();

	return true;
}

ScalarType ScalarFieldTools::computeMeanScalarValue(GenericCloud* theCloud)
{
	//valid input?
	if (!theCloud)
	{
		assert(false);
		return NAN_VALUE;
	}

	double meanValue = 0.0;
	unsigned count = 0;

	for (unsigned i=0; i<theCloud->size(); ++i)
	{
		ScalarType V = theCloud->getPointScalarValue(i);
		if (ScalarField::ValidValue(V))
		{
			meanValue += V;
			++count;
		}
	}

	return (count ? static_cast<ScalarType>(meanValue/count) : 0);
}

ScalarType ScalarFieldTools::computeMeanSquareScalarValue(GenericCloud* theCloud)
{
	//valid input?
	if (!theCloud)
	{
		assert(false);
		return NAN_VALUE;
	}

	double meanValue = 0.0;
	unsigned count = 0;

	for (unsigned i=0; i<theCloud->size(); ++i)
	{
		ScalarType V = theCloud->getPointScalarValue(i);
		if (ScalarField::ValidValue(V))
		{
			double Vd = static_cast<double>(V);
			meanValue += Vd*Vd;
			++count;
		}
	}

	return (count ? static_cast<ScalarType>(meanValue/count) : 0);
}
