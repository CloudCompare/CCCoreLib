// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include <AutoSegmentationTools.h>

//local
#include <FastMarchingForPropagation.h>
#include <GenericProgressCallback.h>
#include <ReferenceCloud.h>
#include <ScalarField.h>
#include <ScalarFieldTools.h>

//System
#include <algorithm>
#include <memory>

using namespace CCCoreLib;

int AutoSegmentationTools::labelConnectedComponents(GenericIndexedCloudPersist* theCloud,
													unsigned char level,
													bool sixConnexity/*=false*/,
													GenericProgressCallback* progressCb/*=nullptr*/,
													DgmOctree* inputOctree/*=nullptr*/)
{
	if (nullptr == theCloud)
	{
		return -1;
	}

	//compute octree if none was provided
	DgmOctree* theOctree = inputOctree;
	if (nullptr == theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb) < 1)
		{
			delete theOctree;
			return -1;
		}
	}

	//we use the default scalar field to store components labels
	if (!theCloud->enableScalarField())
	{
		//failed to enable a scalar field
		return -1;
	}

	int result = theOctree->extractCCs(level, sixConnexity, progressCb);

	//remove octree if it was not provided as input
	if (nullptr == inputOctree)
	{
		delete theOctree;
		theOctree = nullptr;
	}

	return result;
}

bool AutoSegmentationTools::extractConnectedComponents(GenericIndexedCloudPersist* theCloud, ReferenceCloudContainer& cc)
{
	unsigned numberOfPoints = (theCloud ? theCloud->size() : 0);
	if (numberOfPoints == 0)
	{
		return false;
	}

	//components should have already been labeled and labels should have been stored in the active scalar field!
	if (!theCloud->isScalarFieldEnabled())
	{
		return false;
	}

	//empty the input vector if necessary
	for (auto cloud : cc)
	{
		delete cloud;
	} 
	cc.clear();

	for (unsigned i = 0; i < numberOfPoints; ++i)
	{
		ScalarType slabel = theCloud->getPointScalarValue(i);
		if (slabel >= 1) //labels start from 1! (this test rejects NaN values as well)
		{
			int ccLabel = static_cast<int>(theCloud->getPointScalarValue(i)) - 1;

			//we fill the components vector with empty components until we reach the current label
			//(they will be "used" later)
			try
			{
				while (static_cast<std::size_t>(ccLabel) >= cc.size())
				{
					cc.push_back(new ReferenceCloud(theCloud));
				}
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory
				for (auto cloud : cc)
				{
					delete cloud;
				} 
				cc.clear();
				return false;
			}

			//add the point to the current component
			if (!cc[ccLabel]->addPointIndex(i))
			{
				//not enough memory
				for (auto cloud : cc)
				{
					delete cloud;
				} 
				cc.clear();

				return false;
			}
		}
	}

	return true;
}

bool AutoSegmentationTools::frontPropagationBasedSegmentation(	GenericIndexedCloudPersist* theCloud,
																PointCoordinateType radius,
																ScalarType minSeedDist,
																unsigned char octreeLevel,
																ReferenceCloudContainer& theSegmentedLists,
																GenericProgressCallback* progressCb,
																DgmOctree* inputOctree,
																bool applyGaussianFilter,
																float alpha)
{
	unsigned numberOfPoints = (theCloud ? theCloud->size() : 0);
	if (numberOfPoints == 0)
	{
		return false;
	}

	//compute octree if none was provided
	DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb) < 1)
		{
			delete theOctree;
			return false;
		}
	}

	//we compute the gradient (may overwrite the distances SF)
	if (ScalarFieldTools::computeScalarFieldGradient(theCloud, radius, true, true, progressCb, theOctree) < 0)
	{
		if (nullptr == inputOctree)
		{
			delete theOctree;
		}
		return false;
	}

	//we optionally smooth the result
	if (applyGaussianFilter)
	{
		ScalarFieldTools::applyScalarFieldGaussianFilter(radius / 3, theCloud, -1, progressCb, theOctree);
	}

	unsigned seedPoints = 0;
	unsigned numberOfSegmentedLists = 0;

	//start the FastMarching front propagation
	std::unique_ptr<FastMarchingForPropagation> fm(new FastMarchingForPropagation());
	{
		fm->setJumpCoef(50.0);
		fm->setDetectionThreshold(alpha);

		int result = fm->init(theCloud, theOctree, octreeLevel);
		if (result < 0)
		{
			if (nullptr == inputOctree)
			{
				delete theOctree;
			}
			return false;
		}
	}
	int octreeLength = DgmOctree::OCTREE_LENGTH(octreeLevel) - 1;

	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setMethodTitle("FM Propagation");
			char buffer[64];
			snprintf(buffer, 64, "Octree level: %i\nNumber of points: %u", octreeLevel, numberOfPoints);
			progressCb->setInfo(buffer);
		}
		progressCb->update(0);
		progressCb->start();
	}

	ScalarField* theDists = new ScalarField("distances");
	{
		ScalarType d = theCloud->getPointScalarValue(0);
		if (!theDists->resizeSafe(numberOfPoints, true, d))
		{
			if (nullptr == inputOctree)
			{
				delete theOctree;
			}
			theDists->release();
			return false;
		}
	}

	unsigned maxDistIndex = 0;
	unsigned begin = 0;
	CCVector3 startPoint;

	while (true)
	{
		ScalarType maxDist = NAN_VALUE;

		//on cherche la premiere distance superieure ou egale a "minSeedDist"
		while (begin < numberOfPoints)
		{
			const CCVector3* thePoint = theCloud->getPoint(begin);
			const ScalarType theDistance = theDists->getValue(begin);
			++begin;

			if (	(theCloud->getPointScalarValue(begin) >= 0)
				&&	(theDistance >= minSeedDist) )
			{
				maxDist = theDistance;
				startPoint = *thePoint;
				maxDistIndex = begin;
				break;
			}
			else
			{
				//FIXME DGM: what happens if SF is negative?!
			}
		}

		//il n'y a plus de point avec des distances suffisamment grandes !
		if (maxDist < minSeedDist)
		{
			break;
		}

		//on finit la recherche du max
		for (unsigned i = begin; i < numberOfPoints; ++i)
		{
			const CCVector3 *thePoint = theCloud->getPoint(i);
			const ScalarType theDistance = theDists->getValue(i);

			if (	(theCloud->getPointScalarValue(i) >= 0.0)
				&&	(theDistance > maxDist) )
			{
				maxDist = theDistance;
				startPoint = *thePoint;
				maxDistIndex = i;
			}
		}

		//set seed point
		{
			Tuple3i cellPos;
			theOctree->getTheCellPosWhichIncludesThePoint(&startPoint, cellPos, octreeLevel);
			//clipping (important!)
			cellPos.x = std::min(octreeLength, cellPos.x);
			cellPos.y = std::min(octreeLength, cellPos.y);
			cellPos.z = std::min(octreeLength, cellPos.z);
			fm->setSeedCell(cellPos);
			++seedPoints;
		}

		int resultFM = fm->propagate();

		//if the propagation was successful
		if (resultFM >= 0)
		{
			//we extract the corresponding points
			ReferenceCloud* newCloud = new ReferenceCloud(theCloud);

			if (fm->extractPropagatedPoints(newCloud) && newCloud->size() != 0)
			{
				theSegmentedLists.push_back(newCloud);
				++numberOfSegmentedLists;
			}
			else
			{
				//not enough memory?!
				delete newCloud;
				newCloud = nullptr;
			}

			if (progressCb)
			{
				progressCb->update(static_cast<float>(numberOfSegmentedLists % 100));
			}

			fm->cleanLastPropagation();

			//break;
		}

		if (maxDistIndex == begin)
		{
			++begin;
		}
	}

	if (progressCb)
	{
		progressCb->stop();
	}

	for (unsigned i = 0; i < numberOfPoints; ++i)
	{
		theCloud->setPointScalarValue(i, theDists->getValue(i));
	}

	theDists->release();
	theDists = nullptr;

	if (nullptr == inputOctree)
	{
		delete theOctree;
		theOctree = nullptr;
	}

	return true;
}
