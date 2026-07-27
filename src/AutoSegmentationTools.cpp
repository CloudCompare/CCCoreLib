// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include <AutoSegmentationTools.h>

//local
#include <FastMarchingForPropagation.h>
#include <GenericProgressCallback.h>
#include <ReferenceCloud.h>
#include <ScalarField.h>
#include <ScalarFieldTools.h>
#include <GridGraph.h>
#include <CutPursuit.h>
#include "cp_d0_dist.h"

//System
#include <algorithm>
#include <memory>

#if defined(_OPENMP)
// OpenMP
#include <omp.h>
#endif

using namespace CCCoreLib;

int AutoSegmentationTools::labelCutPursuitComponents(GenericIndexedCloudPersist* theCloud,
													int32_t knn,
													double knnRadius,
													int32_t N,
													int32_t D,
													std::vector<float> Y,
													float regularization,
													float spatialWeight,
													int32_t cutoff,
													std::vector<int32_t>& components,
													std::function<void(int)> progressCb,
													DgmOctree* theOctree)
{
	if (nullptr == theCloud)
	{
		return -1;
	}

	//we use the default scalar field to store components labels
	if (!theCloud->enableScalarField())
	{
		//failed to enable a scalar field
		return -1;
	}

	// call cut pursuit
	unsigned char bestLevel = theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(knnRadius);
	CCCoreLib::ReferenceCloud neighbors(theCloud);

	// allocate memory for edges computation
	std::vector<int32_t> edges;
	std::vector<float> distances;

	// compute edges
	progressCb(0);
	float distancesSum = 0.0;
	for (int32_t i = 0; i < N; ++i)
	{
		neighbors.clear(false);
		double maxSquareDist = 0.0;
		int finalNeighbourhoodSize = 0;
		const CCVector3* queryPoint = theCloud->getPoint(i);
		if (theOctree->findPointNeighbourhood(
			queryPoint,             	// Position we are searching around
			&neighbors,             	// Where the resulting neighbor indices will be stored
			knn,                   		// Max number of neighbors (k)
			bestLevel,               	// The optimized octree level we calculated
			maxSquareDist,          	// Output: The squared distance to the furthest neighbor found
			knnRadius,             		// Max search radius (r)
			&finalNeighbourhoodSize) 	// Output: Internal octree box search size metric (optional)
		)
		{
			int32_t source = static_cast<int>(i);
			for (unsigned n = 0; n < neighbors.size(); ++n)
			{
				int32_t target = static_cast<int>(neighbors.getPointGlobalIndex(n));

				// ignore self-loops
				if (source == target)
					continue;

				edges.push_back(source); // 2*e
				edges.push_back(target); // 2*e + 1

				// compute distance
				const CCVector3* p1 = theCloud->getPoint(source);
				const CCVector3* p2 = theCloud->getPoint(target);
				float distance = static_cast<float>((*p1 - *p2).norm());
				distancesSum += distance;
				distances.push_back(distance);
			}
		}

		progressCb(int(double(i + 1) / N * 30.0));
	}

	// more parallel cut pursuit params
	int32_t E = static_cast<int32_t>(edges.size() / 2);
	std::vector<float> edgeWeights(E);
	std::vector<int32_t> first_edge(N + 1);
	std::vector<int32_t> adj_vertices(E);
	std::vector<int32_t> reindex(E);
	std::vector<float> node_size(N, 1.0f);
	std::vector<float> coor_weights(D, 1.0f);
	float cp_dif_tol = 0.01f;
	int cp_it_max = 15;
	int K = 2;
	int split_iter_num = 2;
	float split_damp_ratio = 0.7f;
	int kmpp_init_num = 3;
	int kmpp_iter_num = 3;
	int verbose = 1000;
	int max_num_threads = omp_get_max_threads();
	int32_t max_split_size = N;
	int balance_parallel_split = false;
	int compute_Time = true;
	int compute_List = true;
	int compute_Graph = true;
	int compute_Obj = false;
	int compute_Dif = false;
	
	// monitoring arrays
	float* Obj = nullptr;
	if (compute_Obj){ Obj = (float*) malloc(sizeof(float)*(cp_it_max + 1)); }

	double* Time = nullptr;
	if (compute_Time){
		Time = (double*) malloc(sizeof(double)*(cp_it_max + 1));
	}

	float* Dif = nullptr;
	if (compute_Dif){ Dif = (float*) malloc(sizeof(float)*cp_it_max); }

	// Allocate Comp array using malloc because cut pursuit uses C-style memory tracking
	int32_t* Comp = (int32_t*)calloc(N, sizeof(int32_t));
	if (!Comp)
	{
		return -1;
	}
	
	// compute CSR representation of the graph
	GridGraph graph;
	graph.edge_list_to_forward_star<int32_t, int32_t>(
		N,
		E,
		edges.data(),
		first_edge.data(),
		reindex.data()
	);

	// apply spatial weight
	for (int32_t d = 0; d < 3; ++d)
	{
		coor_weights[d] *= spatialWeight;
	}

	// compute targets and edge weights based on distances in CSR order
	float avgDistance = distancesSum / distances.size();
	for (int32_t e = 0; e < E; ++e)
	{
		// compute weight based on distance
		float distance = distances[e];
		float edgeAttr = distance;
		
		// The target vertex for original edge 'e' is stored at (2 * e + 1)
		adj_vertices[reindex[e]] = edges[2 * e + 1];
		
		// The weight for original edge 'e' maps to the same new position
		edgeWeights[reindex[e]] = edgeAttr * regularization;
	}

	//  cut-pursuit with preconditioned forward-Douglas-Rachford
	Cp_d0_dist<float, int32_t, int32_t>* cp =
		new Cp_d0_dist<float, int32_t, int32_t>
			(N, E, first_edge.data(), adj_vertices.data(), Y.data(), D);

	cp->set_loss(static_cast<float>(D), Y.data(), node_size.data(), coor_weights.data());
	cp->set_edge_weights(edgeWeights.data(), regularization);
	cp->set_cp_param(cp_dif_tol, cp_it_max, verbose);
	cp->set_split_param(max_split_size, K, split_iter_num, split_damp_ratio,
		kmpp_init_num, kmpp_iter_num);
	cp->set_min_comp_weight(static_cast<float>(cutoff));
	cp->set_parallel_param(max_num_threads, balance_parallel_split);
	cp->set_monitoring_arrays(Obj, Time, Dif);
	cp->set_components(0, Comp);

	int cp_it = cp->cut_pursuit(true, progressCb);

	// Get number of components and their lists of indices
	const int32_t* comp_assign;
	const int32_t* first_vertex;
	const int32_t* comp_list;
	auto rV = cp->get_components(&comp_assign, &first_vertex, &comp_list);

	// Copy results
	components.resize(N);
	for (int32_t i = 0; i < N; i++) {
		components[i] = static_cast<int32_t>(comp_assign[i]);
	}

	delete cp;
	return rV;
}

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

	auto theDists = std::make_shared<ScalarField>("distances");
	{
		ScalarType d = theCloud->getPointScalarValue(0);
		if (!theDists->resizeSafe(numberOfPoints, true, d))
		{
			if (nullptr == inputOctree)
			{
				delete theOctree;
			}
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

	if (nullptr == inputOctree)
	{
		delete theOctree;
		theOctree = nullptr;
	}

	return true;
}
