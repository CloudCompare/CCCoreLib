// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include <RegistrationTools.h>

//local
#include <CCMath.h>
#include <CloudSamplingTools.h>
#include <DistanceComputationTools.h>
#include <Garbage.h>
#include <GenericProgressCallback.h>
#include <GenericIndexedMesh.h>
#include <GeometricalAnalysisTools.h>
#include <Jacobi.h>
#include <ManualSegmentationTools.h>
#include <NormalDistribution.h>
#include <ParallelSort.h>
#include <PointCloud.h>
#include <ReferenceCloud.h>
#include <ScalarFieldTools.h>

//system
#include <ctime>

using namespace CCCoreLib;

void RegistrationTools::FilterTransformation(	const ScaledTransformation& inTrans,
												int filters,
												ScaledTransformation& outTrans )
{
	outTrans = inTrans;

	//filter translation
	if (filters & SKIP_TRANSLATION)
	{
		if (filters & SKIP_TX)
			outTrans.T.x = 0;
		if (filters & SKIP_TY)
			outTrans.T.y = 0;
		if (filters & SKIP_TZ)
			outTrans.T.z = 0;
	}

	//filter rotation
	int rotationFilter = (filters & SKIP_ROTATION);
	if (inTrans.R.isValid() && (rotationFilter != 0))
	{
		const SquareMatrix R(inTrans.R); //copy it in case inTrans and outTrans are the same!
		outTrans.R.toIdentity();
		if (rotationFilter == SKIP_RYZ) //keep only the rotation component around X
		{
			//we use a specific Euler angles convention here
			if (R.getValue(0, 2) < 1.0)
			{
				PointCoordinateType phi = -asin(R.getValue(0, 2));
				PointCoordinateType cos_phi = cos(phi);
				PointCoordinateType theta = atan2(R.getValue(1, 2) / cos_phi, R.getValue(2, 2) / cos_phi);
				PointCoordinateType cos_theta = cos(theta);
				PointCoordinateType sin_theta = sin(theta);

				outTrans.R.setValue(1, 1, cos_theta);
				outTrans.R.setValue(2, 2, cos_theta);
				outTrans.R.setValue(2, 1, sin_theta);
				outTrans.R.setValue(1, 2, -sin_theta);
			}
			else
			{
				//simpler/faster to ignore this (very) specific case!
			}
		}
		else if (rotationFilter == SKIP_RXZ) //keep only the rotation component around Y
		{
			//we use a specific Euler angles convention here
			if (R.getValue(2, 1) < 1.0)
			{
				PointCoordinateType theta = asin(R.getValue(2, 1));
				PointCoordinateType cos_theta = cos(theta);
				PointCoordinateType phi = atan2(-R.getValue(2, 0) / cos_theta, R.getValue(2, 2) / cos_theta);
				PointCoordinateType cos_phi = cos(phi);
				PointCoordinateType sin_phi = sin(phi);

				outTrans.R.setValue(0, 0, cos_phi);
				outTrans.R.setValue(2, 2, cos_phi);
				outTrans.R.setValue(0, 2, sin_phi);
				outTrans.R.setValue(2, 0, -sin_phi);
			}
			else
			{
				//simpler/faster to ignore this (very) specific case!
			}
		}
		else if (rotationFilter == SKIP_RXY) //keep only the rotation component around Z
		{
			//we use a specific Euler angles convention here
			if (R.getValue(2, 0) < 1.0)
			{
				PointCoordinateType theta_rad = -asin(R.getValue(2, 0));
				PointCoordinateType cos_theta = cos(theta_rad);
				PointCoordinateType phi_rad = atan2(R.getValue(1, 0) / cos_theta, R.getValue(0, 0) / cos_theta);
				PointCoordinateType cos_phi = cos(phi_rad);
				PointCoordinateType sin_phi = sin(phi_rad);

				outTrans.R.setValue(0, 0, cos_phi);
				outTrans.R.setValue(1, 1, cos_phi);
				outTrans.R.setValue(1, 0, sin_phi);
				outTrans.R.setValue(0, 1, -sin_phi);
			}
			else
			{
				//simpler/faster to ignore this (very) specific case!
			}
		}
		else
		{
			//we ignore all rotation components
		}
	}
}

struct ModelCloud
{
	ModelCloud() : cloud(nullptr), weights(nullptr) {}
	ModelCloud(const ModelCloud& m) = default;
	GenericIndexedCloudPersist* cloud;
	ScalarField* weights;
};

struct DataCloud
{
	DataCloud() : cloud(nullptr), rotatedCloud(nullptr), weights(nullptr), CPSetRef(nullptr), CPSetPlain(nullptr) {}

	ReferenceCloud* cloud;
	PointCloud* rotatedCloud;
	ScalarField* weights;
	ReferenceCloud* CPSetRef;
	PointCloud* CPSetPlain;
};

ICPRegistrationTools::RESULT_TYPE ICPRegistrationTools::Register(	GenericIndexedCloudPersist* inputModelCloud,
																	GenericIndexedMesh* inputModelMesh,
																	GenericIndexedCloudPersist* inputDataCloud,
																	const Parameters& params,
																	ScaledTransformation& transform,
																	double& finalRMS,
																	unsigned& finalPointCount,
																	GenericProgressCallback* progressCb/*=nullptr*/)
{
	if (!inputModelCloud || !inputDataCloud)
	{
		assert(false);
		return ICP_ERROR_INVALID_INPUT;
	}

	//hopefully the user will understand it's not possible ;)
	finalRMS = -1.0;

	Garbage<GenericIndexedCloudPersist> cloudGarbage;
	Garbage<ScalarField> sfGarbage;

	bool registerWithNormals = (params.normalsMatching != NO_NORMAL);

	//DATA CLOUD (will move)
	DataCloud data;
	{
		//we also want to use the same number of points for registration as initially defined by the user!
		unsigned dataSamplingLimit = params.finalOverlapRatio != 1.0 ? static_cast<unsigned>(params.samplingLimit / params.finalOverlapRatio) : params.samplingLimit;

		if (inputDataCloud->size() == 0)
		{
			return ICP_NOTHING_TO_DO;
		}
		else if (inputDataCloud->size() > dataSamplingLimit)
		{
			//we resample the cloud if it's too big (speed increase)
			data.cloud = CloudSamplingTools::subsampleCloudRandomly(inputDataCloud, dataSamplingLimit);
			if (!data.cloud)
			{
				return ICP_ERROR_NOT_ENOUGH_MEMORY;
			}
			cloudGarbage.add(data.cloud);

			//if we need to resample the weights as well
			if (params.dataWeights)
			{
				data.weights = new ScalarField("ResampledDataWeights");
				sfGarbage.add(data.weights);

				unsigned destCount = data.cloud->size();
				if (data.weights->resizeSafe(destCount))
				{
					for (unsigned i = 0; i < destCount; ++i)
					{
						unsigned pointIndex = data.cloud->getPointGlobalIndex(i);
						data.weights->setValue(i, params.dataWeights->getValue(pointIndex));
					}
					data.weights->computeMinAndMax();
				}
				else
				{
					//not enough memory
					return ICP_ERROR_NOT_ENOUGH_MEMORY;
				}
			}
		}
		else //no need to resample
		{
			//we still create a 'fake' reference cloud with all the points
			data.cloud = new ReferenceCloud(inputDataCloud);
			cloudGarbage.add(data.cloud);
			if (!data.cloud->addPointIndex(0, inputDataCloud->size()))
			{
				//not enough memory
				return ICP_ERROR_NOT_ENOUGH_MEMORY;
			}
			if (params.dataWeights)
			{
				//we use the input weights
				data.weights = new ScalarField(*params.dataWeights);
				sfGarbage.add(data.weights);
			}
		}

		//eventually we'll need a scalar field on the data cloud
		if (!data.cloud->enableScalarField())
		{
			//not enough memory
			return ICP_ERROR_NOT_ENOUGH_MEMORY;
		}

		//we need normals to register with normals ;)
		registerWithNormals &= inputDataCloud->normalsAvailable();
	}
	assert(data.cloud);

	//octree level for cloud/mesh distances computation
	unsigned char meshDistOctreeLevel = 8;

	//MODEL ENTITY (reference, won't move)
	ModelCloud model;
	if (inputModelMesh)
	{
		assert(!params.modelWeights);

		if (inputModelMesh->size() == 0)
		{
			return ICP_ERROR_INVALID_INPUT;
		}

		//we'll use the mesh vertices to estimate the right octree level
		DgmOctree dataOctree(data.cloud);
		DgmOctree modelOctree(inputModelCloud);
		if (dataOctree.build() < static_cast<int>(data.cloud->size()) || modelOctree.build() < static_cast<int>(inputModelCloud->size()))
		{
			//an error occurred during the octree computation: probably there's not enough memory
			return ICP_ERROR_NOT_ENOUGH_MEMORY;
		}

		meshDistOctreeLevel = dataOctree.findBestLevelForComparisonWithOctree(&modelOctree);

		//we need normals to register with normals ;)
		registerWithNormals &= inputModelMesh->normalsAvailable();
	}
	else /*if (inputModelCloud)*/
	{
		if (inputModelCloud->size() == 0)
		{
			return ICP_ERROR_INVALID_INPUT;
		}
		else if (inputModelCloud->size() > params.samplingLimit)
		{
			//we resample the cloud if it's too big (speed increase)
			ReferenceCloud* subModelCloud = CloudSamplingTools::subsampleCloudRandomly(inputModelCloud, params.samplingLimit);
			if (!subModelCloud)
			{
				//not enough memory
				return ICP_ERROR_NOT_ENOUGH_MEMORY;
			}
			cloudGarbage.add(subModelCloud);

			//if we need to resample the weights as well
			if (params.modelWeights)
			{
				model.weights = new ScalarField("ResampledModelWeights");
				sfGarbage.add(model.weights);

				unsigned destCount = subModelCloud->size();
				if (model.weights->resizeSafe(destCount))
				{
					for (unsigned i = 0; i < destCount; ++i)
					{
						unsigned pointIndex = subModelCloud->getPointGlobalIndex(i);
						model.weights->setValue(i, params.modelWeights->getValue(pointIndex));
					}
					model.weights->computeMinAndMax();
				}
				else
				{
					//not enough memory
					return ICP_ERROR_NOT_ENOUGH_MEMORY;
				}
			}
			model.cloud = subModelCloud;
		}
		else
		{
			//we use the input cloud and weights
			model.cloud = inputModelCloud;
			model.weights = params.modelWeights;
		}
		assert(model.cloud);

		//we need normals to register with normals ;)
		registerWithNormals &= inputModelCloud->normalsAvailable();
	}

	//for partial overlap
	unsigned maxOverlapCount = 0;
	std::vector<ScalarType> overlapDistances;
	if (params.finalOverlapRatio < 1.0)
	{
		//we pre-allocate the memory to sort distance values later
		try
		{
			overlapDistances.resize(data.cloud->size());
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			return ICP_ERROR_NOT_ENOUGH_MEMORY;
		}
		maxOverlapCount = static_cast<unsigned>(params.finalOverlapRatio*data.cloud->size());
		assert(maxOverlapCount != 0);
	}

	//Closest Point Set (see ICP algorithm)
	if (inputModelMesh)
	{
		data.CPSetPlain = new PointCloud;
		cloudGarbage.add(data.CPSetPlain);
	}
	else
	{
		data.CPSetRef = new ReferenceCloud(model.cloud);
		cloudGarbage.add(data.CPSetRef);
	}

	//per-point couple weights
	ScalarField* coupleWeights = nullptr;
	if (model.weights || data.weights || registerWithNormals)
	{
		coupleWeights = new ScalarField("CoupleWeights");
		sfGarbage.add(coupleWeights);
	}

	//we compute the initial distance between the two clouds (and the CPSet by the way)
	//data.cloud->forEachScalarValue(ScalarFieldTools::SetScalarValueToNaN); //DGM: done automatically in computeCloud2CloudDistances now
	if (inputModelMesh)
	{
		assert(data.CPSetPlain);
		DistanceComputationTools::Cloud2MeshDistancesComputationParams c2mDistParams;
		c2mDistParams.octreeLevel = meshDistOctreeLevel;
		c2mDistParams.signedDistances = params.useC2MSignedDistances;
		c2mDistParams.CPSet = data.CPSetPlain;
		c2mDistParams.maxThreadCount = params.maxThreadCount;
		if (DistanceComputationTools::computeCloud2MeshDistances(data.cloud, inputModelMesh, c2mDistParams, progressCb) < 0)
		{
			//an error occurred during distances computation...
			return ICP_ERROR_DIST_COMPUTATION;
		}
	}
	else if (inputModelCloud)
	{
		assert(data.CPSetRef);
		DistanceComputationTools::Cloud2CloudDistancesComputationParams c2cDistParams;
		c2cDistParams.CPSet = data.CPSetRef;
		c2cDistParams.maxThreadCount = params.maxThreadCount;
		if (DistanceComputationTools::computeCloud2CloudDistances(data.cloud, model.cloud, c2cDistParams, progressCb) < 0)
		{
			//an error occurred during distances computation...
			return ICP_ERROR_DIST_COMPUTATION;
		}
	}
	else
	{
		assert(false);
	}

	FILE* fTraceFile = nullptr;
#ifdef CC_DEBUG
	fTraceFile = fopen("registration_trace_log.csv", "wt");
#endif
	if (fTraceFile)
	{
		fprintf(fTraceFile, "Iteration; RMS; Point count;\n");
	}

	double lastStepRMS = -1.0;
	double initialDeltaRMS = -1.0;
	ScaledTransformation currentTrans;
	RESULT_TYPE result = ICP_ERROR;

	for (unsigned iteration = 0;; ++iteration)
	{
		if (progressCb && progressCb->isCancelRequested())
		{
			result = ICP_ERROR_CANCELED_BY_USER;
			break;
		}

		//shall we remove the farthest points?
		bool pointOrderHasBeenChanged = false;
		if (params.filterOutFarthestPoints)
		{
			NormalDistribution N;
			N.computeParameters(data.cloud);
			if (N.isValid())
			{
				ScalarType mu;
				ScalarType sigma2;
				N.getParameters(mu, sigma2);
				ScalarType maxDistance = static_cast<ScalarType>(mu + 2.5*sqrt(sigma2));

				DataCloud filteredData;
				filteredData.cloud = new ReferenceCloud(data.cloud->getAssociatedCloud());
				cloudGarbage.add(filteredData.cloud);

				if (data.CPSetRef)
				{
					filteredData.CPSetRef = new ReferenceCloud(data.CPSetRef->getAssociatedCloud()); //we must also update the CPSet!
					cloudGarbage.add(filteredData.CPSetRef);
				}
				else if (data.CPSetPlain)
				{
					filteredData.CPSetPlain = new PointCloud; //we must also update the CPSet!
					cloudGarbage.add(filteredData.CPSetPlain);
				}

				if (data.weights)
				{
					filteredData.weights = new ScalarField("ResampledDataWeights");
					sfGarbage.add(filteredData.weights);
				}

				unsigned pointCount = data.cloud->size();
				if (!filteredData.cloud->reserve(pointCount)
					|| (filteredData.CPSetRef && !filteredData.CPSetRef->reserve(pointCount))
					|| (filteredData.CPSetPlain && !filteredData.CPSetPlain->reserve(pointCount))
					|| (filteredData.weights && !filteredData.weights->reserveSafe(pointCount)))
				{
					//not enough memory
					result = ICP_ERROR_NOT_ENOUGH_MEMORY;
					break;
				}

				//we keep only the points with "not too high" distances
				for (unsigned i = 0; i < pointCount; ++i)
				{
					if (data.cloud->getPointScalarValue(i) <= maxDistance)
					{
						filteredData.cloud->addPointIndex(data.cloud->getPointGlobalIndex(i));
						if (filteredData.CPSetRef)
							filteredData.CPSetRef->addPointIndex(data.CPSetRef->getPointGlobalIndex(i));
						else if (filteredData.CPSetPlain)
							filteredData.CPSetPlain->addLocalPoint(*(data.CPSetPlain->getLocalPoint(i)));
						if (filteredData.weights)
							filteredData.weights->addElement(data.weights->getValue(i));
					}
				}

				//resize should be ok as we have called reserve first
				filteredData.cloud->resize(filteredData.cloud->size()); //should always be ok as current size < pointCount
				if (filteredData.CPSetRef)
					filteredData.CPSetRef->resize(filteredData.CPSetRef->size());
				else if (filteredData.CPSetPlain)
					filteredData.CPSetPlain->resize(filteredData.CPSetPlain->size());
				if (filteredData.weights)
					filteredData.weights->resize(filteredData.weights->currentSize());

				//replace old structures by new ones
				cloudGarbage.destroy(data.cloud);
				if (data.CPSetRef)
					cloudGarbage.destroy(data.CPSetRef);
				else if (data.CPSetPlain)
					cloudGarbage.destroy(data.CPSetPlain);
				if (data.weights)
					sfGarbage.destroy(data.weights);
				data = filteredData;

				pointOrderHasBeenChanged = true;
			}
		}

		//shall we ignore/remove some points based on their distance?
		DataCloud trueData;
		unsigned pointCount = data.cloud->size();
		if (maxOverlapCount != 0 && pointCount > maxOverlapCount)
		{
			assert(overlapDistances.size() >= pointCount);
			for (unsigned i = 0; i < pointCount; ++i)
			{
				overlapDistances[i] = data.cloud->getPointScalarValue(i);
				assert(overlapDistances[i] == overlapDistances[i]);
			}

			ParallelSort(overlapDistances.begin(), overlapDistances.begin() + pointCount);

			assert(maxOverlapCount != 0);
			ScalarType maxOverlapDist = overlapDistances[maxOverlapCount - 1];

			DataCloud filteredData;
			filteredData.cloud = new ReferenceCloud(data.cloud->getAssociatedCloud());
			if (data.CPSetRef)
			{
				filteredData.CPSetRef = new ReferenceCloud(data.CPSetRef->getAssociatedCloud()); //we must also update the CPSet!
				cloudGarbage.add(filteredData.CPSetRef);
			}
			else if (data.CPSetPlain)
			{
				filteredData.CPSetPlain = new PointCloud; //we must also update the CPSet!
				cloudGarbage.add(filteredData.CPSetPlain);
			}
			cloudGarbage.add(filteredData.cloud);
			if (data.weights)
			{
				filteredData.weights = new ScalarField("ResampledDataWeights");
				sfGarbage.add(filteredData.weights);
			}

			if (!filteredData.cloud->reserve(pointCount) //should be maxOverlapCount in theory, but there may be several points with the same value as maxOverlapDist!
				|| (filteredData.CPSetRef && !filteredData.CPSetRef->reserve(pointCount))
				|| (filteredData.CPSetPlain && !filteredData.CPSetPlain->reserve(pointCount))
				|| (filteredData.CPSetPlain && !filteredData.CPSetPlain->enableScalarField()) //don't forget the scalar field with the nearest triangle index
				|| (filteredData.weights && !filteredData.weights->reserveSafe(pointCount)))
			{
				//not enough memory
				result = ICP_ERROR_NOT_ENOUGH_MEMORY;
				break;
			}

			//we keep only the points with "not too high" distances
			for (unsigned i = 0; i < pointCount; ++i)
			{
				if (data.cloud->getPointScalarValue(i) <= maxOverlapDist)
				{
					filteredData.cloud->addPointIndex(data.cloud->getPointGlobalIndex(i));
					if (filteredData.CPSetRef)
					{
						filteredData.CPSetRef->addPointIndex(data.CPSetRef->getPointGlobalIndex(i));
					}
					else if (filteredData.CPSetPlain)
					{
						filteredData.CPSetPlain->addLocalPoint(*(data.CPSetPlain->getLocalPoint(i)));
						//don't forget the scalar field with the nearest triangle index!
						filteredData.CPSetPlain->addPointScalarValue(data.CPSetPlain->getPointScalarValue(i));
					}
					if (filteredData.weights)
					{
						filteredData.weights->addElement(data.weights->getValue(i));
					}
				}
			}
			assert(filteredData.cloud->size() >= maxOverlapCount);

			//resize should be ok as we have called reserve first
			filteredData.cloud->resize(filteredData.cloud->size()); //should always be ok as current size < pointCount
			if (filteredData.CPSetRef)
				filteredData.CPSetRef->resize(filteredData.CPSetRef->size());
			else if (filteredData.CPSetPlain)
				filteredData.CPSetPlain->resize(filteredData.CPSetPlain->size());
			if (filteredData.weights)
				filteredData.weights->resize(filteredData.weights->currentSize());

			//(temporarily) replace old structures by new ones
			trueData = data;
			data = filteredData;
		}

		//update couple weights (if any)
		if (coupleWeights)
		{
			unsigned count = data.cloud->size();
			assert(model.weights || data.weights || registerWithNormals);
			assert(!model.weights || (data.CPSetRef && data.CPSetRef->size() == count));

			if (coupleWeights->currentSize() != count && !coupleWeights->resizeSafe(count))
			{
				//not enough memory to store weights
				result = ICP_ERROR_NOT_ENOUGH_MEMORY;
				break;
			}

			for (unsigned i = 0; i < count; ++i)
			{
				double w = 1.0;
				if (registerWithNormals)
				{
					//retrieve the data point normal
					const CCVector3* Nd = data.cloud->getNormal(i);
					
					//retrieve the nearest model point normal
					CCVector3 Nm;
					if (inputModelMesh)
					{
						unsigned triIndex = static_cast<unsigned>(data.CPSetPlain->getPointScalarValue(i));
						assert(triIndex >= 0 && triIndex < inputModelMesh->size());
						CCVector3d globalP;
						data.CPSetPlain->getGlobalPoint(i, globalP);
						inputModelMesh->interpolateNormalsGlobal(triIndex, globalP, Nm);
					}
					else
					{
						Nm = *inputModelCloud->getNormal(i);
					}

					//we assume the vectors are unitary!
					PointCoordinateType dp = Nd->dot(Nm);

					switch (params.normalsMatching)
					{
					case OPPOSITE_NORMALS:
					{
						w = acos(dp) / M_PI; // 0 rad --> w = 0 / pi/2 rad --> w = 0.5 / pi rad --> w = 1
					}
					break;

					case SAME_SIDE_NORMALS:
					{
						w = 1.0 - acos(dp) / M_PI; // 0 rad --> w = 1 / pi/2 rad --> w = 0.5 / pi rad --> w = 0
					}
					break;

					case DOUBLE_SIDED_NORMALS:
					{
						dp = std::abs(dp);
						w = 1.0 - acos(dp) / M_PI_2; // 0 rad --> w = 1 / pi/2 rad --> w = 0
					}
					break;

					default:
						assert(false);
						break;
					}
				}
				if (data.weights)
				{
					w *= data.weights->getValue(i);
				}
				if (model.weights)
				{
					//model weights are only supported with a reference cloud!
					ScalarType wm = model.weights->getValue(data.CPSetRef->getPointGlobalIndex(i));
					w *= wm;
				}
				coupleWeights->setValue(i, static_cast<ScalarType>(w));
			}
			coupleWeights->computeMinAndMax();
		}

		//we can now compute the best registration transformation for this step
		//(now that we have selected the points that will be used for registration!)
		{
			//if we use weights, we have to compute weighted RMS!!!
			double meanSquareValue = 0.0;
			double wiSum = 0.0; //we normalize the weights by their sum

			for (unsigned i = 0; i < data.cloud->size(); ++i)
			{
				ScalarType V = data.cloud->getPointScalarValue(i);
				if (ScalarField::ValidValue(V))
				{
					double wi = 1.0;
					if (coupleWeights)
					{
						ScalarType w = coupleWeights->getValue(i);
						if (!ScalarField::ValidValue(w))
							continue;
						wi = std::abs(w);
					}
					double Vd = wi * V;
					wiSum += wi * wi;
					meanSquareValue += Vd * Vd;
				}
			}

			//12/11/2008 - A.BEY: ICP guarantees only the decrease of the squared distances sum (not the distances sum)
			double meanSquareError = (wiSum != 0 ? static_cast<ScalarType>(meanSquareValue / wiSum) : 0);

			double rms = sqrt(meanSquareError);

			if (fTraceFile)
			{
				fprintf(fTraceFile, "%u; %f; %u;\n", iteration, rms, data.cloud->size());
			}

			if (iteration == 0)
			{
				//progress notification
				if (progressCb)
				{
					//on the first iteration, we init/show the dialog
					if (progressCb->textCanBeEdited())
					{
						progressCb->setMethodTitle("Registration");
						char buffer[32];
						snprintf(buffer, 32, "Initial RMS = %f", rms);
						progressCb->setInfo(buffer);
					}
					progressCb->update(0);
					progressCb->start();
				}

				finalRMS = rms;
				finalPointCount = data.cloud->size();

				if (LessThanEpsilon(rms))
				{
					//nothing to do
					result = ICP_NOTHING_TO_DO;
					break;
				}
			}
			else
			{
				assert(lastStepRMS >= 0.0);

				if (rms > lastStepRMS) //error increase!
				{
					result = iteration == 1 ? ICP_NOTHING_TO_DO : ICP_APPLY_TRANSFO;
					break;
				}

				//error update (RMS)
				double deltaRMS = lastStepRMS - rms;
				//should be better!
				assert(deltaRMS >= 0.0);

				//we update the global transformation matrix
				if (currentTrans.R.isValid())
				{
					if (transform.R.isValid())
						transform.R = currentTrans.R * transform.R;
					else
						transform.R = currentTrans.R;

					transform.T = currentTrans.R * transform.T;
				}

				if (params.adjustScale)
				{
					transform.s *= currentTrans.s;
					transform.T *= currentTrans.s;
				}

				transform.T += currentTrans.T;

				finalRMS = rms;
				finalPointCount = data.cloud->size();

				//stop criterion
				if (	(params.convType == MAX_ERROR_CONVERGENCE && deltaRMS < params.minRMSDecrease) //convergence reached
						||	(params.convType == MAX_ITER_CONVERGENCE && iteration >= params.nbMaxIterations) //max iteration reached
						)
				{
					result = ICP_APPLY_TRANSFO;
					break;
				}

				//progress notification
				if (progressCb)
				{
					if (progressCb->textCanBeEdited())
					{
						char buffer[64];
						snprintf(buffer, 64, (coupleWeights ? "Weighted RMS = %f [-%f]" : "RMS = %f [-%f]"), rms, deltaRMS);
						progressCb->setInfo(buffer);
					}
					if (iteration == 1)
					{
						initialDeltaRMS = deltaRMS;
						progressCb->update(0);
					}
					else
					{
						assert(initialDeltaRMS >= 0.0);
						float progressPercent = static_cast<float>((initialDeltaRMS - deltaRMS) / (initialDeltaRMS - params.minRMSDecrease)*100.0);
						progressCb->update(progressPercent);
					}
				}
			}

			lastStepRMS = rms;
		}

		//single iteration of the registration procedure
		currentTrans = ScaledTransformation();
		if (!RegistrationTools::RegistrationProcedure(	data.cloud,
														data.CPSetRef ? static_cast<GenericCloud*>(data.CPSetRef) : static_cast<GenericCloud*>(data.CPSetPlain),
														currentTrans,
														params.adjustScale,
														coupleWeights))
		{
			result = ICP_ERROR_REGISTRATION_STEP;
			break;
		}

		//restore original data sets (if any were stored)
		if (trueData.cloud)
		{
			cloudGarbage.destroy(data.cloud);
			if (data.CPSetRef)
				cloudGarbage.destroy(data.CPSetRef);
			else if (data.CPSetPlain)
				cloudGarbage.destroy(data.CPSetPlain);
			if (data.weights)
				sfGarbage.destroy(data.weights);
			data = trueData;
		}

		//shall we filter some components of the resulting transformation?
		if (params.transformationFilters != SKIP_NONE)
		{
			//filter translation (in place)
			FilterTransformation(currentTrans, params.transformationFilters, currentTrans);
		}

		//get rotated data cloud
		if (!data.rotatedCloud || pointOrderHasBeenChanged)
		{
			//we create a new structure, with rotated points
			PointCloud* rotatedDataCloud = PointProjectionTools::applyTransformation(data.cloud, currentTrans);
			if (!rotatedDataCloud)
			{
				//not enough memory
				result = ICP_ERROR_NOT_ENOUGH_MEMORY;
				break;
			}
			//replace data.rotatedCloud
			if (data.rotatedCloud)
				cloudGarbage.destroy(data.rotatedCloud);
			data.rotatedCloud = rotatedDataCloud;
			cloudGarbage.add(data.rotatedCloud);

			//update data.cloud
			data.cloud->clear();
			data.cloud->setAssociatedCloud(data.rotatedCloud);
			if (!data.cloud->addPointIndex(0, data.rotatedCloud->size()))
			{
				//not enough memory
				result = ICP_ERROR_NOT_ENOUGH_MEMORY;
				break;
			}
		}
		else
		{
			//we simply have to rotate the existing temporary cloud
			currentTrans.applyToGlobal(*data.rotatedCloud);
			data.rotatedCloud->invalidateBoundingBox(); //invalidate bb

			//DGM: warning, we must manually invalidate the ReferenceCloud bbox after rotation!
			data.cloud->invalidateBoundingBox();
		}

		//compute (new) distances to model
		if (inputModelMesh)
		{
			DistanceComputationTools::Cloud2MeshDistancesComputationParams c2mDistParams;
			c2mDistParams.octreeLevel = meshDistOctreeLevel;
			c2mDistParams.signedDistances = params.useC2MSignedDistances;
			c2mDistParams.CPSet = data.CPSetPlain;
			c2mDistParams.maxThreadCount = params.maxThreadCount;
			if (DistanceComputationTools::computeCloud2MeshDistances(data.cloud, inputModelMesh, c2mDistParams) < 0)
			{
				//an error occurred during distances computation...
				result = ICP_ERROR_REGISTRATION_STEP;
				break;
			}
		}
		else if (inputDataCloud)
		{
			DistanceComputationTools::Cloud2CloudDistancesComputationParams c2cDistParams;
			c2cDistParams.CPSet = data.CPSetRef;
			c2cDistParams.maxThreadCount = params.maxThreadCount;
			if (DistanceComputationTools::computeCloud2CloudDistances(data.cloud, model.cloud, c2cDistParams) < 0)
			{
				//an error occurred during distances computation...
				result = ICP_ERROR_REGISTRATION_STEP;
				break;
			}
		}
		else
		{
			assert(false);
		}
	}

	//end of tracefile
	if (fTraceFile)
	{
		fclose(fTraceFile);
		fTraceFile = nullptr;
	}

	//end of progress notification
	if (progressCb)
	{
		progressCb->stop();
	}

	return result;
}

bool HornRegistrationTools::FindAbsoluteOrientation(GenericCloud* lCloud,
													GenericCloud* rCloud,
													ScaledTransformation& trans,
													bool fixedScale/*=false*/)
{
	return RegistrationProcedure(lCloud, rCloud, trans, !fixedScale);
}

double HornRegistrationTools::ComputeRMS(GenericCloud* lCloud,
										 GenericCloud* rCloud,
										 const ScaledTransformation& trans)
{
	assert(rCloud && lCloud);
	if (!rCloud || !lCloud || rCloud->size() != lCloud->size() || rCloud->size() < 3)
		return false;

	double rms = 0.0;

	rCloud->placeIteratorAtBeginning();
	lCloud->placeIteratorAtBeginning();
	unsigned count = rCloud->size();

	for (unsigned i = 0; i < count; i++)
	{
		CCVector3d Ri = rCloud->getNextGlobalPoint();
		CCVector3d Li = lCloud->getNextGlobalPoint();
		CCVector3d Lit = trans.apply(Li);

		rms += (Ri - Lit).norm2();
	}

	return sqrt(rms / count);
}

bool RegistrationTools::RegistrationProcedure(	GenericCloud* P, //data
												GenericCloud* X, //model
												ScaledTransformation& trans,
												bool adjustScale/*=false*/,
												ScalarField* coupleWeights/*=nullptr*/,
												PointCoordinateType aPrioriScale/*=1.0f*/)
{
	//resulting transformation (R is invalid on initialization, T is (0,0,0) and s==1)
	trans.R.invalidate();
	trans.T = CCVector3d(0, 0, 0);
	trans.s = 1.0;

	if (P == nullptr || X == nullptr || P->size() != X->size() || P->size() < 3)
		return false;

	//centers of mass
	CCVector3 Gp = coupleWeights ? GeometricalAnalysisTools::ComputeWeightedLocalGravityCenter(P, coupleWeights) : GeometricalAnalysisTools::ComputeLocalGravityCenter(P);
	CCVector3 Gx = coupleWeights ? GeometricalAnalysisTools::ComputeWeightedLocalGravityCenter(X, coupleWeights) : GeometricalAnalysisTools::ComputeLocalGravityCenter(X);

	//specific case: 3 points only
	//See section 5.A in Horn's paper
	if (P->size() == 3)
	{
		//compute the first set normal
		P->placeIteratorAtBeginning();
		const CCVector3* Ap = P->getNextLocalPoint();
		const CCVector3* Bp = P->getNextLocalPoint();
		const CCVector3* Cp = P->getNextLocalPoint();
		CCVector3 Np(0, 0, 1);
		{
			Np = (*Bp - *Ap).cross(*Cp - *Ap);
			double norm = Np.normd();
			if (LessThanEpsilon(norm))
			{
				return false;
			}
			Np /= static_cast<PointCoordinateType>(norm);
		}
		//compute the second set normal
		X->placeIteratorAtBeginning();
		const CCVector3* Ax = X->getNextLocalPoint();
		const CCVector3* Bx = X->getNextLocalPoint();
		const CCVector3* Cx = X->getNextLocalPoint();
		CCVector3 Nx(0, 0, 1);
		{
			Nx = (*Bx - *Ax).cross(*Cx - *Ax);
			double norm = Nx.normd();
			if (LessThanEpsilon(norm))
			{
				return false;
			}
			Nx /= static_cast<PointCoordinateType>(norm);
		}
		//now the rotation is simply the rotation from Nx to Np, centered on Gx
		CCVector3 a = Np.cross(Nx);
		if (LessThanEpsilon(a.norm()))
		{
			trans.R = SquareMatrix(3);
			trans.R.toIdentity();
			if (Np.dot(Nx) < 0)
			{
				trans.R.scale(-PC_ONE);
			}
		}
		else
		{
			double cos_t = Np.dot(Nx);
			assert(cos_t > -1.0 && cos_t < 1.0); //see above
			double s = sqrt((1 + cos_t) * 2);
			double q[4] = { s / 2, a.x / s, a.y / s, a.z / s }; //don't forget to normalize the quaternion
			double qnorm = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
			assert( qnorm >= ZERO_TOLERANCE_D );
			qnorm = sqrt(qnorm);
			q[0] /= qnorm;
			q[1] /= qnorm;
			q[2] /= qnorm;
			q[3] /= qnorm;
			trans.R.initFromQuaternion(q);
		}

		if (adjustScale)
		{
			double sumNormP = (*Bp - *Ap).norm() + (*Cp - *Bp).norm() + (*Ap - *Cp).norm();
			sumNormP *= aPrioriScale;
			if (LessThanEpsilon(sumNormP))
			{
				return false;
			}
			double sumNormX = (*Bx - *Ax).norm() + (*Cx - *Bx).norm() + (*Ax - *Cx).norm();
			trans.s = static_cast<PointCoordinateType>(sumNormX / sumNormP); //sumNormX / (sumNormP * Sa) in fact
		}

		//we deduce the first translation
		trans.T = X->toGlobal(Gx) - (trans.R * P->toGlobal(Gp)) * (aPrioriScale*trans.s); //#26 in besl paper, modified with the scale as in jschmidt

		//we need to find the rotation in the (X) plane now
		{
			CCVector3 App = trans.apply(*Ap);
			CCVector3 Bpp = trans.apply(*Bp);
			CCVector3 Cpp = trans.apply(*Cp);

			CCVector3 rx = *Ax - Gx;
			CCVector3 rp = App - Gx;
			double C = rx.dot(rp);
			CCVector3 Ssum = rx.cross(rp);

			rx = *Bx - Gx;
			rp = Bpp - Gx;
			C += rx.dot(rp);
			Ssum += rx.cross(rp);

			rx = *Cx - Gx;
			rp = Cpp - Gx;
			C += rx.dot(rp);
			Ssum += rx.cross(rp);

			double S = Ssum.dot(Nx);
			double Q = sqrt(S*S + C*C);
			if (LessThanEpsilon(Q))
			{
				return false;
			}
			
			PointCoordinateType sin_t = static_cast<PointCoordinateType>(S / Q);
			PointCoordinateType cos_t = static_cast<PointCoordinateType>(C / Q);
			PointCoordinateType inv_cos_t = 1 - cos_t;

			const PointCoordinateType& l1 = Nx.x;
			const PointCoordinateType& l2 = Nx.y;
			const PointCoordinateType& l3 = Nx.z;

			PointCoordinateType l1_inv_cos_t = l1 * inv_cos_t;
			PointCoordinateType l3_inv_cos_t = l3 * inv_cos_t;

			SquareMatrix R(3);
			//1st column
			R.m_values[0][0] = cos_t + l1 * l1_inv_cos_t;
			R.m_values[0][1] = l2 * l1_inv_cos_t + l3 * sin_t;
			R.m_values[0][2] = l3 * l1_inv_cos_t - l2 * sin_t;

			//2nd column
			R.m_values[1][0] = l2 * l1_inv_cos_t - l3 * sin_t;
			R.m_values[1][1] = cos_t + l2 * l2*inv_cos_t;
			R.m_values[1][2] = l2 * l3_inv_cos_t + l1 * sin_t;

			//3rd column
			R.m_values[2][0] = l3 * l1_inv_cos_t + l2 * sin_t;
			R.m_values[2][1] = l2 * l3_inv_cos_t - l1 * sin_t;
			R.m_values[2][2] = cos_t + l3 * l3_inv_cos_t;

			trans.R = R * trans.R;
			trans.T = X->toGlobal(Gx) - (trans.R * P->toGlobal(Gp)) * (aPrioriScale*trans.s); //update T as well
		}
	}
	else
	{
		CCVector3 bbMin;
		CCVector3 bbMax;
		X->getLocalBoundingBox(bbMin, bbMax);

		//if the data cloud is equivalent to a single point (for instance
		//it's the case when the two clouds are very far away from
		//each other in the ICP process) we try to get the two clouds closer
		CCVector3 diag = bbMax - bbMin;
		if (LessThanEpsilon(std::abs(diag.x) + std::abs(diag.y) + std::abs(diag.z)))
		{
			trans.T = (Gx - Gp * aPrioriScale).toDouble();
			return true;
		}

		//Cross covariance matrix, eq #24 in Besl92 (but with weights, if any)
		SquareMatrixd Sigma_px = (coupleWeights ? GeometricalAnalysisTools::ComputeWeightedCrossCovarianceMatrix(P, X, Gp, Gx, coupleWeights)
												: GeometricalAnalysisTools::ComputeCrossCovarianceMatrix(P, X, Gp, Gx));
		if (!Sigma_px.isValid())
			return false;

#define USE_SVD
#ifdef USE_SVD

		SquareMatrixd U, S, V;
		if (!Sigma_px.svd(S, U, V))
			return false;
		SquareMatrixd UT = U.transposed();

		trans.R = V * UT;

#else
		//transpose sigma_px
		SquareMatrixd Sigma_px_t = Sigma_px.transposed();

		SquareMatrixd Aij = Sigma_px - Sigma_px_t;

		double trace = Sigma_px.trace(); //that is the sum of diagonal elements of sigma_px

		SquareMatrixd traceI3(3); //create the I matrix with eigvals equal to trace
		traceI3.m_values[0][0] = trace;
		traceI3.m_values[1][1] = trace;
		traceI3.m_values[2][2] = trace;

		SquareMatrixd bottomMat = Sigma_px + Sigma_px_t - traceI3;

		//we build up the registration matrix (see ICP algorithm)
		SquareMatrixd QSigma(4); //#25 in the paper (besl)

		QSigma.m_values[0][0] = trace;

		QSigma.m_values[0][1] = QSigma.m_values[1][0] = Aij.m_values[1][2];
		QSigma.m_values[0][2] = QSigma.m_values[2][0] = Aij.m_values[2][0];
		QSigma.m_values[0][3] = QSigma.m_values[3][0] = Aij.m_values[0][1];

		QSigma.m_values[1][1] = bottomMat.m_values[0][0];
		QSigma.m_values[1][2] = bottomMat.m_values[0][1];
		QSigma.m_values[1][3] = bottomMat.m_values[0][2];

		QSigma.m_values[2][1] = bottomMat.m_values[1][0];
		QSigma.m_values[2][2] = bottomMat.m_values[1][1];
		QSigma.m_values[2][3] = bottomMat.m_values[1][2];

		QSigma.m_values[3][1] = bottomMat.m_values[2][0];
		QSigma.m_values[3][2] = bottomMat.m_values[2][1];
		QSigma.m_values[3][3] = bottomMat.m_values[2][2];

		//we compute its eigenvalues and eigenvectors
		SquareMatrixd eigVectors;
		std::vector<double> eigValues;
		if (!Jacobi<double>::ComputeEigenValuesAndVectors(QSigma, eigVectors, eigValues, false))
		{
			//failure
			return false;
		}

		//as Besl says, the best rotation corresponds to the eigenvector associated to the biggest eigenvalue
		double qR[4];
		double maxEigValue = 0;
		Jacobi<double>::GetMaxEigenValueAndVector(eigVectors, eigValues, maxEigValue, qR);

		//these eigenvalue and eigenvector correspond to a quaternion --> we get the corresponding matrix
		trans.R.initFromQuaternion(qR);
#endif
		if (adjustScale)
		{
			//two accumulators
			double acc_num = 0.0;
			double acc_denom = 0.0;

			//now deduce the scale (refer to "Point Set Registration with Integrated Scale Estimation", Zinsser et. al, PRIP 2005)
			X->placeIteratorAtBeginning();
			P->placeIteratorAtBeginning();

			unsigned count = X->size();
			assert(P->size() == count);
			for (unsigned i = 0; i < count; ++i)
			{
				//'a' refers to the data 'A' (moving) = P
				//'b' refers to the model 'B' (not moving) = X
				CCVector3d a_tilde = trans.R * (*(P->getNextLocalPoint()) - Gp);	// a_tilde_i = R * (a_i - a_mean)
				CCVector3d b_tilde =           (*(X->getNextLocalPoint()) - Gx);	// b_tilde_j =     (b_j - b_mean)

				acc_num += b_tilde.dot(a_tilde);
				acc_denom += a_tilde.dot(a_tilde);
			}

			//DGM: acc_2 can't be 0 because we already have checked that the bbox is not a single point!
			assert(acc_denom > 0.0);
			trans.s = static_cast<PointCoordinateType>(std::abs(acc_num / acc_denom));
		}

		//and we deduce the translation
		trans.T = X->toGlobal(Gx) - (trans.R * P->toGlobal(Gp)) * (aPrioriScale*trans.s); //#26 in besl paper, modified with the scale as in jschmidt
	}

	return true;
}
