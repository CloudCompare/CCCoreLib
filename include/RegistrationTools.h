// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//Local
#include "PointProjectionTools.h"


namespace CCCoreLib
{
	class GenericProgressCallback;
	class GenericCloud;
	class GenericIndexedMesh;
	class GenericIndexedCloud;
	class KDTree;
	class ScalarField;

	//! Common point cloud registration algorithms
	class CC_CORE_LIB_API RegistrationTools : public CCToolbox
	{
	public:

		//! Shortcut to PointProjectionTools::ScaledTransformation
		using ScaledTransformation = PointProjectionTools::Transformation;

		//! Transformation constraints
		enum TRANSFORMATION_FILTERS
		{
			SKIP_NONE			= 0,
			SKIP_RXY			= 1,
			SKIP_RYZ			= 2,
			SKIP_RXZ			= 4,
			SKIP_ROTATION		= 7,
			SKIP_TX				= 8,
			SKIP_TY				= 16,
			SKIP_TZ				= 32,
			SKIP_TRANSLATION	= 56,
		};

		//! 'Filters' a transformation by constraining it about some rotation axes and/or along some translation directions
		/**	\param inTrans input transformation
			\param transformationFilters filters to be applied on the resulting transformation at each step (experimental) - see RegistrationTools::TRANSFORMATION_FILTERS flags
			\param outTrans output transformation
		**/
		static void FilterTransformation(	const ScaledTransformation& inTrans,
											int transformationFilters,
											const CCVector3& toBeAlignedGravityCenter,
											const CCVector3& referenceGravityCenter,
											ScaledTransformation& outTrans );

		//! Computes RMS between two clouds given a transformation and a scale
		/** Warning: both clouds must have the same size (and at least 3 points)
			RMS = Sqrt ( Sum( square_norm( Pr - s*R*Pl+T ) ) / count )
			\param lCloud left cloud {Pl}
			\param rCloud right cloud {Pr}
			\param trans transformation: Pr = s.R.Pl + T
			\return RMS (or -1.0 if an error occurred)
		**/
		static double ComputeRMS(	GenericCloud* lCloud,
									GenericCloud* rCloud,
									const ScaledTransformation& trans);

	protected:

		//! ICP Registration procedure with optional scale estimation
		/** Determines the best quaternion (a couple qR|qT) and optionally
			a scale 's' (different from a priori scale Sa) to bring the cloud
			P closer to the reference cloud X (one step). Refer to the ICP
			algorithm theory for more details about this procedure, and to
			"Point Set Registration with Integrated Scale Estimation",
			Znisser et al, PRIP 2005 for the scale estimation.

				X = Sa.s.R.P + T (with Sa = s = 1 by default)

			Warning: P and X must have the same size, and must be in the same
			order (i.e. P[i] is the point equivalent to X[i] for all 'i').

			\param[in]  P the cloud to register (data)
			\param[in]  X the reference cloud (model)
			\param[in]  trans the resulting transformation
			\param[in]  adjustScale whether to estimate scale (s) as well (see jschmidt 2005)
			\param[in]  coupleWeights weights for each (Pi,Xi) couple (optional)
			\param[in]  aPrioriScale 'a priori' scale (Sa) between P and X
			\param[out] Gp optional: gravity center of the P cloud (potentially weighted)
			\param[out] Gx optional: gravity center of the X cloud (potentially weighted)
			\return success
		**/
		static bool RegistrationProcedure(	GenericCloud* P,
											GenericCloud* X,
											ScaledTransformation& trans,
											bool adjustScale = false,
											ScalarField* coupleWeights = nullptr,
											PointCoordinateType aPrioriScale = 1.0f,
											CCVector3* Gp = nullptr,
											CCVector3* Gx = nullptr);

	};

	//! Horn point cloud registration algorithm
	/** See 'Closed-form solution of absolute orientation using unit quaternions', B.K.P. Horn, 1987.
	**/
	class CC_CORE_LIB_API HornRegistrationTools : public RegistrationTools
	{
	public:

		//! Returns the transformation (scale + transformation) between two sets of (unordered) points
		/** \warning Both clouds must have the same size (and at least 3 points)
			\param[in] toBeAlignedPoints the points to be aligned
			\param[in] referencePoints the fixed/static points
			\param[out] trans resulting transformation: Pr = s.R.Pl + T
			\param[in] fixedScale force scale parameter to 1.0
			\return success
		**/
		static bool FindAbsoluteOrientation(GenericCloud* toBeAlignedPoints,
											GenericCloud* referencePoints,
											ScaledTransformation& trans,
											bool fixedScale = false);
	};

	//! ICP point cloud registration algorithm (Besl et al.).
	class CC_CORE_LIB_API ICPRegistrationTools : public RegistrationTools
	{
	public:

		//! Convergence control method
		enum CONVERGENCE_TYPE
		{
			MAX_ERROR_CONVERGENCE	= 0,
			MAX_ITER_CONVERGENCE	= 1,
		};

		//! Errors
		enum RESULT_TYPE
		{
			ICP_NOTHING_TO_DO				= 0,
			ICP_APPLY_TRANSFO				= 1,
			ICP_ERROR						= 100,
			//all errors should be greater than ICP_ERROR
			ICP_ERROR_REGISTRATION_STEP		= 101,
			ICP_ERROR_DIST_COMPUTATION		= 102,
			ICP_ERROR_NOT_ENOUGH_MEMORY		= 103,
			ICP_ERROR_CANCELED_BY_USER		= 104,
			ICP_ERROR_INVALID_INPUT			= 105,
		};

		//! Normals matching method
		enum NORMALS_MATCHING
		{
			NO_NORMAL				= 0,
			OPPOSITE_NORMALS		= 1,
			SAME_SIDE_NORMALS		= 2,
			DOUBLE_SIDED_NORMALS	= 3
		};

		//! ICP Parameters
		struct Parameters
		{
			Parameters()
				: convType(MAX_ERROR_CONVERGENCE)
				, minRMSDecrease(1.0e-5)
				, nbMaxIterations(20)
				, adjustScale(false)
				, filterOutFarthestPoints(false)
				, samplingLimit(50000)
				, finalOverlapRatio(1.0)
				, modelWeights(nullptr)
				, dataWeights(nullptr)
				, transformationFilters(SKIP_NONE)
				, maxThreadCount(0)
				, useC2MSignedDistances(false)
				, robustC2MSignedDistances(true)
				, normalsMatching(NO_NORMAL)
			{}

			//! Convergence type
			CONVERGENCE_TYPE convType;

			//! The minimum error (RMS) reduction between two consecutive steps to continue process (ignored if convType is not MAX_ERROR_CONVERGENCE)
			double minRMSDecrease;

			//! The maximum number of iteration (ignored if convType is not MAX_ITER_CONVERGENCE)
			unsigned nbMaxIterations;

			//! Whether to release the scale parameter during the registration procedure or not
			bool adjustScale;

			//! If true, the algorithm will automatically ignore farthest points from the reference, for better convergence
			bool filterOutFarthestPoints;

			//! Maximum number of points per cloud (they are randomly resampled below this limit otherwise)
			unsigned samplingLimit;

			//! Theoretical overlap ratio (at each iteration, only this percentage (between 0 and 1) will be used for registration
			double finalOverlapRatio;

			//! Weights for model points (i.e. only if the model entity is a cloud) (optional)
			ScalarField* modelWeights;

			//! Weights for data points (optional)
			ScalarField* dataWeights;

			//! Filters to be applied on the resulting transformation at each step (experimental) - see RegistrationTools::TRANSFORMATION_FILTERS flags
			int transformationFilters;

			//! Maximum number of threads to use (0 = max)
			int maxThreadCount;

			//! Whether to compute signed C2M distances.
			/** Useful when registering a cloud with a mesh AND partial overlap, to move the cloud towards the outside of the mesh
			**/
			bool useC2MSignedDistances;

			//! Whether to compute robust signed C2M distances.
			bool robustC2MSignedDistances;

			//! Normals matching method
			NORMALS_MATCHING normalsMatching;
		};

		//! Registers two clouds or a cloud and a mesh
		/** This method implements the ICP algorithm (Besl et al.) with various improvements (random sampling, optional weights, normals matching, etc.).
			\warning Be sure to activate an INPUT/OUTPUT scalar field on the point cloud.
			\warning The mesh is always the reference/model entity.
			\param modelCloud the reference cloud or the vertices of the reference mesh --> won't move
			\param modelMesh the reference mesh (optional) --> won't move
			\param dataCloud the cloud to register --> will move
			\param params ICP parameters
			\param[out] totalTrans the resulting transformation (once the algorithm has converged)
			\param[out] finalRMS final error (RMS)
			\param[out] finalPointCount number of points used to compute the final RMS
			\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
			\return algorithm result
		**/
		static RESULT_TYPE Register(	GenericIndexedCloudPersist* modelCloud,
										GenericIndexedMesh* modelMesh,
										GenericIndexedCloudPersist* dataCloud,
										const Parameters& params,
										ScaledTransformation& totalTrans,
										double& finalRMS,
										unsigned& finalPointCount,
										GenericProgressCallback* progressCb = nullptr);


	};
}
