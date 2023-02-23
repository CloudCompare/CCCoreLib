// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//CCCoreLib
#include <CCGeom.h>

//STL
#include <vector>

//! 2D point with an associated scalar value
struct DataPoint : CCVector2d
{
	DataPoint()
		: CCVector2d()
		, value{ 0.0 }
	{}

	DataPoint(double x, double y, double _value)
		: CCVector2d(x, y)
		, value{ _value }
	{
	}

	double value;
};

//! Raster descriptor
class RasterParameters
{
public:

	RasterParameters()
		: minCorner(0.0, 0.0)
		, step(0.0)
		, width(0)
		, height(0)
	{
	}

	RasterParameters(	const CCVector2d& _minCorner,
						double _step,
						unsigned _width,
						unsigned _height )
		: minCorner(_minCorner)
		, step(_step)
		, width(_width)
		, height(_height)
	{
	}

	inline CCVector2d toPoint(unsigned x, unsigned y) const
	{
		return minCorner + CCVector2d(x * step, y * step);
	}

	CCVector2d minCorner;
	double step;
	unsigned width;
	unsigned height;
};

class CholeskyDecomposition;
struct OrdinaryKrigeContext;
struct SimpleKrigeContext;

//! Simple and Ordinary Kriging
/** Inspired by https://github.com/ByteShark/Kriging/
**/
class CC_CORE_LIB_API Kriging
{
	public:

		// Interpolation model
		enum Model
		{
			Spherical = 0,
			Exponential,
			Gaussian,
			Invalid
		};

		//! Vector type
		using Vector = std::vector<double>;
		//! Matrix type
		using Matrix = std::vector<Vector>;

		//! Constructor
		Kriging(const std::vector<DataPoint>& dataPoints,
				const RasterParameters& rasterParams);

		//! Parameters
		struct KrigeParams
		{
			KrigeParams(Model _model = Invalid,
						double _nugget = 0.0,
						double _sill = 0.0,
						double _range = 1.0)
				: model(_model)
				, nugget(_nugget)
				, sill(_sill)
				, range(_range)
			{
			}

			Model model;
			double nugget;
			double sill;
			double range;
		};

		//! Computes default parameters
		KrigeParams computeDefaultParameters() const;

		// Ordinary Kriging (all cells at once, returns a grid)
		bool ordinaryKrige(	const KrigeParams& params,
							unsigned knn,
							std::vector<DataPoint>& output);

		// Ordinary Kriging (cell by cell, with a 'context' object)
		double ordinaryKrigeSingleCell(	const KrigeParams& params,
										unsigned row,
										unsigned col,
										OrdinaryKrigeContext* context,
										bool alreadyHaveCandidates = false);

		// context management
		OrdinaryKrigeContext* createOrdinaryKrigeContext(int knn);
		void releaseOrdinaryKrigeContext(OrdinaryKrigeContext*& context);

protected:

		//! Fills a matrix of covariograms over all point distances
		Matrix calculateCovariogramMatrix(const std::vector<DataPoint>& dataPointCandidates, const KrigeParams& params, bool lagrangeMultiplier) const;

		//! Fills a vector of covariograms over all distances from a given point
		Vector calculateCovariogramVector(const std::vector<DataPoint>& dataPointCandidates, const CCVector2d& point, const KrigeParams& params, bool lagrangeMultiplier) const;

		//! Association of a point index and a (squared) distance
		struct SquareDistanceAndIndex
		{
			double sqDistance;
			size_t index;
		};

		//! Estimates the lag parameters
		void calculateDefaultLagParameters(double& lag, double& lagTolerance) const;
		
		//! Estimates the kriging parameters
		void calculateEstimatedParameters();

		//! Calculates the covariogram
		double calculateCovariogram(const KrigeParams& params, double distance) const;

		//! Simple linear regression
		/** \return slope and intercept
		**/
		std::pair<double, double> linearRegression(const Vector& X, const Vector& Y) const;

		//! Ordinary Kringing for an individual point
		double ordinaryKrigeForPoint(const CCVector2d& point, const KrigeParams& params,
			                         const std::vector<DataPoint>& dataPointCandidates);

	protected: // members

		const std::vector<DataPoint>& m_dataPoints;

		RasterParameters m_rasterParams;
};
