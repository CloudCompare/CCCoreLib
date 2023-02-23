// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include "../include/Kriging.h"

//System
#include <algorithm>
#include <array>
#include <assert.h>
#include <map>
#include <random>
#include <numeric>

//nanoflann
#include <nanoflann.hpp>

//! Cholesky matrix decomposition to lower triangular matrix and its conjugate transpose
/** Restricted to positive-definite matrices
**/
class CholeskyDecomposition
{
public:

	//! Constructor
	/** \warning Input matrix is decomposed in-place
	**/
	CholeskyDecomposition(const Kriging::Matrix& matrix)
		: m_matrix(matrix)
	{
		assert(matrix.size() > 0 && matrix.size() == matrix[0].size());
	}

	//! Decomposition into triangular matrices
	bool decompose()
	{
		// Enumerate matrix columnwise
		for (size_t j = 0; j < m_matrix.size(); ++j)
		{
			for (size_t i = j; i < m_matrix.size(); ++i)
			{
				if (i == j)
				{
					double sum = 0.0;

					for (size_t k = 0; k < i; ++k)
					{
						sum += m_matrix[i][k] * m_matrix[i][k];
					}

					if (m_matrix[i][j] - sum <= 0.0)
					{
						// Not positive definite matrix

						return false;
					}

					m_matrix[i][j] = std::sqrt(m_matrix[i][j] - sum);
				}
				else
				{
					double sum = 0.0;

					for (size_t k = 0; k < j; ++k)
					{
						sum += (m_matrix[i][k] * m_matrix[j][k]);
					}

					m_matrix[i][j] = (1 / m_matrix[j][j]) * (m_matrix[i][j] - sum);
					m_matrix[j][i] = m_matrix[i][j];
				}
			}
		}

		return true;
	}

	//! Solve Ax = b. A is the input matrix.
	Kriging::Vector solve(const Kriging::Vector& b)
	{
		Kriging::Vector y(b.size());

		// First solve lower triangular * y = b with forward substitution
		for (size_t i = 0; i < b.size(); ++i)
		{
			double sum = 0.0;

			for (size_t j = 0; j < i; ++j)
			{
				sum += (m_matrix[i][j] * y[j]);
			}

			y[i] = (b[i] - sum) / m_matrix[i][i];
		}

		// Now solve upper triangular (transpose of lower triangular) * x = y with back substitution.
		// Note that x can be solved in place using the existing y vector.  No need to allocate 
		// another vector.
		for (int i = static_cast<int>(b.size()) - 1; i >= 0; i--)
		{
			double sum = 0.0;

			for (int j = static_cast<int>(b.size()) - 1; j > i; j--)
			{
				sum += (m_matrix[i][j] * y[j]);
			}

			y[i] = (y[i] - sum) / m_matrix[i][i];
		}

		return y;
	}

protected:

	//! Decomposed matrix
	Kriging::Matrix m_matrix;
};

//! Lower/upper decomposition of matrix into a lower triangular matrix and a upper triangular matrix.
class LUDecomposition
{
public:

	//! Constructor
	/** \warning Input matrix is decomposed in-place
	**/
	LUDecomposition(Kriging::Matrix& matrix)
		: m_matrix(matrix)
	{
		assert(matrix.size() > 0 && matrix.size() == matrix[0].size());
	}

	//! Decomposition into triangular matrices
	bool decompose()
	{
		// Initialize the permutation vector
		size_t n = m_matrix.size();

		m_rowPermutation.reserve(n);
		for (size_t i = 0; i < n; ++i)
		{
			m_rowPermutation.push_back(static_cast<int>(i));
		}

		double det = 1.0;

		// LU factorization
		for (size_t p = 1; p <= n - 1; ++p)
		{
			// Find pivot element
			for (size_t i = p + 1; i <= n; ++i)
			{
				if (std::abs(m_matrix[m_rowPermutation[i - 1]][p - 1]) > std::abs(m_matrix[m_rowPermutation[p - 1]][p - 1]))
				{
					// Switch the index for the p-1 pivot row if necessary
					std::swap(m_rowPermutation[p - 1], m_rowPermutation[i - 1]);
					det = -det;
				}
			}

			if (m_matrix[m_rowPermutation[p - 1]][p - 1] == 0.0)
			{
				// The matrix is singular, at least to precision of algorithm
				return false;
			}

			// Multiply the diagonal elements
			det *= m_matrix[m_rowPermutation[p - 1]][p - 1];

			// Form multiplier
			for (size_t i = p + 1; i <= n; ++i)
			{
				m_matrix[m_rowPermutation[i - 1]][p - 1] /= m_matrix[m_rowPermutation[p - 1]][p - 1];

				// Eliminate [p-1]
				for (size_t j = p + 1; j <= n; ++j)
				{
					m_matrix[m_rowPermutation[i - 1]][j - 1] -= m_matrix[m_rowPermutation[i - 1]][p - 1] * m_matrix[m_rowPermutation[p - 1]][j - 1];
				}
			}
		}

		det *= m_matrix[m_rowPermutation[n - 1]][n - 1];

		return (det != 0.0);
	}

	//! Solve Ax = b. A is the input matrix.
	Kriging::Vector solve(const Kriging::Vector& b)
	{
		// The decomposed matrix is comprised of both the lower and upper diagonal matrices.

		// The rows of this matrix have been permutated during the decomposition process.  The
		// m_rowPermutation indicates the proper row order.

		// The lower diagonal matrix only includes elements below the diagonal with diagonal 
		// elements set to 1.

		// The upper diagonal matrix is fully specified.

		// First solve Ly = Pb for y using forward substitution. P is a permutated identity matrix.

		Kriging::Vector y(b.size());

		for (size_t i = 0; i < y.size(); ++i)
		{
			size_t currentRow = m_rowPermutation[i];

			double sum = 0.0;
			for (size_t j = 0; j < i; ++j)
			{
				sum += (m_matrix[currentRow][j] * y[j]);
			}

			y[i] = (b[currentRow] - sum);
		}

		// Now solve Ux = y for x using back substitution.
		// Note that x can be solved in place using the existing y vector. No need to allocate another vector.
		for (int i = static_cast<int>(b.size()) - 1; i >= 0; i--)
		{
			size_t currentRow = m_rowPermutation[i];

			double sum = 0.0;
			for (int j = static_cast<int>(b.size()) - 1; j > i; j--)
			{
				sum += (m_matrix[currentRow][j] * y[j]);
			}

			y[i] = (y[i] - sum) / m_matrix[currentRow][i];
		}

		return y;
	}

protected:

	// remove copy constructors/methods
	LUDecomposition(const LUDecomposition&) = delete;
	void operator=(const LUDecomposition&) = delete;

	// Decomposed matrix
	Kriging::Matrix& m_matrix;

	// Permutation of rows during pivoting
	std::vector<int> m_rowPermutation;
};

Kriging::Kriging(const std::vector<DataPoint>& dataPoints, const RasterParameters& rasterParams)
	: m_rasterParams(rasterParams)
	, m_dataPoints(dataPoints)
{
}

struct OrdinaryKrigeContext
{
	struct NFWrapper
	{
		NFWrapper(const std::vector<DataPoint>& dataPoints)
			: dataPointsRef(dataPoints)
		{}

		// Must return the number of data points
		inline size_t kdtree_get_point_count() const { return dataPointsRef.size(); }

		// Returns the dim'th component of the idx'th point in the class:
		// Since this is inlined and the "dim" argument is typically an immediate
		// value, the
		//  "if/else's" are actually solved at compile time.
		inline double kdtree_get_pt(size_t idx, size_t dim) const
		{
			if (dim == 0)
				return dataPointsRef[idx].x;
			else if (dim == 1)
				return dataPointsRef[idx].y;

			assert(false);
			return 0.0;
		}

		// Optional bounding-box computation: return false to default to a standard
		// bbox computation loop.
		//   Return true if the BBOX was already computed by the class and returned
		//   in "bb" so it can be avoided to redo it again. Look at bb.size() to
		//   find out the expected dimensionality (e.g. 2 or 3 for point clouds)
		template <class BBOX>
		bool kdtree_get_bbox(BBOX& /*bb*/) const
		{
			return false;
		}

		const std::vector<DataPoint>& dataPointsRef;
	};

	using KdTreeType = nanoflann::KDTreeSingleIndexAdaptor<
		nanoflann::L2_Simple_Adaptor<double, NFWrapper>,
		NFWrapper, /*dim=*/2>;

	OrdinaryKrigeContext(const std::vector<DataPoint>& _dataPoints)
		: nfWrapper(_dataPoints)
		, kdTree(nullptr)
		, knn(0)
	{
	}

	~OrdinaryKrigeContext()
	{
		delete kdTree;
		kdTree = nullptr;
	}

	bool prepare(int _knn)
	{
		if (_knn <= 0)
		{
			assert(false);
			return false;
		}

		size_t pointCount = nfWrapper.dataPointsRef.size();
		if (pointCount < static_cast<size_t>(knn))
		{
			// not enough data points
			return false;
		}

		knn = _knn;

		try
		{
			dataPointCandidates.resize(knn);
			kdIndexes.resize(knn);
			kdDistances.resize(knn);
		}
		catch (const std::bad_alloc&)
		{
			// not enough memory
			return false;
		}

		// eventually, instantiate the kd-tree
		kdTree = new KdTreeType(/*dim=*/2, nfWrapper, { /*max leaf=*/10 });

		return true;
	}

	bool getCandidates(const CCVector2d& query)
	{
		if (!kdTree)
		{
			assert(false);
			dataPointCandidates.clear();
			return false;
		}

		nanoflann::KNNResultSet<double> resultSet(knn);
		resultSet.init(kdIndexes.data(), kdDistances.data());
		nanoflann::SearchParams searchParams;
		searchParams.sorted = false;
		if (false == kdTree->findNeighbors(resultSet, query.u, searchParams))
		{
			// nanoflann failed to find the nearest neighbors
			assert(false);
			return false;
		}

		return updateCandidates();
	}

	bool updateCandidates()
	{
		if (kdIndexes.size() == dataPointCandidates.size())
		{
			for (int i = 0; i < kdIndexes.size(); ++i)
			{
				assert(kdIndexes[i] < nfWrapper.dataPointsRef.size());
				dataPointCandidates[i] = nfWrapper.dataPointsRef[kdIndexes[i]];
			}

			return true;
		}

		assert(false);
		return false;
	}

	NFWrapper nfWrapper;

	std::vector<DataPoint> dataPointCandidates;

	std::vector<size_t>		kdIndexes;
	std::vector<double>		kdDistances;
	KdTreeType*				kdTree;
	int						knn;

};

OrdinaryKrigeContext* Kriging::createOrdinaryKrigeContext(int knn)
{
	OrdinaryKrigeContext* context = new OrdinaryKrigeContext(m_dataPoints);
	if (!context->prepare(knn))
	{
		delete context;
		context = nullptr;
	}
	return context;
}

void Kriging::releaseOrdinaryKrigeContext(OrdinaryKrigeContext*& context)
{
	delete context;
	context = nullptr;
}

bool Kriging::ordinaryKrige(const KrigeParams& params,
							unsigned knn,
							std::vector<DataPoint>& output)
{
	if (m_dataPoints.empty())
	{
		// nothing to do
		assert(false);
		return false;
	}

	OrdinaryKrigeContext* context = createOrdinaryKrigeContext(knn);
	if (!context)
	{
		return false;
	}

	bool success = true;
	try
	{
		output.clear();
		output.reserve(m_rasterParams.width * m_rasterParams.height);

		for (unsigned i = 0; i < m_rasterParams.width; ++i)
		{
			for (unsigned j = 0; j < m_rasterParams.height; ++j)
			{
				CCVector2d point = m_rasterParams.toPoint(i, j);

				double estimatedValue = ordinaryKrigeSingleCell(params, i, j, context);

				output.push_back(DataPoint{ point.x, point.y, estimatedValue });
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		success = false;
	}

	releaseOrdinaryKrigeContext(context);

	return success;
}

double Kriging::ordinaryKrigeSingleCell(	const KrigeParams& params,
											unsigned row,
											unsigned col,
											OrdinaryKrigeContext* context,
											bool alreadyHaveCandidates/*=false*/)
{
	if (!context)
	{
		assert(false);
		return false;
	}

	assert(static_cast<int>(m_dataPoints.size()) > context->knn);

	try
	{
		// Current x, y values
		CCVector2d point = m_rasterParams.toPoint(row, col);

		if (!alreadyHaveCandidates)
		{
			if (!context->getCandidates(point))
			{
				return std::numeric_limits<double>::quiet_NaN();
			}
		}

		return ordinaryKrigeForPoint(point, params, context->dataPointCandidates);
	}
	catch (const std::bad_alloc&)
	{
		// not enough memory
	}

	return std::numeric_limits<double>::quiet_NaN();
}

double Kriging::ordinaryKrigeForPoint(const CCVector2d& point, const KrigeParams& params, const std::vector<DataPoint>& dataPointCandidates)
{
	// Find distances between all points and calculate covariograms
	Matrix distanceCovariogramMatrix = calculateCovariogramMatrix(dataPointCandidates, params, true);

	// decompose the covariogram matrix.  Because of the Lagrange multiplier, this will not be a positive definite matrix and we will
	// need to use LU decomposition.
	LUDecomposition luDecomposition(distanceCovariogramMatrix);
	if (!luDecomposition.decompose())
	{
		return std::numeric_limits<double>::quiet_NaN();
	}

	// Find distances over given points and calculate covariograms
	Vector distanceCovariogramVector = calculateCovariogramVector(dataPointCandidates, point, params, true);

	// Solve Ax = b, for x.  A represents the covariogram matrix of all distances and b is the covariogram vector for the current point.
	// x will be a vector of weights.
	Vector weights = luDecomposition.solve(distanceCovariogramVector);
	assert(!weights.empty());

	// Multiply the weights by the residuals to yield estimate for this point.
	return std::inner_product(weights.begin(), weights.end() - 1, dataPointCandidates.begin(), 0.0, std::plus<double>(), [](double weight, const DataPoint& dataPointCandidates)
		{
			return weight * dataPointCandidates.value;
		}
	);
}

Kriging::Vector Kriging::calculateCovariogramVector(const std::vector<DataPoint>& dataPointCandidates, const CCVector2d& point, const KrigeParams& params, bool lagrangeMultiplier) const
{
	Vector distanceVector(lagrangeMultiplier ? dataPointCandidates.size() + 1 : dataPointCandidates.size(), lagrangeMultiplier ? 1.0 : 0.0);

	for (size_t i = 0; i < dataPointCandidates.size(); ++i)
	{
		double distance = (dataPointCandidates[i] - point).norm();

		distanceVector[i] = calculateCovariogram(params, distance);
	}

	return distanceVector;
}

Kriging::Matrix Kriging::calculateCovariogramMatrix(const std::vector<DataPoint>& dataPointCandidates, const KrigeParams& params, bool lagrangeMultiplier) const
{
	Matrix distanceMatrix(lagrangeMultiplier ? dataPointCandidates.size() + 1 : dataPointCandidates.size(), Vector(lagrangeMultiplier ? dataPointCandidates.size() + 1 : dataPointCandidates.size(), lagrangeMultiplier ? 1.0 : 0.0));

	for (size_t i = 0; i < dataPointCandidates.size(); ++i)
	{
		distanceMatrix[i][i] = calculateCovariogram(params, 0.0);

		for (size_t j = i + 1; j < dataPointCandidates.size(); ++j)
		{
			double distance = (dataPointCandidates[i] - dataPointCandidates[j]).norm();

			double covariogram = calculateCovariogram(params, distance);

			distanceMatrix[i][j] = covariogram;
			distanceMatrix[j][i] = distanceMatrix[i][j];
		}
	}

	if (lagrangeMultiplier)
	{
		distanceMatrix[dataPointCandidates.size()][dataPointCandidates.size()] = 0.0;
	}

	return distanceMatrix;
}

std::pair<double, double> Kriging::linearRegression(const Vector& X, const Vector& Y) const
{
	double Xmean = std::accumulate(X.begin(), X.end(), 0.0) / X.size();
	double Ymean = std::accumulate(Y.begin(), Y.end(), 0.0) / Y.size();

	Vector::const_iterator Xi = X.begin();
	Vector::const_iterator Yi = Y.begin();

	double numerator = 0.0;
	double denominator = 0.0;

	while (Xi != X.end())
	{
		numerator += ((*Xi - Xmean) * (*Yi - Ymean));
		denominator += ((*Xi - Xmean) * (*Xi - Xmean));

		++Xi;
		++Yi;
	}

	double slope = std::numeric_limits<double>::max();
	double intercept = 0.0;
	if (denominator > std::numeric_limits<double>::epsilon())
	{
		slope = numerator / denominator;
		intercept = Ymean - (slope * Xmean);
	}

	return { slope, intercept };
}

Kriging::KrigeParams Kriging::computeDefaultParameters() const
{
	// Note: Determination of sill and range are only rough estimates.
	//       It is up to the user to interpret the empirical and model
	//       variogram plots to assess the validity of the parameters 
	//       used in the chosen model.

	// For fixed models, range is the first distance where the sill is reached.

	// For asymptotic models it would be the first distance where the semivariance
	// reaches 95% of the sill.

	// We will assume a fixed model to start.

	KrigeParams estimatedParams(Model::Invalid, 0.0, 0.0, 0.0);

	size_t pointCount = m_dataPoints.size();
	if (pointCount < 2)
	{
		assert(false);
		return estimatedParams;
	}

	estimatedParams.sill = 0.0;
	{
		double sum = 0.0;
		double sum2 = 0.0;
		for (const DataPoint& P : m_dataPoints)
		{
			sum += P.value;
			sum2 += P.value * P.value;
		}
		double mean = sum / m_dataPoints.size();
		estimatedParams.sill = ((sum2 / m_dataPoints.size()) - (mean * mean)) / 2; // semi-variance
	}

	std::multimap<double, double> semiVariogram;
	Vector lagDistance;
	Vector lagSemivariance;

	static const size_t MaxPointCount = 1000;
	bool duplicatePoints = false;
	//estimatedParams.sill = 0.0;
	if (pointCount <= MaxPointCount) // expensive version with an explicit computation of all possible distances!
	{
		for (size_t i = 0; i < pointCount; ++i)
		{
			for (size_t j = i + 1; j < pointCount; ++j)
			{
				double delta = m_dataPoints[i].value - m_dataPoints[j].value;
				double variance = delta * delta;

				//estimatedParams.sill += variance;

				double distance = (m_dataPoints[i] - m_dataPoints[j]).norm();
				if (distance > std::numeric_limits<double>::epsilon()) // ignore duplicate points!
				{
					semiVariogram.insert({ distance, variance });
				}
				else
				{
					duplicatePoints = true;
				}
			}
		}

		// Rather than take the simple variance of the variable, a better estimation should be the
		// average variance amoung all distances.
		//estimatedParams.sill = (estimatedParams.sill / 2.0) / ((pointCount * (pointCount - 1)) / 2.0);
		//estimatedParams.sill = estimatedParams.sill / (pointCount * (pointCount - 1));
	}
	else
	{
		std::mt19937 gen; //Standard mersenne_twister_engine seeded with default seed as we wan't a reproducible behavior
		std::uniform_int_distribution<> distrib(1, MaxPointCount - 1);

		size_t sampleCount = (MaxPointCount * (MaxPointCount - 1)) / 2;
		size_t sqrtSampleCount = static_cast<size_t>(sqrt(sampleCount));

		//estimatedParams.sill = 0.0;
		for (size_t iRand = 0; iRand < sqrtSampleCount; ++iRand)
		{
			int i = distrib(gen);
			for (size_t jRand = 0; jRand < sqrtSampleCount; ++jRand)
			{
				int shift = distrib(gen);
				assert(shift >= 1 && shift < static_cast<int>(MaxPointCount));
				int j = (i + shift) % static_cast<int>(MaxPointCount);
				assert(i != j);
				
				double delta = m_dataPoints[i].value - m_dataPoints[j].value;
				double variance = (delta * delta) / 2;

				//estimatedParams.sill += variance;

				double distance = (m_dataPoints[i] - m_dataPoints[j]).norm();
				if (distance > std::numeric_limits<double>::epsilon()) // ignore duplicate points!
				{
					semiVariogram.insert({ distance, variance });
				}
				else
				{
					duplicatePoints = true;
				}
			}
		}

		// Rather than take the simple variance of the variable, a better estimation should be the
		// average variance amoung all distances.
		//estimatedParams.sill = (estimatedParams.sill / 2.0) / ((semiVariogram.size() * (semiVariogram.size() - 1)) / 2.0);
		//estimatedParams.sill = estimatedParams.sill / (semiVariogram.size() * (semiVariogram.size() - 1));
	}

	if (duplicatePoints)
	{
		//Log::Warning("Kriging", QObject::tr("Duplicate points found. Results may be biased"));
	}

	// compute the data points bounding-box
	CCVector2d minBB, maxBB;
	for (size_t i = 0; i < pointCount; ++i)
	{
		const DataPoint& P = m_dataPoints[i];
		if (i != 0)
		{
			minBB.x = std::min(minBB.x, P.x);
			minBB.y = std::min(minBB.y, P.y);
			maxBB.x = std::max(maxBB.x, P.x);
			maxBB.y = std::max(maxBB.y, P.y);
		}
		else
		{
			minBB = maxBB = P;
		}
	}
	double bbDiag = (maxBB - minBB).norm();

	// estimate the default lag
	double lag = 0.0, lagBoundary = 0.0;
	{
		double minDistance = semiVariogram.begin()->first;
		double maxDistance = semiVariogram.rbegin()->first;

		lag = std::max(minDistance * 2.5, bbDiag / 1000.0); // we can't have a too small lag value!

		// Only consider points over half the distance.
		lagBoundary = (maxDistance - minDistance) / 2.0;

		unsigned lagCount = static_cast<unsigned>(lagBoundary / lag + 0.5);
		static const unsigned MINIMUM_LAG_COUNT = 20;
		if (lagCount < MINIMUM_LAG_COUNT)
		{
			lag = lagBoundary / MINIMUM_LAG_COUNT;
		}
	}

	double lagTolerance = lag / 2.0;
	double currentLagDistance = lagTolerance;
	double currentVarianceSum = 0.0;
	unsigned currentLagCount = 0;

	for (std::multimap<double, double>::const_iterator iterator = semiVariogram.begin(); iterator != semiVariogram.end() && currentLagDistance <= lagBoundary; ++iterator)
	{
		if (iterator->first > currentLagDistance + lagTolerance)
		{
			if (currentLagCount != 0)
			{
				lagDistance.push_back(currentLagDistance);
				lagSemivariance.push_back((currentVarianceSum / currentLagCount) / 2.0);

				currentVarianceSum = 0.0;
				currentLagCount = 0;
			}

			currentLagDistance += lag;
		}

		currentVarianceSum += iterator->second;
		++currentLagCount;
	}

	// Estimated range
	estimatedParams.range = 0.0;
	{
		// look for the maximum peak variance
		size_t peakVarianceIndex = 0;
		for (size_t i = 1; i< lagSemivariance.size(); ++i)
		{
			if (lagSemivariance[i] > lagSemivariance[peakVarianceIndex])
			{
				peakVarianceIndex = i;
			}
		}

		bool foundRangeLimit = false;
		for (size_t i = 0; i < lagSemivariance.size(); ++i)
		{
			double semivariance = lagSemivariance[i];
			
			if (semivariance > estimatedParams.sill)
			{
				estimatedParams.range = (i != 0 ? (lagDistance[i - 1] + lagDistance[i]) / 2.0 : lagDistance[i]);
				peakVarianceIndex = i;
				foundRangeLimit = true;
				break;
			}
		}

		if (!foundRangeLimit)
		{
			// let's use the peak instead
			estimatedParams.range = lagDistance[peakVarianceIndex];
		}

		// we can ignore the rest of the variogram for the fitting below
		lagSemivariance.resize(peakVarianceIndex + 1);
		lagDistance.resize(peakVarianceIndex + 1);
	}

	// Estimated nugget
	estimatedParams.nugget = 0.0;
	if (lagDistance.size() > 1)
	{
		auto slopeAndIntercept = linearRegression(lagDistance, lagSemivariance);
		estimatedParams.nugget = slopeAndIntercept.second; // = intercept
	}

	estimatedParams.model = Model::Spherical;

	return estimatedParams;
}

double Kriging::calculateCovariogram(const KrigeParams& params, double distance) const
{
	// Note: as linear models do not have a sill, it is not possible to calculate a covariogram.
	switch (params.model)
	{
	case Spherical:
		if (distance == 0.0)
		{
			return params.sill;
		}
		else if (distance <= params.range)
		{
			double q = distance / params.range;
			return params.sill * (1.0 - q * (1.5 - 0.5 * (q * q)));
		}
		else
		{
			return 0.0;
		}
		break;
	case Exponential:
		if (distance == 0.0)
		{
			return params.sill;
		}
		else
		{
			double q = distance / params.range;
			return (params.sill - params.nugget) * (std::exp(-q));
		}
		break;
	case Gaussian:
		if (distance == 0.0)
		{
			return params.sill;
		}
		else
		{
			double q = distance / params.range;
			return (params.sill - params.nugget) * (std::exp(-q * q));
		}
		break;
	default:
		assert(false);
		break;
	}

	return std::numeric_limits<double>::quiet_NaN();
}
