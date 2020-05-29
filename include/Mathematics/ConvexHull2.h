// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2020
// Distributed under the Boost Software License, Version 1.0.
// https://www.boost.org/LICENSE_1_0.txt
// https://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// Version: 4.0.2019.08.13

#pragma once

// Compute the convex hull of 2D points using a divide-and-conquer algorithm.
// This is an O(N log N) algorithm for N input points.  The only way to ensure
// a correct result for the input vertices (assumed to be exact) is to choose
// ComputeType for exact rational arithmetic.  You may use BSNumber.  No
// divisions are performed in this computation, so you do not have to use
// BSRational.
//
// The worst-case choices of N for Real of type BSNumber or BSRational with
// integer storage UIntegerFP32<N> are listed in the next table.  The numerical
// computations are encapsulated in PrimalQuery2<Real>::ToLineExtended.  We
// recommend using only BSNumber, because no divisions are performed in the
// convex-hull computations.
//
//    input type | compute type | N
//    -----------+--------------+------
//    float      | BSNumber     |   18
//    double     | BSNumber     |  132
//    float      | BSRational   |  214
//    double     | BSRational   | 1587

#include <Mathematics/Logger.h>
#include <Mathematics/PrimalQuery2.h>
#include <Mathematics/Line.h>
#include <vector>

// Uncomment this to assert when an infinite loop is encountered in
// ConvexHull2::GetTangent.
//#define GTE_THROW_ON_CONVEXHULL2_INFINITE_LOOP

namespace gte
{
    template <typename InputType, typename ComputeType>
    class ConvexHull2
    {
    public:
        // The class is a functor to support computing the convex hull of
        // multiple data sets using the same class object.
        ConvexHull2()
            :
            mEpsilon((InputType)0),
            mDimension(0),
            mLine(Vector2<InputType>::Zero(), Vector2<InputType>::Zero()),
            mNumPoints(0),
            mNumUniquePoints(0),
            mPoints(nullptr)
        {
        }

        // The input is the array of points whose convex hull is required.
        // The epsilon value is used to determine the intrinsic dimensionality
        // of the vertices (d = 0, 1, or 2).  When epsilon is positive, the
        // determination is fuzzy: points approximately the same point,
        // approximately on a line, or planar.  The return value is 'true' if
        // and only if the hull/ construction is successful.
        bool operator()(int numPoints, Vector2<InputType> const* points, InputType epsilon)
        {
            mEpsilon = std::max(epsilon, (InputType)0);
            mDimension = 0;
            mLine.origin = Vector2<InputType>::Zero();
            mLine.direction = Vector2<InputType>::Zero();
            mNumPoints = numPoints;
            mNumUniquePoints = 0;
            mPoints = points;
            mMerged.clear();
            mHull.clear();

            int i, j;
            if (mNumPoints < 3)
            {
                // ConvexHull2 should be called with at least three points.
                return false;
            }

            IntrinsicsVector2<InputType> info(mNumPoints, mPoints, mEpsilon);
            if (info.dimension == 0)
            {
                // mDimension is 0
                return false;
            }

            if (info.dimension == 1)
            {
                // The set is (nearly) collinear.
                mDimension = 1;
                mLine = Line2<InputType>(info.origin, info.direction[0]);
                return false;
            }

            mDimension = 2;

            // Compute the points for the queries.
            mComputePoints.resize(mNumPoints);
            mQuery.Set(mNumPoints, &mComputePoints[0]);
            for (i = 0; i < mNumPoints; ++i)
            {
                for (j = 0; j < 2; ++j)
                {
                    mComputePoints[i][j] = points[i][j];
                }
            }

            // Sort the points.
            mHull.resize(mNumPoints);
            for (i = 0; i < mNumPoints; ++i)
            {
                mHull[i] = i;
            }
            std::sort(mHull.begin(), mHull.end(),
                [points](int i0, int i1)
                {
                    if (points[i0][0] < points[i1][0])
                    {
                        return true;
                    }
                    if (points[i0][0] > points[i1][0])
                    {
                        return false;
                    }
                    return points[i0][1] < points[i1][1];
                }
            );

            // Remove duplicates.
            auto newEnd = std::unique(mHull.begin(), mHull.end(),
                [points](int i0, int i1)
                {
                    return points[i0] == points[i1];
                }
            );
            mHull.erase(newEnd, mHull.end());
            mNumUniquePoints = static_cast<int>(mHull.size());

            // Use a divide-and-conquer algorithm.  The merge step computes
            // the convex hull of two convex polygons.
            mMerged.resize(mNumUniquePoints);
            int i0 = 0, i1 = mNumUniquePoints - 1;
            GetHull(i0, i1);
            mHull.resize(i1 - i0 + 1);
            return true;
        }

        // Dimensional information.  If GetDimension() returns 1, the points
        // lie on a line P+t*D (fuzzy comparison when epsilon > 0).  You can
        // sort these if you need a polyline output by projecting onto the
        // line each vertex X = P+t*D, where t = Dot(D,X-P).
        inline InputType GetEpsilon() const
        {
            return mEpsilon;
        }

        inline int GetDimension() const
        {
            return mDimension;
        }

        inline Line2<InputType> const& GetLine() const
        {
            return mLine;
        }

        // Member access.
        inline int GetNumPoints() const
        {
            return mNumPoints;
        }

        inline int GetNumUniquePoints() const
        {
            return mNumUniquePoints;
        }

        inline Vector2<InputType> const* GetPoints() const
        {
            return mPoints;
        }

        inline PrimalQuery2<ComputeType> const& GetQuery() const
        {
            return mQuery;
        }

        // The convex hull is a convex polygon whose vertices are listed in
        // counterclockwise order.
        inline std::vector<int> const& GetHull() const
        {
            return mHull;
        }

    private:
        // Support for divide-and-conquer.
        void GetHull(int& i0, int& i1)
        {
            int numVertices = i1 - i0 + 1;
            if (numVertices > 1)
            {
                // Compute the middle index of input range.
                int mid = (i0 + i1) / 2;

                // Compute the hull of subsets (mid-i0+1 >= i1-mid).
                int j0 = i0, j1 = mid, j2 = mid + 1, j3 = i1;
                GetHull(j0, j1);
                GetHull(j2, j3);

                // Merge the convex hulls into a single convex hull.
                Merge(j0, j1, j2, j3, i0, i1);
            }
            // else: The convex hull is a single point.
        }

        void Merge(int j0, int j1, int j2, int j3, int& i0, int& i1)
        {
            // Subhull0 is to the left of subhull1 because of the initial
            // sorting of the points by x-components.  We need to find two
            // mutually visible points, one on the left subhull and one on
            // the right subhull.
            int size0 = j1 - j0 + 1;
            int size1 = j3 - j2 + 1;

            int i;
            Vector2<ComputeType> p;

            // Find the right-most point of the left subhull.
            Vector2<ComputeType> pmax0 = mComputePoints[mHull[j0]];
            int imax0 = j0;
            for (i = j0 + 1; i <= j1; ++i)
            {
                p = mComputePoints[mHull[i]];
                if (pmax0 < p)
                {
                    pmax0 = p;
                    imax0 = i;
                }
            }

            // Find the left-most point of the right subhull.
            Vector2<ComputeType> pmin1 = mComputePoints[mHull[j2]];
            int imin1 = j2;
            for (i = j2 + 1; i <= j3; ++i)
            {
                p = mComputePoints[mHull[i]];
                if (p < pmin1)
                {
                    pmin1 = p;
                    imin1 = i;
                }
            }

            // Get the lower tangent to hulls (LL = lower-left,
            // LR = lower-right).
            int iLL = imax0, iLR = imin1;
            GetTangent(j0, j1, j2, j3, iLL, iLR);

            // Get the upper tangent to hulls (UL = upper-left,
            // UR = upper-right).
            int iUL = imax0, iUR = imin1;
            GetTangent(j2, j3, j0, j1, iUR, iUL);

            // Construct the counterclockwise-ordered merged-hull vertices.
            int k;
            int numMerged = 0;

            i = iUL;
            for (k = 0; k < size0; ++k)
            {
                mMerged[numMerged++] = mHull[i];
                if (i == iLL)
                {
                    break;
                }
                i = (i < j1 ? i + 1 : j0);
            }
            LogAssert(k < size0, "Unexpected condition.");

            i = iLR;
            for (k = 0; k < size1; ++k)
            {
                mMerged[numMerged++] = mHull[i];
                if (i == iUR)
                {
                    break;
                }
                i = (i < j3 ? i + 1 : j2);
            }
            LogAssert(k < size1, "Unexpected condition.");

            int next = j0;
            for (k = 0; k < numMerged; ++k)
            {
                mHull[next] = mMerged[k];
                ++next;
            }

            i0 = j0;
            i1 = next - 1;
        }

        void GetTangent(int j0, int j1, int j2, int j3, int& i0, int& i1)
        {
            // In theory the loop terminates in a finite number of steps,
            // but the upper bound for the loop variable is used to trap
            // problems caused by floating-point roundoff errors that might
            // lead to an infinite loop.

            int size0 = j1 - j0 + 1;
            int size1 = j3 - j2 + 1;
            int const imax = size0 + size1;
            int i, iLm1, iRp1;
            Vector2<ComputeType> L0, L1, R0, R1;

            for (i = 0; i < imax; i++)
            {
                // Get the endpoints of the potential tangent.
                L1 = mComputePoints[mHull[i0]];
                R0 = mComputePoints[mHull[i1]];

                // Walk along the left hull to find the point of tangency.
                if (size0 > 1)
                {
                    iLm1 = (i0 > j0 ? i0 - 1 : j1);
                    L0 = mComputePoints[mHull[iLm1]];
                    auto order = mQuery.ToLineExtended(R0, L0, L1);
                    if (order == PrimalQuery2<ComputeType>::ORDER_NEGATIVE
                        || order == PrimalQuery2<ComputeType>::ORDER_COLLINEAR_RIGHT)
                    {
                        i0 = iLm1;
                        continue;
                    }
                }

                // Walk along right hull to find the point of tangency.
                if (size1 > 1)
                {
                    iRp1 = (i1 < j3 ? i1 + 1 : j2);
                    R1 = mComputePoints[mHull[iRp1]];
                    auto order = mQuery.ToLineExtended(L1, R0, R1);
                    if (order == PrimalQuery2<ComputeType>::ORDER_NEGATIVE
                        || order == PrimalQuery2<ComputeType>::ORDER_COLLINEAR_LEFT)
                    {
                        i1 = iRp1;
                        continue;
                    }
                }

                // The tangent segment has been found.
                break;
            }

            // Detect an "infinite loop" caused by floating point round-off
            // errors.
#if defined(GTE_THROW_ON_CONVEXHULL2_INFINITE_LOOP)
            LogAssert(i < imax, "Unexpected condition.");
#endif
        }

        // The epsilon value is used for fuzzy determination of intrinsic
        // dimensionality.  If the dimension is 0 or 1, the constructor
        // returns early.  The caller is responsible for retrieving the
        // dimension and taking an alternate path should the dimension be
        // smaller than 2.  If the dimension is 0, the caller may as well
        // treat all points[] as a single point, say, points[0].  If the
        // dimension is 1, the caller can query for the approximating line
        // and project points[] onto it for further processing.
        InputType mEpsilon;
        int mDimension;
        Line2<InputType> mLine;

        // The array of points used for geometric queries.  If you want to
        // be certain of a correct result, choose ComputeType to be BSNumber.
        std::vector<Vector2<ComputeType>> mComputePoints;
        PrimalQuery2<ComputeType> mQuery;

        int mNumPoints;
        int mNumUniquePoints;
        Vector2<InputType> const* mPoints;
        std::vector<int> mMerged, mHull;
    };
}
