// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2020
// Distributed under the Boost Software License, Version 1.0.
// https://www.boost.org/LICENSE_1_0.txt
// https://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// Version: 4.0.2019.08.13

#pragma once

// Compute the convex hull of 3D points using incremental insertion.  The only
// way to ensure a correct result for the input vertices (assumed to be exact)
// is to choose ComputeType for exact rational arithmetic.  You may use
// BSNumber.  No divisions are performed in this computation, so you do not
// have to use BSRational.
//
// The worst-case choices of N for Real of type BSNumber or BSRational with
// integer storage UIntegerFP32<N> are listed in the next table.  The
// numerical computations are encapsulated in PrimalQuery3<Real>::ToPlane.
// We recommend using only BSNumber, because no divisions are performed in
// the convex-hull computations.
//
//    input type | compute type | N
//    -----------+--------------+------
//    float      | BSNumber     |    27
//    double     | BSNumber     |   197
//    float      | BSRational   |  2882
//    double     | BSRational   | 21688

#include <Mathematics/ETManifoldMesh.h>
#include <Mathematics/PrimalQuery3.h>
#include <Mathematics/Line.h>
#include <Mathematics/Hyperplane.h>
#include <functional>
#include <set>
#include <thread>

namespace gte
{
    template <typename InputType, typename ComputeType>
    class ConvexHull3
    {
    public:
        // The class is a functor to support computing the convex hull of
        // multiple data sets using the same class object.  For
        // multithreading in Update, choose 'numThreads' subject to the
        // constraints 1 <= numThreads <= std::thread::hardware_concurrency().
        ConvexHull3(unsigned int numThreads = 1)
            :
            mEpsilon((InputType)0),
            mDimension(0),
            mLine(Vector3<InputType>::Zero(), Vector3<InputType>::Zero()),
            mPlane(Vector3<InputType>::Zero(), (InputType)0),
            mNumPoints(0),
            mNumUniquePoints(0),
            mPoints(nullptr),
            mNumThreads(numThreads)
        {
        }

        // The input is the array of points whose convex hull is required.
        // The epsilon value is used to determine the intrinsic dimensionality
        // of the vertices (d = 0, 1, 2, or 3).  When epsilon is positive, the
        // determination is fuzzy--points approximately the same point,
        // approximately on a line, approximately planar, or volumetric.
        bool operator()(int numPoints, Vector3<InputType> const* points, InputType epsilon)
        {
            mEpsilon = std::max(epsilon, (InputType)0);
            mDimension = 0;
            mLine.origin = Vector3<InputType>::Zero();
            mLine.direction = Vector3<InputType>::Zero();
            mPlane.normal = Vector3<InputType>::Zero();
            mPlane.constant = (InputType)0;
            mNumPoints = numPoints;
            mNumUniquePoints = 0;
            mPoints = points;
            mHullUnordered.clear();
            mHullMesh.Clear();

            int i, j;
            if (mNumPoints < 4)
            {
                // ConvexHull3 should be called with at least four points.
                return false;
            }

            IntrinsicsVector3<InputType> info(mNumPoints, mPoints, mEpsilon);
            if (info.dimension == 0)
            {
                // The set is (nearly) a point.
                return false;
            }

            if (info.dimension == 1)
            {
                // The set is (nearly) collinear.
                mDimension = 1;
                mLine = Line3<InputType>(info.origin, info.direction[0]);
                return false;
            }

            if (info.dimension == 2)
            {
                // The set is (nearly) coplanar.
                mDimension = 2;
                mPlane = Plane3<InputType>(UnitCross(info.direction[0],
                    info.direction[1]), info.origin);
                return false;
            }

            mDimension = 3;

            // Compute the vertices for the queries.
            mComputePoints.resize(mNumPoints);
            mQuery.Set(mNumPoints, &mComputePoints[0]);
            for (i = 0; i < mNumPoints; ++i)
            {
                for (j = 0; j < 3; ++j)
                {
                    mComputePoints[i][j] = points[i][j];
                }
            }

            // Insert the faces of the (nondegenerate) tetrahedron
            // constructed by the call to GetInformation.
            if (!info.extremeCCW)
            {
                std::swap(info.extreme[2], info.extreme[3]);
            }

            mHullUnordered.push_back(TriangleKey<true>(info.extreme[1],
                info.extreme[2], info.extreme[3]));
            mHullUnordered.push_back(TriangleKey<true>(info.extreme[0],
                info.extreme[3], info.extreme[2]));
            mHullUnordered.push_back(TriangleKey<true>(info.extreme[0],
                info.extreme[1], info.extreme[3]));
            mHullUnordered.push_back(TriangleKey<true>(info.extreme[0],
                info.extreme[2], info.extreme[1]));

            // Incrementally update the hull.  The set of processed points is
            // maintained to eliminate duplicates, either in the original
            // input points or in the points obtained by snap rounding.
            std::set<Vector3<InputType>> processed;
            for (i = 0; i < 4; ++i)
            {
                processed.insert(points[info.extreme[i]]);
            }
            for (i = 0; i < mNumPoints; ++i)
            {
                if (processed.find(points[i]) == processed.end())
                {
                    Update(i);
                    processed.insert(points[i]);
                }
            }
            mNumUniquePoints = static_cast<int>(processed.size());
            return true;
        }

        // Dimensional information.  If GetDimension() returns 1, the points
        // lie on a line P+t*D (fuzzy comparison when epsilon > 0).  You can
        // sort these if you need a polyline output by projecting onto the
        // line each vertex X = P+t*D, where t = Dot(D,X-P). If GetDimension()
        // returns 2, the points line on a plane P+s*U+t*V (fuzzy comparison
        // when epsilon > 0).  You can project each point X = P+s*U+t*V, where
        // s = Dot(U,X-P) and t = Dot(V,X-P), then apply ConvexHull2 to the
        // (s,t) tuples.
        inline InputType GetEpsilon() const
        {
            return mEpsilon;
        }

        inline int GetDimension() const
        {
            return mDimension;
        }

        inline Line3<InputType> const& GetLine() const
        {
            return mLine;
        }

        inline Plane3<InputType> const& GetPlane() const
        {
            return mPlane;
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

        inline Vector3<InputType> const* GetPoints() const
        {
            return mPoints;
        }

        inline PrimalQuery3<ComputeType> const& GetQuery() const
        {
            return mQuery;
        }

        // The convex hull is a convex polyhedron with triangular faces.
        inline std::vector<TriangleKey<true>> const& GetHullUnordered() const
        {
            return mHullUnordered;
        }

        ETManifoldMesh const& GetHullMesh() const
        {
            // Create the mesh only on demand.
            if (mHullMesh.GetTriangles().size() == 0)
            {
                for (auto const& tri : mHullUnordered)
                {
                    mHullMesh.Insert(tri.V[0], tri.V[1], tri.V[2]);
                }
            }

            return mHullMesh;
        }

    private:
        // Support for incremental insertion.
        void Update(int i)
        {
            // The terminator that separates visible faces from nonvisible
            // faces is constructed by this code.  Visible faces for the
            // incoming hull are removed, and the boundary of that set of
            // triangles is the terminator.  New visible faces are added
            // using the incoming point and the edges of the terminator.
            //
            // A simple algorithm for computing terminator edges is the
            // following.  Back-facing triangles are located and the three
            // edges are processed.  The first time an edge is visited,
            // insert it into the terminator.  If it is visited a second time,
            // the edge is removed because it is shared by another back-facing
            // triangle and, therefore, cannot be a terminator edge.  After
            // visiting all back-facing triangles, the only remaining edges in
            // the map are the terminator edges.
            //
            // The order of vertices of an edge is important for adding new
            // faces with the correct vertex winding.  However, the edge
            // "toggle" (insert edge, remove edge) should use edges with
            // unordered vertices, because the edge shared by one triangle has
            // opposite ordering relative to that of the other triangle.  The
            // map uses unordered edges as the keys but stores the ordered
            // edge as the value.  This avoids having to look up an edge twice
            // in a map with ordered edge keys.

            unsigned int numFaces = static_cast<unsigned int>(mHullUnordered.size());
            std::vector<int> queryResult(numFaces);
            if (mNumThreads > 1 && numFaces >= mNumThreads)
            {
                // Partition the data for multiple threads.
                unsigned int numFacesPerThread = numFaces / mNumThreads;
                std::vector<unsigned int> jmin(mNumThreads), jmax(mNumThreads);
                for (unsigned int t = 0; t < mNumThreads; ++t)
                {
                    jmin[t] = t * numFacesPerThread;
                    jmax[t] = jmin[t] + numFacesPerThread - 1;
                }
                jmax[mNumThreads - 1] = numFaces - 1;

                // Execute the point-plane queries in multiple threads.
                std::vector<std::thread> process(mNumThreads);
                for (unsigned int t = 0; t < mNumThreads; ++t)
                {
                    process[t] = std::thread([this, i, t, &jmin, &jmax,
                        &queryResult]()
                        {
                            for (unsigned int j = jmin[t]; j <= jmax[t]; ++j)
                            {
                                TriangleKey<true> const& tri = mHullUnordered[j];
                                queryResult[j] = mQuery.ToPlane(i, tri.V[0], tri.V[1], tri.V[2]);
                            }
                        });
                }

                // Wait for all threads to finish.
                for (unsigned int t = 0; t < mNumThreads; ++t)
                {
                    process[t].join();
                }
            }
            else
            {
                unsigned int j = 0;
                for (auto const& tri : mHullUnordered)
                {
                    queryResult[j++] = mQuery.ToPlane(i, tri.V[0], tri.V[1], tri.V[2]);
                }
            }

            std::map<EdgeKey<false>, std::pair<int, int>> terminator;
            std::vector<TriangleKey<true>> backFaces;
            bool existsFrontFacingTriangle = false;
            unsigned int j = 0;
            for (auto const& tri : mHullUnordered)
            {
                if (queryResult[j++] <= 0)
                {
                    // The triangle is back facing.  These include triangles
                    // that are coplanar with the incoming point.
                    backFaces.push_back(tri);

                    // The current hull is a 2-manifold watertight mesh.  The
                    // terminator edges are those shared with a front-facing
                    // triangle.  The first time an edge of a back-facing
                    // triangle is visited, insert it into the terminator.  If
                    // it is visited a second time, the edge is removed
                    // because it is shared by another back-facing triangle.
                    // After all back-facing triangles are visited, the only
                    // remaining edges are shared by a single back-facing
                    // triangle, which makes them terminator edges.
                    for (int j0 = 2, j1 = 0; j1 < 3; j0 = j1++)
                    {
                        int v0 = tri.V[j0], v1 = tri.V[j1];
                        EdgeKey<false> edge(v0, v1);
                        auto iter = terminator.find(edge);
                        if (iter == terminator.end())
                        {
                            // The edge is visited for the first time.
                            terminator.insert(std::make_pair(edge, std::make_pair(v0, v1)));
                        }
                        else
                        {
                            // The edge is visited for the second time.
                            terminator.erase(edge);
                        }
                    }
                }
                else
                {
                    // If there are no strictly front-facing triangles, then
                    // the incoming point is inside or on the convex hull.  If
                    // we get to this code, then the point is truly outside
                    // and we can update the hull.
                    existsFrontFacingTriangle = true;
                }
            }

            if (!existsFrontFacingTriangle)
            {
                // The incoming point is inside or on the current hull, so no
                // update of the hull is necessary.
                return;
            }

            // The updated hull contains the triangles not visible to the
            // incoming point.
            mHullUnordered = backFaces;

            // Insert the triangles formed by the incoming point and the
            // terminator edges.
            for (auto const& edge : terminator)
            {
                mHullUnordered.push_back(TriangleKey<true>(i, edge.second.second, edge.second.first));
            }
        }

        // The epsilon value is used for fuzzy determination of intrinsic
        // dimensionality.  If the dimension is 0, 1, or 2, the constructor
        // returns early.  The caller is responsible for retrieving the
        // dimension and taking an alternate path should the dimension be
        // smaller than 3.  If the dimension is 0, the caller may as well
        // treat all points[] as a single point, say, points[0].  If the
        // dimension is 1, the caller can query for the approximating line
        // and project points[] onto it for further processing.  If the
        // dimension is 2, the caller can query for the approximating plane
        // and project points[] onto it for further processing.
        InputType mEpsilon;
        int mDimension;
        Line3<InputType> mLine;
        Plane3<InputType> mPlane;

        // The array of points used for geometric queries.  If you want to be
        // certain of a correct result, choose ComputeType to be BSNumber.
        std::vector<Vector3<ComputeType>> mComputePoints;
        PrimalQuery3<ComputeType> mQuery;

        int mNumPoints;
        int mNumUniquePoints;
        Vector3<InputType> const* mPoints;
        std::vector<TriangleKey<true>> mHullUnordered;
        mutable ETManifoldMesh mHullMesh;
        unsigned int mNumThreads;
    };
}
