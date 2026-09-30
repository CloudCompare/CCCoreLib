/*=============================================================================
 * Graph class for CloudCompare (Ioannis Farmakis, 2026)
 *
 * This class builds and manipulates a k-nearest-neighbor graph over a point
 * cloud (using CCCoreLib's octree for neighbor search), and provides
 * cut-pursuit based partitioning/segmentation over that graph.
 *
 *-----------------------------------------------------------------------------
 * Third-party integration notes
 *
 * The `edgeListToForwardStar()` method below is adapted from the
 * edge_list_to_forward_star().cpp file of the grid-graph project by
 * Hugo Raguet (https://github.com/1a7r0ch3/grid-graph).
 *
 * The method was de-templated (fixed vertex/edge index types)
 * for direct use with the data types required by CloudCompare.
 *
 * No other component, method, or documentation from the grid-graph project
 * applies to this class; the graph structure and connectivity used
 * elsewhere in this class are unrelated to the grid-graph project described
 * above, and are original to this class.
 *
 * The grid-graph project is distributed under the GNU General Public
 * License. As this file incorporates a derivative of one of its functions,
 * this file as a whole is likewise distributed under the GPL, and the
 * corresponding license notice is reproduced below, as required.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *-----------------------------------------------------------------------------
 *===========================================================================*/

#pragma once
#include <cstddef>

namespace CCCoreLib
{
    class GenericIndexedCloudPersist;
    class DgmOctree;

    class Graph
    {
    public:
        // constructor
        Graph(int32_t N, GenericIndexedCloudPersist* cloud, DgmOctree* octree);

        /* return the number of nodes in the graph */
        int32_t numNodes() const { return m_N; }
        /* return the number of edges in the graph */
        size_t numEdges() const { return static_cast<size_t>(m_edges.size() / 2); }

        /* compute edges of the graph
        * knn - number of nearest neighbors
        * knnRadius - radius for nearest neighbors search
        * progressCb - progress callback
        */
        void computeEdges(int32_t knn, double knnRadius, GenericProgressCallback* progressCb = nullptr);

        /* convert edge list to forward-star representation */
        /* adapted from the edge_list_to_forward_star().cpp file of the
         * grid-graph project by Hugo Raguet (https://github.com/1a7r0ch3/grid-graph) */ 
        void edgeListToForwardStar(int32_t V, size_t E, const int32_t* edges,
            int32_t* first_edge, int32_t* reindex);
        /* first_edge is an array of length V + 1, already allocated;
        * reindex is the permutation indices so that all edges starting from a
        * same vertex are consecutive, array of length E, already allocated;
        * adj_vertices can be thus deduced from the edges by permuting the ending
        * vertices according to reindex */

        int partitionCutPursuit(
            int32_t D,
            const std::vector<float>& Y,
            std::vector<int32_t>& components,
            float regularization,
            float spatialWeight,
            int32_t cutoff,
            float cp_dif_tol = 0.01f,
            int cp_it_max = 15,
            int K = 2,
            int split_iter_num = 2,
            float split_damp_ratio = 0.7f,
            int kmpp_init_num = 3,
            int kmpp_iter_num = 3,
            int verbose = 1000,
            int balance_parallel_split = false,
            int compute_Time = true,
            int compute_Obj = false,
            int compute_Dif = false,
            int max_num_threads = -1,
            GenericProgressCallback* progressCb = nullptr);

    protected:
        GenericIndexedCloudPersist* m_cloud;
        DgmOctree* m_octree;
        int32_t m_N; // number of nodes
        std::vector<int32_t> m_edges; // edge list representation of the graph
        std::vector<float> m_distances;
    };
}
