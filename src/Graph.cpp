/*=============================================================================
 * Hugo Raguet 2019
 *===========================================================================*/
#include <cstdint>

// Local
#include "GenericIndexedCloudPersist.h"
#include <ReferenceCloud.h>
#include "DgmOctree.h"
#include "Graph.h"
#include "OMPNumThreads.h"
#include <CutPursuit.h>
#include "cp_d0_dist.h"


using namespace CCCoreLib;

Graph::Graph(int32_t N, GenericIndexedCloudPersist* cloud, DgmOctree* octree)
    : m_N(N),
      m_cloud(cloud),
      m_octree(octree),
      m_edges(),
      m_distances()
{
}

void Graph::computeEdges(   int32_t knn, 
                            double knnRadius, 
                            std::function<void(int)> progressCb)
{
    if (m_cloud && m_octree)
    {
        unsigned char bestLevel = m_octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(knnRadius);
        CCCoreLib::ReferenceCloud neighbors(m_cloud);

        if (progressCb)
        {
            progressCb(0);
        }

        // compute edges
        for (int32_t i = 0; i < m_N; ++i)
        {
            neighbors.clear(false);
            double maxSquareDist = 0.0;
            int finalNeighbourhoodSize = 0;
            const CCVector3* queryPoint = m_cloud->getPoint(i);
            if (m_octree->findPointNeighbourhood(
                queryPoint,             	// Position we are searching around
                &neighbors,             	// Where the resulting neighbor indices will be stored
                knn,                   		// Max number of neighbors (k)
                bestLevel,               	// The optimized octree level we calculated
                maxSquareDist,          	// Output: The squared distance to the furthest neighbor found
                knnRadius,             		// Max search radius (r)
                &finalNeighbourhoodSize) 	// Output: Internal octree box search size metric (optional)
            )
            {
                int32_t source = static_cast<int32_t>(i);
                for (unsigned n = 0; n < neighbors.size(); ++n)
                {
                    int32_t target = static_cast<int32_t>(neighbors.getPointGlobalIndex(n));

                    // ignore self-loops
                    if (source == target)
                        continue;

                    m_edges.push_back(source); // 2*e
                    m_edges.push_back(target); // 2*e + 1

                    // compute distance
                    const CCVector3* p1 = m_cloud->getPoint(source);
                    const CCVector3* p2 = m_cloud->getPoint(target);
                    float distance = static_cast<float>((*p1 - *p2).norm());
                    m_distances.push_back(distance);
                }
            }

            if (progressCb)
            {
                progressCb(static_cast<int>(100.0 * i / m_N));
            }
        }
    }
}

void Graph::edgeListToForwardStar(int32_t V, size_t E, const int32_t* edges,
    int32_t* first_edge, int32_t* reindex)
{
    /* compute number of edges for each vertex and keep track of indices */
    for (int32_t v = 0; v < V; v++){ first_edge[v] = 0; }
    for (size_t e = 0; e < E; e++){ reindex[e] = first_edge[edges[2*e]]++; }

    /* compute cumulative sum and shift to the right */
    int32_t sum = 0; // first_edge[0] is always 0
    for (int32_t v = 0; v <= V; v++){
        int32_t tmp = first_edge[v];
        first_edge[v] = sum;
        sum += tmp;
    } // first_edge[V] should be total number of edges

    /* finalize reindex */
    #pragma omp parallel for NUM_THREADS(E)
    /* unsigned loop counter is allowed since OpenMP 3.0 (2008)
     * but MSVC compiler still does not support it as of 2020 */
    for (long long e = 0; e < (long long) E; e++){
        reindex[e] += first_edge[edges[2*e]];
    }
}

int Graph::partitionCutPursuit(
            int32_t D,
            std::vector<float> Y,
            std::vector<int32_t>& components,
            float regularization, 
            float spatialWeight, 
            int32_t cutoff,
            int32_t knn,
            double knnRadius,
            float cp_dif_tol,
            int cp_it_max,
            int K,
            int split_iter_num,
            float split_damp_ratio,
            int kmpp_init_num,
            int kmpp_iter_num,
            int verbose,
            int balance_parallel_split,
            int compute_Time,
            int compute_List,
            int compute_Graph,
            int compute_Obj,
            int compute_Dif,
            int max_num_threads,
            std::function<void(int)> progressCb)
{

    // more parallel cut pursuit params
    int32_t E = static_cast<int32_t>(m_edges.size() / 2);
    std::vector<float> edgeWeights(E);
    std::vector<int32_t> first_edge(m_N + 1);
    std::vector<int32_t> adj_vertices(E);
    std::vector<int32_t> reindex(E);
    std::vector<float> node_size(m_N, 1.0f);
    std::vector<float> coor_weights(D, 1.0f);

    // settings for parallel processing
    int32_t max_split_size = m_N;
    if (max_num_threads < 0)
    {
        max_num_threads = omp_get_max_threads();
    }

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
	int32_t* Comp = (int32_t*)calloc(m_N, sizeof(int32_t));
	if (!Comp)
	{
		return -1; // not enough memory
	}

    // compute CSR representation of the graph
	edgeListToForwardStar(
		m_N,
		E,
		m_edges.data(),
		first_edge.data(),
		reindex.data()
	);

    // apply spatial weight
	for (int32_t d = 0; d < 3; ++d)
	{
		coor_weights[d] *= spatialWeight;
	}

    // compute targets and edge weights based on distances in CSR order
	for (int32_t e = 0; e < E; ++e)
	{		
		// compute weight based on distance
		float distance = m_distances[e];
        float edgeAttr = distance;

        // The target vertex for original edge 'e' is stored at (2 * e + 1)
		adj_vertices[reindex[e]] = m_edges[2 * e + 1];
		
		// The weight for original edge 'e' maps to the same new position
		edgeWeights[reindex[e]] =  edgeAttr * regularization;
	}

    //  cut-pursuit with preconditioned forward-Douglas-Rachford
	Cp_d0_dist<float, int32_t, int32_t>* cp =
		new Cp_d0_dist<float, int32_t, int32_t>
			(m_N, E, first_edge.data(), adj_vertices.data(), Y.data(), D);

	cp->set_loss(static_cast<float>(D), Y.data(), node_size.data(), coor_weights.data());
	cp->set_edge_weights(edgeWeights.data(), regularization);
	cp->set_cp_param(cp_dif_tol, cp_it_max, verbose);
	cp->set_split_param(max_split_size, K, split_iter_num, split_damp_ratio,
		kmpp_init_num, kmpp_iter_num);
	cp->set_min_comp_weight(static_cast<float>(cutoff));
	cp->set_parallel_param(max_num_threads, balance_parallel_split);
	cp->set_monitoring_arrays(Obj, Time, Dif);
	cp->set_components(0, Comp);

	if (progressCb)
    {
        progressCb(0);
        int cp_it = cp->cut_pursuit(true, progressCb);
    }
    else
    {
        int cp_it = cp->cut_pursuit(true);
    }
    
    // Get number of components and their lists of indices
	const int32_t* comp_assign;
	const int32_t* first_vertex;
	const int32_t* comp_list;
	auto rV = cp->get_components(&comp_assign, &first_vertex, &comp_list);

    // Copy results
	for (int32_t i = 0; i < m_N; i++) {
		components[i] = static_cast<int32_t>(comp_assign[i]);
	}

    delete cp;
    return rV;
}
