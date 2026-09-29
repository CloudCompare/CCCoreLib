/*=============================================================================
 * Hugo Raguet 2019
 *===========================================================================*/
#include <cstdint>

// Local
#include "GenericIndexedCloudPersist.h"
#include <ReferenceCloud.h>
#include "DgmOctree.h"
#include "Graph.h"
#include "GenericProgressCallback.h"
#include "OMPNumThreads.h"
#include <CutPursuit.h>


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
                            GenericProgressCallback* progressCb)
{
    if (m_cloud && m_octree)
    {
        unsigned char bestLevel = m_octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(knnRadius);

        //progress notification (optional)
        if (progressCb)
        {
            if (progressCb->textCanBeEdited())
            {
                progressCb->setMethodTitle("Building graph");
                char infosBuffer[64];
                snprintf(infosBuffer, 64, "Computing %u edges from %i nodes", knn, m_N);
                progressCb->setInfo(infosBuffer);
            }
            progressCb->update(0);
            progressCb->start();
        }
        NormalizedProgress nprogress(progressCb, m_N, 100);

        // parallel compute pre-thread edges
        int numThreads = omp_get_max_threads();
        std::vector<std::vector<int32_t>> threadEdges(numThreads);
        std::vector<std::vector<float>> threadDistances(numThreads);
        
        // pre-allocate space per thread (assuming KNN neighbors per point)
        for (int t = 0; t < numThreads; ++t)
        {
            int pointsPerThread = (m_N + numThreads - 1) / numThreads;
            threadEdges[t].reserve(pointsPerThread * knn * 2);
            threadDistances[t].reserve(pointsPerThread * knn);
        }

        #pragma omp parallel
        {
            int threadId = omp_get_thread_num();
            CCCoreLib::ReferenceCloud neighbors(m_cloud);
            
            #pragma omp for schedule(dynamic, 64)
            for (int32_t i = 0; i < m_N; ++i)
            {
                neighbors.clear(false);
                double maxSquareDist = 0.0;
                int finalNeighbourhoodSize = 0;
                const CCVector3* queryPoint = m_cloud->getPoint(i);
                
                if (m_octree->findPointNeighbourhood(
                    queryPoint,
                    &neighbors,
                    knn,
                    bestLevel,
                    maxSquareDist,
                    knnRadius,
                    &finalNeighbourhoodSize))
                {
                    int32_t source = static_cast<int32_t>(i);
                    for (unsigned n = 0; n < neighbors.size(); ++n)
                    {
                        int32_t target = static_cast<int32_t>(neighbors.getPointGlobalIndex(n));

                        // ignore self-loops
                        if (source == target)
                            continue;

                        threadEdges[threadId].push_back(source);
                        threadEdges[threadId].push_back(target);

                        // compute distance
                        const CCVector3* p1 = m_cloud->getPoint(source);
                        const CCVector3* p2 = m_cloud->getPoint(target);
                        float distance = static_cast<float>((*p1 - *p2).norm());
                        threadDistances[threadId].push_back(distance);
                    }
                }

                // Progress update from master thread only
                if (progressCb && threadId == 0 && (i % 100 == 0))
                {
                    nprogress.oneStep();
                }
            }
        }
        
        // Concatenate results from all threads
        size_t totalEdges = 0;
        size_t totalDistances = 0;
        for (int t = 0; t < numThreads; ++t)
        {
            totalEdges += threadEdges[t].size();
            totalDistances += threadDistances[t].size();
        }
        
        m_edges.reserve(totalEdges);
        m_distances.reserve(totalDistances);
        
        for (int t = 0; t < numThreads; ++t)
        {
            m_edges.insert(m_edges.end(), threadEdges[t].begin(), threadEdges[t].end());
            m_distances.insert(m_distances.end(), threadDistances[t].begin(), threadDistances[t].end());
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
            const std::vector<float>& Y,
            std::vector<int32_t>& components,
            float regularization, 
            float spatialWeight, 
            int32_t cutoff,
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
            int compute_Obj,
            int compute_Dif,
            int max_num_threads,
            GenericProgressCallback* progressCb)
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
	CP* cp = new CP(m_N, E, first_edge.data(), adj_vertices.data(), Y.data(), D);

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
    components.resize(m_N);
	for (int32_t i = 0; i < m_N; i++) {
		components[i] = static_cast<int32_t>(comp_assign[i]);
	}

    delete cp;
    return rV;
}
