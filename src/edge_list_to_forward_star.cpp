/*=============================================================================
 * Hugo Raguet 2019
 *===========================================================================*/
#include <cstdint>
#include "grid_graph.h"
#include "omp_num_threads.h"

template <typename vertex_t, typename edge_t>
void edge_list_to_forward_star(vertex_t V, size_t E, const vertex_t* edges,
    edge_t* first_edge, edge_t* reindex)
{
    /* compute number of edges for each vertex and keep track of indices */
    for (vertex_t v = 0; v < V; v++){ first_edge[v] = 0; }
    for (size_t e = 0; e < E; e++){ reindex[e] = first_edge[edges[2*e]]++; }

    /* compute cumulative sum and shift to the right */
    edge_t sum = 0; // first_edge[0] is always 0
    for (vertex_t v = 0; v <= V; v++){
        edge_t tmp = first_edge[v];
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

/**  instantiate for compilation  **/

#define INSTANCE(vertex_t, edge_t) \
    template void edge_list_to_forward_star<vertex_t, edge_t> \
        (vertex_t, size_t, const vertex_t*, edge_t*, edge_t*);

INSTANCE(int32_t, int32_t)
