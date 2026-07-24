/*=============================================================================
 * Tools for manipulating nearest-neighbors graph defined on regular grids
 *
 * A grid graph in dimension D is defined by the following parameters:
 * D - the number of dimensions
 * shape - array of length D, giving the grid size in each dimension
 * connectivity - defines the neighboring relationship;
 *      corresponds to the _square_ of the maximum Euclidean distance between
 *      two neighbors;
 *      if less than 4, it defines the number of coordinates allowed
 *      to simultaneously vary (+1 or -1) to define a neighbor; in that case,
 *      each level l of connectivity in dimension D adds binom(D, l)*2^l
 *      neighbors; corresponding number of neighbors for D = 2 and 3:
 *
 *      connectivity |  1   2   3
 *      --------------------------
 *                2D |  4   8  (8)
 *                3D |  6  18  26
 *
 *      note that a connectivity of 4 or more includes neighbors whose
 *      coordinates might differ by 2 or more from the coordinates of the
 *      considered vertex. Interestingly, in dimension 4 or more, including
 *      all surrounding vertices would then also include vertices from a "more
 *      distant" surround: the neighbor v + (2, 0, 0, 0) is at the same
 *      distance as the neighbor v + (1, 1, 1, 1).
 *
 * A graph with V vertices and E edges is represented either as edge list
 * (array of E edges given as ordered pair of vertices), or as  forward-star,
 * where edges are numeroted so that all edges originating from a same vertex
 * are consecutive, and represented by the following parameters:
 * first_edge - array of length V + 1, indicating for each vertex, the first
 *      edge starting from the vertex (or, if there are none, starting from
 *      the next vertex); the first value is always zero and the last value is
 *      always the total number of edges
 * adj_vertices - array of length E, indicating for each edge, its ending
 *      vertex
 *
 * Vertices of the grid are indexed in _column-major_ order, that is indices
 * increase first along the first dimension specified in the 'shape' array
 * (in the usual convention in 2D, this corresponds to columns), and then along
 * the second dimension, and so on up to the last dimension.
 * Indexing in _row-major_ order (indices increase first along the last
 * dimension and so on up to the first) can be obtained by simply reverting
 * the order of the grid dimensions in the shape array (in 2D, this amounts to
 * transposition).
 *
 * Parallel implementation with OpenMP API
 *
 * Hugo Raguet 2019
 *===========================================================================*/
#pragma once
#include <cstddef>

/* vertex_t is supposed to be an unsigned integer type able to hold the total
 * number of _vertices_ of the manipulated graphs;
 * edge_t is supposed to be an unsigned integer type able to hold the total
 * number of _edges_ of the manipulated graphs */

/* compute the number of edges in the resulting graph */
template <typename vertex_t = unsigned int, typename conn_t = unsigned char>
size_t num_edges_grid_graph(size_t D, vertex_t* shape, conn_t connectivity);

/* compute the graph structure */
template <typename vertex_t = unsigned int, typename conn_t = unsigned char>
void edge_list_grid_graph(size_t D, vertex_t* shape, conn_t connectivity,
    vertex_t* edges, conn_t* connectivities = nullptr,
    vertex_t offset_u = 0, vertex_t offset_v = 0,
    conn_t recursive_connectivity = 0, bool recursive_call = false);
/* edges is an array of length twice the number of edges, already allocated;
 * connectivities is an array of length the number of edges, already allocated;
 * the number of edges can be found using the num_edges_grid_graph function;
 * connectivities are computed unless corresponding argument is null */

/* convert edge list to forward-star representation */
template <typename vertex_t = unsigned int, typename edge_t = vertex_t>
void edge_list_to_forward_star(vertex_t V, size_t E, const vertex_t* edges,
    edge_t* first_edge, edge_t* reindex);
/* first_edge is an array of length V + 1, already allocated;
 * reindex is the permutation indices so that all edges starting from a
 * same vertex are consecutive, array of length E, already allocated;
 * adj_vertices can be thus deduced from the edges by permuting the ending
 * vertices according to reindex */
