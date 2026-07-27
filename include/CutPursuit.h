/*=============================================================================
 * Base class for cut-pursuit algorithm
 * 
 * L. Landrieu and G. Obozinski, Cut Pursuit: Fast Algorithms to Learn 
 * Piecewise Constant Functions on General Weighted Graphs, SIAM Journal on 
 * Imaging Sciences, 2017, 10, 1724-1766
 *
 * Hugo Raguet 2018, 2020, 2022
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
 *===========================================================================*/
#pragma once

//Local
#include "OMPNumThreads.h"
#include "Maxflow.h"

//system
#include <cstdint> // for uintmax_t, requires C++11
#include <cstdlib> // for size_t, malloc, exit
#include <chrono>
#include <limits>
#include <functional>
#include <iostream>

/* real_t is the real numeric type, used for objective functional computation
 * and thus for edge weights and flow graph capacities;
 * index_t is an integer type able to hold the number of vertices and of edges
 * in the main graph;
 * comp_t is an integer type able to hold the maximum number of constant
 * connected components in the reduced graph;
 * value_t is the type associated to the space to which the values belong, it
 * is usually real_t, and if multidimensional, this must be specified in the
 * parameter D (e.g. for R^3, specify value_t = real_t and D = 3) */
template <typename real_t, typename index_t, typename comp_t,
    typename value_t = real_t>
class Cp
{
public:
    /**  constructor, destructor  **/

    Cp(index_t V, index_t E, const index_t* first_edge, 
        const index_t* adj_vertices, size_t D = 1);

    /* the destructor does not free pointers which are supposed to be provided 
     * by the user (forward-star graph structure given at construction, 
     * monitoring arrays, etc.); IT DOES FREE THE REST (components assignment 
     * and reduced problem elements, etc.), but this can be prevented by
     * getting the corresponding pointer member and setting it to null
     * beforehand */
    virtual ~Cp();

    /**  methods for manipulating parameters  **/

    void reset_edges(); // bind all edges

    /* if 'edge_weights' is null, homogeneously equal to 'homo_edge_weight' */
    void set_edge_weights(const real_t* edge_weights = nullptr,
        real_t homo_edge_weight = 1.0);

    void set_monitoring_arrays(real_t* objective_values = nullptr,
        double* elapsed_time = nullptr, real_t* iterate_evolution = nullptr);

    /* if rV is zero or unity, comp_assign will be automatically initialized;
     * if rV is zero, arbitrary components will be assigned at initialization,
     * in an attempt to optimize parallelization along components;
     * if rV is greater than one, comp_assign must be given and initialized;
     * comp_assign is free()'d by destructor, unless set to null beforehand */
    void set_components(comp_t rV = 0, comp_t* comp_assign = nullptr);

    void set_cp_param(real_t dif_tol, int it_max, int verbose, real_t eps);
    /* overload for default eps parameter */
    void set_cp_param(real_t dif_tol = 0.0, int it_max = 10,
        int verbose = 1000)
    {
        set_cp_param(dif_tol, it_max, verbose,
            std::numeric_limits<real_t>::epsilon());
    }

    /* tune split parameters; set max_split_size to V for no max */
    void set_split_param(index_t max_split_size, comp_t K = 2,
        int split_iter_num = 1, real_t split_damp_ratio = 1.0,
        int split_values_init_num = 1, int split_values_iter_num = 1);

    void set_parallel_param(int max_num_threads,
        bool balance_parallel_split = true);
    /* overload for default max_num_threads parameter */
    void set_parallel_param(bool balance_parallel_split)
    {
        set_parallel_param(omp_get_max_threads(), balance_parallel_split);
    }

    /* the 'get' methods takes pointers to pointers as arguments; a null means
     * that the user is not interested by the corresponding pointer; NOTA:
     * 1) if not explicitely set by the user, memory pointed by these members
     * is allocated using malloc(), and thus should be deleted with free()
     * 2) they are free()'d by destructor, unless set to null beforehand */

    comp_t get_components(const comp_t** comp_assign = nullptr,
        const index_t** first_vertex = nullptr,
        const index_t** comp_list = nullptr) const;

    /* return the number of reduced edges */
    index_t get_reduced_graph(const comp_t** reduced_edges = nullptr,
        const real_t** reduced_edge_weights = nullptr);

    /* retrieve the reduced iterate (values of the components);
     * WARNING: reduced values are free()'d by destructor */
    const value_t* get_reduced_values() const;

    /* set the reduced iterate (values of the components);
     * WARNING: if not set to null before deletion of the main cp object,
     * this will be deleted by free() so the given pointer must have been
     * allocated with malloc() and the likes */
    void set_reduced_values(value_t* rX);

    /* solve the main problem */
    int cut_pursuit(bool init, std::function<void(int)> progressCb);

protected:
    /**  main graph  **/

    const index_t V, E; // number of vertices, of edges

    /**  forward-star graph representation  **/
    /* - edges are numeroted so that all edges originating from a same vertex
     * are consecutive;
     * - for each vertex, 'first_edge' indicates the first edge starting
     * from the vertex (or, if there are none, starting from the next vertex);
     * array of length V + 1, the first value is always zero and the last
     * value is always the total number of edges E
     * - for each edge, 'adj_vertices' indicates its ending vertex */
    const index_t *first_edge, *adj_vertices; 
    
    const real_t *edge_weights; // array of length E, weights of edges
    real_t homo_edge_weight; // homogeneous weights, set edge_weights to null

    /* dimension of the data; total size signal is V*D */
    const size_t D;

    /**  reduced graph  **/

    /* last_* are used to identify saturated components and to compute 
     * iterate evolution */
    comp_t rV, last_rV; // number of components (reduced vertices)
    value_t *rX, *last_rX; // reduced iterate (values of the components)
    index_t rE; // number of reduced edges
    /* assignment of each vertex to a component */
    comp_t* comp_assign, *last_comp_assign;
    /* list the vertices of each components:
     * - vertices are gathered in 'comp_list' so that all vertices belonging
     * to a same components are consecutive
     * - for each component, 'first_vertex' indicates the index of its first
     * vertex in 'comp_list' */
    index_t *comp_list, *first_vertex;
    /* reverse mapping of comp list: index of a given vertex within its
     * components (useful for working within components in parallel) */
    index_t *index_in_comp;
    /* components saturation */
    bool* is_saturated;
    comp_t saturated_comp; // number of saturated components
    index_t saturated_vert; // number of vertices within saturated components

    /* reduced connectivity
     * reduced edges represented with edges list (array of size twice the 
     * number of reduced edges, consecutive indices are linked components)
     * guarantees:
     * 1) starting component identifiers are smaller than ending components
     * 2) increasing order of starting and ending components identifiers
     *  (this eases some routines, like conversion to forward-star)
     * 3) each edge appears only once
     * 4) isolated components (not linked to any other component) are linked
     *  to themselves with epsilon reduced weight */
    comp_t* reduced_edges;

    /* easy accessors for reduced_edges */
    const comp_t& reduced_edges_u(index_t re) const
        { return reduced_edges[((size_t) 2)*re]; }
    const comp_t& reduced_edges_v(index_t re) const
        { return reduced_edges[((size_t) 2)*re + 1]; }
    comp_t& reduced_edges_u(index_t re)
        { return reduced_edges[((size_t) 2)*re]; }
    comp_t& reduced_edges_v(index_t re)
        { return reduced_edges[((size_t) 2)*re + 1]; }

    real_t* reduced_edge_weights;

    /**  parameters  **/

    real_t dif_tol, eps; // eps gives a characteristic precision 
    /* with nonzero verbose information on the process will be printed;
     * for convex methods, this will be passed on to the reduced problem
     * subroutine, controlling the number of subiterations between prints */
    int verbose; 

    /**  split components with graph cuts  **/
    struct Split_info {
        comp_t rv; // component to split
        comp_t K; // number of alternative values in the component's split
        /* first alternative to compete, useful to avoid competing with a value
         * already assigned to all vertices, or for single cut with K = 2 */
        comp_t first_k; 
        value_t* sX; // D-by-K array with alternative values in the split
        Split_info(comp_t rv);
        ~Split_info();
    };
    comp_t K; // maximum number of alternative values in any component's split
    int split_iter_num; // number of partition-and-update iterations
    real_t split_damp_ratio; // split damping along iterations
    /* number of repetitions in case of stochastic split values computation */
    int split_values_init_num;
    int split_values_iter_num;

    virtual index_t split();

    virtual void split_component(comp_t rv, Maxflow<index_t, real_t>* maxflow);

    /* initialize candidate split values, schedule and assignments;
     * base class version implements a kmeans++, with distances replaced by
     * split costs, and centroids assignments and updates made purely virtual
     * for specializing data and computations */
    virtual Split_info initialize_split_info(comp_t rv);
    /* make split value k that would optimaly fit vertex v */
    virtual void set_split_value(Split_info& split_info, comp_t k, index_t v)
        const = 0;
    /* usually some kind of averaging; must remove alternative values which are
     * no longer interesting (e.g. associated to no vertex) */
    virtual void update_split_info(Split_info& split_info) const = 0;
    /* rough estimate of the number of operations for initializing the split
     * values and all subsequent updates */
    virtual uintmax_t split_values_complexity() const;
    /* compute unary cost of split value k at vertex v in component rv;
     * can be +infinity, not -infinity */
    virtual real_t vert_split_cost(const Split_info& split_info, index_t v,
        comp_t k) const = 0;
    /* overload for possibly saving computations for the difference when
     * choosing alternative k against alternative l */
    virtual real_t vert_split_cost(const Split_info& split_info, index_t v,
        comp_t k, comp_t l) const;
    /* compute binary cost of choosing alternatives lu and lv at edge e */
    virtual real_t edge_split_cost(const Split_info& split_info, index_t e,
        comp_t lu, comp_t lv) const = 0;    

    /* methods for setting and checking edge status */
    bool is_cut(index_t e) const // check if edge e is cut (active)
        { return edge_status[e] == CUT; }
    bool is_bind(index_t e) const // check if edge e is binding (inactive)
        { return edge_status[e] == BIND; }
    bool is_separation(index_t e) const // check if edge is a separation
        { return edge_status[e] == SEPARATION; }
    void cut(index_t e) // flag a cut (active) edge
        { edge_status[e] = CUT; }
    void bind(index_t e) // flag a binding (inactive) edge
        { edge_status[e] = BIND; }
    void separate(index_t e) // flag a balancing separation edge
        { edge_status[e] = SEPARATION; }

    /* split large components for balancing split, either for parallelism or
     * for preventing bad maxflow performance on huge components;
     * new components are computed by breadth-first search, restarting when a
     * maximum size is reached;
     * reorder comp_list and populate first vertex accordingly;
     * rV_new is the number of components resulting from such split;
     * rV_big is the number of large original components split this way;
     * first_vertex_big holds the first vertices of components split this way;
     * returns the number of useful parallel threads */
    int balance_split(comp_t& rV_new, comp_t& rV_big,
        index_t*& first_vertex_big);

    /* after splitting, separation edges must be removed or activated;
     * when called, first_vertex contains additional components due to large
     * components being split by balance_split();
     * NOTA: currently, separation edges must be either removed or activated
     * at this step; this cannot wait for a future split step, because
     * components list of vertex must be kept consecutive for parallel
     * treatment of the resulting connected components, and removing parallel
     * separation edges in a later step might connect components whose list of
     * vertices are not consecutive */
    virtual index_t remove_balance_separations(comp_t rV_new);

    /* revert the above process;
     * no change to comp_list, only suppress elements from first_vertex */
    void revert_balance_split(comp_t rV_new, comp_t rV_big,
        index_t* first_vertex_big);

    /* rough estimate of the number of operations for split step;
     * useful for estimating the number of parallel threads */
    uintmax_t maxflow_complexity() const
        { return (uintmax_t) 2*E + V; } // just for a graph cut; heuristic
    virtual uintmax_t split_complexity() const;

    /* prefered alternative value for each vertex */
    comp_t*& label_assign = comp_assign; // reuse the same storage

    /**  compute reduced values  **/

    virtual void solve_reduced_problem() = 0;

    /**  merging components when deemed useful  **/

    /* during the merging step, merged components are stored as chains,
     * represented by arrays of length rV 'merge_chains_root', '_next' and
     * '_leaf'; merge chain involving component rv follows the scheme
     *   root[rv] -> ... -> rv -> next[rv] -> ... -> leaf[rv] ;
     * NOTA: CHAIN_END is a special values, and:
     * - only next[rv] is always up-to-date;
     * - root[rv] is always a strictly preceding component in its chain, or
     *   CHAIN_END if rv is a root;
     * - leaf[rv] is up-to-date if rv is a root;
     * - rv is the leaf of its chain if, and only if next[rv] == CHAIN_END;
     * an additional requirement is that the root of each chain should be the
     * component in the chain with lowest index */
    comp_t get_merge_chain_root(comp_t rv) const;

    /* merge the merge chains of the two given roots;
     * the root of the resulting chain will be the component in the chains
     * with lowest index, which is returned by the function */ 
    comp_t merge_components(comp_t ru, comp_t rv);

    /* compute the merge chains and return the number of effective merges */
    virtual comp_t compute_merge_chains() = 0;

    /* main routine using the above to perform the merge step;
     * NOTA: reduced edges must guarantee 1-4), see member declaration */
    virtual index_t merge();

    /**  monitoring evolution  **/

    /* test if computation of evolution is required */
    virtual bool monitor_evolution() const
        { return dif_tol > (real_t) 0.0 || iterate_evolution; }

    /* compute relative iterate evolution */
    virtual real_t compute_evolution() const = 0;

    /* compute objective functional, often on the reduced problem objects */
    virtual real_t compute_objective() const = 0;

    /* allocate memory and fail with error message if not successful */
    static void* malloc_check(size_t size)
    {
        void *ptr = malloc(size);
        if (!ptr){
            std::cerr << "Cut-pursuit: not enough memory." << std::endl;
            exit(EXIT_FAILURE);
        }
        return ptr;
    }

    /* simply free if size is zero */
    static void* realloc_check(void* ptr, size_t size)
    {
        if (!size){
           free(ptr); 
           return nullptr; 
        }
        ptr = realloc(ptr, size);
        if (!ptr){
            std::cerr << "Cut-pursuit: not enough memory." << std::endl;
            exit(EXIT_FAILURE);
        }
        return ptr;
    }

    /**  control parallelization  **/
    int max_num_threads; // maximum number of parallel threads 
    /* take into account max_num_threads attribute */
    int compute_num_threads(uintmax_t num_ops, uintmax_t max_threads) const
    {
        int num_threads = ::compute_num_threads(num_ops, max_threads);
        return num_threads < max_num_threads ? num_threads : max_num_threads;
    }
    /* overload for max_threads defaulting to num_ops */
    int compute_num_threads(uintmax_t num_ops) const
    { return compute_num_threads(num_ops, num_ops); }

    /* representing infinite values (has_infinity checked by constructor) */
    static real_t real_inf(){ return std::numeric_limits<real_t>::infinity(); }

private:
    enum Edge_status : char // requires C++11 to ensure 1 byte
        {BIND, CUT, SEPARATION};
    Edge_status* edge_status; // edge activation

    /* parameters */
    int it_max; // maximum number of cut-pursuit iterations
    bool balance_parallel_split; // switch parallel split balancing
    index_t max_split_size; // ensure maxflow not working on components too big

    /* monitoring */
    real_t* objective_values;
    double* elapsed_time;
    real_t* iterate_evolution;

    /* during the merging step, merged components are stored as chains */
    comp_t *merge_chains_root, *merge_chains_next, *merge_chains_leaf;

    double monitor_time(std::chrono::steady_clock::time_point start) const;

    void print_progress(int it, real_t dif, double t) const;

    /* set components assignment and values (and allocate them if needed);
     * assumes that no edge of the graph are cut when it is called */
    void initialize();

    /* initialize with components specified in 'comp_assign' */
    void assign_connected_components();

    /* initialize with only one component and reduced graph accordingly */
    void single_connected_component();

    /* compute binding reverse edge forward star graph structure */
    void get_bind_reverse_edges(comp_t rv, index_t*& first_edge_r,
        index_t*& adj_vertices_r);

    /* update connected components and count saturated ones */
    void compute_connected_components();

    /* allocate and compute reduced graph structure;
     * NOTA: reduced edges must guarantee 1-4), see member declaration */
    void compute_reduced_graph();
};
