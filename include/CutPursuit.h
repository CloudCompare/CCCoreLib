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
#include <GenericProgressCallback.h>

//system
#include <cstdint> // for uintmax_t, requires C++11
#include <cstdlib> // for size_t, malloc, exit
#include <chrono>
#include <limits>
#include <functional>
#include <iostream>
#include <cmath>


class CP
{
public:
    /**  constructor, destructor  **/

    CP(int32_t V, int32_t E, const int32_t* first_edge, 
        const int32_t* adj_vertices, const float* Y, size_t D = 1);

    /* the destructor does not free pointers which are supposed to be provided 
     * by the user (forward-star graph structure given at construction, 
     * monitoring arrays, etc.); IT DOES FREE THE REST (components assignment 
     * and reduced problem elements, etc.), but this can be prevented by
     * getting the corresponding pointer member and setting it to null
     * beforehand */
    virtual ~CP();

    /**  methods for manipulating parameters  **/

    void reset_edges(); // bind all edges

    /* if 'edge_weights' is null, homogeneously equal to 'homo_edge_weight' */
    void set_edge_weights(const float* edge_weights = nullptr,
        float homo_edge_weight = 1.0);

    void set_monitoring_arrays(float* objective_values = nullptr,
        double* elapsed_time = nullptr, float* iterate_evolution = nullptr);

    /* if rV is zero or unity, comp_assign will be automatically initialized;
     * if rV is zero, arbitrary components will be assigned at initialization,
     * in an attempt to optimize parallelization along components;
     * if rV is greater than one, comp_assign must be given and initialized;
     * comp_assign is free()'d by destructor, unless set to null beforehand */
    void set_components(int32_t rV = 0, int32_t* comp_assign = nullptr);

    void set_cp_param(float dif_tol, int it_max, int verbose, float eps);
    /* overload for default eps parameter */
    void set_cp_param(float dif_tol = 0.0, int it_max = 10,
        int verbose = 1000)
    {
        set_cp_param(dif_tol, it_max, verbose,
            std::numeric_limits<float>::epsilon());
    }

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

    int32_t get_components(const int32_t** comp_assign = nullptr,
        const int32_t** first_vertex = nullptr,
        const int32_t** comp_list = nullptr) const;

    /* return the number of reduced edges */
    int32_t get_reduced_graph(const int32_t** reduced_edges = nullptr,
        const float** reduced_edge_weights = nullptr);

    /* retrieve the reduced iterate (values of the components);
     * WARNING: reduced values are free()'d by destructor */
    const float* get_reduced_values() const;

    /* set the reduced iterate (values of the components);
     * WARNING: if not set to null before deletion of the main cp object,
     * this will be deleted by free() so the given pointer must have been
     * allocated with malloc() and the likes */
    void set_reduced_values(float* rX);

    /* parameters of d0 penalization (w_d0_uv) can be set using base class Cp
     * method set_edge_weights() */

    /* specific loss */
    float quadratic_loss() const { return D; }

    /* Y is changed only if the corresponding argument is not null */
    void set_loss(float loss, const float* Y = nullptr,
        const float* vert_weights = nullptr,
        const float* coor_weights = nullptr);

    /* overload for changing only loss weights */
    void set_loss(const float* vert_weights = nullptr,
        const float* coor_weights = nullptr)
        { set_loss(loss, nullptr, vert_weights, coor_weights); }

    /* tune split parameters; set max_split_size to V for no max */
    void set_split_param(int32_t max_split_size, int32_t K = 2,
        int split_iter_num = 1, float split_damp_ratio = 1.0,
        int split_values_init_num = 3, int split_values_iter_num = 3);

    void set_min_comp_weight(float min_comp_weight = 1.0);

    /* solve the main problem */
    int cut_pursuit(bool init, CCCoreLib::GenericProgressCallback* progressCb = nullptr);

protected:
    /**  main graph  **/

    const int32_t V, E; // number of vertices, of edges

    /**  forward-star graph representation  **/
    /* - edges are numeroted so that all edges originating from a same vertex
     * are consecutive;
     * - for each vertex, 'first_edge' indicates the first edge starting
     * from the vertex (or, if there are none, starting from the next vertex);
     * array of length V + 1, the first value is always zero and the last
     * value is always the total number of edges E
     * - for each edge, 'adj_vertices' indicates its ending vertex */
    const int32_t *first_edge, *adj_vertices; 
    
    const float *edge_weights; // array of length E, weights of edges
    float homo_edge_weight; // homogeneous weights, set edge_weights to null

    /* dimension of the data; total size signal is V*D */
    const size_t D;

    /**  reduced graph  **/

    /* last_* are used to identify saturated components and to compute 
     * iterate evolution */
    int32_t rV, last_rV; // number of components (reduced vertices)
    float *rX, *last_rX; // reduced iterate (values of the components)
    int32_t rE; // number of reduced edges
    /* assignment of each vertex to a component */
    int32_t* comp_assign, *last_comp_assign;
    /* list the vertices of each components:
     * - vertices are gathered in 'comp_list' so that all vertices belonging
     * to a same components are consecutive
     * - for each component, 'first_vertex' indicates the index of its first
     * vertex in 'comp_list' */
    int32_t *comp_list, *first_vertex;
    /* reverse mapping of comp list: index of a given vertex within its
     * components (useful for working within components in parallel) */
    int32_t *index_in_comp;
    /* components saturation */
    bool* is_saturated;
    int32_t saturated_comp; // number of saturated components
    int32_t saturated_vert; // number of vertices within saturated components

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
    int32_t* reduced_edges;

    /* easy accessors for reduced_edges */
    const int32_t& reduced_edges_u(int32_t re) const
        { return reduced_edges[((size_t) 2)*re]; }
    const int32_t& reduced_edges_v(int32_t re) const
        { return reduced_edges[((size_t) 2)*re + 1]; }
    int32_t& reduced_edges_u(int32_t re)
        { return reduced_edges[((size_t) 2)*re]; }
    int32_t& reduced_edges_v(int32_t re)
        { return reduced_edges[((size_t) 2)*re + 1]; }

    float* reduced_edge_weights;

    /**  parameters  **/

    float dif_tol, eps; // eps gives a characteristic precision 
    /* with nonzero verbose information on the process will be printed;
     * for convex methods, this will be passed on to the reduced problem
     * subroutine, controlling the number of subiterations between prints */
    int verbose; 

    /**  split components with graph cuts  **/
    struct Split_info {
        int32_t rv; // component to split
        int32_t K; // number of alternative values in the component's split
        /* first alternative to compete, useful to avoid competing with a value
         * already assigned to all vertices, or for single cut with K = 2 */
        int32_t first_k; 
        float* sX; // D-by-K array with alternative values in the split
        Split_info(int32_t rv);
        ~Split_info();
    };
    int32_t K; // maximum number of alternative values in any component's split
    int split_iter_num; // number of partition-and-update iterations
    float split_damp_ratio; // split damping along iterations
    /* number of repetitions in case of stochastic split values computation */
    int split_values_init_num;
    int split_values_iter_num;

    virtual int32_t split();

    virtual void split_component(int32_t rv, Maxflow<int32_t, float>* maxflow);

    /* initialize candidate split values, schedule and assignments;
     * implements a kmeans++, with distances replaced by split costs */
    virtual Split_info initialize_split_info(int32_t rv);
    /* make split value k that would optimaly fit vertex v (sets Yv) */
    void set_split_value(Split_info& split_info, int32_t k, int32_t v) const;
    /* average of observations Y; must remove alternative values which are
     * no longer interesting (e.g. associated to no vertex) */
    void update_split_info(Split_info& split_info) const;
    /* rough estimate of the number of operations for initializing the split
     * values and all subsequent updates */
    virtual uintmax_t split_values_complexity() const;
    /* compute unary cost of split value k at vertex v in component rv;
     * can be +infinity, not -infinity */
    float vert_split_cost(const Split_info& split_info, int32_t v,
        int32_t k) const;
    /* overload for possibly saving computations for the difference when
     * choosing alternative k against alternative l */
    virtual float vert_split_cost(const Split_info& split_info, int32_t v,
        int32_t k, int32_t l) const;
    /* compute binary cost of choosing alternatives lu and lv at edge e */
    float edge_split_cost(const Split_info& split_info, int32_t e,
        int32_t lu, int32_t lv) const;

    /* methods for setting and checking edge status */
    bool is_cut(int32_t e) const // check if edge e is cut (active)
        { return edge_status[e] == CUT; }
    bool is_bind(int32_t e) const // check if edge e is binding (inactive)
        { return edge_status[e] == BIND; }
    bool is_separation(int32_t e) const // check if edge is a separation
        { return edge_status[e] == SEPARATION; }
    void cut(int32_t e) // flag a cut (active) edge
        { edge_status[e] = CUT; }
    void bind(int32_t e) // flag a binding (inactive) edge
        { edge_status[e] = BIND; }
    void separate(int32_t e) // flag a balancing separation edge
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
    int balance_split(int32_t& rV_new, int32_t& rV_big,
        int32_t*& first_vertex_big);

    /* after splitting, separation edges must be removed or activated;
     * when called, first_vertex contains additional components due to large
     * components being split by balance_split();
     * NOTA: currently, separation edges must be either removed or activated
     * at this step; this cannot wait for a future split step, because
     * components list of vertex must be kept consecutive for parallel
     * treatment of the resulting connected components, and removing parallel
     * separation edges in a later step might connect components whose list of
     * vertices are not consecutive */
    virtual int32_t remove_balance_separations(int32_t rV_new);

    /* revert the above process;
     * no change to comp_list, only suppress elements from first_vertex */
    void revert_balance_split(int32_t rV_new, int32_t rV_big,
        int32_t* first_vertex_big);

    /* rough estimate of the number of operations for split step;
     * useful for estimating the number of parallel threads */
    uintmax_t maxflow_complexity() const
        { return (uintmax_t) 2*E + V; } // just for a graph cut; heuristic
    virtual uintmax_t split_complexity() const;

    /* prefered alternative value for each vertex */
    int32_t*& label_assign = comp_assign; // reuse the same storage

    /**  compute reduced values  **/

    /* allocate and compute reduced values */
    void solve_reduced_problem();

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
    int32_t get_merge_chain_root(int32_t rv) const;

    /* merge the merge chains of the two given roots;
     * the root of the resulting chain will be the component in the chains
     * with lowest index, which is returned by the function */ 
    int32_t merge_components(int32_t ru, int32_t rv);

    /* arrays are indexed by reduced edges */
    float* merge_gains; // gain on the objective if components are merged
    float** merge_values; // the value of the components if they are merged

    /* compute merge information of the given reduced edge;
     * populate member arrays merge_gains and merge_values; allocate value
     * with malloc; negative gain values might still get accepted,
     * inacceptable merge candidate must be deleted */
    void compute_merge_candidate(int32_t re);

    /* accept and delete the merge candidate, and return the component root
     * of the resulting merge chain; also transfers component weights to the
     * root component */
    int32_t accept_merge_candidate(int32_t re);

    /* frees the merge value and flag it to null pointer */
    void delete_merge_candidate(int32_t re);

    /* rough estimate of the number of operations for computing merge info of
     * a reduced edge; useful for estimating the number of parallel threads */
    size_t merge_info_complexity() const;

    /* compute the merge chains and return the number of effective merges */
    int32_t compute_merge_chains();

    /* main routine using the above to perform the merge step;
     * NOTA: reduced edges must guarantee 1-4), see member declaration */
    int32_t merge();

    /**  monitoring evolution  **/

    /* test if computation of evolution is required */
    virtual bool monitor_evolution() const
        { return dif_tol > (float) 0.0 || iterate_evolution; }

    /* compute relative iterate evolution (in terms of distance relative to
     * distance to Y) */
    float compute_evolution() const;

    /* compute graph contour length; use reduced edges and reduced weights */
    float compute_graph_d0() const;

    /* compute objective functional */
    float compute_objective() const;

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
    static float real_inf(){ return std::numeric_limits<float>::infinity(); }

private:
/**  separable loss term: weighted square l2 or smoothed KL **/
    const float* Y; // observations, D-by-V array, column major format

    /* D (or public method quadratic_loss()) for quadratic 
     *      f(x) = 1/2 ||y - x||_{l2,W}^2 ,
     * where W is a diagonal metric (separable product along ℝ^V and ℝ^D),
     * that is ||y - x||_{l2,W}^2 = sum_{v in V} w_v ||x_v - y_v||_{l2,M}^2
     *                            = sum_{v in V} w_v sum_d m_d (x_vd - y_vd)^2.
     *
     * 0 < loss < 1 for smoothed Kullback-Leibler divergence (equivalent to
     * cross-entropy) on the probability simplex
     *     f(x) = sum_v w_v KLs_m(x_v, y_v),
     * with KLs(y_v, x_v) = KL(s u + (1 - s) y_v ,  s u + (1 - s) x_v), where
     *     KL is the regular Kullback-Leibler divergence,
     *     u is the uniform discrete distribution over {1,...,D}, and
     *     s = loss is the smoothing parameter
     * it yields
     *     KLs(y_v, x_v) = - H(s u + (1 - s) y_v)
     *         - sum_d (s/D + (1 - s) y_{v,d}) log(s/D + (1 - s) x_{v,d}) ,
     * where H_m is the entropy, that is H(s u + (1 - s) y_v)
     *       = - sum_d (s/D + (1 - s) y_{v,d}) log(s/D + (1 - s) y_{v,d}) ;
     * note that the choosen order of the arguments in the Kullback-Leibler
     * does not favor the entropy of x (H(s u + (1 - s) y_v) is a constant),
     * hence this loss is actually equivalent to cross-entropy;
     *
     * 1 <= loss < D for both: quadratic on coordinates from 1 to loss, and
     * Kullback-Leibler divergence on coordinates from loss + 1 to D;
     *
     * the weights w_v are set in vert_weights and m_d are set in coor_weights;
     * set corresponding pointer to null for no weight; note that coordinate
     * weights makes no sense for Kullback-Leibler divergence alone, but should
     * be used for weighting quadratic and KL when mixing both, in which case
     * coor_weights should be of length loss + 1 */
    float loss;
    const float *vert_weights, *coor_weights;

    /* minimum weight allowed for a component */
    float min_comp_weight;

    /* compute the functional f at a single vertex */
    /* NOTA: not actually a metric, in spite of its name */
    float distance(const float* Xv, const float* Yv) const;
    float fv(int32_t v, const float* Xv) const;
    /* raw sum of fv over all vertices (no fYY subtraction, not cached) */
    float compute_f_sum() const;
    /* stored values (used for iterate evolution): fXY - fYY, or recomputed */
    float compute_f() const;
    float fXY; // dist(X, Y), reinitialized when freeing rX
    float fYY; // dist(Y, Y), reinitialized when modifying the loss

    /**  reduced problem  **/
    float* comp_weights;

    enum Edge_status : char // requires C++11 to ensure 1 byte
        {BIND, CUT, SEPARATION};
    Edge_status* edge_status; // edge activation

    /* parameters */
    int it_max; // maximum number of cut-pursuit iterations
    bool balance_parallel_split; // switch parallel split balancing
    int32_t max_split_size; // ensure maxflow not working on components too big

    /* monitoring */
    float* objective_values;
    double* elapsed_time;
    float* iterate_evolution;

    /* during the merging step, merged components are stored as chains */
    int32_t *merge_chains_root, *merge_chains_next, *merge_chains_leaf;

    double monitor_time(std::chrono::steady_clock::time_point start) const;

    void print_progress(int it, float dif, double t) const;

    /* set components assignment and values (and allocate them if needed);
     * assumes that no edge of the graph are cut when it is called */
    void initialize();

    /* initialize with components specified in 'comp_assign' */
    void assign_connected_components();

    /* initialize with only one component and reduced graph accordingly */
    void single_connected_component();

    /* compute binding reverse edge forward star graph structure */
    void get_bind_reverse_edges(int32_t rv, int32_t*& first_edge_r,
        int32_t*& adj_vertices_r);

    /* update connected components and count saturated ones */
    void compute_connected_components();

    /* allocate and compute reduced graph structure;
     * NOTA: reduced edges must guarantee 1-4), see member declaration */
    void compute_reduced_graph();
};
