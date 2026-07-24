/*=============================================================================
 * Derived class for cut-pursuit algorithm with d0 (weighted contour length) 
 * penalization, with a loss akin to a distance:
 *
 * minimize functional over a graph G = (V, E)
 *
 *        F(x) = sum_v loss(y_v, x_v) + ||x||_d0
 *
 * where for each vertex, y_v and x_v are D-dimensional vectors, the loss is
 * a mix of the sum of square differences and a Kullback-Leibler divergence
 * (equivalent to cross-entropy in this formulation); see the 'loss' attribute,
 *   and ||x||_d0 = sum_{uv in E : xu != xv} w_d0_uv ,
 *
 * using greedy cut-pursuit approach with splitting initialized with k-means++.
 *
 * Parallel implementation with OpenMP API.
 *
 * L. Landrieu and G. Obozinski, Cut Pursuit: fast algorithms to learn
 * piecewise constant functions on general weighted graphs, SIAM Journal on
 * Imaging Science, 10(4):1724-1766, 2017
 *
 * L. Landrieu et al., A structured regularization framework for spatially
 * smoothing semantic labelings of 3D point clouds, ISPRS Journal of
 * Photogrammetry and Remote Sensing, 132:102-118, 2017
 *
 * Hugo Raguet 2019, 2022, 2023
 *===========================================================================*/
#pragma once
#include <cmath>
#include "cut_pursuit_d0.h"

/* real_t is the real numeric type, used for the base field and for the
 * objective functional computation;
 * index_t must be able to represent the number of vertices and of (undirected)
 * edges in the main graph;
 * comp_t must be able to represent the number of constant connected components
 * in the reduced graph */
template <typename real_t, typename index_t, typename comp_t>
class Cp_d0_dist : public Cp_d0<real_t, index_t, comp_t>
{
public:
    /**  constructor, destructor  **/

    /* only creates BK graph structure and assign Y, D */
    Cp_d0_dist(index_t V, index_t E, const index_t* first_edge,
        const index_t* adj_vertices, const real_t* Y, size_t D = 1);

    /* the destructor does not free pointers which are supposed to be provided 
     * by the user (forward-star graph structure given at construction, 
     * monitoring arrays, observation arrays); IT DOES FREE THE REST 
     * (components assignment and reduced problem elements, etc.), but this can
     * be prevented by getting the corresponding pointer member and setting it
     * to null beforehand */
	~Cp_d0_dist();

    /**  methods for manipulating parameters  **/

    /* parameters of d0 penalization (w_d0_uv) can be set using base class Cp
     * method set_edge_weights() */

    /* specific loss */
    real_t quadratic_loss() const { return D; }

    /* Y is changed only if the corresponding argument is not null */
    void set_loss(real_t loss, const real_t* Y = nullptr,
        const real_t* vert_weights = nullptr,
        const real_t* coor_weights = nullptr);

    /* overload for changing only loss weights */
    void set_loss(const real_t* vert_weights = nullptr,
        const real_t* coor_weights = nullptr)
        { set_loss(loss, nullptr, vert_weights, coor_weights); }

    /* overload base method for higher init and iter num */
    void set_split_param(index_t max_split_size, comp_t K = 2,
        int split_iter_num = 1, real_t split_damp_ratio = 1.0,
        int split_values_init_num = 3, int split_values_iter_num = 3);

    void set_min_comp_weight(real_t min_comp_weight = 1.0);

private:
    /**  separable loss term: weighted square l2 or smoothed KL **/
    const real_t* Y; // observations, D-by-V array, column major format

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
    real_t loss;
    const real_t *vert_weights, *coor_weights;

    /* minimum weight allowed for a component */
    real_t min_comp_weight;

    /* compute the functional f at a single vertex */
    /* NOTA: not actually a metric, in spite of its name */
    real_t distance(const real_t* Xv, const real_t* Yv) const;
    real_t fv(index_t v, const real_t* Xv) const override;
    /* override for storing values (used for iterate evolution) */
    real_t compute_f() const override;
    real_t fXY; // dist(X, Y), reinitialized when freeing rX
    real_t fYY; // dist(Y, Y), reinitialized when modifying the loss

    /**  reduced problem  **/
    real_t* comp_weights;

    /* allocate and compute reduced values;
     * do nothing if the array of reduced values is not null */
    void solve_reduced_problem() override;

    /**  greedy splitting  **/

    /* override for setting observation Yv */
    using typename Cp<real_t, index_t, comp_t>::Split_info;
    void set_split_value(Split_info& split_info, comp_t k, index_t v) const
        override;
    /* override for average of observations Y */
    void update_split_info(Split_info& split_info) const override;

    /**  merging components **/

    /* compute merge information of the given reduced edge;
     * populate member arrays merge_gains and merge_values; allocate value with
     * malloc; negative gain values might still get accepted, inacceptable
     * merge candidate must be deleted */
    void compute_merge_candidate(index_t re) override;

    /* override for transfering component weights to root component */
    comp_t accept_merge_candidate(index_t re) override;

    /* rough estimate of the number of operations for computing merge info of a
     * reduced edge; useful for estimating the number of parallel threads */
    size_t merge_info_complexity() const override;

    index_t merge() override; // override for freeing comp_weights

    /**  monitoring evolution  **/

    /* iterate evolution in terms of distance relative to distance to Y */
    real_t compute_evolution() const override;

    /**  type resolution for base template class members
     * https://isocpp.org/wiki/faq/templates#nondependent-name-lookup-members
     **/
    using Cp_d0<real_t, index_t, comp_t>::delete_merge_candidate;
    using Cp_d0<real_t, index_t, comp_t>::merge_gains;
    using Cp_d0<real_t, index_t, comp_t>::merge_values;
    using Cp<real_t, index_t, comp_t>::set_split_param;
    using Cp<real_t, index_t, comp_t>::saturated_vert;
    using Cp<real_t, index_t, comp_t>::last_comp_assign;
    using Cp<real_t, index_t, comp_t>::eps;
    using Cp<real_t, index_t, comp_t>::D;
    using Cp<real_t, index_t, comp_t>::V;
    using Cp<real_t, index_t, comp_t>::rV;
    using Cp<real_t, index_t, comp_t>::rE;
    using Cp<real_t, index_t, comp_t>::rX;
    using Cp<real_t, index_t, comp_t>::last_rX;
    using Cp<real_t, index_t, comp_t>::monitor_evolution;
    using Cp<real_t, index_t, comp_t>::comp_assign;
    using Cp<real_t, index_t, comp_t>::label_assign;
    using Cp<real_t, index_t, comp_t>::comp_list;
    using Cp<real_t, index_t, comp_t>::first_vertex;
    using Cp<real_t, index_t, comp_t>::reduced_edges_u;
    using Cp<real_t, index_t, comp_t>::reduced_edges_v;
    using Cp<real_t, index_t, comp_t>::reduced_edge_weights;
    using Cp<real_t, index_t, comp_t>::is_saturated;
    using Cp<real_t, index_t, comp_t>::malloc_check;
    using Cp<real_t, index_t, comp_t>::real_inf;
};
