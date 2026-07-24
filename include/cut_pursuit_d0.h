/*=============================================================================
 * Derived class for cut-pursuit algorithm with d0 (weighted contour length) 
 * penalization, with a separable loss term over a given space:
 *
 * minimize functional over a graph G = (V, E)
 *
 *        F(x) = f(x) + ||x||_d0
 *        
 * where for each vertex, x_v belongs in a possibly multidimensional space Ω,
 *       f(x) = sum_{v in V} f_v(x_v) is separable along V with f_v : Ω → ℝ
 *   and ||x||_d0 = sum_{uv in E : xu != xv} w_d0_uv ,
 *
 * using greedy cut-pursuit approach.
 *
 * Parallel implementation with OpenMP API.
 *
 * References: 
 *
 * L. Landrieu and G. Obozinski, Cut Pursuit: fast algorithms to learn
 * piecewise constant functions on general weighted graphs, SIAM Journal on
 * Imaging Science, 10(4):1724-1766, 2017
 *
 * L. Landrieu et al., A structured regularization framework for spatially
 * smoothing semantic labelings of 3D point clouds, ISPRS Journal of
 * Photogrammetry and Remote Sensing, 132:102-118, 2017
 *
 * Hugo Raguet 2019, 2020
 *===========================================================================*/
#pragma once
#include "cut_pursuit.h"

/* real_t is the real numeric type, used for objective functional computation;
 * index_t must be able to represent the number of vertices and of (undirected)
 * edges in the main graph;
 * comp_t must be able to represent the number of constant connected components
 * in the reduced graph;
 * value_t is the type associated to the space to which the values belong, it
 * is usually real_t, and if multidimensional, this must be specified in the
 * parameter D (e.g. for R^3, specify value_t = real_t and D = 3) */
template <typename real_t, typename index_t, typename comp_t,
    typename value_t = real_t>
class Cp_d0 : public Cp<real_t, index_t, comp_t, value_t>
{
public:
    Cp_d0(index_t V, index_t E, const index_t* first_edge, 
        const index_t* adj_vertices, size_t D = 1);

protected:
    /* compute the functional f at a single vertex */
    virtual real_t fv(index_t v, const value_t* Xv) const = 0; 

    /* compute graph contour length; use reduced edges and reduced weights */
    real_t compute_graph_d0() const;

    /* compute objective functional */
    virtual real_t compute_f() const;
    real_t compute_objective() const override;

    /**  greedy splitting  **/

    /* compute unary cost of split value k at vertex v in component rv */
    using typename Cp<real_t, index_t, comp_t>::Split_info;
    real_t vert_split_cost(const Split_info& split_info, index_t v,
        comp_t k) const override;
    /* compute binary cost of choosing alternatives lu and lv at edge e */
    real_t edge_split_cost(const Split_info& split_info, index_t e,
        comp_t lu, comp_t lv) const override;

    /**  merging components  **/

    /* the strategy is to compute the gain on the functional for the merge of
     * each reduced edge, and accept greedily the candidates with greatest
     * gain; merge gains and values of neighboring components might be impacted
     * by the merge of two components, see implementation for details; override
     * the following virtual merge methods for taking additional information
     * into account
     * NOTA: during the merging step, merged components are stored as chains,
     * see header `cut_pursuit.hpp` for details */

    /* arrays are indexed by reduced edges */
    real_t* merge_gains; // gain on the objective if components are merged
    value_t** merge_values; // the value of the components if they are merged

    /* compute merge information of the given reduced edge;
     * populate member arrays merge_gains and merge_values; allocate value with
     * malloc; negative gain values might still get accepted, inacceptable
     * merge candidate must be deleted */
    virtual void compute_merge_candidate(index_t re) = 0;

    /* accept and delete the merge candidate, and return the component root of
     * the resulting merge chain; can be overriden to take into account other
     * merge effects */
    virtual comp_t accept_merge_candidate(index_t re);

    /* frees the merge value and flag it to null pointer */
    void delete_merge_candidate(index_t re);

    /* rough estimate of the number of operations for computing merge info of a
     * reduced edge; useful for estimating the number of parallel threads */
    virtual size_t merge_info_complexity() const = 0;



    /**  type resolution for base template class members
     * https://isocpp.org/wiki/faq/templates#nondependent-name-lookup-members
     **/
    using Cp<real_t, index_t, comp_t>::get_merge_chain_root;
    using Cp<real_t, index_t, comp_t>::K;
    using Cp<real_t, index_t, comp_t>::split_iter_num;
    using Cp<real_t, index_t, comp_t>::split_damp_ratio;
    using Cp<real_t, index_t, comp_t>::split_values_init_num;
    using Cp<real_t, index_t, comp_t>::split_values_iter_num;
    using Cp<real_t, index_t, comp_t>::V;
    using Cp<real_t, index_t, comp_t>::E;
    using Cp<real_t, index_t, comp_t>::D;
    using Cp<real_t, index_t, comp_t>::rV;
    using Cp<real_t, index_t, comp_t>::rE;
    using Cp<real_t, index_t, comp_t>::rX;
    using Cp<real_t, index_t, comp_t>::edge_weights;
    using Cp<real_t, index_t, comp_t>::homo_edge_weight;
    using Cp<real_t, index_t, comp_t>::comp_list;
    using Cp<real_t, index_t, comp_t>::first_vertex;
    using Cp<real_t, index_t, comp_t>::reduced_edges_u;
    using Cp<real_t, index_t, comp_t>::reduced_edges_v;
    using Cp<real_t, index_t, comp_t>::reduced_edge_weights;
    using Cp<real_t, index_t, comp_t>::merge_components;
    using Cp<real_t, index_t, comp_t>::realloc_check;
    using Cp<real_t, index_t, comp_t>::malloc_check;

private:
    /* compute the merge chains and return the number of effective merges */
    comp_t compute_merge_chains() override;
};
