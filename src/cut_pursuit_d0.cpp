/*=============================================================================
 * Hugo Raguet 2019
 *===========================================================================*/
#include "cut_pursuit_d0.h"
#include <set>
#include <algorithm>

#define EDGE_WEIGHTS_(e) (edge_weights ? edge_weights[(e)] : homo_edge_weight)

#define TPL template <typename real_t, typename index_t, typename comp_t, \
    typename value_t>
#define CP_D0 Cp_d0<real_t, index_t, comp_t, value_t>

using namespace std;

TPL CP_D0::Cp_d0(index_t V, index_t E, const index_t* first_edge,
    const index_t* adj_vertices, size_t D)
    : Cp<real_t, index_t, comp_t>(V, E, first_edge, adj_vertices, D)
{
    K = 2;
    split_iter_num = 2;
    split_damp_ratio = 1.0;
    split_values_init_num = 3;
    split_values_iter_num = 3;
    merge_gains = nullptr;
    merge_values = nullptr;
}

TPL real_t CP_D0::compute_graph_d0() const
{
    real_t weighted_contour_length = 0.0;
    #pragma omp parallel for schedule(static) NUM_THREADS(rE) \
        reduction(+:weighted_contour_length)
    for (index_t re = 0; re < rE; re++){
        weighted_contour_length += reduced_edge_weights[re];
    }
    return weighted_contour_length;
}

TPL real_t CP_D0::compute_f() const
{
    real_t f = 0.0;
    #pragma omp parallel for schedule(dynamic) NUM_THREADS(D*V, rV) \
        reduction(+:f)
    for (comp_t rv = 0; rv < rV; rv++){
        real_t* rXv = rX + D*rv;
        for (index_t i = first_vertex[rv]; i < first_vertex[rv + 1]; i++){
            f += fv(comp_list[i], rXv);
        }
    }
    return f;
}

TPL real_t CP_D0::compute_objective() const
{ return compute_f() + compute_graph_d0(); } // f(x) + ||x||_d0

TPL real_t CP_D0::vert_split_cost(const Split_info& split_info, index_t v,
        comp_t k) const
{ return fv(v, split_info.sX + D*k); }

/* compute binary cost of choosing alternatives lu and lv at edge e */
TPL real_t CP_D0::edge_split_cost(const Split_info& split_info, index_t e,
    comp_t lu, comp_t lv) const
{ return lu == lv ? 0.0 : EDGE_WEIGHTS_(e); }

TPL comp_t CP_D0::accept_merge_candidate(index_t re)
{
    comp_t ru = reduced_edges_u(re);
    comp_t rv = reduced_edges_v(re);
    ru = merge_components(ru, rv);
    value_t* rXu = rX + D*ru;
    for (size_t d = 0; d < D; d++){ rXu[d] = merge_values[re][d]; }
    delete_merge_candidate(re);
    return ru;
}

TPL void CP_D0::delete_merge_candidate(index_t re)
{ free(merge_values[re]); merge_values[re] = nullptr; }

TPL comp_t CP_D0::compute_merge_chains()
{
    comp_t merge_count = 0;

    /* compute merge candidates in parallel */
    merge_gains = (real_t*) malloc_check(sizeof(real_t)*rE);
    merge_values = (value_t**) malloc_check(sizeof(value_t*)*rE);
    for (index_t re = 0; re < rE; re++){ merge_values[re] = nullptr; }
    index_t num_pos_candidates = 0, num_neg_candidates = 0;
    #pragma omp parallel for NUM_THREADS(merge_info_complexity()*rE, rE) \
        schedule(static) reduction(+:num_pos_candidates, num_neg_candidates)
    for (index_t re = 0; re < rE; re++){
        comp_t ru = reduced_edges_u(re);
        comp_t rv = reduced_edges_v(re);
        if (ru == rv){ continue; }
        compute_merge_candidate(re);
        if (merge_values[re]){
            if (merge_gains[re] > 0.0){ num_pos_candidates++; }
            else{ num_neg_candidates++; }
        }
    }

    if (!(num_pos_candidates || num_neg_candidates)){
        free(merge_gains); free(merge_values);
        return 0;
    }

    /* local read-only access to merge_gains; useful for lambdas below,
     * since one cannot directly capture member variables */
    const real_t* _merge_gains = merge_gains;

    if (num_pos_candidates){
    /**  merge candidates with positive gains;
     **  these are important enough to be merged in decreasing gain order, and
     **  to update surrounding merge candidates after each merge:
     **  1) maintain candidates in a priority order on the gain
     **  2) maintain access to all reduced edges involving a given vertex, and
     **  to their potential corresponding candidate in the priority order;
     **  because of 2), the best choice for 1) is a binary search tree **/

    /* 1) binary search tree on the gain */
    auto compare_candidates = [_merge_gains] (index_t mc1, index_t mc2) -> bool
        { return _merge_gains[mc1] > _merge_gains[mc2] ||
            /* ensure unique identification of merge candidates */
            (_merge_gains[mc1] == _merge_gains[mc2] && mc1 < mc2); };
    set<index_t, decltype(compare_candidates)>
        candidates_queue(compare_candidates);
    for (index_t re = 0; re < rE; re++){
        if (merge_values[re] && merge_gains[re] > 0.0){
            candidates_queue.insert(re);
        }
    }

    /* 2) linked list structure for updating reduced graph while merging */
    /* - given a component, we need access to the list of merge candidates
     * whose corresponding reduced edge involves the considered component;
     * - to that purpose, we maintain for each component a linked list of such
     * merge candidates; we call "merge candidate cell" the data structure with
     * the merge candidate identifier and the access to the next cell in such a
     * linked list;
     * - each active merge candidates is thus referenced in two such cells: one
     * within both lists of starting and ending components of the corresponding
     * reduced edge;
     * - one can thus compact information mapping unequivocally each merge
     * candidate mc to merge candidate cells identifiers 2*mc and 2*mc + 1;
     * conversely, the merge candidate of a cell mcc is mcc/2
     * - the link list structure can thus be maintained with the following
     * tables:
     *  first_candidate_cell[ru] is the index of the first merge candidate
     *      cell of the list of adjacent candidates for component ru
     *  next_candidate_cell[mcc] is the index of the merge candidate cell
     *      that comes after mcc within the list containing it
     */
    typedef size_t Cell_id;
    #define EMPTY_CELL (std::numeric_limits<Cell_id>::max())
    Cell_id* first_candidate_cell = (Cell_id*)
        malloc_check(sizeof(Cell_id*)*rV);
    Cell_id* next_candidate_cell = (Cell_id*)
        malloc_check(sizeof(Cell_id*)*2*rE);
    for (comp_t rv = 0; rv < rV; rv++){
        first_candidate_cell[rv] = EMPTY_CELL;
    }
    for (Cell_id mcc = 0; mcc < ((Cell_id) 2)*rE; mcc++){
        next_candidate_cell[mcc] = EMPTY_CELL;
    }
    #define GET_REDUCED_EDGE(mcc) (*mcc/2)
    #define FIRST_CELL(mcc, rv) (mcc = &first_candidate_cell[rv])
    #define NEXT_CELL(mcc) (mcc = &next_candidate_cell[*mcc])
    #define DELETE_CELL(mcc) (*mcc = next_candidate_cell[*mcc])
    #define IS_EMPTY(mcc) (*mcc == EMPTY_CELL)
    
    /* construct the linked list structure;
     * last_candidate_cell[ru] is the index of the last merge candidate cell
     *      of the list of adjacent candidates for component ru;
     *      useful only for constructing the list in linear time */
    Cell_id* last_candidate_cell = (Cell_id*)
        malloc_check(sizeof(index_t*)*rV);
    for (comp_t rv = 0; rv < rV; rv++){ last_candidate_cell[rv] = EMPTY_CELL; }
    for (index_t re = 0; re < rE; re++){
        comp_t ru = reduced_edges_u(re);
        comp_t rv = reduced_edges_v(re);
        if (ru == rv){ continue; }
        #define INSERT_CELL(rv, mcc) \
            if (last_candidate_cell[rv] == EMPTY_CELL){ \
                first_candidate_cell[rv] = mcc; \
                last_candidate_cell[rv] = mcc; \
            }else{ \
                next_candidate_cell[last_candidate_cell[rv]] = mcc; \
                last_candidate_cell[rv] = mcc; \
            }
        Cell_id mcc_ru = ((Cell_id) 2)*re, mcc_rv = ((Cell_id) 2)*re + 1;
        INSERT_CELL(ru, mcc_ru); INSERT_CELL(rv, mcc_rv);
    }
    free(last_candidate_cell);

    /* iterative merge following the above order */
    while (!candidates_queue.empty()){
        typename set<index_t>::iterator candidate = candidates_queue.begin();
        index_t re = *candidate;
        comp_t ru = reduced_edges_u(re);
        comp_t rv = reduced_edges_v(re);

        /**  accept the merge and remove from the queue  **/
        comp_t ro = accept_merge_candidate(re); // merge ru and rv
        if (ro != ru){ rv = ru; ru = ro; } // makes sure ru is the root
        candidates_queue.erase(candidate);
        merge_count++;

        /**  update reduced graph structure and adjacent merge candidates  **/
        Cell_id *mcc_ru, *mcc_rv;

        /* first pass on the list of rv: cleanup deleted candidates, remove
         * current merging candidate, update vertices by replacing rv by ru */
        FIRST_CELL(mcc_rv, rv);
        while (!IS_EMPTY(mcc_rv)){
            index_t re_rv = GET_REDUCED_EDGE(mcc_rv);
            if (!reduced_edge_weights[re_rv]){ DELETE_CELL(mcc_rv); continue; }
            comp_t end_re_rv;
            if (reduced_edges_u(re_rv) == rv){ 
                reduced_edges_u(re_rv) = ru;
                end_re_rv = reduced_edges_v(re_rv);
            }else{
                reduced_edges_v(re_rv) = ru;
                end_re_rv = reduced_edges_u(re_rv);
            }
            if (end_re_rv == ru){ DELETE_CELL(mcc_rv); continue; }
            NEXT_CELL(mcc_rv);
        }

        /* cleanup deleted candidates and delete current merging candidate from
         * ru list, and search candidates adjacent to both ru and rv with same
         * end vertex;
         * NOTA: bilinear time cost in orders of merging components cannot be
         * avoided; in particular, ordering lists by end vertex identifiers
         * would require reordering of all adjacent candidates of rv, bilinear
         * in order of rv and sum of orders of its adjacent candidates
         * NOTA: might be done in parallel along ru list, but current merging 
         * candidate must be removed before, and might not be worth it */
        FIRST_CELL(mcc_ru, ru);
        while (!IS_EMPTY(mcc_ru)){
            index_t re_ru = GET_REDUCED_EDGE(mcc_ru);
            if (!reduced_edge_weights[re_ru]){ DELETE_CELL(mcc_ru); continue; }
            comp_t end_re_ru = reduced_edges_u(re_ru) == ru ?
                reduced_edges_v(re_ru) : reduced_edges_u(re_ru);
            if (end_re_ru == ru){ DELETE_CELL(mcc_ru); continue; }
            for (FIRST_CELL(mcc_rv, rv); !IS_EMPTY(mcc_rv); NEXT_CELL(mcc_rv)){
                index_t re_rv = GET_REDUCED_EDGE(mcc_rv);
                comp_t end_re_rv = reduced_edges_u(re_rv) == ru ?
                    reduced_edges_v(re_rv) : reduced_edges_u(re_rv);
                if (end_re_ru == end_re_rv){
                    reduced_edge_weights[re_ru] += reduced_edge_weights[re_rv];
                    reduced_edge_weights[re_rv] = 0.0; // sum must be constant
                    if (merge_gains[re_rv] > 0.0){ /* remove from queue */
                        candidate = candidates_queue.find(re_rv);
                        candidate = candidates_queue.erase(candidate);
                        merge_gains[re_rv] = 0.0;
                    }
                    delete_merge_candidate(re_rv);
                    DELETE_CELL(mcc_rv);
                    /* NOTA: sister candidate cell for re_rv still exists in
                     * the list of adjacent candidates of end_re_rv; but this
                     * situation is flagged with zero reduced edge weight */
                    break;
                }
            }
            NEXT_CELL(mcc_ru);
        }

        /* at that point, mcc_ru is the last (empty) cell of the ru list;
         * concatenate adjacent candidate list of rv after the one of ru  */
        *mcc_ru = first_candidate_cell[rv];

        /* update all adjacent candidates */
        for (FIRST_CELL(mcc_ru, ru); !IS_EMPTY(mcc_ru); NEXT_CELL(mcc_ru)){
            index_t re = GET_REDUCED_EDGE(mcc_ru);
            if (merge_gains[re] > 0.0){ /* already in the queue */
                candidate = candidates_queue.find(re);
                candidate = candidates_queue.erase(candidate);
            }else{
                candidate = candidates_queue.end();
            }
            compute_merge_candidate(re);
            if (merge_gains[re] > 0.0){
                candidates_queue.insert(candidate, re);
            }
        }
    } // end while candidates queue not empty

    free(first_candidate_cell); free(next_candidate_cell);
    } // end if num_pos_candidates

    if (num_neg_candidates){
    /**  merge candidates with negative gains;
     **  these are less important, no update of adajacent candidates;
     **  only sort once and merge in that order **/
    index_t bufsize = num_neg_candidates;
    index_t* neg_candidates = (index_t*) malloc_check(sizeof(index_t)*bufsize);
    num_neg_candidates = 0; // recounting
    for (index_t re = 0; re < rE; re++){
        if (merge_values[re]){
            if (num_neg_candidates == bufsize){
                bufsize += bufsize/2 + 1;
                neg_candidates = (index_t*) realloc_check(neg_candidates,
                    sizeof(index_t)*bufsize);
            }
            neg_candidates[num_neg_candidates++] = re;
        }
    }
    sort(neg_candidates, neg_candidates + num_neg_candidates,
        [_merge_gains] (index_t re1, index_t re2) -> bool
        { return _merge_gains[re1] > _merge_gains[re2]; });
    for (index_t mc = 0; mc < num_neg_candidates; mc++){
        index_t re = neg_candidates[mc];
        /* ensure candidate info is up-to-date */
        comp_t ru = get_merge_chain_root(reduced_edges_u(re));
        comp_t rv = get_merge_chain_root(reduced_edges_v(re));
        if (ru == rv){
            delete_merge_candidate(re);
        }else{
            reduced_edges_u(re) = ru;
            reduced_edges_v(re) = rv;
            compute_merge_candidate(re);
            if (merge_values[re]){
                accept_merge_candidate(re);
                merge_count++;
            }
        }
    }

    free(neg_candidates);
    } // end if num_neg_candidates

    free(merge_gains); free(merge_values);
    return merge_count;
}

template class Cp_d0<float, int32_t, int16_t>;
template class Cp_d0<double, int32_t, int16_t>;
template class Cp_d0<float, int32_t, int32_t>;
template class Cp_d0<double, int32_t, int32_t>;
