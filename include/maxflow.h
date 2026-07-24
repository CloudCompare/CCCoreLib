/* maxflow.hpp */
/* modified from graph.h by Hugo Raguet 2020, for use with cut-pursuit
 * algorithms */
/*
    Copyright Vladimir Kolmogorov and Yuri Boykov

    This file is part of MAXFLOW.

    MAXFLOW is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MAXFLOW is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MAXFLOW.  If not, see <http://www.gnu.org/licenses/>.
=============================================================================*/

#pragma once
#include "block.h"

/* index_t is an integer type able to hold the number of nodes and of edges;
 * flow_t is a numeric type for the flow (capacities) */
template <typename index_t, typename flow_t> class Maxflow
{
public:
    Maxflow(index_t node_num, index_t edge_num);

    ~Maxflow();

    void add_edge(index_t i, index_t j);

    flow_t& terminal_capacity(index_t i);

    void set_edge_capacities(index_t e, flow_t cap, flow_t rev_cap);

    /* retrieve (signed) flow passing through a given edge
       second parameter is the initial capacity */
    flow_t get_edge_flow(index_t e, flow_t cap);

    void maxflow();

    bool is_sink(index_t i, bool default_side = false);

private:
    struct node;
    struct arc;

    // internal variables and functions

    struct node
    {
        arc *first; // first outcoming arc
        arc *parent; // node's parent
        node *next; // pointer to the next active node
        index_t TS; // timestamp showing when DIST was computed
        index_t DIST; // distance to the terminal
        bool is_sink : 1; // indicate source or sink tree, if parent not null
        /* positive if connected to source, negative if connected to sink */
        flow_t term_res_cap;
    };

    struct arc
    {
        node* head; // node the arc points to
        arc* next; // next arc with the same originating node
        arc* sister; // reverse arc
        flow_t res_cap; // residual capacity
    };

    struct nodeptr
    {
        node        *ptr;
        nodeptr        *next;
    };
    static const int NODEPTR_BLOCK_SIZE = 128;

    node *nodes, *node_last; 
    arc *arcs, *arc_last; 

    /* special constants for parent arcs */
    arc reserved_terminal_arc; // the parent is an arc to terminal
    arc* const terminal;
    arc reserved_orphan_arc; // no parent
    arc* const orphan;

    DBlock<nodeptr>        *nodeptr_block;

    node *queue_first[2], *queue_last[2]; // list of active nodes
    nodeptr *orphan_first, *orphan_last; // list of pointers to orphans
    index_t TIME; // monotonically increasing global counter

    // functions for processing active list
    void set_active(node *i);
    node *next_active();

    // functions for processing orphans list
    void set_orphan_front(node* i); // add to the beginning of the list
    void set_orphan_rear(node* i);  // add to the end of the list

    void maxflow_init();             // called if reuse_trees == false
    void augment(arc *middle_arc);
    void process_source_orphan(node *i);
    void process_sink_orphan(node *i);
};

#define TPL template <typename index_t, typename flow_t>
#define MXFL Maxflow<index_t, flow_t>

TPL inline void MXFL::add_edge(index_t _i, index_t _j)
{
    arc *a = arc_last++;
    arc *a_rev = arc_last++;

    node* i = nodes + _i;
    node* j = nodes + _j;

    a->sister = a_rev;
    a_rev->sister = a;
    a->next = i->first;
    i->first = a;
    a_rev->next = j->first;
    j->first = a_rev;
    a->head = j;
    a_rev->head = i;
}

TPL inline flow_t& MXFL::terminal_capacity(index_t i)
{
    return nodes[i].term_res_cap;
}

TPL inline void MXFL::set_edge_capacities(index_t e, flow_t cap,
    flow_t rev_cap)
{
    arc* a = arcs + (size_t) 2*e;
    a->res_cap = cap;
    (a + 1)->res_cap = rev_cap;
}

TPL inline flow_t MXFL::get_edge_flow(index_t e, flow_t cap)
{
    arc* a = arcs + (size_t) 2*e;
    return cap - a->res_cap;
}

TPL inline bool MXFL::is_sink(index_t i, bool default_side)
{
    return nodes[i].parent ? nodes[i].is_sink : default_side;
}

#undef TPL
#undef MXFL
