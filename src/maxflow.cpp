/* maxflow.cpp */

#include <cstdlib>
#include <limits>
#include <cstdint> // for instantiation
#include "maxflow.h"

/* special constants for parent arcs */
#define TERMINAL terminal
#define ORPHAN orphan
/* infinite distance to the terminal */
#define INFINITE_D (std::numeric_limits<index_t>::max())

#define TPL template <typename index_t, typename flow_t>
#define MXFL Maxflow<index_t, flow_t>

using namespace std;

TPL MXFL::Maxflow(index_t node_num, index_t edge_num)
    : terminal(&reserved_terminal_arc), orphan(&reserved_orphan_arc),
      nodeptr_block(nullptr)
{
    nodes = (node*) malloc(sizeof(node)*node_num);
    arcs = (arc*) malloc(sizeof(arc)*2*edge_num);
    if (!nodes || !arcs) {
        cerr << "Maxflow: not enough memory." << endl;
        exit(EXIT_FAILURE);
    }

    node_last = nodes + node_num;
    arc_last = arcs; // arcs not created yet

    for (node* i = nodes; i < node_last; i++){ i->first = nullptr; }
}

TPL MXFL::~Maxflow()
{
    if (nodeptr_block){ 
        delete nodeptr_block; 
        nodeptr_block = nullptr; 
    }
    free(nodes);
    free(arcs);
}

/***********************************************************************/

/*
    Functions for processing active list.
    i->next points to the next node in the list
    (or to i, if i is the last node in the list).
    If i->next is nullptr iff i is not in the list.

    There are two queues. Active nodes are added
    to the end of the second queue and read from
    the front of the first queue. If the first queue
    is empty, it is replaced by the second queue
    (and the second queue becomes empty).
*/


TPL inline void MXFL::set_active(node *i)
{
    if (!i->next){
        /* it's not in the list yet */
        if (queue_last[1]) queue_last[1]->next = i;
        else               queue_first[1]      = i;
        queue_last[1] = i;
        i->next = i;
    }
}

/*
    Returns the next active node.
    If it is connected to the sink, it stays in the list, (???)
    otherwise it is removed from the list
*/
TPL inline typename MXFL::node* MXFL::next_active()
{
    node *i;

    while (true){
        if (!(i = queue_first[0])){
            queue_first[0] = i = queue_first[1];
            queue_last[0]  = queue_last[1];
            queue_first[1] = nullptr;
            queue_last[1]  = nullptr;
            if (!i) return nullptr;
        }

        /* remove it from the active list */
        if (i->next == i) queue_first[0] = queue_last[0] = nullptr;
        else              queue_first[0] = i->next;
        i->next = nullptr;

        /* a node in the list is active iff it has a parent */
        if (i->parent) return i;
    }
}

/***********************************************************************/

TPL inline void MXFL::set_orphan_front(node *i)
{
    nodeptr *np;
    i->parent = ORPHAN;
    np = nodeptr_block->New();
    np->ptr = i;
    np->next = orphan_first;
    orphan_first = np;
}

TPL inline void MXFL::set_orphan_rear(node *i)
{
    nodeptr *np;
    i->parent = ORPHAN;
    np = nodeptr_block->New();
    np->ptr = i;
    if (orphan_last) orphan_last->next = np;
    else             orphan_first      = np;
    orphan_last = np;
    np->next = nullptr;
}

/***********************************************************************/

TPL void MXFL::maxflow_init()
{
    node *i;

    queue_first[0] = queue_last[0] = nullptr;
    queue_first[1] = queue_last[1] = nullptr;
    orphan_first = nullptr;

    TIME = 0;

    for (i = nodes; i < node_last; i++){
        i->next = nullptr;
        i->TS = TIME;
        if (i->term_res_cap > 0){
            /* i is connected to the source */
            i->is_sink = false;
            i->parent = TERMINAL;
            set_active(i);
            i->DIST = 1;
        }else if (i->term_res_cap < 0){
            /* i is connected to the sink */
            i->is_sink = true;
            i->parent = TERMINAL;
            set_active(i);
            i->DIST = 1;
        }else{
            i->parent = nullptr;
        }
    }
}

TPL void MXFL::augment(arc *middle_arc)
{
    node *i;
    arc *a;
    flow_t bottleneck;


    /* 1. Finding bottleneck capacity */
    /* 1a - the source tree */
    bottleneck = middle_arc->res_cap;
    for (i = middle_arc->sister->head; ; i = a->head)
    {
        a = i->parent;
        if (a == TERMINAL) break;
        if (bottleneck > a->sister->res_cap) bottleneck = a->sister->res_cap;
    }
    if (bottleneck > i->term_res_cap) bottleneck = i->term_res_cap;
    /* 1b - the sink tree */
    for (i = middle_arc->head; ; i = a->head)
    {
        a = i->parent;
        if (a == TERMINAL) break;
        if (bottleneck > a->res_cap) bottleneck = a->res_cap;
    }
    if (bottleneck > - i->term_res_cap) bottleneck = - i->term_res_cap;


    /* 2. Augmenting */
    /* 2a - the source tree */
    middle_arc->sister->res_cap += bottleneck;
    middle_arc->res_cap -= bottleneck;
    for (i = middle_arc->sister->head; ; i = a->head)
    {
        a = i->parent;
        if (a == TERMINAL) break;
        a->res_cap += bottleneck;
        a->sister->res_cap -= bottleneck;
        if (!a->sister->res_cap){ set_orphan_front(i); }
    }
    i->term_res_cap -= bottleneck;
    if (!i->term_res_cap){ set_orphan_front(i); }
    /* 2b - the sink tree */
    for (i = middle_arc->head; ; i = a->head)
    {
        a = i->parent;
        if (a == TERMINAL) break;
        a->sister->res_cap += bottleneck;
        a->res_cap -= bottleneck;
        if (!a->res_cap){ set_orphan_front(i); }
    }
    i->term_res_cap += bottleneck;
    if (!i->term_res_cap){ set_orphan_front(i); }
}

/***********************************************************************/

TPL void MXFL::process_source_orphan(node *i)
{
    node *j;
    arc *a0, *a0_min = nullptr, *a;
    index_t d, d_min = INFINITE_D;

    /* trying to find a new parent */
    for (a0 = i->first; a0; a0 = a0->next)
    if (a0->sister->res_cap){
        j = a0->head;
        if (!j->is_sink && (a = j->parent)){
            /* checking the origin of j */
            d = 0;
            while (true){
                if (j->TS == TIME){
                    d += j->DIST;
                    break;
                }
                a = j->parent;
                d++;
                if (a == TERMINAL){
                    j->TS = TIME;
                    j->DIST = 1;
                    break;
                }
                if (a == ORPHAN){
                    d = INFINITE_D;
                    break;
                }
                j = a->head;
            }
            if (d < INFINITE_D){ /* j originates from the source - done */
                if (d < d_min){
                    a0_min = a0;
                    d_min = d;
                }
                /* set marks along the path */
                for (j = a0->head; j->TS != TIME; j = j->parent->head){
                    j->TS = TIME;
                    j->DIST = d--;
                }
            }
        }
    }

    if ((i->parent = a0_min)){
        i->TS = TIME;
        i->DIST = d_min + 1;
    }else{
        /* process neighbors */
        for (a0 = i->first; a0; a0 = a0->next){
            j = a0->head;
            if (!j->is_sink && (a = j->parent)){
                if (a0->sister->res_cap){ set_active(j); }
                if (a != TERMINAL && a != ORPHAN && a->head == i){
                    set_orphan_rear(j); 
                }
            }
        }
    }
}

TPL void MXFL::process_sink_orphan(node *i)
{
    node *j;
    arc *a0, *a0_min = nullptr, *a;
    index_t d, d_min = INFINITE_D;

    /* trying to find a new parent */
    for (a0 = i->first; a0; a0 = a0->next)
    if (a0->res_cap){
        j = a0->head;
        if (j->is_sink && (a = j->parent)){
            /* checking the origin of j */
            d = 0;
            while (true){
                if (j->TS == TIME){
                    d += j->DIST;
                    break;
                }
                a = j->parent;
                d++;
                if (a == TERMINAL){
                    j->TS = TIME;
                    j->DIST = 1;
                    break;
                }
                if (a == ORPHAN){
                    d = INFINITE_D;
                    break;
                }
                j = a->head;
            }
            if (d < INFINITE_D){ /* j originates from the sink - done */
                if (d < d_min){
                    a0_min = a0;
                    d_min = d;
                }
                /* set marks along the path */
                for (j = a0->head; j->TS != TIME; j = j->parent->head){
                    j->TS = TIME;
                    j->DIST = d--;
                }
            }
        }
    }

    if ((i->parent = a0_min)){
        i->TS = TIME;
        i->DIST = d_min + 1;
    }else{
        /* process neighbors */
        for (a0 = i->first; a0; a0 = a0->next){
            j = a0->head;
            if (j->is_sink && (a = j->parent)){
                if (a0->res_cap) set_active(j);
                if (a != TERMINAL && a != ORPHAN && a->head == i){
                    set_orphan_rear(j);
                }
            }
        }
    }
}

/***********************************************************************/

TPL void MXFL::maxflow()
{
    node *i, *j, *current_node = nullptr;
    arc *a;
    nodeptr *np, *np_next;

    if (!nodeptr_block){
        nodeptr_block = new DBlock<nodeptr>(NODEPTR_BLOCK_SIZE);
    }

    maxflow_init();

    while (true){

        if ((i = current_node)){
            i->next = nullptr; /* remove active flag */
            if (!i->parent) i = nullptr;
        }

        if (!i){ if (!(i = next_active())){ break; } }

        /* growth */
        if (!i->is_sink){
            /* grow source tree */
            for (a = i->first; a; a = a->next)
            if (a->res_cap){
                j = a->head;
                if (!j->parent){
                    j->is_sink = false;
                    j->parent = a->sister;
                    j->TS = i->TS;
                    j->DIST = i->DIST + 1;
                    set_active(j);
                }else if (j->is_sink){
                    break;
                }else if (j->TS <= i->TS && j->DIST > i->DIST){
                    /* heuristic - trying to make the distance from j to the
                       source shorter */
                    j->parent = a->sister;
                    j->TS = i->TS;
                    j->DIST = i->DIST + 1;
                }
            }
        }else{
            /* grow sink tree */
            for (a = i->first; a; a = a->next)
            if (a->sister->res_cap){
                j = a->head;
                if (!j->parent){
                    j->is_sink = true;
                    j->parent = a->sister;
                    j->TS = i->TS;
                    j->DIST = i->DIST + 1;
                    set_active(j);
                }else if (!j->is_sink){
                    a = a->sister; break;
                }else if (j->TS <= i->TS && j->DIST > i->DIST){
                    /* heuristic - trying to make the distance from j to the
                       sink shorter */
                    j->parent = a->sister;
                    j->TS = i->TS;
                    j->DIST = i->DIST + 1;
                }
            }
        }

        if (++TIME <= 0){
        /* changed type from long to index_t */
        /* can't we prove this won't overflow? */
            cerr << "Maxflow: timestamp overflow." << endl;
            exit(EXIT_FAILURE);
        }

        if (a){ /* found a valid path; a is the middle arc */
            i->next = i; // set active flag
            current_node = i;

            augment(a);

            /* adoption */
            while ((np = orphan_first)){
                np_next = np->next;
                np->next = nullptr;

                while ((np = orphan_first)){
                    orphan_first = np->next;
                    i = np->ptr;
                    nodeptr_block->Delete(np);
                    if (!orphan_first) orphan_last = nullptr;
                    if (i->is_sink) process_sink_orphan(i);
                    else            process_source_orphan(i);
                }

                orphan_first = np_next;
            }
        }else{
            current_node = nullptr;
        }

    } // end main loop

    delete nodeptr_block; 
    nodeptr_block = nullptr; 
}

/**  instantiate for compilation  **/
template class Maxflow<int32_t, float>;
template class Maxflow<int32_t, double>;
