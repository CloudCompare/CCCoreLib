/*=============================================================================
 * Adapted/integrated by Ioannis Farmakis (2026) from the original 
 * parallel-cut-pursuit project implementation by Hugo Raguet 2018
 * (https://github.com/1a7r0ch3/parallel-cut-pursuit), 2026, for use in 
 * CloudCompare: merged and de-templated from the original cut_pursuit.hpp,
 * cut_pursuit_d0.hpp and cp_d0_dist.hpp to implement the d0 distance only.
 * See the integration notes in CutPursuit.h for full details.
 *===========================================================================*/
#include <set>
#include <algorithm>
#include <random>

// local
#include <CutPursuit.h>
#include <GenericProgressCallback.h>

#define ADD1(i) (((size_t)i) + (size_t)1) // avoid overflows
#define EDGE_WEIGHTS_(e) (edge_weights ? edge_weights[(e)] : homo_edge_weight)
#define VERT_WEIGHTS_(v) (vert_weights ? vert_weights[(v)] : (float) 1.0)
#define COOR_WEIGHTS_(d) (coor_weights ? coor_weights[(d)] : (float) 1.0)

/** specific flags **/
/* enusre number of components do not exceed integer representation */
#define MAX_NUM_COMP (std::numeric_limits<int32_t>::max())
/* use maximum number of components; no component can have this identifier */
#define NOT_ASSIGNED (std::numeric_limits<int32_t>::max())
#define CHAIN_END (std::numeric_limits<int32_t>::max())
#define NO_COMP (std::numeric_limits<int32_t>::max())
#define ASSIGNED ((int32_t)0)
#define ASSIGNED_ROOT ((int32_t)1)     // must differ from ASSIGNED
#define ASSIGNED_ROOT_SAT ((int32_t)2) // must differ from ASSIGNED_ROOT
#define NOT_SATURATED ((int32_t)1)     // must differ from ASSIGNED
/* use maximum number of edges; no edge can have this identifier */
#define NO_EDGE (std::numeric_limits<int32_t>::max())
#define NOT_ISOLATED (std::numeric_limits<int32_t>::max())
#define ISOLATED ((int32_t)0)


using namespace std;

CP::CP(int32_t V, int32_t E, const int32_t* first_edge, const int32_t* adj_vertices, const float* Y, size_t D)
    : V(V)
    , E(E)
    , first_edge(first_edge)
    , adj_vertices(adj_vertices)
    , D(D)
    , Y(Y)
{
	/* real type with infinity is handy */
	static_assert(numeric_limits<float>::has_infinity,
	              "Cut-pursuit: float must be able to represent infinity.");

	/* edge activation */
	edge_status = (Edge_status*)malloc_check(sizeof(Edge_status) * E);
	for (int32_t e = 0; e < E; e++)
	{
		bind(e);
	}

	/* reduced graph **/
	rV               = 1;
	rE               = 0;
	last_rV          = 0;
	saturated_comp   = 0;
	saturated_vert   = 0;
	edge_weights     = nullptr;
	homo_edge_weight = 1.0;
	comp_assign = last_comp_assign = nullptr;
	comp_list = first_vertex = index_in_comp = nullptr;
	is_saturated                             = nullptr;
	reduced_edge_weights                     = nullptr;
	reduced_edges                            = nullptr;
	elapsed_time                             = nullptr;
	objective_values = iterate_evolution = nullptr;
	rX = last_rX = nullptr;

	/* some algorithmic parameters */
	it_max                = 10;
	verbose               = 1000;
	dif_tol               = 0.0;
	eps                   = numeric_limits<float>::epsilon();
	K                     = 2;
	split_iter_num        = 1;
	split_damp_ratio      = 1.0;
	split_values_init_num = 1;
	split_values_iter_num = 1;

	max_num_threads        = omp_get_max_threads();
	balance_parallel_split = max_num_threads > 1 && compute_num_threads(maxflow_complexity()) > 1;
	max_split_size         = V;

	vert_weights = coor_weights = nullptr;
    comp_weights = nullptr;
    merge_gains = nullptr;
    merge_values = nullptr;

    loss = quadratic_loss();
    fYY = 0.0;
    fXY = real_inf();

    min_comp_weight = 0.0;
}

CP::~CP()
{
	free(edge_status);
	free(comp_assign);
	free(last_comp_assign);
	free(first_vertex);
	free(comp_list);
	free(index_in_comp);
	free(is_saturated);
	free(reduced_edges);
	free(reduced_edge_weights);
	free(rX);
	free(last_rX);
	free(comp_weights);
}

void CP::reset_edges()
{
	for (int32_t e = 0; e < E; e++)
	{
		bind(e);
	}
}

void CP::set_edge_weights(const float* edge_weights,
                              float        homo_edge_weight)
{
	this->edge_weights     = edge_weights;
	this->homo_edge_weight = homo_edge_weight;
}

void CP::set_monitoring_arrays(float* objective_values,
                                   double* elapsed_time,
                                   float* iterate_evolution)
{
	this->objective_values  = objective_values;
	this->elapsed_time      = elapsed_time;
	this->iterate_evolution = iterate_evolution;
}

float CP::distance(const float* Yv, const float* Xv) const
{
    float dist = 0.0;
    size_t Q = loss; // number of coordinates for quadratic part
    if (Q != 0){ /* quadratic part */
        for (size_t d = 0; d < Q; d++){
            dist += COOR_WEIGHTS_(d)*(Yv[d] - Xv[d])*(Yv[d] - Xv[d]);
        }
    }
    if (Q != D){ /* smoothed Kullback-Leibler;
                    just compute cross-entropy here */
        float distKL = 0.0;
        const float s = loss < 1.0 ? loss : eps;
        const float c = 1.0 - s;
        const float u = s/(D - Q);
        for (size_t d = Q; d < D; d++){
            distKL -= (u + c*Yv[d])*log(u + c*Xv[d]);
        }
        dist += COOR_WEIGHTS_(Q)*distKL;
    }
    return dist;
}

void CP::set_loss(float loss, const float* Y,
    const float* vert_weights, const float* coor_weights)
{
    if (loss < 0.0 || (loss > 1.0 && ((size_t) loss) != loss) || loss > D){
        cerr << "Cut-pursuit d0 distance: loss parameter should be positive,"
            "either in (0,1) or an integer that do not exceed the dimension "
            "(" << loss << " given)." << endl;
        exit(EXIT_FAILURE);
    }
    if (loss == 0.0){ loss = eps; } // avoid singularities
    this->loss = loss;
    if (Y){ this->Y = Y; }
    this->vert_weights = vert_weights;
    if (0.0 < loss && loss < 1.0 && coor_weights){
        cerr << "Cut-pursuit d0 distance: no sense in weighting coordinates of"
            " the probability space in Kullback-Leibler divergence." << endl;
        exit(EXIT_FAILURE);
    }
    this->coor_weights = coor_weights;
    if (loss == quadratic_loss()){ fYY = 0.0; return; }
    /* recompute the constant dist(Y, Y) for Kullback-Leibler */
    const size_t Q = loss; // number of coordinates for quadratic part
    const float s = loss < 1.0 ? loss : eps;
    const float c = 1.0 - s;
    const float u = s/(D - Q);
    float fYY_par = 0.0; // auxiliary variable for parallel region

    for (int32_t v = 0; v < V; v++){
        const float* Yv = Y + D*v;
        float H_Yv = 0.0;
        for (size_t d = Q; d < D; d++){
            H_Yv -= (u + c*Yv[d])*log(u + c*Yv[d]);
        }
        fYY_par += VERT_WEIGHTS_(v)*H_Yv;
    }
    fYY = fYY_par;
}

void CP::set_components(int32_t rV, int32_t* comp_assign)
{
	if (rV > 1 && !comp_assign)
	{
		cerr << "Cut-pursuit: if an initial number of components greater than "
		        "one is given, components assignment must be provided."
		     << endl;
		exit(EXIT_FAILURE);
	}
	this->rV          = rV;
	this->comp_assign = comp_assign;
}

void CP::set_cp_param(float dif_tol, int it_max, int verbose, float eps)
{
	this->dif_tol = dif_tol;
	this->it_max  = it_max;
	this->verbose = verbose;
	this->eps     = 0.0 < dif_tol && dif_tol < eps ? dif_tol : eps;
}

void CP::set_split_param(int32_t max_split_size, int32_t K, int split_iter_num, float split_damp_ratio, int split_values_init_num, int split_values_iter_num)
{
	if (K < 2)
	{
		cerr << "Cut-pursuit: there must be at least two alternative values"
		        "in the split ("
		     << K << " specified)." << endl;
		exit(EXIT_FAILURE);
	}
	if (split_iter_num < 1)
	{
		cerr << "Cut-pursuit: there must be at least one iteration in the "
		        "split ("
		     << split_iter_num << " specified)." << endl;
		exit(EXIT_FAILURE);
	}
	if (split_damp_ratio <= 0 || split_damp_ratio > 1.0)
	{
		cerr << "Cut-pursuit: split damping ratio must be between zero "
		        "excluded and one included ("
		     << split_damp_ratio << " specified)."
		     << endl;
		exit(EXIT_FAILURE);
	}
	if (split_values_init_num < 1)
	{
		cerr << "Cut-pursuit: split values must be computed at least once per"
		        "split ("
		     << split_values_init_num << " specified)." << endl;
		exit(EXIT_FAILURE);
	}
	if (split_values_iter_num < 1)
	{
		cerr << "Cut-pursuit: split values must be updated at least once per"
		        "split ("
		     << split_values_iter_num << " specified)." << endl;
		exit(EXIT_FAILURE);
	}
	this->max_split_size        = max_split_size;
	this->K                     = K;
	this->split_iter_num        = split_iter_num;
	this->split_damp_ratio      = split_damp_ratio;
	this->split_values_init_num = split_values_init_num;
	this->split_values_iter_num = split_values_iter_num;
}

void CP::set_min_comp_weight(float min_comp_weight)
{
    if (min_comp_weight < 0.0){
        cerr << "Cut-pursuit d0 distance: min component weight parameter "
            "should be positive (" << min_comp_weight << " given)." << endl;
        exit(EXIT_FAILURE);
    }
    this->min_comp_weight = min_comp_weight;
}

float CP::fv(int32_t v, const float* Xv) const
{ return VERT_WEIGHTS_(v)*distance(Y + D*v, Xv); }

float CP::compute_f_sum() const
{
    float f = 0.0;
    #pragma omp parallel for schedule(dynamic) NUM_THREADS(D*V, rV) \
        reduction(+:f)
    for (int32_t rv = 0; rv < rV; rv++){
        float* rXv = rX + D*rv;
        for (int32_t i = first_vertex[rv]; i < first_vertex[rv + 1]; i++){
            f += fv(comp_list[i], rXv);
        }
    }
    return f;
}

float CP::compute_f() const
{
    return fXY == real_inf() ? compute_f_sum() - fYY : fXY - fYY;
}

float CP::compute_graph_d0() const
{
    float weighted_contour_length = 0.0;
    #pragma omp parallel for schedule(static) NUM_THREADS(rE) \
        reduction(+:weighted_contour_length)
    for (int32_t re = 0; re < rE; re++){
        weighted_contour_length += reduced_edge_weights[re];
    }
    return weighted_contour_length;
}

float CP::compute_objective() const
{ return compute_f() + compute_graph_d0(); } // f(x) + ||x||_d0


void CP::set_parallel_param(int  max_num_threads,
                                bool balance_parallel_split)
{
	if (max_num_threads <= 0)
	{
		max_num_threads = omp_get_max_threads();
	}
	this->max_num_threads        = max_num_threads;
	this->balance_parallel_split = balance_parallel_split
	                               && max_num_threads > 1
	                               && compute_num_threads(split_complexity()) > 1;
}

int32_t CP::get_components(const int32_t**  comp_assign,
                              const int32_t** first_vertex,
                              const int32_t** comp_list) const
{
	if (comp_assign)
	{
		*comp_assign = this->comp_assign;
	}
	if (first_vertex)
	{
		*first_vertex = this->first_vertex;
	}
	if (comp_list)
	{
		*comp_list = this->comp_list;
	}
	return this->rV;
}

void CP::solve_reduced_problem()
{
    free(comp_weights);
    comp_weights = (float*) malloc_check(sizeof(float)*rV);

    for (int32_t rv = 0; rv < rV; rv++){
        float* rXv = rX + D*rv;
        comp_weights[rv] = 0.0;
        for (size_t d = 0; d < D; d++){ rXv[d] = 0.0; }
        for (int32_t i = first_vertex[rv]; i < first_vertex[rv + 1]; i++){
            int32_t v = comp_list[i];
            comp_weights[rv] += VERT_WEIGHTS_(v);
            const float* Yv = Y + D*v;
            for (size_t d = 0; d < D; d++){ rXv[d] += VERT_WEIGHTS_(v)*Yv[d]; }
        }
        if (comp_weights[rv] <= 0.0){
            cerr << "Cut-pursuit d0 distance: nonpositive total component "
                "weight; something went wrong." << endl;
            exit(EXIT_FAILURE);
        }
        for (size_t d = 0; d < D; d++){ rXv[d] /= comp_weights[rv]; }
    }
}

int32_t CP::get_reduced_graph(const int32_t** reduced_edges,
                                  const float** reduced_edge_weights)
{

	if (reduced_edges)
	{
		if (!this->reduced_edges)
		{
			compute_reduced_graph();
		}
		*reduced_edges = this->reduced_edges;
	}
	if (reduced_edge_weights)
	{
		*reduced_edge_weights = this->reduced_edge_weights;
	}
	return this->rE;
}

const float* CP::get_reduced_values() const
{
	return rX;
}

void CP::set_reduced_values(float* rX)
{
	this->rX = rX;
}

int CP::cut_pursuit(bool init, CCCoreLib::GenericProgressCallback* progressCb)
{
	int    it    = 0;
	double timer = 0.0;
	float dif   = real_inf();

	chrono::steady_clock::time_point start;
	if (elapsed_time)
	{
		start = chrono::steady_clock::now();
	}
	if (init)
	{
		if (verbose)
		{
			cout << "Cut-pursuit initialization:" << endl;
		}
		initialize();
		if (objective_values)
		{
			objective_values[0] = compute_objective();
		}
	}

	//progress notification (optional)
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setMethodTitle("Cut-Pursuit Segmentation");
			char infosBuffer[64];
			snprintf(infosBuffer, 64, "Computing graph partition ...");
			progressCb->setInfo(infosBuffer);
		}
		progressCb->update(0);
		progressCb->start();
	}
	CCCoreLib::NormalizedProgress nprogress(progressCb, it_max, 100);

	while (true)
	{
		if (elapsed_time)
		{
			elapsed_time[it] = timer = monitor_time(start);
		}
		if (verbose)
		{
			print_progress(it, dif, timer);
		}
		if (it == it_max || dif <= dif_tol)
		{
			break;
		}

		if (progressCb)
		{
			nprogress.oneStep();
		}

		if (verbose)
		{
			cout << "Cut-pursuit iteration " << it + 1 << " (max. " << it_max
			     << "): " << endl;
		}

		if (verbose)
		{
			cout << "\tSplit... " << flush;
		}
		int32_t activation = split();
		if (verbose)
		{
			cout << activation << " new activated edge(s)." << endl;
		}

		if (!activation)
		{ /* do not recompute reduced problem */
			saturated_comp = rV;
			saturated_vert = V;

			if (monitor_evolution())
			{
				dif = 0.0;
				if (iterate_evolution)
				{
					iterate_evolution[it] = dif;
				}
			}

			it++;

			if (objective_values)
			{
				objective_values[it] = objective_values[it - 1];
			}

			continue;
		}

		/* store previous component assignment */
		last_comp_assign = (int32_t*)malloc_check(sizeof(int32_t) * V);
		for (int32_t v = 0; v < V; v++)
		{
			last_comp_assign[v] = comp_assign[v];
		}
		last_rV = rV;
		if (monitor_evolution())
		{ /* store also last iterate values */
			last_rX = (float*)malloc_check(sizeof(float) * D * rV);
			for (size_t i = 0; i < D * rV; i++)
			{
				last_rX[i] = rX[i];
			}
		}
		/* reduced graph and components will be updated */
		free(rX);
		rX = nullptr;

		if (verbose)
		{
			cout << "\tCompute connected components... " << flush;
		}
		compute_connected_components();
		if (verbose)
		{
			cout << rV << " connected component(s), " << saturated_comp << " saturated." << endl;
		}

		if (verbose)
		{
			cout << "\tCompute reduced graph... " << flush;
		}
		compute_reduced_graph();
		if (verbose)
		{
			cout << rE << " reduced edge(s)." << endl;
		}

		if (verbose)
		{
			cout << "\tSolve reduced problem: " << endl;
		}
		rX = (float*)malloc_check(sizeof(float) * D * rV);
		solve_reduced_problem();

		if (verbose)
		{
			cout << "\tMerge... " << flush;
		}
		int32_t deactivation = merge();
		if (verbose)
		{
			cout << deactivation << " deactivated edge(s)." << endl;
		}

		if (dif_tol > 0.0 || iterate_evolution)
		{
			dif = compute_evolution();
			if (iterate_evolution)
			{
				iterate_evolution[it] = dif;
			}
			free(last_rX);
			last_rX = nullptr;
		}

		free(last_comp_assign);
		last_comp_assign = nullptr;

		it++;

		if (objective_values)
		{
			objective_values[it] = compute_objective();
		}

		free(reduced_edges);
		reduced_edges = nullptr;
		free(reduced_edge_weights);
		reduced_edge_weights = nullptr;

	} /* endwhile true */

	return it;
}

double CP::monitor_time(chrono::steady_clock::time_point start) const
{
	using namespace chrono;
	steady_clock::time_point current = steady_clock::now();
	return ((current - start).count()) * steady_clock::period::num
	       / static_cast<double>(steady_clock::period::den);
}

void CP::print_progress(int it, float dif, double timer) const
{
	if (it && monitor_evolution())
	{
		cout.precision(2);
		cout << scientific << "\trelative iterate evolution " << dif
		     << " (tol. " << dif_tol << ")\n";
	}
	cout << "\t" << rV << " connected component(s), " << saturated_comp << " saturated, and " << rE << " reduced edge(s).\n";
	if (timer > 0.0)
	{
		cout.precision(1);
		cout << fixed << "\telapsed time " << timer << " s.\n";
	}
	cout << endl;
}

void CP::single_connected_component()
{
	free(first_vertex);
	first_vertex    = (int32_t*)malloc_check(sizeof(int32_t) * 2);
	first_vertex[0] = 0;
	first_vertex[1] = V;
	rV              = 1;
	for (int32_t v = 0; v < V; v++)
	{
		comp_assign[v] = 0;
	}
	for (int32_t v = 0; v < V; v++)
	{
		comp_list[v] = v;
	}
}

void CP::assign_connected_components()
{
/* activate edges between components */
#pragma omp parallel for schedule(static) NUM_THREADS(E, V)
	for (int32_t v = 0; v < V; v++)
	{
		int32_t rv = comp_assign[v];
		for (int32_t e = first_edge[v]; e < first_edge[v + 1]; e++)
		{
			if (rv != comp_assign[adj_vertices[e]])
			{
				cut(e);
			}
		}
	}

	/* translate 'comp_assign' into dual representation 'comp_list' */
	free(first_vertex);
	first_vertex = (int32_t*)malloc_check(sizeof(int32_t) * ADD1(rV));
	for (int32_t rv = 0; rv < ADD1(rV); rv++)
	{
		first_vertex[rv] = 0;
	}
	for (int32_t v = 0; v < V; v++)
	{
		first_vertex[comp_assign[v] + 1]++;
	}
	for (int32_t rv = 1; rv < rV - 1; rv++)
	{
		first_vertex[rv + 1] += first_vertex[rv];
	}
	for (int32_t v = 0; v < V; v++)
	{
		comp_list[first_vertex[comp_assign[v]]++] = v;
	}
	for (int32_t rv = rV; rv > 0; rv--)
	{
		first_vertex[rv] = first_vertex[rv - 1];
	}
	first_vertex[0] = 0;
}

void CP::get_bind_reverse_edges(int32_t rv, int32_t*& first_edge_r, int32_t*& adj_vertices_r)
{
	const int32_t* comp_list_rv = comp_list + first_vertex[rv];
	int32_t        comp_size    = first_vertex[rv + 1] - first_vertex[rv];
	first_edge_r                = (int32_t*)malloc_check(sizeof(int32_t) * ADD1(comp_size));
	/* set index of each vertex in the component */
	for (int32_t i = 0; i < comp_size; i++)
	{
		index_in_comp[comp_list_rv[i]] = i;
	}
	/* count reverse edges for each vertex (shift by one index) */
	for (int32_t i = 0; i < ADD1(comp_size); i++)
	{
		first_edge_r[i] = 0;
	}
	for (int32_t i = 0; i < comp_size; i++)
	{
		int32_t v = comp_list_rv[i];
		for (int32_t e = first_edge[v]; e < first_edge[v + 1]; e++)
		{
			if (is_bind(e))
			{ /* keep only binding edges */
				first_edge_r[index_in_comp[adj_vertices[e]] + 1]++;
			}
		}
	}
	/* cumulative sum for actual first binding edge id for each vertex */
	first_edge_r[0] = 0;
	for (int32_t i = 2; i < ADD1(comp_size); i++)
	{
		first_edge_r[i] += first_edge_r[i - 1];
	}
	/* store adjacent vertices, using previous sum as starting indices */
	adj_vertices_r = (int32_t*)
	    malloc_check(sizeof(int32_t) * first_edge_r[comp_size]);
	for (int32_t i = 0; i < comp_size; i++)
	{
		int32_t v = comp_list_rv[i];
		for (int32_t e = first_edge[v]; e < first_edge[v + 1]; e++)
		{
			if (is_bind(e))
			{
				int32_t j           = index_in_comp[adj_vertices[e]];
				int32_t e_r         = first_edge_r[j]++;
				adj_vertices_r[e_r] = v;
			}
		}
	}
	/* first reverse edges have been shifted in the process, shift back */
	for (int32_t i = comp_size; i > 0; i--)
	{
		first_edge_r[i] = first_edge_r[i - 1];
	}
	first_edge_r[0] = 0;
}

void CP::compute_connected_components()
{
	/**  new connected components hierarchically derives from previous ones,
	 **  we can thus compute them in parallel along previous components  **/

	/* auxiliary variables for parallel region */
	int32_t  saturated_comp_par = 0;
	int32_t saturated_vert_par = 0;
	int32_t tmp_rV             = 0; // identify and count components, prevent overflow

	/** there is need to scan all edges involving a given vertex without
	 * running through all edges of the graph, so we create the list of
	 * 'reverse edges' within each component; to facilitate this, we keep the
	 * index of each vertex within its component **/
	index_in_comp = (int32_t*)malloc_check(sizeof(int32_t) * V);

#pragma omp parallel for schedule(dynamic) NUM_THREADS(2 * E, rV) \
    reduction(+ : tmp_rV, saturated_comp_par, saturated_vert_par)
	for (int32_t rv = 0; rv < rV; rv++)
	{
		int32_t comp_size = first_vertex[rv + 1] - first_vertex[rv];

		if (is_saturated[rv])
		{ /* component stays the same */
			int32_t i                 = first_vertex[rv];
			comp_assign[comp_list[i]] = ASSIGNED_ROOT_SAT; // flag the root
			for (i++; i < first_vertex[rv + 1]; i++)
			{
				comp_assign[comp_list[i]] = ASSIGNED;
			}
			saturated_comp_par++;
			saturated_vert_par += comp_size;
			tmp_rV++;
			continue;
		} /* else component has been split */

		/* cleanup assigned components */
		for (int32_t i = first_vertex[rv]; i < first_vertex[rv + 1]; i++)
		{
			comp_assign[comp_list[i]] = NOT_ASSIGNED;
		}

		/* get reverse binding edges for breadth-first search */
		int32_t *first_edge_r, *adj_vertices_r;
		get_bind_reverse_edges(rv, first_edge_r, adj_vertices_r);

		/* auxiliary component list for reordering vertices */
		int32_t* tmp_comp_list_rv = (int32_t*)
		    malloc_check(sizeof(int32_t) * comp_size);

		/**  compute the connected components  **/
		int32_t i = 0, j = 0;
		for (int32_t k = first_vertex[rv]; k < first_vertex[rv + 1]; k++)
		{
			int32_t u = comp_list[k];
			if (comp_assign[u] != NOT_ASSIGNED)
			{
				continue;
			}
			comp_assign[u] = ASSIGNED_ROOT; // flag a component's root
			/* put in connected components list */
			tmp_comp_list_rv[j++] = u;
			while (i < j)
			{ /* breadth-first search */
				int32_t v = tmp_comp_list_rv[i++];
				/* add neighbors to the connected component list */
				int32_t        e        = first_edge[v];
				int32_t        l        = index_in_comp[v];
				const int32_t* adj_vert = adj_vertices;
				while (adj_vert == adj_vertices || e < first_edge_r[l + 1])
				{
					if (adj_vert == adj_vertices)
					{
						if (e == first_edge[v + 1])
						{
							e        = first_edge_r[l];
							adj_vert = adj_vertices_r;
							continue;
						}
						else if (!is_bind(e))
						{
							e++;
							continue;
						}
					}
					int32_t w = adj_vert[e];
					if (comp_assign[w] == NOT_ASSIGNED)
					{
						comp_assign[w]        = ASSIGNED;
						tmp_comp_list_rv[j++] = w;
					}
					e++;
				}
			} /* the current connected component is complete */
			tmp_rV++;
		}
		free(first_edge_r);
		free(adj_vertices_r);

		int32_t* comp_list_rv = comp_list + first_vertex[rv];
		for (int32_t i = 0; i < comp_size; i++)
		{
			comp_list_rv[i] = tmp_comp_list_rv[i];
		}

		free(tmp_comp_list_rv);
	}

	free(index_in_comp);
	index_in_comp = nullptr;

	saturated_comp = saturated_comp_par;
	saturated_vert = saturated_vert_par;

	if (tmp_rV > MAX_NUM_COMP)
	{
		cerr << "Cut-pursuit: number of components (" << tmp_rV << ") greater "
		                                                           "than can be represented by int32_t ("
		     << MAX_NUM_COMP << ")"
		     << endl;
		exit(EXIT_FAILURE);
	}

	/**  update components lists, assignments and saturation  **/
	rV = tmp_rV;
	free(first_vertex);
	first_vertex = (int32_t*)malloc_check(sizeof(int32_t) * ADD1(rV));
	free(is_saturated);
	is_saturated = (bool*)malloc_check(sizeof(int32_t) * rV);

	int32_t rv = (int32_t)-1;
	for (int32_t i = 0; i < V; i++)
	{
		int32_t v = comp_list[i];
		if (comp_assign[v] == ASSIGNED_ROOT || comp_assign[v] == ASSIGNED_ROOT_SAT)
		{
			first_vertex[++rv] = i;
			is_saturated[rv]   = comp_assign[v] == ASSIGNED_ROOT_SAT;
		}
		comp_assign[v] = rv;
	}
	first_vertex[rV] = V;
}

void CP::compute_reduced_graph()
/* this could actually be parallelized, but is it worth the pain? */
{
	free(reduced_edges);
	free(reduced_edge_weights);

	if (rV == 1)
	{ /* reduced graph only edge from the component to itself
	   * this is only useful for solving reduced problems with
	   * certain implementations where isolated vertices must be
	   * linked to themselves */
		rE                 = 1;
		reduced_edges      = (int32_t*)malloc_check(sizeof(int32_t) * 2);
		reduced_edges_u(0) = reduced_edges_v(0) = 0;
		reduced_edge_weights                    = (float*)malloc_check(sizeof(float) * 1);
		reduced_edge_weights[0]                 = eps;
		return;
	}

	/* to avoid allocating rV*(rV - 1)/2, we work component by component;
	 * when dealing with component ru, reduced_edge_to[rv] is the identifier of
	 * the reduced edge ru -> rv, or NO_EDGE if the edge is not created yet */
	int32_t* reduced_edge_to = (int32_t*)malloc_check(sizeof(int32_t) * rV);
	/* same storage can also be used to indicate isolated vertices */
	int32_t* is_isolated = reduced_edge_to;
	for (int32_t rv = 0; rv < rV; rv++)
	{
		is_isolated[rv] = ISOLATED;
	}

	/**  get all active (cut) edges linking a component to another
	 **  forward-star representation (first_active_edge, adj_components)  **/
	int32_t* first_active_edge = (int32_t*)
	    malloc_check(sizeof(int32_t) * ADD1(rV));
	/* count the number of such edges for each component (ind shift by one) */
	for (int32_t rv = 0; rv < ADD1(rV); rv++)
	{
		first_active_edge[rv] = 0;
	}
	for (int32_t v = 0; v < V; v++)
	{
		int32_t ru = comp_assign[v];
		for (int32_t e = first_edge[v]; e < first_edge[v + 1]; e++)
		{
			if (!is_bind(e) && EDGE_WEIGHTS_(e) > 0.0)
			{
				int32_t rv = comp_assign[adj_vertices[e]];
				if (ru != rv)
				{
					/* a nonzero edge involving ru and rv exists */
					is_isolated[ru] = is_isolated[rv] = NOT_ISOLATED;
					if (ru < rv)
					{ // count only undirected edges
						first_active_edge[ru + 1]++;
					}
					else
					{
						first_active_edge[rv + 1]++;
					}
				}
			}
		}
	}
	/* cumulative sum, giving first active edge id for each vertex */
	for (int32_t rv = 2; rv < ADD1(rV); rv++)
	{
		first_active_edge[rv] += first_active_edge[rv - 1];
	}
	/* store adjacent components and edge weights using previous sum as
	 * starting indices */
	int32_t* adj_components = (int32_t*)
	    malloc_check(sizeof(int32_t) * first_active_edge[rV]);
	float* active_edge_weights = edge_weights ? (float*)malloc_check(sizeof(float) * first_active_edge[rV]) : nullptr;
	for (int32_t v = 0; v < V; v++)
	{
		int32_t ru = comp_assign[v];
		for (int32_t e = first_edge[v]; e < first_edge[v + 1]; e++)
		{
			if (!is_bind(e) && EDGE_WEIGHTS_(e) > 0.0)
			{
				int32_t  rv = comp_assign[adj_vertices[e]];
				int32_t ae = NO_EDGE;
				if (ru < rv)
				{ // count only undirected edges
					ae                 = first_active_edge[ru]++;
					adj_components[ae] = rv;
				}
				else if (rv < ru)
				{
					ae                 = first_active_edge[rv]++;
					adj_components[ae] = ru;
				}
				if (edge_weights && ae != NO_EDGE)
				{
					active_edge_weights[ae] = edge_weights[e];
				}
			}
		}
	}
	/* first active edges have been shifted in the process, shift back */
	for (int32_t rv = rV; rv > 0; rv--)
	{
		first_active_edge[rv] = first_active_edge[rv - 1];
	}
	first_active_edge[0] = 0;

	/* temporary buffer size */
	size_t bufsize = rE > rV * (double)E / V ? rE : rV * (double)E / V;

	reduced_edges        = (int32_t*)malloc_check(sizeof(int32_t) * 2 * bufsize);
	reduced_edge_weights = (float*)malloc_check(sizeof(float) * bufsize);

	/**  convert to edge list representation with weights  **/

	rE              = 0; // current number of reduced edges
	int32_t last_rE = 0; // keep track of number of processed edges
	for (int32_t ru = 0; ru < rV; ru++)
	{ /* iterate over the components */

		if (is_isolated[ru] == ISOLATED)
		{ /* this is only useful for solving
		   * reduced problems with certain implementations where isolated
		   * vertices must be linked to themselves */
			if (rE == bufsize)
			{ // reach buffer size
				bufsize += bufsize / 2 + 1;
				reduced_edges        = (int32_t*)realloc_check(reduced_edges,
				                                              sizeof(int32_t) * 2 * bufsize);
				reduced_edge_weights = (float*)realloc_check(
				    reduced_edge_weights, sizeof(float) * bufsize);
			}
			reduced_edges_u(rE) = reduced_edges_v(rE) = ru;
			reduced_edge_weights[rE++]                = eps;
			continue;
		}

		for (int32_t ae = first_active_edge[ru];
		     ae < first_active_edge[ru + 1];
		     ae++)
		{
			float  edge_weight = edge_weights ? active_edge_weights[ae]
			                                   : homo_edge_weight;
			int32_t  rv          = adj_components[ae];
			int32_t re          = reduced_edge_to[rv];
			if (re == NO_EDGE)
			{ // a new edge must be created
				if (rE == bufsize)
				{ // reach buffer size
					bufsize += bufsize / 2 + 1;
					reduced_edges        = (int32_t*)realloc_check(reduced_edges,
					                                              sizeof(int32_t) * 2 * bufsize);
					reduced_edge_weights = (float*)realloc_check(
					    reduced_edge_weights, sizeof(float) * bufsize);
				}
				reduced_edges_u(rE)      = ru;
				reduced_edges_v(rE)      = rv;
				reduced_edge_weights[rE] = edge_weight;
				reduced_edge_to[rv]      = rE++;
			}
			else
			{ /* edge already exists */
				reduced_edge_weights[re] += edge_weight;
			}
		}

		/* reset reduced_edge_to */
		for (; last_rE < rE; last_rE++)
		{
			reduced_edge_to[reduced_edges_v(last_rE)] = NO_EDGE;
		}
	}

	free(adj_components);
	free(active_edge_weights);
	free(first_active_edge);
	free(reduced_edge_to);

	if (bufsize > rE)
	{
		reduced_edges        = (int32_t*)realloc_check(reduced_edges,
		                                              sizeof(int32_t) * 2 * rE);
		reduced_edge_weights = (float*)realloc_check(reduced_edge_weights,
		                                              sizeof(float) * rE);
	}
}

void CP::initialize()
{
	free(rX);
	if (!comp_assign)
	{
		comp_assign = (int32_t*)malloc_check(sizeof(int32_t) * V);
	}
	if (!comp_list)
	{
		comp_list = (int32_t*)malloc_check(sizeof(int32_t) * V);
	}

	last_rV = 0;

	reset_edges();

	if (rV > 1)
	{
		assign_connected_components();
	}
	else
	{
		single_connected_component();
	}

	/* start with no saturated component */
	free(is_saturated);
	is_saturated = (bool*)malloc_check(sizeof(bool) * rV);
	for (int32_t rv = 0; rv < rV; rv++)
	{
		is_saturated[rv] = false;
	}

	compute_reduced_graph();
	rX = (float*)malloc_check(sizeof(float) * D * rV);
	solve_reduced_problem();
	merge();
}

int CP::balance_split(int32_t& rV_big, int32_t& rV_new, int32_t*& first_vertex_big)
/* rV_big will be the number of big components to be split
   rV_new will be the number of resulting new components
   first_vertex_big will store info on list of vertices of big components */
{
	int num_thrds = compute_num_threads(split_complexity());

	/**  sort components by decreasing size
	 * even if no balancing is required, sorting is useful for dynamic
	 * scheduling of parallel split */
	if (num_thrds > 1 || max_split_size < V)
	{
		/* get component sizes */
		int32_t* comp_sizes = (int32_t*)malloc_check(sizeof(int32_t) * rV);
		for (int32_t rv = 0; rv < rV; rv++)
		{
			/* saturated components need no processing */
			comp_sizes[rv] = is_saturated[rv] ? 0 : first_vertex[rv + 1] - first_vertex[rv];
		}
		/* get sorting permutation indices */
		int32_t* sort_comp = (int32_t*)malloc_check(sizeof(int32_t) * rV);
		for (int32_t rv = 0; rv < rV; rv++)
		{
			sort_comp[rv] = rv;
		}
		/* sorting can be parallelized as well...
		 * libstdc++ users can simply compile with -D_GLIBCXX_PARALLEL
		 * in which case, omp_set_num_threads() will determine the number of
		 * threads used; scaling linearly with rV seems to work best */
		omp_set_num_threads(compute_num_threads(rV));
		sort(sort_comp, sort_comp + rV, [comp_sizes](int32_t ru, int32_t rv) -> bool
		     { return comp_sizes[ru] > comp_sizes[rv]; }); // decreasing order
		omp_set_num_threads(omp_get_num_procs());
		/* reorder saturation */
		for (int32_t rv = 0; rv < rV; rv++)
		{
			is_saturated[rv] = !comp_sizes[sort_comp[rv]];
		}
		/* reorder components list */
		int32_t* tmp_comp_list    = (int32_t*)malloc_check(sizeof(int32_t) * V);
		int32_t* tmp_first_vertex = comp_sizes; /* reuse storage */
		int32_t  i                = 0;
		for (int32_t rv = 0; rv < rV; rv++)
		{
			int32_t sort_rv       = sort_comp[rv];
			tmp_first_vertex[rv] = i;
			for (int32_t j = first_vertex[sort_rv];
			     j < first_vertex[sort_rv + 1];
			     j++)
			{
				tmp_comp_list[i++] = comp_list[j];
			}
		}
		for (int32_t v = 0; v < V; v++)
		{
			comp_list[v] = tmp_comp_list[v];
		}
		for (int32_t rv = 0; rv < rV; rv++)
		{
			first_vertex[rv] = tmp_first_vertex[rv];
		}
		free(tmp_comp_list);
		free(comp_sizes); /* also storage of tmp_first_vertex */
		/* reorder component values */
		float* tmp_rX = (float*)malloc_check(sizeof(float) * D * rV);
		for (int32_t rv = 0; rv < rV; rv++)
		{
			float* tmp_rXv = tmp_rX + D * rv;
			float* rXv     = rX + D * sort_comp[rv];
			for (size_t d = 0; d < D; d++)
			{
				tmp_rXv[d] = rXv[d];
			}
		}
		free(rX);
		rX = tmp_rX;

		free(sort_comp);
	}

	if (!balance_parallel_split && max_split_size >= first_vertex[1] - first_vertex[0])
	{
		rV_new = 0;
		rV_big = 0;
		return (int32_t)num_thrds < rV ? num_thrds : rV;
	}

	/* maximum component size for parallelism or maxflow performance */
	int32_t max_comp_size = (V - 1) / num_thrds + 1;
	if (max_comp_size > max_split_size)
	{
		max_comp_size = max_split_size;
	}

	/**  get number of components to split  **/
	rV_big = 0; // the number of components to split
	while (rV_big < rV && !is_saturated[rV_big] && first_vertex[rV_big + 1] - first_vertex[rV_big] > max_comp_size)
	{
		rV_big++;
	}

	if (!rV_big)
	{
		rV_new = 0;
		return (int32_t)num_thrds < rV ? num_thrds : rV;
	}

	/**  split big components and create balanced component list  **/
	/* the number of resulting new components */
	int32_t rV_new_par = 0; // auxiliary variable for parallel region

	/* there is need to scan all edges involving a given vertex without
	 * running through all edges of the graph, so we create the list of
	 * 'reverse edges' within each component; to facilitate this, we keep the
	 * index of each vertex within its component */
	index_in_comp = (int32_t*)malloc_check(sizeof(int32_t) * V);

#pragma omp parallel for schedule(dynamic) \
    NUM_THREADS(2 * E * first_vertex[rV_big] / V, rV_big) reduction(+ : rV_new_par)
	for (int32_t rv = 0; rv < rV_big; rv++)
	{
		int32_t comp_size = first_vertex[rv + 1] - first_vertex[rv];

		/* cleanup assigned components */
		for (int32_t i = first_vertex[rv]; i < first_vertex[rv + 1]; i++)
		{
			comp_assign[comp_list[i]] = NOT_ASSIGNED;
		}

		/* get reverse binding edges for breadth-first search */
		int32_t *first_edge_r, *adj_vertices_r;
		get_bind_reverse_edges(rv, first_edge_r, adj_vertices_r);

		/* auxiliary component list for reordering vertices */
		int32_t* tmp_comp_list_rv = (int32_t*)
		    malloc_check(sizeof(int32_t) * comp_size);

		/**  compute the new components  **/
		int32_t residual_comp_size = comp_size;
		int32_t i = 0, j = 0;
		for (int32_t k = first_vertex[rv]; k < first_vertex[rv + 1]; k++)
		{
			int32_t u = comp_list[k];
			if (comp_assign[u] != NOT_ASSIGNED)
			{
				continue;
			}
			/* start a new component with u as root */
			comp_assign[u] = ASSIGNED_ROOT;
			/* adjust maximum component size */
			int32_t n               = (residual_comp_size - 1) / max_comp_size + 1;
			int32_t max_comp_size_u = (residual_comp_size - 1) / n + 1;
			/* put u in the new component list */
			tmp_comp_list_rv[j++] = u;
			int32_t size          = 1;
			while (i < j)
			{ /* breadth-first search up to max component size */
				int32_t v = tmp_comp_list_rv[i++];
				/* add neighbors to the connected component list */
				int32_t        e        = first_edge[v];
				int32_t        l        = index_in_comp[v];
				const int32_t* adj_vert = adj_vertices;
				while (adj_vert == adj_vertices || e < first_edge_r[l + 1])
				{
					if (adj_vert == adj_vertices)
					{
						if (e == first_edge[v + 1])
						{
							e        = first_edge_r[l];
							adj_vert = adj_vertices_r;
							continue;
						}
						else if (!is_bind(e))
						{
							e++;
							continue;
						}
					}
					int32_t w = adj_vert[e];
					if (comp_assign[w] == NOT_ASSIGNED)
					{
						comp_assign[w]        = ASSIGNED;
						tmp_comp_list_rv[j++] = w;
						size++;
						if (size == max_comp_size_u)
						{
							i = j; // swallow the queue
							break;
						}
					}
					e++;
				}
			} /* the current new component is complete */
			residual_comp_size -= size;
			rV_new_par++;
		}

		free(first_edge_r);
		free(adj_vertices_r);

		int32_t* comp_list_rv = comp_list + first_vertex[rv];
		for (int32_t i = 0; i < comp_size; i++)
		{
			comp_list_rv[i] = tmp_comp_list_rv[i];
		}

		free(tmp_comp_list_rv);
	}

	rV_new = rV_new_par;

	free(index_in_comp);
	index_in_comp = nullptr;

	int32_t rV_dif = rV_new - rV_big;

	if ((int32_t)rV + rV_dif > MAX_NUM_COMP)
	{
		cerr << "Cut-pursuit: number of balanced components (" << (int32_t)rV + rV_dif << ") greater "
		     << "than can be represented by int32_t (" << MAX_NUM_COMP << ")"
		     << endl;
		exit(EXIT_FAILURE);
	}

	/**  first vertices of balanced components  **/
	int32_t   rV_bal           = rV + rV_dif;
	int32_t* first_vertex_bal = (int32_t*)
	    malloc_check(sizeof(int32_t) * ADD1(rV_bal));

	/* new components first vertices, and assignments for later */
	int32_t rv_new = (int32_t)-1;
	for (int32_t i = 0; i < first_vertex[rV_big]; i++)
	{
		int32_t v = comp_list[i];
		if (comp_assign[v] == ASSIGNED_ROOT)
		{
			first_vertex_bal[++rv_new] = i;
		}
		comp_assign[v] = rv_new;
	}

	/* add the small components first vertices */
	for (int32_t rv = rV_big; rv < ADD1(rV); rv++)
	{
		first_vertex_bal[rv + rV_dif] = first_vertex[rv];
	}

/**  set separation on edges between new components  **/
#pragma omp parallel for schedule(static) \
    NUM_THREADS(E* first_vertex_bal[rV_new] / V, rV_new)
	for (int32_t rv_new = 0; rv_new < rV_new; rv_new++)
	{
		for (int32_t i = first_vertex_bal[rv_new];
		     i < first_vertex_bal[rv_new + 1];
		     i++)
		{
			int32_t v = comp_list[i];
			for (int32_t e = first_edge[v]; e < first_edge[v + 1]; e++)
			{
				if (is_bind(e) && rv_new != comp_assign[adj_vertices[e]])
				{
					separate(e);
				}
			}
		}
	}

	/**  duplicate component values and saturation accordingly  **/
	rX           = (float*)realloc_check(rX, sizeof(float) * D * rV_bal);
	is_saturated = (bool*)realloc_check(is_saturated, sizeof(bool) * rV_bal);
	/* small components; in-place, start by the end */
	for (int32_t rv = rV - 1; rv >= rV_big; rv--)
	{ // rVbig > 0
		float* rXv     = rX + D * rv;
		float* rXv_bal = rX + D * (rv + rV_dif);
		for (size_t d = 0; d < D; d++)
		{
			rXv_bal[d] = rXv[d];
		}
		is_saturated[rv + rV_dif] = is_saturated[rv];
	}
	/* big components; in-place, slightly more complicated */
	rv_new = rV_new - 1;
	for (int32_t rv = rV_big; rv-- > 0;)
	{ // nice trick for unsigned int32_t
		float* rXv = rX + D * rv;
		while (rv_new != 0 && first_vertex_bal[rv_new] >= first_vertex[rv])
		{
			float* rXv_bal = rX + D * rv_new;
			for (size_t d = 0; d < D; d++)
			{
				rXv_bal[d] = rXv[d];
			}
			is_saturated[rv_new] = is_saturated[rv]; // should be false
			rv_new--;
		}
	}

	/**  replace the component list by the balanced one  **/
	/* store info abount big components */
	first_vertex_big = (int32_t*)realloc_check(first_vertex,
	                                           sizeof(int32_t) * (rV_big + 1));
	first_vertex     = first_vertex_bal;
	rV               = rV_bal;

	return (int32_t)num_thrds < rV ? num_thrds : rV;
}

int32_t CP::remove_balance_separations(int32_t rV_new)
{
	int32_t activation = 0;

/* reconstruct component assignment (only on new components) */
#pragma omp parallel for schedule(static) \
    NUM_THREADS(first_vertex[rV_new], rV_new)
	for (int32_t rv_new = 0; rv_new < rV_new; rv_new++)
	{
		for (int32_t i = first_vertex[rv_new]; i < first_vertex[rv_new + 1];
		     i++)
		{
			comp_assign[comp_list[i]] = rv_new;
		}
	}

/* parallel separation edges are cut if at least one end vertex belongs
 * to a nonsaturated component, to favor cutting */
#pragma omp parallel for schedule(static) reduction(+ : activation) \
    NUM_THREADS(E* first_vertex[rV_new] / V, rV_new)
	for (int32_t rv_new = 0; rv_new < rV_new; rv_new++)
	{
		const bool sat = is_saturated[rv_new];
		for (int32_t i = first_vertex[rv_new]; i < first_vertex[rv_new + 1];
		     i++)
		{
			int32_t v = comp_list[i];
			for (int32_t e = first_edge[v]; e < first_edge[v + 1]; e++)
			{
				if (is_separation(e))
				{
					if (sat && is_saturated[comp_assign[adj_vertices[e]]])
					{
						bind(e);
					}
					else
					{
						cut(e);
						activation++;
					}
				}
			}
		}
	}

	return activation;
}

void CP::revert_balance_split(int32_t rV_big, int32_t rV_new, int32_t* first_vertex_big)
{
	int32_t* first_vertex_bal = first_vertex;    // make clear which one is which
	int32_t   rV_dif           = rV_new - rV_big; // additional components due to balancing
	int32_t   rV_ini           = rV - rV_dif;     // number of components prior to balancing

	/**  remove duplicated component values and aggregate saturation **/
	/* big components */
	int32_t rv_new = 0;
	for (int32_t rv = 0; rv < rV_big; rv++)
	{
		float* rXv     = rX + D * rv;
		float* rXv_bal = rX + D * rv_new;
		for (size_t d = 0; d < D; d++)
		{
			rXv[d] = rXv_bal[d];
		}

		/* each new component which has not been cut has been declared
		 * saturated; an original large component is declared saturated if all
		 * new components within are saturated */
		bool saturation = true;
		while (first_vertex_bal[rv_new] < first_vertex_big[rv + 1])
		{
			saturation = saturation && is_saturated[rv_new];
			rv_new++;
		}
		is_saturated[rv] = saturation;
	}
	/* small components */
	for (int32_t rv = rV_big; rv < rV_ini; rv++)
	{
		float* rXv     = rX + D * rv;
		float* rXv_bal = rX + D * (rv + rV_dif);
		for (size_t d = 0; d < D; d++)
		{
			rXv[d] = rXv_bal[d];
		}
		is_saturated[rv] = is_saturated[rv + rV_dif];
	}
	rX           = (float*)realloc_check(rX, sizeof(float) * D * rV_ini);
	is_saturated = (bool*)realloc_check(is_saturated, sizeof(bool) * rV_ini);

	/**  revert to initial component list  **/
	/* big components */
	for (int32_t rv = 0; rv < rV_big; rv++)
	{
		first_vertex[rv] = first_vertex_big[rv];
	}
	/* small components; in-place */
	for (int32_t rv = rV_big; rv <= rV_ini; rv++)
	{
		first_vertex[rv] = first_vertex[rv + rV_dif];
	}
	first_vertex = (int32_t*)realloc_check(first_vertex,
	                                       sizeof(int32_t) * (rV_ini + 1));
	free(first_vertex_big);
	rV = rV_ini;
}

uintmax_t CP::split_values_complexity() const
{
	uintmax_t complexity = 0;
	/* initialization: k-means++ */
	complexity += D * V * K * (K - 1) / 2;                 // draw initialization
	complexity += D * V * (K + 1) * split_values_iter_num; // k-means
	complexity *= split_values_init_num;                   // repetition
	/* updates */
	complexity += D * (K + V) * (split_iter_num - 1);
	return complexity;
}

uintmax_t CP::split_complexity() const
{
	/* graph cut */
	uintmax_t complexity = D * V;       // account unary split cost and final labeling
	complexity += E;                    // account for binary split cost capacities
	complexity += maxflow_complexity(); // graph cut
	if (K > 2)
	{
		complexity *= K;
	}                             // K alternative labels
	complexity *= split_iter_num; // repeated
	/* all split value computations (init and updates) */
	complexity += split_values_complexity();
	return complexity * (V - saturated_vert) / V; // account saturation linearly
}

void CP::set_split_value(Split_info& split_info, int32_t k,
    int32_t v) const
{
    const float* Yv = Y + D*v;
    float* sXk = split_info.sX + D*k;
    for (size_t d = 0; d < D; d++){ sXk[d] = Yv[d]; }
}

float CP::vert_split_cost(const Split_info& split_info, int32_t v,
    int32_t k) const
{ return fv(v, split_info.sX + D*k); }

float CP::edge_split_cost(const Split_info& split_info, int32_t e,
    int32_t lu, int32_t lv) const
{
    (void)split_info;
    return lu == lv ? 0.0 : EDGE_WEIGHTS_(e);
}

float CP::vert_split_cost(const Split_info& split_info, int32_t v, int32_t k, int32_t l) const
{
	if (k == l)
	{
		return 0.0;
	}
	return vert_split_cost(split_info, v, k)
	       - vert_split_cost(split_info, v, l);
}

void CP::update_split_info(Split_info& split_info) const
{
    int32_t rv = split_info.rv;
    float* sX = split_info.sX;
    float* total_weights = (float*)
        malloc_check(sizeof(float)*split_info.K);
    for (int32_t k = 0; k < split_info.K; k++){
        total_weights[k] = 0.0;
        float* sXk = sX + D*k;
        for (size_t d = 0; d < D; d++){ sXk[d] = 0.0; }
    }
    for (int32_t i = first_vertex[rv]; i < first_vertex[rv + 1]; i++){
        int32_t v = comp_list[i];
        int32_t k = label_assign[v];
        total_weights[k] += VERT_WEIGHTS_(v);
        const float* Yv = Y + D*v;
        float* sXk = sX + D*k;
        for (size_t d = 0; d < D; d++){ sXk[d] += VERT_WEIGHTS_(v)*Yv[d]; }
    }
    int32_t kk = 0; // actual number of alternatives kept
    for (int32_t k = 0; k < split_info.K; k++){
        const float* sXk = sX + D*k;
        float* sXkk = sX + D*kk;
        if (total_weights[k]){
            for (size_t d = 0; d < D; d++){
                sXkk[d] = sXk[d]/total_weights[k];
            }
            kk++;
        } // else no vertex assigned to k, discard this alternative
    }
    split_info.K = kk;
    free(total_weights);
}

CP::Split_info::Split_info(int32_t rv)
    : rv(rv)
    , K(0)
    , first_k(0)
    , sX(nullptr)
{
}

CP::Split_info::~Split_info()
{
	free(sX);
}

CP::Split_info CP::initialize_split_info(int32_t rv)
{
	Split_info split_info(rv);

	split_info.sX = (float*)malloc_check(sizeof(float) * D * K);
	float* sX   = split_info.sX;

	int32_t        comp_size    = first_vertex[rv + 1] - first_vertex[rv];
	const int32_t* comp_list_rv = comp_list + first_vertex[rv];

	/* split cost map and random device for k-means++ */
	float*               near_cost = (float*)malloc_check(sizeof(float) * comp_size);
	default_random_engine rand_gen; // default seed also enough for our purpose

	/* best centroids, assignment and corresponding sum of split costs */
	float   current_sum_cost = real_inf();
	float   best_sum_cost    = real_inf();
	int32_t   best_K           = K;
	int32_t*  best_assign      = split_values_init_num == 1 ? nullptr : (int32_t*)malloc_check(sizeof(int32_t) * comp_size);
	float* best_centroids   = split_values_init_num == 1 ? nullptr : (float*)malloc_check(sizeof(float) * D * K);

	/**  kmeans ++  **/
	for (int init = 0; init < split_values_init_num; init++)
	{
		split_info.K = K;

		/**  initialization  **/
		for (int32_t k = 0; k < split_info.K; k++)
		{
			int32_t rand_i;
			if (k == 0)
			{ /* draw a value uniformly */
				uniform_int_distribution<int32_t> unif_distr(0, comp_size - 1);
				rand_i = unif_distr(rand_gen);
			}
			else
			{ /* draw value with higher probability to vertices not
			   * satisfied with centroids already computed, that is
			   * with higher unary split costs of the centroids */
				for (int32_t i = 0; i < comp_size; i++)
				{
					int32_t v    = comp_list_rv[i];
					near_cost[i] = real_inf();
					for (int32_t l = 0; l < k; l++)
					{
						float c = vert_split_cost(split_info, v, l);
						if (c < near_cost[i])
						{
							near_cost[i] = c;
						}
					}
				}
				/* ensure positivity and deal with infinite costs;
				 * concerning positivity, absolute values of costs are not
				 * meaningful here anyway, only their differences are, so one
				 * can subtract the minimum;
				 * concerning infinite costs, they might concern
				 * non-informative centroids, so we do note encourage them;
				 * nonzero values ensures weights are not all zero, and prevent
				 * from drawing twice the same vertex */
				float min = near_cost[0];
				for (int32_t i = 1; i < comp_size; i++)
				{
					if (near_cost[i] < min)
					{
						min = near_cost[i];
					}
				}
				for (int32_t i = 0; i < comp_size; i++)
				{
					if (near_cost[i] == real_inf())
					{
						near_cost[i] = eps;
					}
					else
					{
						(near_cost[i] -= min) += eps;
					}
				}
				discrete_distribution<int32_t> split_cost_distr(near_cost,
				                                                near_cost + comp_size);
				rand_i = split_cost_distr(rand_gen);
			}
			int32_t rand_v = comp_list_rv[rand_i];
			set_split_value(split_info, k, rand_v);
		} // end for k

		/**  k-means  **/
		for (int iter = 0; iter < split_values_iter_num; iter++)
		{
			/* assign clusters to centroids */
			for (int32_t i = 0; i < comp_size; i++)
			{
				int32_t v        = comp_list_rv[i];
				float  min_cost = real_inf();
				for (int32_t k = 0; k < split_info.K; k++)
				{
					float c = vert_split_cost(split_info, v, k);
					if (c < min_cost)
					{
						min_cost        = c;
						label_assign[v] = k;
					}
				}
			}
			/* update centroids of clusters */
			update_split_info(split_info);
		}

		if (split_values_init_num > 1)
		{ /* keep the best sum of costs */
			float current_sum_cost = 0.0;
			for (int32_t i = 0; i < comp_size; i++)
			{
				int32_t v = comp_list_rv[i];
				int32_t  k = label_assign[v];
				current_sum_cost += vert_split_cost(split_info, v, k);
			}
			if (current_sum_cost < best_sum_cost)
			{
				best_sum_cost = current_sum_cost;
				best_K        = split_info.K;
				for (size_t dk = 0; dk < D * split_info.K; dk++)
				{
					best_centroids[dk] = sX[dk];
				}
				for (int32_t i = 0; i < comp_size; i++)
				{
					int32_t v      = comp_list_rv[i];
					best_assign[i] = label_assign[v];
				}
			}
		}

	} // end for init

	free(near_cost);

	if (current_sum_cost != best_sum_cost)
	{
		/* copy best centroids and assignment */
		split_info.K = best_K;
		for (size_t dk = 0; dk < D * split_info.K; dk++)
		{
			sX[dk] = best_centroids[dk];
		}
		for (int32_t i = 0; i < comp_size; i++)
		{
			int32_t v       = comp_list_rv[i];
			label_assign[v] = best_assign[i];
		}
	}

	free(best_centroids);
	free(best_assign);

	if (split_info.K == 2)
	{
		split_info.first_k = 1;
	}

	return split_info;
}

void CP::split_component(int32_t rv, Maxflow<int32_t, float>* maxflow)
{
	int32_t        comp_size    = first_vertex[rv + 1] - first_vertex[rv];
	const int32_t* comp_list_rv = comp_list + first_vertex[rv];

	Split_info split_info = initialize_split_info(rv);

	float damping = split_damp_ratio;
	for (int split_it = 0; split_it < split_iter_num; split_it++)
	{
		damping += (1.0 - split_damp_ratio) / split_iter_num;

		if (split_it > 0)
		{
			update_split_info(split_info);
		}

		bool no_reassignment = true;

		/**  assign split values with graph cuts;
		 **  for K = 2, one graph cut 0 vs 1 in enough; otherwise iterate
		 **  over K alternative values like alpha-expansion  **/
		for (int32_t k = split_info.first_k; k < split_info.K; k++)
		{

			/* set the source/sink capacities */
			for (int32_t i = 0; i < comp_size; i++)
			{
				int32_t v = comp_list_rv[i];
				int32_t  l = split_info.K == 2 ? 0 : label_assign[v];
				/* unary cost: choosing alternative k against alternative l */
				maxflow->terminal_capacity(i) = vert_split_cost(split_info, v, k, l);
			}

			/* set edge capacities */
			int32_t e_in_comp = 0;
			for (int32_t i = 0; i < comp_size; i++)
			{
				int32_t u  = comp_list_rv[i];
				int32_t  lu = split_info.K == 2 ? 0 : label_assign[u];
				for (int32_t e = first_edge[u]; e < first_edge[u + 1]; e++)
				{
					if (!is_bind(e))
					{
						continue;
					}
					int32_t v  = adj_vertices[e];
					int32_t  lv = split_info.K == 2 ? 0 : label_assign[v];
					if (lu == lv)
					{
						/* special case useful for avoiding additional flow,
						 * and getting meaningful residual flows (e.g. for
						 * directionnaly differentiable problems, where they
						 * might represent subgradients) */
						float cap = damping * edge_split_cost(split_info, e, lu, k);
						maxflow->set_edge_capacities(e_in_comp++, cap, cap);
					}
					else
					{
						/* horizontal and source/sink capacities are modified
						 * according to Kolmogorov & Zabih (2004); in their
						 * notations, functional E(u,v) is decomposed as
						 *
						 * E(0,0) | E(0,1)    A | B
						 * --------------- = -------
						 * E(1,0) | E(1,1)    C | D
						 *
						 *               0 | 0        0 | D-C      0 |B+C-A-D
						 *  =   A   +  ---------  +  --------  +  -----------
						 *             C-A | C-A      0 | D-C      0 |   0
						 *
						 * constant +      unary terms         +  binary term
						 */
						/* A = E(0,0) binary cost of the current assignment */
						float A = damping * edge_split_cost(split_info, e, lu, lv);
						/* B = E(0,1) binary cost of changing lv to k */
						float B = damping * edge_split_cost(split_info, e, lu, k);
						/* C = E(1,0) binary cost of changing lu to k */
						float C = damping * edge_split_cost(split_info, e, k, lv);
						/* D = E(1,1) = 0 binary cost for changing both to k */
						/* set capacities with horizontal orientation u -> v */
						maxflow->terminal_capacity(i) += C - A;
						maxflow->terminal_capacity(index_in_comp[v]) -= C;
						maxflow->set_edge_capacities(e_in_comp++, B + C - A, 0.0);
					}
				} // end for all edges of vertex
			} // end for all vertices

			/* find min cut and set assignment accordingly */
			maxflow->maxflow();

			for (int32_t i = 0; i < comp_size; i++)
			{
				int32_t v = comp_list_rv[i];
				int32_t  l = maxflow->is_sink(i) ? k : split_info.K == 2 ? 0
				                                                        : label_assign[v];
				if (label_assign[v] != l)
				{
					label_assign[v] = l;
					no_reassignment = false;
				}
			}
		} // end for k

		if (no_reassignment)
		{
			break;
		}

	} // end for split_it
}

int32_t CP::split()
{
	int32_t  activation = 0;
	int32_t   rV_new, rV_big;
	int32_t* first_vertex_big;
	int      num_thrds = balance_split(rV_big, rV_new, first_vertex_big);
	(void)num_thrds; /* prevent "unused variable" warning */

	/* components are processed in parallel but graph structure specifies edges
	 * ends with global indexing; the following table enables constant time
	 * conversion to indexing within components */
	index_in_comp = (int32_t*)malloc_check(sizeof(int32_t) * V);

#pragma omp parallel for schedule(dynamic) num_threads(num_thrds) \
    reduction(+ : activation)
	for (int32_t rv = 0; rv < rV; rv++)
	{
		if (is_saturated[rv])
		{
			continue;
		}
		/**  build flow graph structure  **/
		/* set indexing within component and get number of binding edge */
		int32_t        comp_size       = first_vertex[rv + 1] - first_vertex[rv];
		const int32_t* comp_list_rv    = comp_list + first_vertex[rv];
		int32_t        number_of_edges = 0;
		for (int32_t i = 0; i < comp_size; i++)
		{
			int32_t v        = comp_list_rv[i];
			index_in_comp[v] = i;
			for (int32_t e = first_edge[v]; e < first_edge[v + 1]; e++)
			{
				if (is_bind(e))
				{
					number_of_edges++;
				}
			}
		}
		/* build flow graph structure and set edges */
		Maxflow<int32_t, float>* maxflow = new Maxflow<int32_t, float>(comp_size, number_of_edges);
		for (int32_t i = 0; i < comp_size; i++)
		{
			int32_t v = comp_list_rv[i];
			for (int32_t e = first_edge[v]; e < first_edge[v + 1]; e++)
			{
				int32_t j = index_in_comp[adj_vertices[e]];
				if (is_bind(e))
				{
					maxflow->add_edge(i, j);
				}
			}
		}

		/**  set capacities and compute maximum flow  **/
		split_component(rv, maxflow);

		/**  activate edges accordingly  **/
		int32_t rv_activation = 0;
		for (int32_t i = first_vertex[rv]; i < first_vertex[rv + 1]; i++)
		{
			int32_t v = comp_list[i];
			int32_t  l = label_assign[v];
			for (int32_t e = first_edge[v]; e < first_edge[v + 1]; e++)
			{
				if (is_bind(e) && l != label_assign[adj_vertices[e]])
				{
					cut(e);
					rv_activation++;
				}
			}
		}

		is_saturated[rv] = rv_activation == 0;
		activation += rv_activation;

		delete maxflow;
	}

	free(index_in_comp);
	index_in_comp = nullptr;

	if (rV_new != rV_big)
	{
		activation += remove_balance_separations(rV_new);
		revert_balance_split(rV_big, rV_new, first_vertex_big);
	}

/* reconstruct components assignment */
#pragma omp parallel for schedule(static) NUM_THREADS(V, rV)
	for (int32_t rv = 0; rv < rV; rv++)
	{
		for (int32_t i = first_vertex[rv]; i < first_vertex[rv + 1]; i++)
		{
			comp_assign[comp_list[i]] = rv;
		}
	}

	return activation;
}

int32_t CP::get_merge_chain_root(int32_t rv) const
{
	while (merge_chains_root[rv] != CHAIN_END)
	{
		rv = merge_chains_root[rv];
	}
	return rv;
}

void CP::compute_merge_candidate(int32_t re)
{
    int32_t ru = reduced_edges_u(re);
    int32_t rv = reduced_edges_v(re);
    float edge_weight = reduced_edge_weights[re];

    float* rXu = rX + D*ru;
    float* rXv = rX + D*rv;
    float wru = comp_weights[ru]/(comp_weights[ru] + comp_weights[rv]);
    float wrv = comp_weights[rv]/(comp_weights[ru] + comp_weights[rv]);

    float gain = edge_weight;
    size_t Q = loss; // number of coordinates for quadratic part

    if (Q != 0){
        /* quadratic gain */
        float gainQ = 0.0;
        for (size_t d = 0; d < Q; d++){
            gainQ -= COOR_WEIGHTS_(d)*(rXu[d] - rXv[d])*(rXu[d] - rXv[d]);
        }
        gain += comp_weights[ru]*wrv*gainQ;
    }

    if (gain > 0.0 || comp_weights[ru] < min_comp_weight
                    || comp_weights[rv] < min_comp_weight){
        if (!merge_values[re]){
            merge_values[re] = (float*) malloc_check(sizeof(float)*D);
        }
        float* value = merge_values[re];
        for (size_t d = 0; d < D; d++){ value[d] = wru*rXu[d] + wrv*rXv[d]; }

        if (Q != D){
            /* smoothed Kullback-Leibler gain */
            float gainKLu = 0.0, gainKLv = 0.0;
            const float s = loss < 1.0 ? loss : eps;
            const float c = 1.0 - s;
            const float u = s/(D - Q);
            for (size_t d = Q; d < D; d++){
                float u_value_d = u + c*value[d];
                float u_rXu_d = u + c*rXu[d];
                float u_rXv_d = u + c*rXv[d];
                gainKLu -= (u_rXu_d)*log(u_rXu_d/u_value_d);
                gainKLv -= (u_rXv_d)*log(u_rXv_d/u_value_d);
            }
            gain += COOR_WEIGHTS_(Q)*
                (comp_weights[ru]*gainKLu + comp_weights[rv]*gainKLv);
        }
    }

    merge_gains[re] = gain;
    if (gain <= 0.0 && comp_weights[ru] >= min_comp_weight
                     && comp_weights[rv] >= min_comp_weight){
        delete_merge_candidate(re);
    }
}

size_t CP::merge_info_complexity() const
{ return 2*D; }

void CP::delete_merge_candidate(int32_t re)
{ free(merge_values[re]); merge_values[re] = nullptr; }

int32_t CP::accept_merge_candidate(int32_t re)
{
    int32_t ru = reduced_edges_u(re);
    int32_t rv = reduced_edges_v(re);
    int32_t ro = merge_components(ru, rv); // ro is the root of the merge chain
    float* rXo = rX + D*ro;
    for (size_t d = 0; d < D; d++){ rXo[d] = merge_values[re][d]; }
    delete_merge_candidate(re);
    if (ro != ru){ rv = ru; } // rv now designates the non-root component
    comp_weights[ro] += comp_weights[rv];
    return ro;
}

float CP::compute_evolution() const
{
    float dif = 0.0;
    for (int32_t rv = 0; rv < rV; rv++){
        if (is_saturated[rv]){ continue; }
        const float* rXv = rX + D*rv;
        float distXX = 0.0;
        if (loss != quadratic_loss()){
            const size_t Q = loss; // number of coordinates for quadratic part
            const float s = loss < 1.0 ? loss : eps;
            const float c = 1.0 - s;
            const float u = s/(D - Q);
            for (size_t d = Q; d < D; d++){
                distXX -= (u + c*rXv[d])*log(u + c*rXv[d]);
            }
            distXX *= COOR_WEIGHTS_(Q);
        }
        for (int32_t i = first_vertex[rv]; i < first_vertex[rv + 1]; i++){
            int32_t v = comp_list[i];
            const float* lrXv = last_rX + D*last_comp_assign[v];
            dif += VERT_WEIGHTS_(v)*(distance(rXv, lrXv) - distXX);
        }
    }
    float amp = compute_f();
    return amp > eps ? dif/amp : dif/eps;
}

int32_t CP::compute_merge_chains()
{
    int32_t merge_count = 0;

    /* compute merge candidates in parallel */
    merge_gains = (float*) malloc_check(sizeof(float)*rE);
    merge_values = (float**) malloc_check(sizeof(float*)*rE);
    for (int32_t re = 0; re < rE; re++){ merge_values[re] = nullptr; }
    int32_t num_pos_candidates = 0, num_neg_candidates = 0;
    #pragma omp parallel for NUM_THREADS(merge_info_complexity()*rE, rE) \
        schedule(static) reduction(+:num_pos_candidates, num_neg_candidates)
    for (int32_t re = 0; re < rE; re++){
        int32_t ru = reduced_edges_u(re);
        int32_t rv = reduced_edges_v(re);
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
    const float* _merge_gains = merge_gains;

    if (num_pos_candidates){
    /**  merge candidates with positive gains;
     **  these are important enough to be merged in decreasing gain order, and
     **  to update surrounding merge candidates after each merge:
     **  1) maintain candidates in a priority order on the gain
     **  2) maintain access to all reduced edges involving a given vertex, and
     **  to their potential corresponding candidate in the priority order;
     **  because of 2), the best choice for 1) is a binary search tree **/

    /* 1) binary search tree on the gain */
    auto compare_candidates = [_merge_gains] (int32_t mc1, int32_t mc2) -> bool
        { return _merge_gains[mc1] > _merge_gains[mc2] ||
            /* ensure unique identification of merge candidates */
            (_merge_gains[mc1] == _merge_gains[mc2] && mc1 < mc2); };
    set<int32_t, decltype(compare_candidates)>
        candidates_queue(compare_candidates);
    for (int32_t re = 0; re < rE; re++){
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
        malloc_check(sizeof(Cell_id)*rV);
    Cell_id* next_candidate_cell = (Cell_id*)
        malloc_check(sizeof(Cell_id)*2*rE);
    for (int32_t rv = 0; rv < rV; rv++){
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
        malloc_check(sizeof(Cell_id)*rV);
    for (int32_t rv = 0; rv < rV; rv++){ last_candidate_cell[rv] = EMPTY_CELL; }
    for (int32_t re = 0; re < rE; re++){
        int32_t ru = reduced_edges_u(re);
        int32_t rv = reduced_edges_v(re);
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
        typename set<int32_t>::iterator candidate = candidates_queue.begin();
        int32_t re = *candidate;
        int32_t ru = reduced_edges_u(re);
        int32_t rv = reduced_edges_v(re);

        /**  accept the merge and remove from the queue  **/
        int32_t ro = accept_merge_candidate(re); // merge ru and rv
        if (ro != ru){ rv = ru; ru = ro; } // makes sure ru is the root
        candidates_queue.erase(candidate);
        merge_count++;

        /**  update reduced graph structure and adjacent merge candidates  **/
        Cell_id *mcc_ru, *mcc_rv;

        /* first pass on the list of rv: cleanup deleted candidates, remove
         * current merging candidate, update vertices by replacing rv by ru */
        FIRST_CELL(mcc_rv, rv);
        while (!IS_EMPTY(mcc_rv)){
            int32_t re_rv = (int32_t) GET_REDUCED_EDGE(mcc_rv);
            if (!reduced_edge_weights[re_rv]){ DELETE_CELL(mcc_rv); continue; }
            int32_t end_re_rv;
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
            int32_t re_ru = (int32_t) GET_REDUCED_EDGE(mcc_ru);
            if (!reduced_edge_weights[re_ru]){ DELETE_CELL(mcc_ru); continue; }
            int32_t end_re_ru = reduced_edges_u(re_ru) == ru ?
                reduced_edges_v(re_ru) : reduced_edges_u(re_ru);
            if (end_re_ru == ru){ DELETE_CELL(mcc_ru); continue; }
            for (FIRST_CELL(mcc_rv, rv); !IS_EMPTY(mcc_rv); NEXT_CELL(mcc_rv)){
                int32_t re_rv = (int32_t) GET_REDUCED_EDGE(mcc_rv);
                int32_t end_re_rv = reduced_edges_u(re_rv) == ru ?
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
            int32_t re = (int32_t) GET_REDUCED_EDGE(mcc_ru);
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
    int32_t bufsize = num_neg_candidates;
    int32_t* neg_candidates = (int32_t*) malloc_check(sizeof(int32_t)*bufsize);
    num_neg_candidates = 0; // recounting
    for (int32_t re = 0; re < rE; re++){
        if (merge_values[re]){
            if (num_neg_candidates == bufsize){
                bufsize += bufsize/2 + 1;
                neg_candidates = (int32_t*) realloc_check(neg_candidates,
                    sizeof(int32_t)*bufsize);
            }
            neg_candidates[num_neg_candidates++] = re;
        }
    }
    sort(neg_candidates, neg_candidates + num_neg_candidates,
        [_merge_gains] (int32_t re1, int32_t re2) -> bool
        { return _merge_gains[re1] > _merge_gains[re2]; });
    for (int32_t mc = 0; mc < num_neg_candidates; mc++){
        int32_t re = neg_candidates[mc];
        /* ensure candidate info is up-to-date */
        int32_t ru = get_merge_chain_root(reduced_edges_u(re));
        int32_t rv = get_merge_chain_root(reduced_edges_v(re));
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

int32_t CP::merge_components(int32_t ru, int32_t rv)
{
	/* ensure the component with smallest identifier will be the root of the
	 * merge chain */
	if (ru > rv)
	{
		int32_t tmp = ru;
		ru         = rv;
		rv         = tmp;
	}
	/* link both chains; update leaf of the merge chain; update root info */
	merge_chains_next[merge_chains_leaf[ru]] = rv;
	merge_chains_leaf[ru]                    = merge_chains_leaf[rv];
	merge_chains_root[rv] = merge_chains_root[merge_chains_leaf[rv]] = ru;
	/* saturation considerations are taken care of in merge method */
	return ru; // root of the resulting merge chain
}

int32_t CP::merge()
{
	/**  create the chains representing the merged components  **/
	merge_chains_root = (int32_t*)malloc_check(sizeof(int32_t) * rV);
	merge_chains_next = (int32_t*)malloc_check(sizeof(int32_t) * rV);
	merge_chains_leaf = (int32_t*)malloc_check(sizeof(int32_t) * rV);
	for (int32_t rv = 0; rv < rV; rv++)
	{
		merge_chains_root[rv] = CHAIN_END;
		merge_chains_next[rv] = CHAIN_END;
		merge_chains_leaf[rv] = rv;
	}
	int32_t merge_count = compute_merge_chains();

	/**  at this point, three different component assignments exists:
	 **  the one from previous iteration (in last_comp_assign),
	 **  the current one after the split (in comp_assign), and
	 **  the final one after the merge (to be computed now)  **/

	/**  recompute saturation: compare previous iterate and final assignment,
	 **  and flag nonevolving components as saturated  **/
	if (!last_rV)
	{ /* first iteration, no previous assignment available */
		for (int32_t rv = 0; rv < rV; rv++)
		{
			is_saturated[rv] = false;
		}
	}
	else
	{
		/* a previous component is flagged nonevolving if it can be assigned a
		 * unique final component */
		/* we can reuse storage since for now last_rV <= rV */
		int32_t* saturation_flag = merge_chains_leaf;
		for (int32_t last_rv = 0; last_rv < last_rV; last_rv++)
		{
			saturation_flag[last_rv] = NOT_ASSIGNED;
		}
		/* run along each final component, from their root */
		for (int32_t ru = 0; ru < rV; ru++)
		{
			if (merge_chains_root[ru] != CHAIN_END)
			{
				continue;
			}
			int32_t last_ru = last_comp_assign[comp_list[first_vertex[ru]]];
			if (saturation_flag[last_ru] == NOT_ASSIGNED)
			{
				saturation_flag[last_ru] = ASSIGNED;
			}
			else
			{ /* was already assigned another final component */
				saturation_flag[last_ru] = NOT_SATURATED;
			}
			/* run along the merge chain */
			int32_t rv = ru;
			while (rv != CHAIN_END)
			{
				int32_t last_rv = last_comp_assign[comp_list[first_vertex[rv]]];
				if (last_ru != last_rv)
				{ /* previous components do not agree */
					saturation_flag[last_ru] = saturation_flag[last_rv] =
					    NOT_SATURATED;
				}
				rv = merge_chains_next[rv];
			}
		}
		/* resulting saturation for each final component */
		for (int32_t rv = 0; rv < rV; rv++)
		{
			if (merge_chains_root[rv] != CHAIN_END)
			{
				continue;
			}
			int32_t last_rv   = last_comp_assign[comp_list[first_vertex[rv]]];
			is_saturated[rv] = saturation_flag[last_rv] != NOT_SATURATED;
		}
	}
	free(merge_chains_leaf); // also storage of saturation_flag

	/**  if no merge take place, no update needed  **/
	if (!merge_count)
	{
		free(merge_chains_root);
		free(merge_chains_next);
		return 0;
	}

	/**  construct the final component lists in temporary storage, and update
	 **  components saturation, values and first vertex indices in-place  **/
	saturated_comp = 0;
	saturated_vert = 0;

	/* auxiliary components lists */
	int32_t* tmp_comp_list = (int32_t*)malloc_check(sizeof(int32_t) * V);

	int32_t  rn = 0; // component number
	int32_t i  = 0; // index in the final comp_list
	/* each current component is assigned its final component;
	 * this can use the same storage as merge chains root, because the only
	 * required information is to flag roots (no need to get back to roots),
	 * and roots are processed before getting assigned a final component */
	int32_t* final_comp = merge_chains_root;
	for (int32_t ru = 0; ru < rV; ru++)
	{
		if (merge_chains_root[ru] != CHAIN_END)
		{
			continue;
		}
		/**  ru is a root, create the corresponding final component  **/
		/* copy component value and saturation;
		 * can be done in-place because rn <= ru guaranteed */
		const float* rXu = rX + D * ru;
		float*       rXn = rX + D * rn;
		for (size_t d = 0; d < D; d++)
		{
			rXn[d] = rXu[d];
		}
		if ((is_saturated[rn] = is_saturated[ru]))
		{
			saturated_comp++;
		}
		/* run along the merge chain */
		int32_t first = i; // holds index of first vertex of the component
		int32_t  rv    = ru;
		while (rv != CHAIN_END)
		{
			final_comp[rv] = rn;
			/* assign all vertices to final component */
			for (int32_t j = first_vertex[rv]; j < first_vertex[rv + 1]; j++)
			{
				tmp_comp_list[i++] = comp_list[j];
			}
			if (is_saturated[rn])
			{
				saturated_vert += first_vertex[rv + 1] - first_vertex[rv];
			}
			rv = merge_chains_next[rv];
		}
		/* the root of each chain is the component with smallest id in the
		 * chain, and the current components are visited in increasing order,
		 * so now that 'rn' final components have been constructed, at least
		 * the first 'rn' current components have been copied, hence
		 * 'first_vertex' will not be accessed before position 'rn' anymore;
		 * can thus modify in-place */
		first_vertex[rn++] = first;
	}

	/* finalize and shrink arrays to fit the reduced number of components */
	first_vertex[rV = rn] = V;
	first_vertex          = (int32_t*)realloc_check(first_vertex,
	                                                sizeof(int32_t) * (rV + 1));
	rX                    = (float*)realloc_check(rX, sizeof(float) * D * rV);
	is_saturated          = (bool*)realloc_check(is_saturated, sizeof(bool) * rV);

	/* update components assignments */
	for (int32_t v = 0; v < V; v++)
	{
		comp_list[v]   = tmp_comp_list[v];
		comp_assign[v] = final_comp[comp_assign[v]];
	}
	free(tmp_comp_list);

	/* deactivate edges between merged components */
	int32_t deactivation = 0;
#pragma omp parallel for schedule(static) NUM_THREADS(E, rV) \
    reduction(+ : deactivation)
	for (int32_t rv = 0; rv < rV; rv++)
	{
		for (int32_t i = first_vertex[rv]; i < first_vertex[rv + 1]; i++)
		{
			int32_t v = comp_list[i];
			for (int32_t e = first_edge[v]; e < first_edge[v + 1]; e++)
			{
				if (is_bind(e))
				{
					continue;
				}
				if (!is_bind(e) && rv == comp_assign[adj_vertices[e]])
				{
					bind(e);
					deactivation++;
				}
			}
		}
	}

	/**  update reduced edges  **/

	/* update current reduced edges ends with final components */
	int32_t* is_isolated = merge_chains_next; // reuse storage
	for (int32_t rv = 0; rv < rV; rv++)
	{
		is_isolated[rv] = ((int32_t) true);
	}

	for (int32_t re = 0; re < rE; re++)
	{
		int32_t ru = final_comp[reduced_edges_u(re)];
		int32_t rv = final_comp[reduced_edges_v(re)];
		if (ru > rv)
		{
			int32_t tmp = ru;
			ru         = rv;
			rv         = tmp;
		}
		reduced_edges_u(re) = ru;
		reduced_edges_v(re) = rv;
		if (ru != rv && reduced_edge_weights[ru] > 0.0)
		{
			is_isolated[ru] = is_isolated[rv] = ((int32_t) false);
		}
	}

	free(merge_chains_root); // also storage of final_comp

	/* reorder by increasing lexicographic order on the components */
	int32_t* permutation = (int32_t*)malloc_check(sizeof(int32_t) * rE);
	for (int32_t re = 0; re < rE; re++)
	{
		permutation[re] = re;
	}
	sort(permutation, permutation + rE, [this](int32_t re1, int32_t re2) -> bool
	     { return reduced_edges_u(re1) < reduced_edges_u(re2) || (reduced_edges_u(re1) == reduced_edges_u(re2) && reduced_edges_v(re1) < reduced_edges_v(re2)); });

	/* remove duplicates and accumulate edge weights */
	int32_t* new_red_edg       = (int32_t*)malloc_check(sizeof(int32_t) * 2 * rE);
	float* new_red_edg_wghts = (float*)malloc_check(sizeof(float) * rE);
	int32_t re                = 0;
	int32_t final_re          = 0;
	while (re < rE)
	{
		/* draw next edge */
		int32_t ru = reduced_edges_u(permutation[re]);
		int32_t rv = reduced_edges_v(permutation[re]);
		/* put it in the list if regular or isolated */
		if (ru != rv || is_isolated[ru])
		{
			new_red_edg[((size_t)2) * final_re]     = ru;
			new_red_edg[((size_t)2) * final_re + 1] = rv;
			/* compute edge weight */
			if (is_isolated[ru])
			{
				new_red_edg_wghts[final_re] = eps;
				do
				{
					re++;
				} while (re < rE && ru == reduced_edges_u(permutation[re]));
			}
			else
			{
				float new_red_wght = 0.0;
				do
				{
					new_red_wght += reduced_edge_weights[permutation[re]];
					re++;
				} while (re < rE && ru == reduced_edges_u(permutation[re])
				         && rv == reduced_edges_v(permutation[re]));
				new_red_edg_wghts[final_re] = new_red_wght;
			}
			final_re++;
		}
		else
		{
			re++;
		}
	}

	free(permutation);
	free(reduced_edges);
	free(reduced_edge_weights);
	free(merge_chains_next); // also storage of is_isolated

	rE                   = final_re;
	reduced_edges        = (int32_t*)realloc_check(new_red_edg, sizeof(int32_t) * 2 * rE);
	reduced_edge_weights = (float*)realloc_check(new_red_edg_wghts,
	                                              sizeof(float) * rE);

	return deactivation;
}