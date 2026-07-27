/*==========================  omp_num_threads.hpp  ============================
 * include openmp and provide a function to compute a smart number of threads
 *
 * Hugo Raguet 2018
 *===========================================================================*/
#pragma once

#ifdef _OPENMP

    #include <omp.h>
    #include <cstdint>  // requires C++11, needed for uintmax_t

    /* rough minimum number of operations per thread */
    #ifndef MIN_OPS_PER_THREAD
        #define MIN_OPS_PER_THREAD 10000
    #endif

    /* num_ops is a rough estimation of the total number of operations 
     * max_threads is the maximum number of jobs performed in parallel */
    static inline int compute_num_threads(uintmax_t num_ops,
        uintmax_t max_threads)
    {
        uintmax_t num_threads = num_ops/MIN_OPS_PER_THREAD;
        if (num_threads > (unsigned) omp_get_max_threads()){
            num_threads = omp_get_max_threads();
        }
        if (num_threads > (unsigned) omp_get_num_procs()){
            num_threads = omp_get_num_procs();
        }
        if (num_threads > max_threads){ num_threads = max_threads; }
        return num_threads > 1 ? num_threads : 1;
    }

    /* overload for max_threads defaulting to num_ops */
    static inline int compute_num_threads(uintmax_t num_ops)
    { return compute_num_threads(num_ops, num_ops); }

    #define NUM_THREADS(...) \
        num_threads(compute_num_threads((uintmax_t) __VA_ARGS__))

#else /* provide default definitions for basic openmp queries */

    static inline int omp_get_num_procs(){ return 1; }
    static inline int omp_get_thread_num(){ return 0; }
    static inline int omp_get_max_threads(){ return 1; }
    static inline void omp_set_num_threads(int){ /* do nothing */ }
    static inline int compute_num_threads(int num_ops, int max_threads = 1)
        { return 1; }

#endif
