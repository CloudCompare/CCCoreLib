// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © CloudCompare Project

#pragma once

#ifdef ParallelSort
#undef ParallelSort
#pragma message "Replacing preprocessor symbol 'ParallelSort' with the one defined in Parallel.h"
#endif

#if defined(_MSC_VER) && (_MSC_VER >= 1800)

   //Parallel Patterns Library (for parallel sort)
   #include <ppl.h>

   #define ParallelSort Concurrency::parallel_sort

#elif CC_CORE_LIB_USES_TBB

   #include <tbb/parallel_sort.h>

   #define ParallelSort tbb::parallel_sort

#else

   #include <algorithm>

   #define ParallelSort std::sort

#endif
