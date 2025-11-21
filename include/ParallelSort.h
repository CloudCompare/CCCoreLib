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

	#ifndef Q_MOC_RUN
	#if defined(emit)
		#undef emit
		#include <oneapi/tbb/parallel_sort.h>
		#define emit // restore the macro definition of "emit", as it was defined in gtmetamacros.h
	#else
	#include <oneapi/tbb/parallel_sort.h>
	#endif // defined(emit)
	#endif // Q_MOC_RUN

	#define ParallelSort oneapi::tbb::parallel_sort

#else

	#include <algorithm>
	#define ParallelSort std::sort

#endif
