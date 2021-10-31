// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//Defines the following macros (depending on the compilation platform/settings)
//	- CC_WINDOWS / CC_MAC_OS / CC_LINUX
//	- CC_ENV32 / CC_ENV64
#if defined(_WIN32) || defined(_WIN64) || defined(WIN32)
	#define CC_WINDOWS
#if defined(_WIN64)
	#define CC_ENV_64
#else
	#define CC_ENV_32
#endif
#else
#if defined(__APPLE__)
	#define CC_MAC_OS
#else
	#define CC_LINUX
#endif
#if defined(__x86_64__) || defined(__ppc64__) || defined(__arm64__)
	#define CC_ENV_64
#else
	#define CC_ENV_32
#endif
#endif
