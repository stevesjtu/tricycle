
// Should be included by Opcode.h if needed

	#define ICE_DONT_CHECK_COMPILER_OPTIONS

	// From Windows...
	typedef int                 BOOL;
	#ifndef FALSE
	#define FALSE               0
	#endif

	#ifndef TRUE
	#define TRUE                1
	#endif

	#include <stdio.h>
	#include <stdlib.h>
	#include <assert.h>
	#include <string.h>
	#include <float.h>
	#include <math.h>

	#ifndef ASSERT
		#define	ASSERT(exp)	{}
	#endif
	#define ICE_COMPILE_TIME_ASSERT(exp)	extern char ICE_Dummy[ (exp) ? 1 : -1 ]

	#define	Log				{}
	#define	SetIceError(a,b)	false
	#define	EC_OUTOFMEMORY	"Out of memory"

	#include "Ice/IcePreprocessor.h"

	#undef ICECORE_API
	#define ICECORE_API	OPCODE_API

	#include "opcode/Ice/IceTypes.h"
	#include "opcode/Ice/IceFPU.h"
	#include "opcode/Ice/IceMemoryMacros.h"

	namespace IceCore
	{
		#include "opcode/Ice/IceUtils.h"
		#include "opcode/Ice/IceContainer.h"
		#include "opcode/Ice/IcePairs.h"
		//#include "Ice/IceRevisitedRadix.h"
		#include "opcode/Ice/IceRandom.h"
	}
	using namespace IceCore;

	#define ICEMATHS_API	OPCODE_API
	namespace IceMaths
	{
		#include "opcode/Ice/IceAxes.h"
		#include "opcode/Ice/IcePoint.h"
		#include "opcode/Ice/IceHPoint.h"
		#include "opcode/Ice/IceMatrix3x3.h"
		#include "opcode/Ice/IceMatrix4x4.h"
		#include "opcode/Ice/IcePlane.h"
		//#include "Ice/IceRay.h"
		#include "opcode/Ice/IceIndexedTriangle.h"
		#include "opcode/Ice/IceTriangle.h"
		//#include "Ice/IceTriList.h"
		#include "opcode/Ice/IceAABB.h"
		#include "opcode/Ice/IceOBB.h"
		#include "opcode/Ice/IceBoundingSphere.h"
		#include "opcode/Ice/IceSegment.h"
		#include "opcode/Ice/IceLSS.h"
	}
	using namespace IceMaths;
