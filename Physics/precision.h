#ifndef __PRECISION_H_INCLUDED__
#define __PRECISION_H_INCLUDED__


#include <math.h>

#if 0
	typedef float real;
	const real PI = (real)3.14159265358;
	#define REAL_MAX FLT_MAX
	#define real_sqrt sqrtf
	#define real_pow powf
	#define real_abs fabsf
	#define real_sin sinf
	#define real_cos cosf
	#define real_exp expf
	#define real_fmax fmaxf
	#define real_fmin fminf
#else
	typedef double real;
	const real PI = (real)3.14159265358;
	#define REAL_MAX DBL_MAX
	#define real_sqrt sqrt
	#define real_pow pow
	#define real_abs fabs
	#define real_sin sin
	#define real_cos cos
	#define real_exp exp
	#define real_fmax fmax
	#define real_fmin fmin
#endif


#endif // __PRECISION_H_INCLUDED__