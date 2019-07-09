/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_auxiliary_sim_functions.h"

/** Print ACADO code generation notice. */
void acado_printHeader( )
{
	printf(
		"\nACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.\n"
		"Copyright (C) 2008-2015 by Boris Houska, Hans Joachim Ferreau,\n" 
		"Milan Vukov and Rien Quirynen, KU Leuven.\n"
	);
	
	printf(
		"Developed within the Optimization in Engineering Center (OPTEC) under\n"
		"supervision of Moritz Diehl. All rights reserved.\n\n"
		"ACADO Toolkit is distributed under the terms of the GNU Lesser\n"
		"General Public License 3 in the hope that it will be useful,\n"
		"but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
		"MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the\n"
		"GNU Lesser General Public License for more details.\n\n"
	);
}

#if (defined _WIN32 || _WIN64)

void acado_tic( acado_timer* t )
{
	QueryPerformanceFrequency(&t->freq);
	QueryPerformanceCounter(&t->tic);
}

real_t acado_toc( acado_timer* t )
{
	QueryPerformanceCounter(&t->toc);
	return ((t->toc.QuadPart - t->tic.QuadPart) / (real_t)t->freq.QuadPart);
}


#elif (defined __APPLE__)

void acado_tic( acado_timer* t )
{
    /* read current clock cycles */
    t->tic = mach_absolute_time();
}

real_t acado_toc( acado_timer* t )
{

    uint64_t duration; /* elapsed time in clock cycles*/
    
    t->toc = mach_absolute_time();
    duration = t->toc - t->tic;
    
    /*conversion from clock cycles to nanoseconds*/
    mach_timebase_info(&(t->tinfo));
    duration *= t->tinfo.numer;
    duration /= t->tinfo.denom;

    return (real_t)duration / 1e9;
}

#else

/* read current time */
void acado_tic( acado_timer* t )
{
	gettimeofday(&t->tic, 0);
}

/* return time passed since last call to tic on this timer */
real_t acado_toc( acado_timer* t )
{
	struct timeval temp;
	
	gettimeofday(&t->toc, 0);
    
	if ((t->toc.tv_usec - t->tic.tv_usec) < 0)
	{
		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec - 1;
		temp.tv_usec = 1000000 + t->toc.tv_usec - t->tic.tv_usec;
	}
	else
	{
		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec;
		temp.tv_usec = t->toc.tv_usec - t->tic.tv_usec;
	}
	
	return (real_t)temp.tv_sec + (real_t)temp.tv_usec / 1e6;
}

#endif /* (defined _WIN32 || _WIN64) */
