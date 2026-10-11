/*
 * Mercury: A configurable open-source software-defined modem.
 * Copyright (C) 2022-2024 Fadi Jerji
 * Author: Fadi Jerji
 * Email: fadi.jerji@  <gmail.com, caisresearch.com, ieee.org>
 * ORCID: 0000-0002-2076-5831
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, version 3 of the
 * License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "physical_layer/awgn.h"

cl_awgn::cl_awgn()
{
}

cl_awgn::~cl_awgn()
{
}

void cl_awgn::set_seed(long seed)
{
	if(seed>0)
	{
		srand(seed);
	}
}
double  cl_awgn::awgn_value_generator()
{
	double x1;
	double x2;
	double result;

	// Avoid x1=0 which causes log(0)=-inf → sqrt(inf)=inf → NaN in FFT
	// On Windows RAND_MAX=32767, so rand()=0 occurs ~1/32768 calls
	x1=(rand() + 1.0) / ((double)RAND_MAX + 1.0);
	x2=rand() / ((double)RAND_MAX);

	result=sqrt(- log(x1)) * (sqrt(2) * cos(2.0 * M_PI * x2));

	return result;
	// based on  Box-Muller method
	//https://ieeexplore.ieee.org/abstract/document/953647
}






