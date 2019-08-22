/////////////////////
///// shuffle.h /////
/////////////////////

#ifndef __SHUFFLE_HH
#define __SHUFFLE_HH


//=======================//
struct RandomGenerator
//=======================//
{
	RandomGenerator()
	{
		std::srand(static_cast<unsigned>(std::time(NULL)));
	}

	int operator()(int n) const
	{
		const int i = static_cast<int>(static_cast<double>(std::rand()) * n / RAND_MAX);

		return std::max(0, std::min(i, n - 1));
	}

};

#endif

/******** USAGE ***************************************************
#include <deque>
#include "shuffle.h"
...
std::deque<TYPE> point_list ;
...
RandomGenerator rnd;
std::random_shuffle(point_list.begin(), point_list.end(), rnd);
******************************************************************/


// end of shuffle.h

