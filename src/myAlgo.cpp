#include "myAlgo.h"

howde::MyAlgo::MyAlgo(int param)
{
	someVec.push_back(param);
}

int howde::MyAlgo::run(int x, int y)
{
	return x + MyAlgo::someFunction(y) + someVec[0];
}

int howde::MyAlgo::someFunction(int z)
{
	return z / 2;
}