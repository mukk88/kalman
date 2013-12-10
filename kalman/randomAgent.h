#ifndef RANDOM_H
#define RANDOM_H
#include "command.h"
#include <stdlib.h> 
#include <time.h>

using namespace std;

class RandomAgent {

public:

	RandomAgent(int i, BZRC* bzrc, int cycles) : index(i), commandCenter(bzrc), ticker(0), changeCycles(cycles) {
		srand(time(NULL));
	}

	void move(){
		if (ticker++ > changeCycles){
			commandCenter->speed(index, randomNumber());
			commandCenter->angvel(index, randomNumber());
			ticker = 0;
		}
	}

private:
	int index;
	BZRC* commandCenter;
	int ticker;
	const int changeCycles;

	double randomNumber(){
		return (double (rand() % 1000)) / (1000.0);
	}
};

#endif