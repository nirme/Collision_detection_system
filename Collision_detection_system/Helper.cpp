#include "Helper.h"


int itoas(int num, char* str, unsigned int len)
{
	int i = num;
	unsigned int n = i < 0 ? 2 : 1;
	while (i /= 10) ++n;
	assert(n <= len && "number too big");

	i = n;
	if (num < 0)
		str[0] = '-';

	while (num)
	{
		str[--i] = '0' + num % 10;
		num /= 10;
	}

	return n;
};

