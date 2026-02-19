#include <stdlib.h>
	
int
main(int argc, char **argv)
{
	unsigned long long arr_size = 64ULL << 10;
	int arr[arr_size];

	for (long i = 0; i < 1ULL << 15; i++) {
		arr[rand() % arr_size]++;
	}

	return 0;
}
