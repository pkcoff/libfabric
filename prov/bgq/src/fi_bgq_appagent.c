
#include <stdio.h>


void * fi_bgq_agent_start (void * arg);

int main (int argc, char ** argv) {

	setbuf(stdout, NULL);
	setbuf(stderr, NULL);

	fi_bgq_agent_start(NULL);

	return 0;
}
