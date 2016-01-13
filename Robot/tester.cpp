#include "strategy.h"

int main()
{
	motorControl_init();


	receiveCoords((coord3){1, 2, 3}, (coord2){4, 5});

	pressKey(4);

}
