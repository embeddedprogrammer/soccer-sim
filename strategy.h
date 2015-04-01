#include "utility.h"

#ifndef STRATEGY_H_
#define STRATEGY_H_

enum strategy_states
	{strategy_none, testGoToPoint, testPlayRushGoal, testPlayDefense, testStrategySwitch} strategy_state;

void enterStrategyState(strategy_states newState);

void receiveCoords(coord3 pRobot1, coord3 pRobot2, coord2 pBall, double t);

void strategy_tick();

void pressKey(int key);

#endif /* STRATEGY_H_ */
