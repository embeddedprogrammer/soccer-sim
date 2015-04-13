#include "calibrate.h"
#include "stdio.h"
#include "time.h"
#include "strategy.h"
#include "skill.h"
#include "play.h"
#include "stdbool.h"
#include "motorControl.h"

enum strategy_states
	{strategy_state_idle, strategy_state_bestStrategy} strategy_state;

enum bestStrategy_states {bestStrategy_state_start, bestStrategy_state_rushGoal,
	bestStrategy_state_blockGoal, bestStrategy_state_goToCenter} bestStrategy_state;

void strategy_bestStrategy()
{
	strategy_state = strategy_state_bestStrategy;
	bestStrategy_state = bestStrategy_state_start;
}

bool strategy_shouldRushGoal()
{
	return robot2cameraPosition.x > 0 ||
			utility_dist(utility_3to2(robot1currentPosition), ball) <
			utility_dist(utility_3to2(robot2cameraPosition), ball);
}

void strategy_continueBestStrategy()
{
	switch (bestStrategy_state)
	{
	case bestStrategy_state_start:
		if(isnan(ball.x))
		{
			printf("------ bestStrategy_state_goToCenter\n");
			bestStrategy_state = bestStrategy_state_goToCenter;
			skill_goToPoint((coord3){P_START.x, P_START.y, 0});
		}
		else if(strategy_shouldRushGoal())
		{
			printf("------ bestStrategy_state_rushGoal\n");
			play_rushGoal();
			bestStrategy_state = bestStrategy_state_rushGoal;
		}
		else
		{
			printf("------ bestStrategy_state_blockGoal\n");
			play_blockGoal();
			bestStrategy_state = bestStrategy_state_blockGoal;
		}
		break;
	case bestStrategy_state_rushGoal:
		if(isnan(ball.x))
		{
			printf("------ bestStrategy_state_goToCenter\n");
			bestStrategy_state = bestStrategy_state_goToCenter;
			skill_goToPoint((coord3){P_START.x, P_START.y, 0});
		}
		else if(!strategy_shouldRushGoal())
		{
			printf("------ bestStrategy_state_blockGoal\n");
			play_blockGoal();
			bestStrategy_state = bestStrategy_state_blockGoal;
		}
		break;
	case bestStrategy_state_blockGoal:
		if(isnan(ball.x))
		{
			printf("------ bestStrategy_state_goToCenter\n");
			bestStrategy_state = bestStrategy_state_goToCenter;
			skill_goToPoint((coord3){P_START.x, P_START.y, 0});
		}
		else if(strategy_shouldRushGoal())
		{
			printf("------ bestStrategy_state_rushGoal\n");
			play_rushGoal();
			bestStrategy_state = bestStrategy_state_rushGoal;
		}
		break;
	case bestStrategy_state_goToCenter:
		if(isnan(ball.x));
		else if(strategy_shouldRushGoal())
		{
			printf("------ bestStrategy_state_rushGoal\n");
			play_rushGoal();
			bestStrategy_state = bestStrategy_state_rushGoal;
		}
		else
		{
			printf("------ bestStrategy_state_blockGoal\n");
			play_blockGoal();
			bestStrategy_state = bestStrategy_state_blockGoal;
		}
		break;
	}
}

void strategy_stop()
{
	printf("stop strategy\n");
	strategy_state = strategy_state_idle;
}

void strategy_tick()
{
	switch (strategy_state)
	{
	case strategy_state_idle:
		break;
	case strategy_state_bestStrategy:
		strategy_continueBestStrategy();
		break;
	}
}
