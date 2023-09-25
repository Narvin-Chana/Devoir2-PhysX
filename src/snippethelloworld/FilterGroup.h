#pragma once

struct FilterGroup
{
	enum Enum
	{
		BALL_FLY = (1 << 0),
		BALL_BOUNCE = (1 << 1),
		BALL_THROUGH = (1 << 2),
		BALL_SLOW = (1 << 3),
		WALL = (1 << 4)
	};
};