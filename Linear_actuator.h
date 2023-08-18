#pragma once
#include "stddef.h"
#include "AXL.h"
#include "AXM.h"
#include <iostream>
#include <conio.h>
#include <thread>
#include <chrono>

class Linear_actuator
{
public:
	bool init();
	void close();
	void move_by_position(int pos);
	void move_by_position_and_wait(int pos);
	void move_by_velocity(int vel);
	void set_goal_position(int goal_pos, unsigned int goal_vel, int tolerance);
	void set_goal_position_and_wait(int goal_pos, unsigned int goal_vel, int tolerance);
	void stop();
	void sstop();
	void wait_motion();
	double get_position();
	void fetch_tracking(int x, int prev_x, int pp_x, int term, int prev_term);
};