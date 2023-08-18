#pragma once
#include "dynamixel_sdk.h"
#include <thread>
#include <chrono>

class Dynamixel_wrapper
{
public:
	bool init();
	void close();
	bool is_moving();
	void fetch_pose(int pos);
	void fetch_angle(int vel);
	void wait_motion();
	bool check_position(int pos);
};