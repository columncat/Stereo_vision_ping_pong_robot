#include "Linear_actuator.h"


char mot_file[] = "ajin20190628.mot";
int standard_vel = 700;
int standard_accel = 300;

double current_pos = 0;
int current_vel = 0;

const int max_vel = 300;


// Opening linear actuator port
// Initial position must be middle since position is returned relatively
bool Linear_actuator::init()
{
	DWORD Code = AxlOpen(7);
	if (Code == AXT_RT_SUCCESS)
	{
		printf("Linear axis found. \n");
		//Check for Motion Module
		DWORD uStatus;
		Code = AxmInfoIsMotionModule(&uStatus);
		if (Code == AXT_RT_SUCCESS)
		{
			if (uStatus == STATUS_EXIST)
			{
				AxmMotLoadParaAll(mot_file);

				AxmStatusSetActPos(0, 0.0);
				AxmStatusSetCmdPos(0, 0.0);

				AxmSignalServoOn(0, ENABLE);

				AxmMotSetAbsRelMode(0, 1); //0->abs, 1->Rel
				AxmMotSetProfileMode(0, 3);	//0->symetric trapezode, 1->unsymetric trapezode, 2->reserved, 3->symetric S Curve, 4->unsymetric S Cuve
			}
		}
	}
	return true;
}

// Returning actuator to home and close linear actuator port
void Linear_actuator::close()
{
	set_goal_position_and_wait(0, 25, 2);

	AxmSignalServoOn(0, 0);
	AxlClose();

	printf("Successfully closed linear actuator port.\n");
}

// Fetch position command only and not wait
void Linear_actuator::move_by_position(int pos)
{
	AxmMoveStartPos(0, pos, standard_vel, standard_accel, standard_accel);
}

// Fetch position command and wait until motion ends
void Linear_actuator::move_by_position_and_wait(int pos)
{
	AxmMovePos(0, pos, standard_vel, standard_accel, standard_accel);
}

// Fetch velocity command
void Linear_actuator::move_by_velocity(int input_vel)
{
	if (input_vel == 0)
	{
		stop();
		current_vel = input_vel;
	}
	else if (input_vel * current_vel > 0)
	{
		//printf("overriding velocity value\n");
		AxmOverrideSetMaxVel(0, max_vel);
		AxmOverrideVel(0, abs(input_vel));
		current_vel = input_vel;
	}
	else
	{
		if (current_vel != 0) stop();
		//printf("entering velocity control mode\n");
		AxmOverrideSetMaxVel(0, max_vel);
		AxmMoveVel(0, input_vel, standard_accel, standard_accel);
		current_vel = input_vel;
	}
}

// Fetch velocity command if actuator is not at goal position
// MUST be called repeatedly
void Linear_actuator::set_goal_position(int goal_pos, unsigned int goal_vel, int tolerance)
{
	current_pos = get_position();
	int rel_goal_pos = goal_pos - current_pos;
	int cmd_vel = rel_goal_pos > 0 ? goal_vel : -1 * goal_vel;
	if (abs(rel_goal_pos) > tolerance)
	{
		// actuate
		move_by_velocity(cmd_vel);
	}
	else
	{
		// stop
		if (current_vel != 0) stop();
	}
}

// Fetch velocity command until actuator reaches to target point
void Linear_actuator::set_goal_position_and_wait(int goal_pos, unsigned int goal_vel, int tolerance)
{
	current_pos = get_position();
	int rel_goal_pos = goal_pos - current_pos;
	int cmd_vel = rel_goal_pos > 0 ? goal_vel : -1 * goal_vel;
	if (abs(rel_goal_pos) > tolerance)
	{
		// actuate
		move_by_velocity(cmd_vel);
		while (abs(goal_pos - current_pos) > tolerance)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			current_pos = get_position();
		}
		stop();
	}
	else
	{
		// stop
		if (current_vel != 0) stop();
	}
}

// Promptly stop actuator
void Linear_actuator::stop()
{
	AxmMoveEStop(0);
	current_vel = 0;
	wait_motion();
}

// Smoothly stop actuator
void Linear_actuator::sstop()
{
	AxmMoveSStop(0);
	current_vel = 0;
	wait_motion();
}

// Synchronizily wait until motion stops
void Linear_actuator::wait_motion()
{
	DWORD status = 1;
	while (true)
	{
		AxmStatusReadInMotion(0, &status);
		if (status == 1) std::this_thread::sleep_for(std::chrono::milliseconds(5));
		else break;
	}
}

// Return Current position
double Linear_actuator::get_position()
{
	double current_position = 0;
	AxmStatusGetActPos(0, &current_position);
	current_pos = current_position;
	return current_position;
}

// Fetch PD control command to reach to position that x equals 0
// MUST be called repeatedly
void Linear_actuator::fetch_tracking(int x, int prev_x, int pp_x, int term, int prev_term)
{
	double unit_cvt = 50;
	double kp = 1.8;
	double kd = 100;

	int p_ctrl = kp * x;
	int d_ctrl = kd * ((x - prev_x) / (double)term - (prev_x - pp_x) / (double)prev_term);

	current_pos = get_position();
	int linear_command = p_ctrl + d_ctrl;
	linear_command = linear_command > max_vel ? 300 : linear_command < -1 * max_vel ? -300 : linear_command;

	// prevent actuator from collision
	if (current_pos > 40 && current_vel > 0)
	{
		std::cout << "EMERGENCY STOP: RIGHT (" << current_pos << ")" << std::endl;
		stop();
		if (linear_command < 0)
			move_by_velocity(linear_command);
	}
	else if (current_pos < -40 && current_vel < 0)
	{
		std::cout << "EMERGENCY STOP: LEFT (" << current_pos << ")" << std::endl;
		stop();
		if (linear_command > 0)
			move_by_velocity(linear_command);
	}
	else if (current_vel != linear_command)
	{
		move_by_velocity(linear_command);
	}
}
