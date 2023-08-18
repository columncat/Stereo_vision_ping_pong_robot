#include "Dynamixel_wrapper.h"

#define MX64_TORQUE_ENABLE          24
#define MX64_GOAL_POSITION          30
#define MX64_MOVING_SPEED           32
#define MX64_TORQUE_LIMIT           34
#define MX64_PRESENT_POSITION       36
#define MX64_IS_MOVING              46

#define XM430_TORQUE_ENABLE         64
#define XM430_Profile_Velocity      112
#define XM430_GOAL_POSITION         116
#define XM430_PRESENT_POSITION      132

// 1st set for default pose
// 2nd set for stroked pose
// order in 1st, 2nd, 3rd motor
int cmd_set[2][3] = { {1300, 3330, 1480},
                      {2275, 2580, 917} };
int current_pose = 0;
int current_angle = 2048;

dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler("COM5");
dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);

// Initializing dynamixel settings and fetching default pose
bool Dynamixel_wrapper::init()
{
    if (!portHandler->openPort()) return false;
    if (!portHandler->setBaudRate(57600)) return false;

    printf("Dynamixel port found. \n");

    // Torque ON
    packetHandler->write1ByteTxRx(portHandler, 1, MX64_TORQUE_ENABLE, 1);
    packetHandler->write1ByteTxRx(portHandler, 2, MX64_TORQUE_ENABLE, 1);
    packetHandler->write1ByteTxRx(portHandler, 3, XM430_TORQUE_ENABLE, 1);
    packetHandler->write1ByteTxRx(portHandler, 4, XM430_TORQUE_ENABLE, 1);

    // Setting torque limit
    packetHandler->write2ByteTxRx(portHandler, 1, MX64_TORQUE_LIMIT, 1023);
    packetHandler->write2ByteTxRx(portHandler, 2, MX64_TORQUE_LIMIT, 1023);

    // Setting moving speed
    packetHandler->write2ByteTxRx(portHandler, 1, MX64_MOVING_SPEED, 150);
    packetHandler->write2ByteTxRx(portHandler, 2, MX64_MOVING_SPEED, 150);

    // Enable DYNAMIXEL Torque
    packetHandler->write2ByteTxRx(portHandler, 1, MX64_GOAL_POSITION, cmd_set[0][0]);
    packetHandler->write2ByteTxRx(portHandler, 2, MX64_GOAL_POSITION, cmd_set[0][1]);
    packetHandler->write4ByteTxRx(portHandler, 3, XM430_GOAL_POSITION, cmd_set[0][2]);
    packetHandler->write4ByteTxRx(portHandler, 4, XM430_GOAL_POSITION, 2048);

    // init : 800, 800, 600
    packetHandler->write2ByteTxRx(portHandler, 1, MX64_MOVING_SPEED, 800);
    packetHandler->write2ByteTxRx(portHandler, 2, MX64_MOVING_SPEED, 800);
    packetHandler->write4ByteTxRx(portHandler, 3, XM430_Profile_Velocity, 125);
    packetHandler->write4ByteTxRx(portHandler, 4, XM430_Profile_Velocity, 600);

    return true;
}

// Fetching default pose before closing
void Dynamixel_wrapper::close()
{
    if (current_pose != 0) fetch_pose(0);
    if (current_angle != 2048) fetch_angle(2048);

    wait_motion();

    portHandler->closePort();
    printf("Successfully closed dynamixel port.\n");
}

// cmd 0 for default pose
// cmd 1 for stroked pose
void Dynamixel_wrapper::fetch_pose(int cmd)
{
    packetHandler->write2ByteTxRx(portHandler, 1, MX64_GOAL_POSITION, cmd_set[cmd][0]);
    packetHandler->write2ByteTxRx(portHandler, 2, MX64_GOAL_POSITION, cmd_set[cmd][1]);
    packetHandler->write4ByteTxRx(portHandler, 3, XM430_GOAL_POSITION, cmd_set[cmd][2]);
    current_pose = cmd;
}

// Fetching position for 4th motor
void Dynamixel_wrapper::fetch_angle(int angle)
{
    packetHandler->write4ByteTxRx(portHandler, 4, XM430_GOAL_POSITION, angle);
    current_angle = angle;
}

// Fetch single command for single motor
bool Dynamixel_wrapper::is_moving()
{
    uint8_t moving_1 = 1, moving_2 = 1;
    packetHandler->read1ByteTxRx(portHandler, 1, MX64_IS_MOVING, &moving_1);
    packetHandler->read1ByteTxRx(portHandler, 2, MX64_IS_MOVING, &moving_2);
    return moving_1 == 1 || moving_2 == 1;
}

// Return true if dynamixel is still moving
void Dynamixel_wrapper::wait_motion()
{
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    while (interval.count() < 3000)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        end = std::chrono::steady_clock::now();
        interval = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    }
}

// wait synchronizily until motion ends
bool Dynamixel_wrapper::check_position(int pos)
{
    uint16_t pos_1 = 0, pos_2 = 0;
    uint32_t pos_3 = 0;
    packetHandler->read2ByteTxRx(portHandler, 1, MX64_PRESENT_POSITION, &pos_1);
    packetHandler->read2ByteTxRx(portHandler, 2, MX64_PRESENT_POSITION, &pos_2);
    packetHandler->read4ByteTxRx(portHandler, 3, XM430_PRESENT_POSITION, &pos_3);
    if (abs((int)pos_1 - cmd_set[pos][0]) > 10) return false;
    if (abs((int)pos_2 - cmd_set[pos][1]) > 10) return false;
    if (abs((int)pos_3 - cmd_set[pos][2]) > 10) return false;
    return true;
}
