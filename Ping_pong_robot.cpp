#include "opencv2/core/core.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cstdlib>
#include <conio.h>
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "Dynamixel_wrapper.h"
#include "Linear_actuator.h"

#define ENABLE_DYNAMIXEL    true
#define ENABLE_LINEAR       true
#define ENABLE_VISION       true

#define INVERT_CAM          true

#define FOCAL_LENGTH_PX     368
#define BASELINE            169

using namespace cv;
using namespace std;

cv::Scalar lower_orange(0, 60, 160);
cv::Scalar upper_orange(40, 140, 255);

cv::Mat lorange_mask, rorange_mask;
std::vector<std::vector<cv::Point>> lcontours, rcontours;
std::vector<cv::Vec4i> lhierarchy, rhierarchy;

cv::VideoCapture *lc, *rc;
cv::Mat *lf, *rf;

bool *operating;
bool *updated;

void capture_frame()
{
    while (*operating)
    {
        *lc >> *lf;
        *rc >> *rf;
        *updated = true;
    }
}

bool get_coordination(int& displacement, int& distance)
{
    while (!(*updated)) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }

    *updated = false;

    cv::inRange(*lf, lower_orange, upper_orange, lorange_mask);
    cv::inRange(*rf, lower_orange, upper_orange, rorange_mask);

    cv::findContours(lorange_mask, lcontours, lhierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(rorange_mask, rcontours, rhierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (rcontours.size() == 0 && lcontours.size() != 0) displacement = -240;
    if (lcontours.size() == 0 && rcontours.size() != 0) displacement = 240;
    for (size_t i = 0; i < lcontours.size(); ++i)
    {
        if (contourArea(lcontours[i]) > 5)
        {
            //cv::drawContours(*lf, lcontours, i, cv::Scalar(255), 2, cv::LINE_8, lhierarchy, 0);
            for (size_t j = 0; j < rcontours.size(); ++j)
            {
                if (contourArea(rcontours[j]) > 5)
                {
                    //cv::drawContours(*rf, rcotours, j, cv::Scalar(255), 2, cv::LINE_8, rhierarchy, 0);

                    cv::Moments m_left = cv::moments(lcontours[i]);
                    cv::Point centroid_left(m_left.m10 / m_left.m00, m_left.m01 / m_left.m00);

                    cv::Moments m_right = cv::moments(rcontours[j]);
                    cv::Point centroid_right(m_right.m10 / m_right.m00, m_right.m01 / m_right.m00);

                    distance = (int)(FOCAL_LENGTH_PX * BASELINE / abs(centroid_left.x - centroid_right.x));
                    displacement = (centroid_left.x + centroid_right.x - 640) / 2;

                    return true;
                }
            }
        }
    }
    return false;
}

void linear_regression(int* x, int* y, int n, double& slope, int& centoid_x, int& centoid_y)
{
    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;

    for (int i = 0; i < n; i++) {
        sum_x += *(x + i);
        sum_y += *(y + i);
        sum_xy += *(x + i) * *(y + i);
        sum_x2 += *(x + i) * *(x + i);
    }

    double a = ((n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x));
    slope = a / ((sum_y - a * sum_x) / n);
    centoid_x = sum_x / n;
    centoid_y = sum_y / n;
}

int get_timing(double slope, int centoid_x, int centoid_y, int target_y)
{
    return ((target_y - centoid_y) / slope + centoid_x) / 1000;
}

int predict_x(double slope, int centoid_x, int centoid_y, int timing) {
    return (int)(centoid_y + (timing - centoid_x) * slope);
}


int main(int argc, char** argv)
{
    printf("[Booting device]\n\n");

    // Declaration part
    Dynamixel_wrapper dw;
    cv::VideoCapture leftCam, rightCam;
    Linear_actuator la;

    cv::Mat leftFrame, rightFrame;
    bool udt = false, opr = true;
    bool found = false;
    bool hit = false;

    const int initial_prediction_data = 4, array_len = 50;
    int xarr[array_len] = { 0, }, yarr[array_len] = { 0, }, time[array_len] = { 0, };
    int pp_x = 0, prev_x = 0, curr_x = 0;
    int prev_term = 0, term = 0;
    int frame_idx = 1, cnt_not_found = 0;
    int centoid_x = 0, centoid_y = 0, centoid_time = 0;
    int timing = 0, time_left = 0;
    int frame_count = 1;
    int delay = 170;
    int current_time_left = 0;

    double prev_pos = 0, curr_pos = 0;
    double xslope = 0, yslope = 0;
    
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    auto curr = std::chrono::steady_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(curr - start);

    leftCam >> leftFrame;
    rightCam >> rightFrame;

    lc = &leftCam;
    rc = &rightCam;
    lf = &leftFrame;
    rf = &rightFrame;
    updated = &udt;
    operating = &opr;


    // Initialize elements
    if (ENABLE_VISION)
    {
        printf("Cameras found.\n");
        std::vector<int> cam_params = {
            cv::CAP_PROP_FRAME_WIDTH, 640,
            cv::CAP_PROP_FRAME_HEIGHT, 480,
            cv::CAP_PROP_FPS, 90,
            cv::CAP_PROP_EXPOSURE, -6,
            cv::CAP_PROP_AUTO_WB, 1,
        };
        leftCam.open(INVERT_CAM ? 1 : 0, cv::CAP_DSHOW, cam_params);
        rightCam.open(INVERT_CAM ? 0 : 1, cv::CAP_DSHOW, cam_params);
        leftCam.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        rightCam.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    }
    if (ENABLE_DYNAMIXEL) dw.init();
    if (ENABLE_LINEAR) la.init();

    // Instantly get frames from cameras by multi-threading
    thread start_capture(capture_frame);

    std::cout << endl << endl << "[MOdule Ping Pong READY]" << endl;


    while (ENABLE_VISION)
    {
        if (_kbhit())
        {
            std::cout << "keyboard interupt detected.exiting process..." << endl;
            break;
        }

        start = std::chrono::steady_clock::now();

        found = get_coordination(xarr[0], yarr[0]);
        /* cv::imshow("Left Camera", leftFrame);
        cv::imshow("Right Camera", rightFrame);
        if (cv::waitKey(1) == 27) break; */

        curr = std::chrono::steady_clock::now();
        interval = std::chrono::duration_cast<std::chrono::milliseconds>(curr - start);
        time[0] = interval.count();

        // reset all variables if ball is found
        if (found && ENABLE_LINEAR)
        {
            std::cout << endl;

            // double check if dynamixel position is default
            if (!dw.check_position(0)) dw.fetch_pose(0);

            frame_idx = 1, cnt_not_found = 0;
            centoid_x = 0, centoid_y = 0, centoid_time = 0;
            timing = 0, time_left = 0;
            xslope = 0, yslope = 0;
            hit = false;
            frame_count = 1;

            pp_x = prev_x = curr_x = xarr[0];
            prev_pos = curr_pos = la.get_position();
            prev_term = term = interval.count();

            while (cnt_not_found < 6 || frame_idx >= initial_prediction_data)
            {
                found = get_coordination(xarr[frame_idx], yarr[frame_idx]);
                frame_count++;
                curr = std::chrono::steady_clock::now();
                interval = std::chrono::duration_cast<std::chrono::milliseconds>(curr - start);
                time[frame_idx] = interval.count();

                // update datas if ball is found
                if (found)
                {
                    pp_x = prev_x;
                    prev_x = curr_x;
                    curr_x = xarr[frame_idx];
                    prev_pos = curr_pos;
                    curr_pos = la.get_position();
                    prev_term = term;
                    term = time[frame_idx] - time[frame_idx - 1];
                    frame_idx++;
                }
                la.fetch_tracking(curr_x, prev_x, pp_x, term, prev_term);

                // discard datas if too many frames are lost
                if (frame_idx < initial_prediction_data && !found)
                {
                    cnt_not_found++;
                    continue;
                }

                // calculate timing that ball reaches to robot if there are enough datas for linear regression
                if (frame_idx >= initial_prediction_data)
                {
                    linear_regression(time, yarr, frame_idx, yslope, centoid_time, centoid_y);
                    // discard data if ball is getting away from robot
                    if (yslope >= 0) break;
                    if (!hit) timing = get_timing(yslope, centoid_time, centoid_y, 220);
                    // discard data if it takes too long for the ball to get to robot
                    if (centoid_time + timing - interval.count() > 1000) break;

                    // get the time left until the calculated timing
                    // delay is roughly 50% of the time needed to end stroke motion from fetching the command
                    // subtract delay from time left to ensure the stroke to hit the ball
                    current_time_left = centoid_time + timing - interval.count() - delay;
                    cout << "ball meets at " << centoid_time + timing - interval.count() << endl;

                    if (current_time_left < 35)
                    {
                        // to match the timing precisely, wait for the timing
                        if (current_time_left > 0)
                        {
                            std::cout << "waiting for timing for " << current_time_left << "ms" << endl;
                            std::this_thread::sleep_for(std::chrono::milliseconds(current_time_left));
                        }

                        // fetch stroke motion if not fetched
                        if (!hit)
                        {
                            std::cout << "hit process" << endl;
                            dw.fetch_pose(1);
                            dw.fetch_angle(2048 + (la.get_position() * 3));
                            hit = true;
                        }
                        // finish the loop if stroke motion is over
                        else if (!dw.is_moving())
                        {
                            la.stop();
                            dw.fetch_pose(0);
                            dw.fetch_angle(2048);

                            curr = std::chrono::steady_clock::now();
                            interval = std::chrono::duration_cast<std::chrono::milliseconds>(curr - start);
                            printf("Tracking process returned with average fps %.2fHz.\n", (double)frame_count / interval.count() * 1000);

                            break;
                        }
                    }
                }

                // break if it takes too long to end the loop
                if (interval.count() > 2000)
                {
                    std::cout << "lost detection" << endl;
                    break;
                }
            }
        }

        // set default linear position to 0
        la.set_goal_position(0, 25, 2);
    }

    std::cout << endl << "[Program in return]" << endl;

    // Kill capture thread
    *operating = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    start_capture.join();

    // Close elements
    if (ENABLE_LINEAR) la.close();
    if (ENABLE_DYNAMIXEL) dw.close();
    if (ENABLE_VISION)
    {
        leftCam.release();
        rightCam.release();
        printf("Successfully closed camera port.\n");
    }

    return 0;
}
