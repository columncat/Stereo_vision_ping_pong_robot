# Stereo_vision_ping_pong_robot
Ping pong robot using stereo vision with linear actuator and dynamixel arm.
GIST의 2023 창의공학 경진대회에 참여한 부산대의 Pusan Ping Pong 팀의 코드입니다.

[![GIST 2023 창의공학 경진대회 PPP팀 시연 영상](https://img.youtube.com/vi/FdBpEktACxE/0.jpg)](https://www.youtube.com/watch?v=FdBpEktACxE)

Front picture
![20230811_115217](https://github.com/columncat/Stereo_vision_ping_pong_robot/assets/127417901/ab13db30-af20-487e-9fc3-004c88132b00)

Back picture
![20230811_115143](https://github.com/columncat/Stereo_vision_ping_pong_robot/assets/127417901/2b92670e-eeb3-4333-8e37-29f60fac8ee3)



* Hardware configuration


- Logitech Brio 4k pro *2
- Dynamixel XM430 *2
- Dynamixel MX-64 *2
- OpenCR
- Linear actuator



------------------------------------------------



* Hardware Info


1. RRRP robot arm used for strokes

RRRP mechanisms to simulate human stroke




2. Racket with batteries

Batteries are stored at edges of the racket to improve moment of inertia to ensure the ball hit by the racket to move normal to racket plane




3. Two cameras with distance

Used to achieve stereo vision



------------------------------------------------



* Software Info


1. Multi-threading used to copy frames from cameras

As opencv always waits for camera to get next image, multi-threading is used to save time.




2. Ball detection by inRange method and findContour

Gets binary image using inRange method and get masked region using findContour




3. Stereo vision used to get ball distance

From pixel disparity read from two cameras, calculate distance of the ball




4. Linear regression used to calculate when the ball reaches to robot

And also update the timing as each frame is updated




