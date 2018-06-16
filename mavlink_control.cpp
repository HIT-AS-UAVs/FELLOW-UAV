/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"
#include <cv.h>
#include "ellipse/EllipseDetectorYaed.h"
#include "autopilot_interface.h"
#include <thread>//多线程
#include <fstream>
#include <cmath>

using namespace cv;
using namespace std;

vector<target> target_ellipse_position, ellipse_T, ellipse_F;


// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

    // --------------------------------------------------------------------------
    //   PARSE THE COMMANDS
    // --------------------------------------------------------------------------

    // Default input arguments
#ifdef __APPLE__
    char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
    char *uart_name = (char*)"/dev/ttyTHS2";
    char *WL_uart = (char*)"/dev/ttyS0";
//    char *uart_name = (char*)"/dev/ttyUSB0";
//    char *WL_uart = (char*)"/dev/ttyUSB1";
#endif
    int baudrate = 57600;

    // do the parse, will throw an int if it fails
    parse_commandline(argc, argv, uart_name, baudrate);
    parse_commandline(argc, argv, WL_uart, baudrate);


    // --------------------------------------------------------------------------
    //   PORT and THREAD STARTUP
    // --------------------------------------------------------------------------

    /*
     * Instantiate a serial port object
     *
     * This object handles the opening and closing of the offboard computer's
     * serial port over which it will communicate to an autopilot.  It has
     * methods to read and write a mavlink_message_t object.  To help with read
     * and write in the context of pthreading, it gaurds port operations with a
     * pthread mutex lock.
     *
     */
    Serial_Port serial_port(uart_name, baudrate);
    Serial_Port WL_serial_port(WL_uart,baudrate);


    /*
     * Instantiate an autopilot interface object
     *
     * This starts two threads for read and write over MAVlink. The read thread
     * listens for any MAVlink message and pushes it to the current_messages
     * attribute.  The write thread at the moment only streams a position target
     * in the local NED frame (mavlink_set_position_target_local_ned_t), which
     * is changed by using the method update_setpoint().  Sending these messages
     * are only half the requirement to get response from the autopilot, a signal
     * to enter "offboard_control" mode is sent by using the enable_offboard_control()
     * method.  Signal the exit of this mode with disable_offboard_control().  It's
     * important that one way or another this program signals offboard mode exit,
     * otherwise the vehicle will go into failsafe.
     *
     */
    Autopilot_Interface autopilot_interface(&serial_port, &WL_serial_port);

    /*
     * Setup interrupt signal handler
     *
     * Responds to early exits signaled with Ctrl-C.  The handler will command
     * to exit offboard mode if required, and close threads and the port.
     * The handler in this example needs references to the above objects.
     *
     */
    serial_port_quit         = &serial_port;
    serial_port_quit         = &WL_serial_port;
    autopilot_interface_quit = &autopilot_interface;
    signal(SIGINT,quit_handler);

    /*
     * Start the port and autopilot_interface
     * This is where the port is opened, and read and write threads are started.
     */



//    serial_port.start();
//    WL_serial_port.start();
//    autopilot_interface.start();
//视觉定位线程
    thread t1(videothread, ref(autopilot_interface));//ref可以使autopilot_interface引用被正确传递给videothread.
    // --------------------------------------------------------------------------
    //   RUN COMMANDS
    // --------------------------------------------------------------------------
    sleep(10);
    updateellipse = true;
    stable = true;
    sleep(20);
    drop = true;
    /*
     * Now we can implement the algorithm we want on top of the autopilot interface
     */


//   commands(autopilot_interface);
    while(1){
               sleep(10);


    }
    // --------------------------------------------------------------------------
    //   THREAD and PORT SHUTDOWN
    // --------------------------------------------------------------------------
    //  Now that we are done we can stop the threads and close the port

    autopilot_interface.stop();
    serial_port.stop();
    t1.join();

    // --------------------------------------------------------------------------
    //   DONE
    // --------------------------------------------------------------------------

    // woot!
    return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands(Autopilot_Interface &api)
{
    mavlink_global_position_int_t gp;
    float dist, distance, XYdis;
    bool flag = true;
    bool goback = true;
    TargetNum = 0;
    int TNum = 0;
    stable = false;
    updateellipse = false;
    drop = false;
//    while(flag)
//    {
//        if((api.current_messages.param_value.param_type==9)&&(api.current_messages.param_value.param_index == 65535))
//        {
//            break;
//        }
//        else
//        {
//            usleep(1000);
//        }
//    }
    // --------------------------------------------------------------------------
    //   START OFFBOARD MODE
    //   设置guided（offboard）模式/解锁、起飞
    // --------------------------------------------------------------------------
    api.enable_offboard_control();
    usleep(100); // give some time to let it sink in

    // --------------------------------------------------------------------------
    //   SEND OFFBOARD COMMANDS
    // --------------------------------------------------------------------------
    printf("Start Mission!\n");
//sleep(100);
    while(flag)
    {
        gp = api.global_position;
        stable = false;
        updateellipse = false;
        float yaw;
        //设置触发节点
        if (target_ellipse_position.size() > TargetNum)
        {
            // --------------------------------------------------------------------------
            // 设置guided模式
            // --------------------------------------------------------------------------
            api.Set_Mode(05);
            usleep(100);
            api.Set_Mode(04);
            usleep(100);

            //设置成为只更新椭圆位置,不添加新的椭圆坐标
            updateellipse = true;
            mavlink_set_position_target_local_ned_t sp;
            //现在用当前高度,最终高度确定时使用
            float local_alt = -gp.relative_alt/1000.0;
            sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
            yaw = D2R(gp.hdg);
            while(goback)
            {
                // --------------------------------------------------------------------------
                // 给定局部坐标(local_ned)位置，并执行
                // --------------------------------------------------------------------------

                // -------------------------------------------------------------------------
                // 							设置位置/朝向
                // -------------------------------------------------------------------------
                set_position(  target_ellipse_position[TargetNum].x, // [m]
                               target_ellipse_position[TargetNum].y, // [m]
                               local_alt, // [m]
                               sp);
                set_yaw( yaw, // [rad]
                         sp     );

                // SEND THE COMMAND
                api.update_local_setpoint(sp);

                while(1)
                {
                    mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
                    float XYdis = XYDistance(pos.x,pos.y,sp.x,sp.y);
                    if(XYdis >10 && XYdis< 20)
                    {
                        local_alt = -25;
                        // -------------------------------------------------------------------------
                        // 							设置位置/朝向
                        // -------------------------------------------------------------------------
                        //                       set_position(  target_ellipse_position[TargetNum].x, // [m]
                        //                                      target_ellipse_position[TargetNum].y, // [m]
                        //                                      local_alt, // [m]
                        //                                      sp);
                        sp.z = local_alt;
                        // SEND THE COMMAND
                        api.update_local_setpoint(sp);
                        sleep(1);
                    }
                    else if (XYdis < 10)
                    {
                        local_alt = -20;
                        // -------------------------------------------------------------------------
                        // 							设置位置/朝向
                        // -------------------------------------------------------------------------
                        set_position(  target_ellipse_position[TargetNum].x, // [m]
                                       target_ellipse_position[TargetNum].y, // [m]
                                       local_alt, // [m]
                                       sp);
                        set_yaw(yaw, // [rad]
                                sp);
                        // SEND THE COMMAND
                        api.update_local_setpoint(sp);
                        sleep(1);

                        float distance = Distance(pos.x,pos.y,pos.z,sp.x,sp.y,sp.z);
                        if(distance < 5)
                        {
                            int TF=0;
                            stable = true;
                             //------------------------------------------------------------------------------
                             //    调用判断T/F的函数
                             //    若为T则将当前全局坐标系发送给从机[TagetNum+1]
                             //------------------------------------------------------------------------------
                           // 需添加固定时间内未检测到任何东西直接break
                            while (stable)
                            {
                                sleep(1);
                                TF++;
                                if (TF==10)
                                {
                                    int TplusF = target_ellipse_position[TargetNum].T_N + target_ellipse_position[TargetNum].F_N;
                                    if(TplusF <= 10 )
                                    {
                                        break;
                                    }
                                    else
                                    {
                                        continue;
                                    }
                                }
                                else
                                {
                                    ;
                                }
                            }
//                            sleep(30);
                            if (ellipse_T.size() > TNum)
                            {
                                if ((ellipse_T.size() == 1)&&(TNum == 0))
                                {
                                    TNum = api.Throw(yaw,TNum);
                                    mavlink_global_position_int_t Target_Global_Position;
                                    Target_Global_Position = api.current_messages.global_position_int;
                                    if (Target_Global_Position.lat < 1000)
                                    {
                                        Target_Global_Position = api.current_messages.global_position_int;
                                    }
                                    //后续添加判断采集全局坐标的正确性,如果错误重新选择
                                    int Globallen=  api.Send_WL_Global_Position(TNum + 1, Target_Global_Position);
                                    TargetNum = TargetNum + 1;
                                }
                                else
                                {
                                    mavlink_global_position_int_t Target_Global_Position;
                                    Target_Global_Position = api.current_messages.global_position_int;
                                    if (Target_Global_Position.lat < 1000)
                                    {
                                        Target_Global_Position = api.current_messages.global_position_int;
                                    }
                                    //后续添加判断采集全局坐标的正确性,如果错误重新选择
                                    int Globallen=  api.Send_WL_Global_Position(TNum + 1, Target_Global_Position);
                                    // ------------------------------------------------------------------------------
                                    //	驱动舵机：<PWM_Value:1100-1900> 打开：1700、关闭：1250
                                    //	ServoId：AUX_OUT1-6 对应148-153/9-14
                                    // ------------------------------------------------------------------------------
                                    api.Servo_Control(10, 1700);
                                    TNum = TNum + 1;
                                    TargetNum = TargetNum + 1;
                                }

                            }
                            else
                            {
                                TargetNum = TargetNum + 1;
                            }
                            stable = false;
                            drop = false;
                            break;
                        }
                        else
                        {
                            sleep(1);
                        }

                    }
                        //目标位于20m以外,向目标靠近
                    else
                    {
                        sleep(1);
                    }
                }

                if(TNum == 3)
                {
                    flag = false;
                    break;
                }
                else
                {
                    //判定是否执行完已有目标
                    if (target_ellipse_position.size() == TargetNum)
                    {
                        goback = false;
                        usleep(100);
                    }
                    else
                    {
                        goback = true;
                        usleep(100);
                    }
                }

            }

            printf("\n");

            if(goback == false)
            {
                // -------------------------------------------------------------------------
                // --------------- 全局坐标系下设置目标位置坐标 -------------------------------
                // -------------------------------------------------------------------------
                mavlink_set_position_target_global_int_t gsp;
                mavlink_global_position_int_t ggsp = gp; //api.global_position;
                mavlink_set_position_target_global_int_t global_int_pos;
                global_int_pos.lat_int = ggsp.lat;
                global_int_pos.lon_int = ggsp.lon;
                global_int_pos.alt = ggsp.alt;
                global_int_pos.vx = ggsp.vx;
                global_int_pos.vy = ggsp.vy;
                global_int_pos.vz = ggsp.vz;
                float gyaw = D2R(ggsp.hdg);
                global_int_pos.yaw = gyaw;
                gsp.time_boot_ms = (uint32_t) (get_time_usec() / 1000);
                gsp.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;

                // ------------------------------------------------------------------------
                //			配置设定巡视高度
                // ------------------------------------------------------------------------
                int32_t High = 25;
                //set global_point 经度，纬度，相对home高度
                set_global_position(global_int_pos.lat_int,
                                    global_int_pos.lon_int,
                                    High,
                                    gsp);
//				set_global_yaw(gyaw,
//						       gsp);

                api.update_global_setpoint(gsp);
                while(1)
                {
                    mavlink_global_position_int_t current_global = api.global_position;
                    float distan = Distance(current_global.lat,current_global.lon,current_global.relative_alt,gp.lat,gp.lon,gp.relative_alt);
                    if(distan < 5)
                    {
                        usleep(200);
                        goback = true;
                        api.Set_Mode(03);
                        sleep(2);
                        updateellipse = false;
                        break;
                    }
                    else
                    {
                        usleep(200000);
                    }
                }
            }

        }
        else
        {

            if(TNum == 3)
            {
                flag = false;
            }
            else
            {
                usleep(100000);
            }
            if(api.current_messages.mission_item_reached.seq == 8)
            {
                updateellipse = true;
                sort(ellipse_F.begin(),ellipse_F.end());
                if(TNum == 1)
                {
                    //将F中识别出来的T概率最高的点发送给从机[TNum+1]

                    mavlink_global_position_int_t Target_Global_Position;
                    Target_Global_Position.lat = ellipse_F[0].lat;
                    Target_Global_Position.lon = ellipse_F[0].lon;
                    int Globallen=  api.Send_WL_Global_Position(TNum + 1, Target_Global_Position);
                    usleep(2000);
                    //设置成guided模式,到达F中T的概率第二高的位置,到达指定位置后,再次确定T||F,决定投或者不投
                    api.ThrowF(yaw,ellipse_F[1].lat,ellipse_F[1].lon,ellipse_F[1].num);
                    TNum = TNum + 1;
                    flag = false;
                }
                else if(TNum == 2)
                {

                    //设置成guided模式,到达F中T的概率第二高的位置,到达指定位置后,再次确定T||F,决定投或者不投
                    api.ThrowF(yaw,ellipse_F[0].lat,ellipse_F[0].lon,ellipse_F[0].num);
                    flag = false;
                }
                else
                {
                    flag = false;
                    updateellipse = true;
                }
            }
        }

        printf("\n");

    }

    // --------------------------------------------------------------------------
    // RETURN Home 以及
    //
    // STOP OFFBOARD MODE
    // --------------------------------------------------------------------------
    sleep(5);

//    mavlink_mission_clear_all_t comclearall;
//    comclearall.target_system = 01;
//    comclearall.target_component = 01;
//
//    mavlink_message_t Clearcom;
//    mavlink_msg_mission_clear_all_encode(255,190,&Clearcom,&comclearall);
//    int Clearlen = api.write_message(Clearcom);
    sleep(1);

    //返航
    mavlink_command_long_t com3 = { 0 };
    com3.target_system= 01;
    com3.target_component = 01;
    com3.command = 20;

    mavlink_message_t message3;
    mavlink_msg_command_long_encode(255, 190, &message3, &com3);
    int len3 = api.write_message(message3);


    // now pixhawk isn't listening to setpoint commands

    // --------------------------------------------------------------------------
    //   END OF COMMANDS
    // --------------------------------------------------------------------------

    return;

}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

    // string for command line usage
    const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

    // Read input arguments
    for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

        // Help
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("%s\n",commandline_usage);
            throw EXIT_FAILURE;
        }

        // UART device ID
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
            if (argc > i + 1) {
                uart_name = argv[i + 1];

            } else {
                printf("%s\n",commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Baud rate
        if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
            if (argc > i + 1) {
                baudrate = atoi(argv[i + 1]);

            } else {
                printf("%s\n",commandline_usage);
                throw EXIT_FAILURE;
            }
        }

    }
    // end: for each input argument

    // Done!
    return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
    printf("\n");
    printf("TERMINATING AT USER REQUEST\n");
    printf("\n");

    // autopilot interface
    try {
        autopilot_interface_quit->handle_quit(sig);
    }
    catch (int error){}

    // serial port
    try {
        serial_port_quit->handle_quit(sig);
    }
    catch (int error){}

    // end program here
    exit(0);

}

///////////////视觉定位线程
void videothread(Autopilot_Interface& api){

    VideoCapture cap(0);
//    VideoCapture cap;
//    cap.open("T_rotation.avi");
//    cap.open("F.avi");
//    cap.open("T.avi");
    if(!cap.isOpened()) return;
    int width = 640;
    int height = 360;
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
    cap.set(CAP_PROP_AUTOFOCUS,0);


//	 Parameters Settings (Sect. 4.2)
    int		iThLength = 16;
    float	fThObb = 3.0f;
    float	fThPos = 1.0f;
    float	fTaoCenters = 0.05f;
    int 	iNs = 16;
    float	fMaxCenterDistance = sqrt(float(width*width + height*height)) * fTaoCenters;

    float	fThScoreScore = 0.4f;

    // Other constant parameters settings.

    // Gaussian filter parameters, in pre-processing
    Size	szPreProcessingGaussKernelSize = Size(5, 5);
    double	dPreProcessingGaussSigma = 1.0;

    float	fDistanceToEllipseContour = 0.1f;	// (Sect. 3.3.1 - Validation)
    float	fMinReliability = 0.4f;	// Const parameters to discard bad ellipses


    // Initialize Detector with selected parameters
    CEllipseDetectorYaed* yaed = new CEllipseDetectorYaed();
    yaed->SetParameters(szPreProcessingGaussKernelSize,
                        dPreProcessingGaussSigma,
                        fThPos,
                        fMaxCenterDistance,
                        iThLength,
                        fThObb,
                        fDistanceToEllipseContour,
                        fThScoreScore,
                        fMinReliability,
                        iNs
    );

Mat1b gray, gray_big;
VideoWriter writer1("小图.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5.0, Size(640, 360));
	while(true) {

        Mat3b image, image_r;
        cap >> image;
        resize(image, image_r, Size(640, 360), 0, 0, CV_INTER_LINEAR);
        cvtColor(image_r, gray, COLOR_BGR2GRAY);
        cvtColor(image, gray_big, COLOR_BGR2GRAY);

        vector<Ellipse> ellsYaed, ellipse_in, ellipse_big, ellipseok;
        vector<Mat1b> img_roi;
        yaed->Detect(gray, ellsYaed);
        Mat3b resultImage = image_r.clone();
        Mat3b resultImage2 = image_r.clone();
        vector<coordinate> ellipse_out, ellipse_TF, ellipse_out1;
        if(getlocalposition){
            OptimizEllipse(ellipse_in, ellsYaed);//对椭圆检测部分得到的椭圆进行预处理，输出仅有大圆的vector
            yaed->big_vector(resultImage2, ellipse_in, ellipse_big);
            filtellipse(api, ellipseok, ellipse_big);
            yaed->DrawDetectedEllipses(resultImage, ellipse_out, ellipseok);//绘制检测到的椭圆
            if (!drop) {
            vector<vector<Point> > contours;
            if (stable) {
                yaed->extracrROI(gray_big, ellipse_out, img_roi);
                visual_rec(img_roi, ellipse_out, ellipse_TF, contours);//T和F的检测程序
                ellipse_out1 = ellipse_TF;
            } else
                ellipse_out1 = ellipse_out;
            for (auto &p:contours) {
                vector<vector<Point> > contours1;
                contours1.push_back(p);
                drawContours(image, contours1, 0, Scalar(255, 255, 0), 1);
            }
            possible_ellipse(api, ellipse_out1, target_ellipse_position);

            if(stable) {
                resultTF(api, target_ellipse_position, ellipse_T, ellipse_F);
            }

        } else {
            getdroptarget(api, droptarget, ellipse_out);
        }
    }
        cout << "target_ellipse.size = " << target_ellipse_position.size() << endl;
        for (int i = 0; i < target_ellipse_position.size(); ++i) {
            cout << "x = " << target_ellipse_position[i].x << endl
                 << "y = " << target_ellipse_position[i].y << endl
                 << "T = " << target_ellipse_position[i].T_N << endl
                 << "F = " << target_ellipse_position[i].F_N << endl
                 << "possible = " << target_ellipse_position[i].possbile << endl;
        }
        cout << "ellipse_T.size = " << ellipse_T.size() << endl;
        for (int i = 0; i < ellipse_T.size(); ++i) {
            cout << "x = " << ellipse_T[i].x << endl
                 << "y = " << ellipse_T[i].y << endl
                 << "possbile = " << ellipse_T[i].possbile << endl
                 << "lat:" << ellipse_T[i].lat << "lon:" << ellipse_T[i].lon << endl
                 <<"No.:"<<ellipse_T[i].num<<endl;
        }
        cout << "ellipse_F.size = " << ellipse_F.size() << endl;
        for (int i = 0; i < ellipse_F.size(); ++i) {
            cout << "x = " << ellipse_F[i].x << endl
                 << "y = " << ellipse_F[i].y << endl
                 << "possbile = " << ellipse_F[i].possbile << endl
                 << "lat:" << ellipse_F[i].lat << "lon:" << ellipse_F[i].lon << endl
                 <<"No.:"<<ellipse_F[i].num<<endl;
        }
        cout<<"local_position.x:"<<api.current_messages.local_position_ned.x<<endl
            <<"local_position.y:"<<api.current_messages.local_position_ned.y<<endl
            <<"local_position.z:"<<api.current_messages.local_position_ned.z<<endl;
        cout<<"stable:"<<stable<<endl<<"updateellipise:"<<updateellipse<<endl<<"drop:"<<drop<<endl;
//		namedWindow("原图",1);
//		imshow("原图", image);
		namedWindow("缩小",1);
		imshow("缩小", resultImage);
		writer1.write(resultImage);
        ellipse_out.clear();
		waitKey(10);
		ellipse_out1.clear();
		usleep(1000);
	}
}
// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{

    // This program uses throw, wrap one big try/catch here
    try
	{
		int result = top(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}

}


