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
//    char *WL_uart = (char*)"/dev/ttyS0";
//    char *uart_name = (char*)"/dev/ttyUSB0";
    char *WL_uart = (char*)"/dev/ttyUSB0";
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



    serial_port.start();
    WL_serial_port.start();
    autopilot_interface.start();
    // --------------------------------------------------------------------------
    //   RUN COMMANDS
    // --------------------------------------------------------------------------

    /*
     * Now we can implement the algorithm we want on top of the autopilot interface
     */


   commands(autopilot_interface);
//    while(1){
//               sleep(1);
//    }
    // --------------------------------------------------------------------------
    //   THREAD and PORT SHUTDOWN
    // --------------------------------------------------------------------------
    //  Now that we are done we can stop the threads and close the port

    autopilot_interface.stop();
    serial_port.stop();
    WL_serial_port.stop();

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
    mavlink_global_position_int_t gp,global_pos;
    mavlink_set_position_target_global_int_t gsp;
    float dist, distance, XYdis;
    bool flag = true;
    bool detect = false;
    TargetNum = 0;
    int TNum = 0;
    stable = false;
    updateellipse = false;
    drop = false;

    // --------------------------------------------------------------------------
    //   START OFFBOARD MODE
    //   设置guided（offboard）模式/解锁、起飞
    // --------------------------------------------------------------------------
    while(flag)
    {
        if(api.Inter_message.command_long.command == 400)
        {
            api.enable_offboard_control();
            usleep(100); // give some time to let it sink in
            break;
        }
        else
        {
            usleep(200000);
        }
    }


    // --------------------------------------------------------------------------
    //   SEND OFFBOARD COMMANDS
    // --------------------------------------------------------------------------
    printf("Start Mission!\n");

    while(flag)
    {
        float yaw;
        //设置触发节点
        if(api.getposition == 1)
        {
            global_pos = api.current_messages.global_position_int;
            gp = api.Inter_message.global_position_int;
            // --------------------------------------------------------------------------
            // 设置guided模式
            // --------------------------------------------------------------------------
            api.Set_Mode(05);
            usleep(100);
            api.Set_Mode(04);
            usleep(100);
            float galt = (float)global_pos.relative_alt/1000.0;
            yaw = D2R(gp.hdg);
            gsp.yaw = yaw;
            gsp.time_boot_ms = (uint32_t) (get_time_usec() / 1000);
            gsp.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            float High = 20;
            //set global_point 经度，纬度，相对home高度
            set_global_position(gp.lat,
                                gp.lon,
                                galt,
                                gsp);
            set_global_yaw(yaw,gsp);
            api.update_global_setpoint(gsp);
            while(flag)
            {
                mavlink_global_position_int_t current_global = api.global_position;
                float distan = Distance(current_global.lat,current_global.lon,current_global.relative_alt,gp.lat,gp.lon,gp.relative_alt);
                if(distan < 8)
                {
                    usleep(200);
                    sleep(2);
                    api.getposition = 0;
                    detect= true;
                    break;
                }
                else
                {
                    api.update_global_setpoint(gsp);
                    usleep(200000);
                }
            }
        }
        else
        {
            usleep(200000);
        }
        while(detect)
        {
            mavlink_set_position_target_local_ned_t locsp;
            set_velocity(0, 0, 0.8, locsp);
            set_yaw(yaw, locsp);
            locsp.z = -20;
            api.update_local_setpoint(locsp);
            while((api.current_messages.local_position_ned.z + 20.5) <=0)
            {
                api.update_local_setpoint(locsp);
                usleep(200000);
            }
            set_velocity(0, 0, 0, locsp);
            api.update_local_setpoint(locsp);
            thread t1(videothread, ref(api));//ref可以使autopilot_interface引用被正确传递给videothread.
            sleep(8);//设置一定时间启动视觉线程,并识别圆
            //停止添加圆,然后开始识别字符
            updateellipse = true;

            while(target_ellipse_position.size() > TargetNum)
            {
                int TF=0;
                stable = true;
                mavlink_set_position_target_local_ned_t sp;
                float Disx = target_ellipse_position[TargetNum].x - api.current_messages.local_position_ned.x;
                float Disy = target_ellipse_position[TargetNum].y - api.current_messages.local_position_ned.y;
                float Adisx = fabsf(Disx);
                float Adisy = fabsf(Disy);

                while((Adisx >= 2)||(Adisy >= 2))
                {

                    if (Adisx >= Adisy)
                    {
                        set_velocity(0.5*(Disx / Adisx),0.5*( Disy / Adisx), 0, sp);
                    }
                    else
                    {
                        set_velocity(0.5*(Disx / Adisy), 0.5*(Disy / Adisy), 0, sp);
                    }
                    set_yaw(yaw, // [rad]
                            sp);
                    api.update_local_setpoint(sp);
                    usleep(200000);
                    Disx = target_ellipse_position[TargetNum].x - api.current_messages.local_position_ned.x;
                    Disy = target_ellipse_position[TargetNum].y - api.current_messages.local_position_ned.y;
                    Adisx = fabsf(Disx);
                    Adisy = fabsf(Disy);

                }

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
                if (ellipse_T.size() > TNum)
                {

                    TNum = api.Throw(yaw,TNum);
                    TargetNum = target_ellipse_position.size();
                    detect= false;
                    flag = false;
                    break;
                }
                else
                {
                    TargetNum = TargetNum + 1;
                }
            }
            t1.join();
        }
    }

    api.Set_Mode(05);
    usleep(200);
    api.Set_Mode(04);
    usleep(200);
    //返航高度
    float High = 20;
    mavlink_set_position_target_local_ned_t locsp;
    set_velocity(0, 0, -1, locsp);
    api.update_local_setpoint(locsp);
    while((api.current_messages.local_position_ned.z + High) >=0)
    {
        api.update_local_setpoint(locsp);
        usleep(200000);
    }
    set_velocity(0, 0, 0, locsp);
    api.update_local_setpoint(locsp);
    //返航
    mavlink_command_long_t com3 = { 0 };
    com3.target_system= 01;
    com3.target_component = 01;
    com3.command = 20;

    mavlink_message_t message3;
    mavlink_msg_command_long_encode(255, 190, &message3, &com3);

    int len3 = api.write_message(message3);
    while (len3 <= 0)
    {
        len3 = api.write_message(message3);
    }


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


