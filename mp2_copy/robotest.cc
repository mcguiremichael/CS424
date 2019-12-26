#include <SerialStream.h>
#include "irobot-create.hh"
#include <ctime>
#include <iostream>
#include <chrono>
#include <thread>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <mutex>
#include <semaphore.h>

using namespace iRobot;
using namespace LibSerial;
using namespace std;

std::thread song_handler;
thread plot_handler;
mutex song_mtx;
sem_t plot_sem;

/*
 * distances stores the distance the robot travels between every start and stop.
 * angles stores the counter clockwise angle the robot turns through at every turn.
 * angles immediately has the starting angle added to it. We will adjust this value in testing     
 *
 * These two vectors will be used to plot the image contour.
 * */
static std::vector<double> distances = std::vector<double>();
static std::vector<double> angles = std::vector<double>();
// static std::vector<std::vector<double>> positions = std::vector<std::vector<double>>();

/**
 * This double will keep track of the current angle orientation.
 */

/**
 * Width and height of the contour image
 */
static int w = 1000;
static int time_ms = 0;

void DrawLine(cv::Mat &img, cv::Point start, cv::Point end)
{
    int thickness = 5;
    int type = 8;
    line(img,
         start,
         end,
         cv::Scalar(255, 255, 255),
         thickness,
         type);
}

std::vector<double> homogenous_coord_init(double x, double y, double z)
{
    std::vector<double> output = std::vector<double>();
    output.push_back(x);
    output.push_back(y);
    output.push_back(z);
    return output;
}

void write_positions_to_image(std::vector<std::vector<double>> &positions)
{
    cv::Mat contour_img = cv::Mat::zeros(w, w, CV_8UC3);
    int len = positions.size();
    std::vector<cv::Point> points = std::vector<cv::Point>();
    for (int i = 0; i < len; i++)
    {
        std::vector<double> pos = positions[i];
        cout << "pos: " << pos[0] << "," << pos[1] << endl;
        cv::Point p = cv::Point(pos[0] + (w * 3 / 4), pos[1] + (w * 3 / 4));
        points.push_back(p);
    }
    cout << "**********" << endl;
    for (int i = 0; i < len - 1; i++)
    {
        cout << "point: " << points[i] << "," << points[i + 1] << endl;
        DrawLine(contour_img, points[i], points[i + 1]);
    }

    cv::imwrite("contour.jpg", contour_img);
}

void plot_contour()
{
    std::vector<double> curr_loc = homogenous_coord_init(0.0, 0.0, 1.0);
    std::vector<std::vector<double>> positions = std::vector<std::vector<double>>();

    positions.push_back(curr_loc);
    int len = distances.size();
    double u1 = -1.0;
    double u2 = 0.0;
    cout << "***********" << endl;
    cout << "length position bef: " << positions.size() << endl;

    for (int i = 0; i < len; i++)
    {
        double theta = angles[i] * (3.141592653 / 180.0);
        double d = distances[i];
        std::vector<double> d_vec = homogenous_coord_init(d * u1,
                                                          d * u2,
                                                          1.0);
        curr_loc[0] += d_vec[0];
        curr_loc[1] += d_vec[1];
        cout << "d: " << d << endl;
        cout << "angles: " << theta << endl;
        // cout << "u1: " << u1 << ", u2: " << u2 << endl;
        // cout << curr_loc[0] << "," << curr_loc[1] << endl;
        positions.push_back(curr_loc);
        float temp_u1 = u1 * cos(theta) - u2 * sin(theta);
        float temp_u2 = u1 * sin(theta) + u2 * cos(theta);
        u1 = temp_u1;
        u2 = temp_u2;
    }
    cout << "length position aft: " << positions.size() << endl;
    write_positions_to_image(positions);
}

/*
 * Appends new_dist to the distances vector
 */
void add_distance(double new_dist)
{
    distances.push_back(new_dist);
}

/*
 * Appends speed * time to the distances vector
 */
void add_distance(int speed, double time)
{
    add_distance((double)speed * time);
}

/*
 * Appends new_angle to the angles vector
 */
void add_angle(double new_angle)
{
    angles.push_back(new_angle);
}

/*
 * Appends a 90 degree counter-clockwise angle to the angles vector
 * */
void add_ccw_angle()
{
    add_angle(90.0);
}

/*
 * Appends a 90 degree clockwise angle to the angles vector
 * */
void add_cw_angle()
{
    add_angle(-90.0);
}

void plot_handler_func()
{
    while (1)
    {
        sem_wait(&plot_sem);
        plot_contour();
    }
}

void song_handler_func(Create robot)
{
    while (1)
    {
        song_mtx.lock();
        robot.sendPlaySongCommand(0);
        song_mtx.unlock();
        this_thread::sleep_for(chrono::milliseconds(100));
    }
}

int POCL = 0;
int POCR = 0;

int robot_safe(Create &robot)
{
    // robot.readStream();
    int OCL = robot.leftWheelOvercurrent();
    int OCR = robot.rightWheelOvercurrent();
    int WDL = robot.wheeldropLeft();
    int WDR = robot.wheeldropRight();
    int CLFL = robot.cliffFrontLeftSignal();
    int CLFR = robot.cliffFrontRightSignal();
    int CLL = robot.cliffLeftSignal();
    int CLR = robot.cliffRightSignal();

    // cout << "************" << endl;
    // cout << "OCL: " << OCL << endl;
    // cout << "OCR: " << OCR << endl;
    // cout << "WD: " << WDL << endl;
    // cout << "CLFL: " << CLFL << endl;
    // cout << "CLFR: " << CLFR << endl;
    // cout << "CLL: " << CLL << endl;
    // cout << "CLR: " << CLR << endl;

    if (WDL || WDR || CLFL < 50 || CLFR < 50 || CLL < 50 || CLR < 50 || ((OCL && POCL) && (OCR && POCR)))
    {
        // while(1);
        POCL = OCL;
        POCR = OCR;
        return 0;
    }
    POCL = OCL;
    POCR = OCR;
    return 1;
}

int main()
{
    char serial_loc[] = "/dev/ttyUSB0";

    try
    {
        raspicam::RaspiCam_Cv Camera;
        cv::Mat rgb_image, bgr_image;
        if (!Camera.open())
        {
            cerr << "Error opening the camera" << endl;
            return -1;
        }
        cout << "Opened Camera" << endl;
        SerialStream stream(serial_loc, LibSerial::SerialStreamBuf::BAUD_57600);
        cout << "Opened Serial Stream to" << serial_loc << endl;
        this_thread::sleep_for(chrono::milliseconds(1000));
        Create robot(stream);
        cout << "Created iRobot Object" << endl;
        robot.sendFullCommand();
        cout << "Setting iRobot to Full Mode" << endl;
        this_thread::sleep_for(chrono::milliseconds(1000));
        cout << "Robot is ready" << endl;

        // Let's stream some sensors.
        Create::sensorPackets_t sensors;
        sensors.push_back(Create::SENSOR_BUMPS_WHEELS_DROPS);
        sensors.push_back(Create::SENSOR_WALL_SIGNAL);
        sensors.push_back(Create::SENSOR_BUTTONS);
        sensors.push_back(Create::SENSOR_CLIFF_LEFT_SIGNAL);
        sensors.push_back(Create::SENSOR_CLIFF_FRONT_LEFT_SIGNAL);
        sensors.push_back(Create::SENSOR_CLIFF_FRONT_RIGHT_SIGNAL);
        sensors.push_back(Create::SENSOR_CLIFF_RIGHT_SIGNAL);
        sensors.push_back(Create::SENSOR_OVERCURRENTS);

        robot.sendStreamCommand(sensors);
        cout << "Sent Stream Command" << endl;
        // Let's turn!
        int speed = 200;
        int ledColor = Create::LED_COLOR_GREEN;
        robot.sendDriveCommand(speed, Create::DRIVE_STRAIGHT);
        robot.sendLedCommand(Create::LED_PLAY, 0, 0);
        cout << "Sent Drive Command" << endl;

        while (!robot.bumpLeft() && !robot.bumpRight())
        {
            // if (!robot_safe(robot))
            // {
            //     break;
            // }
            this_thread::sleep_for(chrono::microseconds(100));
        }

        robot.sendDriveCommand(-speed, Create::DRIVE_STRAIGHT);
        this_thread::sleep_for(chrono::milliseconds(200));

        robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
        int prev = 0;
        int prev1 = 0;
        while (robot.wallSignal() <= 90 || robot.wallSignal() >= prev)
        {
            if (!robot_safe(robot))
            {
                break;
            }
            prev = robot.wallSignal();
            this_thread::sleep_for(chrono::microseconds(100));
        }

        song_mtx.lock();

        robot.sendDriveCommand(speed, Create::DRIVE_STRAIGHT);

        song_handler = thread(song_handler_func, robot);
        Create::note_t note = make_pair(64, 10);
        Create::song_t song;
        song.push_back(note);
        robot.sendSongCommand(0, song);

        plot_handler = thread(plot_handler_func);

        short wallSignal, prevWallSignal = 0;
        int safe_flag = 0;
        auto start = chrono::steady_clock::now();
        while (!robot.playButton())
        {

            if (!robot_safe(robot))
            {
                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                song_mtx.unlock();
                safe_flag = 1;
            }
            else if (safe_flag)
            {
                if (robot.advanceButton())
                {
                    song_mtx.lock();
                    safe_flag = 0;
                }
            }
            //drive command
            else if (robot.bumpLeft() || robot.bumpRight())
            {
                // move back
                double duration = (double)chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
                cout << "left turn corner, duration time: " << duration << endl;
                robot.sendDriveCommand(-speed, Create::DRIVE_STRAIGHT);
                this_thread::sleep_for(chrono::milliseconds(200));

                // turn counter clock
                robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
                this_thread::sleep_for(chrono::milliseconds(800));
                while (robot.wallSignal() > 90)
                {
                    this_thread::sleep_for(chrono::microseconds(100));
                    if (!robot_safe(robot))
                    {
                        break;
                    }
                }
                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                cout << "finish turn left corner" << endl;
                add_ccw_angle();
                add_distance(20, (double)duration / 1000.0);
                sem_post(&plot_sem);
                start = chrono::steady_clock::now();
            }
            else if (robot.wallSignal() <= 5)
            {
                // robot.sendDriveCommand(speed, Create::DRIVE_STRAIGHT);
                // this_thread::sleep_for(chrono::milliseconds(1000));

                // while(robot.wallSignal()<10){
                double duration = (double)chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
                cout << "right turn corner, duration time: " << duration << endl;
                robot.sendDriveCommand(speed, Create::DRIVE_STRAIGHT);
                this_thread::sleep_for(chrono::milliseconds(1250));

                robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_CLOCKWISE);
                this_thread::sleep_for(chrono::milliseconds(900));
                // }
                if (!robot_safe(robot))
                {
                    continue;
                }
                robot.sendDriveCommand(speed, Create::DRIVE_STRAIGHT);
                this_thread::sleep_for(chrono::milliseconds(1100));
                if (!robot_safe(robot))
                {
                    continue;
                }
                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                cout << "finish turn right corner" << endl;
                add_cw_angle();
                add_distance(20, (double)duration / 1000.0);
                sem_post(&plot_sem);
                start = chrono::steady_clock::now();
            }
            else if (robot.wallSignal() > 110)
            {
                // turn counter clock
                robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);

                while (robot.wallSignal() > 100)
                {
                    // cout << robot.wallSignal() << endl;
                    this_thread::sleep_for(chrono::microseconds(100));
                    if (!robot_safe(robot))
                    {
                        break;
                    }
                }
                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                cout << "finish turn left" << endl;
            }
            else if (robot.wallSignal() < 30)
            {
                // turn clock
                robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_CLOCKWISE);

                prev = 0;
                while (robot.wallSignal() >= prev && robot.wallSignal() < 40)
                {
                    // cout << robot.wallSignal() << endl;
                    prev = robot.wallSignal();
                    this_thread::sleep_for(chrono::microseconds(100));
                    if (!robot_safe(robot))
                    {
                        break;
                    }
                }
                robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
                cout << "finish turn right" << endl;
            }
            else
            {
                // drive straight
                robot.sendDriveCommand(speed, Create::DRIVE_STRAIGHT);
            }
            // You can add more commands here.
            this_thread::sleep_for(chrono::milliseconds(100));
        }

        cout << "Play button pressed, stopping Robot" << endl;
        robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
    }
    catch (InvalidArgument &e)
    {
        cerr << e.what() << endl;
        return 3;
    }
    catch (CommandNotAvailable &e)
    {
        cerr << e.what() << endl;
        return 4;
    }
}
