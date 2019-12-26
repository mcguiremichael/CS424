#include <SerialStream.h>
#include "irobot-create.hh"
#include <ctime>
#include <iostream>
#include <chrono>
#include <thread>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <mutex>

using namespace iRobot;
using namespace LibSerial;
using namespace std;

int LED_FLAG_OFF = 0;

//The thread and lock variables to make multiple threads running
std::thread led_handler;
std::thread song_handler;
mutex led_mtx;
mutex song_mtx;

// specify the owner of the song mutex
// 0 is nobody
// 1 is the song thread
// 2 is the main process
// set after getting the lock
int song_mtx_owner = 0;

//The routine for LED shows
//robot: the robot object
void led_handler_func(Create robot)
{
    while (1)
    {
//Go over the delegated LED sequences
        cout << "led_handler_func_test" << endl;
        led_mtx.lock();
        robot.sendLedCommand(Create::LED_PLAY, 0, Create::LED_INTENSITY_FULL);
        led_mtx.unlock();
        this_thread::sleep_for(chrono::milliseconds(200));

        led_mtx.lock();
        robot.sendLedCommand(Create::LED_ALL, 0, Create::LED_INTENSITY_OFF);
        led_mtx.unlock();
        this_thread::sleep_for(chrono::milliseconds(200));

        led_mtx.lock();
        robot.sendLedCommand(Create::LED_ADVANCE, 255, Create::LED_INTENSITY_FULL);
        led_mtx.unlock();
        this_thread::sleep_for(chrono::milliseconds(200));

        led_mtx.lock();
        robot.sendLedCommand(Create::LED_PLAY, 255, Create::LED_INTENSITY_FULL);
        led_mtx.unlock();
        this_thread::sleep_for(chrono::milliseconds(200));

        led_mtx.lock();
        robot.sendLedCommand(Create::LED_ALL, 0, Create::LED_INTENSITY_OFF);
        led_mtx.unlock();
        this_thread::sleep_for(chrono::milliseconds(200));

        led_mtx.lock();
        robot.sendLedCommand(Create::LED_ADVANCE, 0, Create::LED_INTENSITY_FULL);
        led_mtx.unlock();
        this_thread::sleep_for(chrono::milliseconds(200));
    }
}

//dist is a global variable that is used to control the frequency of sounds
int dist = 0;

//The routine for emitting sound
//robot: the robot object
void song_handler_func(Create robot){
  while(1){
    cout << "Wall signal " << dist << endl;
    song_mtx.lock();
    song_mtx_owner = 1;
    // cout<<"play song"<<endl;
    robot.sendPlaySongCommand(0);
    song_mtx_owner = 0;
    song_mtx.unlock();

    //Play different sounds accoording to distances to walls
    if(dist<50){
        this_thread::sleep_for(chrono::milliseconds(500));
    }
    else if (dist<110){
        this_thread::sleep_for(chrono::milliseconds(50));
    }
    else{
        this_thread::sleep_for(chrono::milliseconds(5));
    }
  }
}


int main()
{
    char serial_loc[] = "/dev/ttyUSB0";

    try
    {

        // Keep this
        //std::thread mythread = std::thread(thread_test);
        //cout << "mythread" << mythread.get_id() << endl;

        // Keep this
        // pthread_cancel(mythread.native_handle());

        raspicam::RaspiCam_Cv Camera;
        cv::Mat rgb_image, bgr_image;
        if (!Camera.open())
        {
            cerr << "Error opening the camera" << endl;
            return -1;
        }

        //Make sure various robot components are set up correctly
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

        robot.sendStreamCommand(sensors);
        cout << "Sent Stream Command" << endl;
        // Let's turn!
        int speed = 200;
        int speed_straight = 287;
        int speed_reverse = 165;
        int speed_rotate = 107;

        // Inches; check later
        int reverse_distance = 15;

        // Rotation hyperparameters
        int min_angle = 120;
        int max_angle = 240;

        //Send drive commands to the robot
        int ledColor = Create::LED_COLOR_GREEN;
        robot.sendDriveCommand(speed_straight, Create::DRIVE_STRAIGHT);
        robot.sendLedCommand(Create::LED_PLAY, 0, 0);
        cout << "Sent Drive Command" << endl;

        led_mtx.lock();
        song_mtx.lock();
        song_mtx_owner = 2;

        //Set up threads for LED and song routines
        led_handler = std::thread(led_handler_func, robot);
        song_handler = thread(song_handler_func, robot);

        // about play song
        Create::note_t note = make_pair(64, 10);
        Create::song_t song;
        song.push_back(note);
        robot.sendSongCommand(0, song);

        short wallSignal, prevWallSignal = 0;

        while (!robot.playButton())
        {
            if (robot.bumpLeft() || robot.bumpRight())
            {
                if(song_mtx_owner!=2){
                    song_mtx.lock();
                    song_mtx_owner = 2;
                }
                cout<<"stop play song"<<endl;
                cout << "Bump !" << endl;
                cout << "Beginning LED show" << endl;

                // Change to 15 inches
                robot.sendDriveCommand(-speed_reverse, Create::DRIVE_STRAIGHT);
                led_mtx.unlock();
                this_thread::sleep_for(chrono::milliseconds(2309));

                //Taking pictures during bumping
                Camera.grab();
                Camera.retrieve(bgr_image);
                cv::cvtColor(bgr_image, rgb_image, CV_RGB2BGR);
		            cv::imwrite("irobot_image.jpg", rgb_image);
                cout << "Taking photo" << endl;

                cout << "Ending LED show" << endl;
                led_mtx.lock();
                cout << "Lock again" << endl;
                robot.sendDriveCommand(speed_rotate, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);

                this_thread::sleep_for(chrono::milliseconds(3000));

                cout << "finish turning" << endl;
                robot.sendDriveCommand(speed_straight, Create::DRIVE_STRAIGHT);
                cout << "straight" << endl;
            }
            wallSignal = robot.wallSignal();
            dist = wallSignal;

            //Start playing songs when the robot gets close to walls
            if (wallSignal > 5)
            {
                if(song_mtx_owner == 2){
                    song_mtx_owner = 0;
                    song_mtx.unlock();
                    cout<<"start play song"<<endl;
                }
            }
            //The main thread should dominate the song mutex when the robot is far away from walls
            if (wallSignal == 0 && song_mtx_owner!=2)
            {
                song_mtx.lock();
                song_mtx_owner = 2;
            }
            prevWallSignal = wallSignal;

            //Handle situations when the advance button is pressed
            if (robot.advanceButton())
            {
                cout << "Advance button pressed" << endl;
                speed = -1 * speed;
                ledColor += 10;
                if (ledColor > 255)
                    ledColor = 0;

                robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_CLOCKWISE);
                if (speed < 0)
                    robot.sendLedCommand(Create::LED_PLAY,
                                         ledColor,
                                         Create::LED_INTENSITY_FULL);
                else
                    robot.sendLedCommand(Create::LED_ADVANCE,
                                         ledColor,
                                         Create::LED_INTENSITY_FULL);
            }

            // You can add more commands here.
            this_thread::sleep_for(chrono::milliseconds(200));
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
