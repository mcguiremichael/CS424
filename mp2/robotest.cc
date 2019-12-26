
#include <chrono>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <SerialStream.h>
#include "irobot-create.hh"
#include <ctime>
#include <thread>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <mutex>
#include <semaphore.h>
#include <dirent.h>
#include <vector>
#include <string>
#include <sys/types.h>
#include <errno.h>

using namespace cv;
using namespace cv::xfeatures2d;

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::chrono::duration;
using std::chrono::steady_clock;

using namespace iRobot;
using namespace LibSerial;
using namespace std;

mutex cam_mtx;

vector<Mat> captured_images;
vector<Mat> query_image;

bool alignPerspective(vector<Point2f> &query, vector<Point2f> &scene,
                      Mat &img_query, Mat &img_scene, vector<Point2f> &scene_corners);
void cropBottom(Mat &img_scene_full, Mat &img_scene, float crop_fraction);
void drawProjection(Mat &img_matches, Mat &img_query,
                    vector<Point2f> &scene_corners);
string type2str(int type);
bool object_indetification(Mat img_query, Mat img_scene_full, float keep_top_fraction);
void getFiles(string dir, vector<Mat> &files)
{
    files.clear();
    DIR *dp;
    struct dirent *dirp;
    if ((dp = opendir(dir.c_str())) == NULL)
    {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return;
    }

    while ((dirp = readdir(dp)) != NULL)
    {
        string file_name(string(dirp->d_name));
        string end = ".jpg";
        if (file_name.find(end) != string::npos)
        {
            Mat imgfile = imread(dir + file_name, IMREAD_GRAYSCALE);
            files.push_back(imgfile);
        }
        //files.push_back(string(dirp->d_name));
    }
    closedir(dp);
    return;
}

void Camera_Capture()
{ // run in thread
    raspicam::RaspiCam_Cv Camera;
    if (!Camera.open())
    {
        cerr << "Error opening the camera" << endl;
        return;
    }
    string sname = "irobot_capture_";
    string ext = ".jpg";
    int counter = 0;
    while (1)
    {
        cout << "Camera_Capture running" << endl;
        cam_mtx.lock();
        cv::Mat img_capture, img_scene;
        Camera.grab();
        Camera.retrieve(img_capture);
        cvtColor(img_capture, img_scene, CV_BGR2GRAY);
        string name_s = sname + to_string(counter) + ext;
        captured_images.push_back(img_scene);
        // cv::imwrite(name_s,img_scene );
        //cout<<"Storing img"<<endl;         //query_image.erase(query_image.begin());
        counter++;
        cout << "Num of " << captured_images.size() << endl;
        cam_mtx.unlock();
        this_thread::sleep_for(chrono::milliseconds(1000));
    }
}

void Identification(Mat captured_images, int &counter)
{ // run in thread
    // Find a way to stop this after we finish.

    //int counter = 0;
    string s = "irobot_stored_";
    string sname = "irobot_matched_";
    string ext = ".jpg";

    //  data_lock.unlock();
    for (int i = 0; i < query_image.size(); i++)
    {
        bool res = object_indetification(query_image[i], captured_images, 0.85);
        if (res)
        {

            string name = s + to_string(counter) + ext;
            string name_s = sname + to_string(counter) + ext;
            cout << "Object Matched !!, Storing as " << name << endl;
            cv::imwrite(name, captured_images);
            cv::imwrite(name_s, query_image[i]);
            //query_image.erase(query_image.begin());
            counter++;
            break;
        }
        else
        {
            cout << "Image not matched !!" << endl;
        }
    }
    //cout<<"Identification done"<<endl;
    return;
}

void Group_Identification()
{ // run in thread
    // Find a way to stop this after we finish.
    cout << "Identification Run" << endl;
    getFiles("object_identification/query-image/low-resolution/", query_image);
    int counter = 0;
    string s = "irobot_stored_";
    string ext = ".jpg";

    for (int i = 0; i < captured_images.size(); i++)
    {
        //data_lock.lock();
        int N = captured_images.size();
        Mat img_scene = captured_images[i];
        cout << "Read image .." << N - i << "  !!" << endl;
        //captured_imgages.erase(captured_imgages.begin());
        //data_lock.unlock();
        Identification(img_scene, counter);
    }
    cout << "Object Identification Finished!!!" << endl;
}

std::thread cam_handler;

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
        cv::Point p = cv::Point(pos[0] + (w / 2), (w / 2 - pos[1]));
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
    /* Added to give the contour the right orientation  */
    //add_angle(0.0);
    try
    {
        //raspicam::RaspiCam_Cv Camera;
        cv::Mat rgb_image, bgr_image;
        /*if (!Camera.open())
        {
            cerr << "Error opening the camera" << endl;
            return -1;
        }*/

        //cam_mtx.unlock();
        thread cam_thread(Camera_Capture);
        //cam_thread.detach();
        // cout << "Opened Camera" << endl;
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
        robot_safe(robot);
        while (!robot.bumpLeft() && !robot.bumpRight())
        {
            this_thread::sleep_for(chrono::microseconds(100));
        //     if (!robot_safe(robot))
        //     {
        //         break;
        //     }
        // 
        }

        robot.sendDriveCommand(-speed, Create::DRIVE_STRAIGHT);
        this_thread::sleep_for(chrono::milliseconds(200));

        robot.sendDriveCommand(speed, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
        int prev = 0;
        int prev1 = 0;
        auto start = chrono::steady_clock::now();

        while (robot.wallSignal() <= 90 || robot.wallSignal() >= prev)
        {
            if (!robot_safe(robot))
            {
                break;
            }
            prev = robot.wallSignal();
            this_thread::sleep_for(chrono::microseconds(100));
            double duration = (double)chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
            if (duration > 3000)
            {
                break;
            }
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
        start = chrono::steady_clock::now();
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

        double duration = (double)chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        add_distance(20, (double)duration / 1000.0);
        cout << "Play button pressed, stopping Robot" << endl;
        robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
        add_angle(0.0);
        sem_post(&plot_sem);
        cam_mtx.lock();
        cout << "Camera Stopped !!" << endl;
        Group_Identification();
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

bool object_indetification(Mat img_query, Mat img_scene_full, float keep_top_fraction)
{

    if (!img_query.data || !img_scene_full.data)
    {
        cout << "Error reading images" << endl;
        return -1;
    }

    Mat img_scene;
    // Crop bottom
    // Images taken by mounting the camera on the robot will have some portion
    // of the side of the robot at the bottom. To reduce ambiguity during
    // detection and to speed up feature extraction, we crop it.
    // The fraction of cropping will be different depending on where the camera
    // is mounted on the robot. We find the useful portion of the picture is
    // the top 62.5% when camera mounted on front. When camera mounted on the
    // left side its the top 85% that contains useful information.
    cropBottom(img_scene_full, img_scene, keep_top_fraction);

    // Detect the keypoints and extract descriptors using SURF
    // Surf keypoint detector and descriptor.
    int minHessian = 100;
    int nOctaves = 4;
    int nOctaveLayers = 3;
    Ptr<SURF> detector = SURF::create(
        minHessian, nOctaves, nOctaveLayers, true);

    vector<KeyPoint> keypoints_query, keypoints_scene;
    Mat descriptors_query, descriptors_scene;

    auto sttime = steady_clock::now();
    detector->detectAndCompute(
        img_query, Mat(), keypoints_query, descriptors_query);
    cout << "Feature extraction query image "
         << (duration<double>(steady_clock::now() - sttime)).count()
         << " sec" << endl;

    sttime = steady_clock::now();
    detector->detectAndCompute(
        img_scene, Mat(), keypoints_scene, descriptors_scene);
    cout << "Feature extraction scene image "
         << (duration<double>(steady_clock::now() - sttime)).count()
         << " sec" << endl;
    sttime = steady_clock::now();

    // Matching descriptor vectors using Brute Force matcher
    BFMatcher matcher(NORM_L2);
    vector<vector<DMatch>> matches;
    matcher.knnMatch(descriptors_query, descriptors_scene, matches, 2);

    vector<DMatch> good_matches;
    for (int i = 0; i < descriptors_query.rows; i++)
    {
        if (matches[i][0].distance < 0.75 * matches[i][1].distance)
            good_matches.push_back(matches[i][0]);
    }

    // Find the location of the query in the scene
    vector<Point2f> query;
    vector<Point2f> scene;
    for (size_t i = 0; i < good_matches.size(); i++)
    {
        query.push_back(keypoints_query[good_matches[i].queryIdx].pt);
        scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
    }

    vector<Point2f> scene_corners(4);
    bool res = alignPerspective(
        query, scene, img_query, img_scene, scene_corners);

    if (res)
    {
        //store the image
    }

    return res;
    /*cout << "Matching and alignment "
      << (duration <double>(steady_clock::now() - sttime)).count()
      << " sec" << endl;

    // Write output to file
    Mat img_matches;
    drawMatches(img_query, keypoints_query, img_scene, keypoints_scene,
        good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
        vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    // Fill the extra area in almost white (Saves ink when printing)
    if (img_query.rows < img_scene.rows) {
      rectangle(img_matches, Point2f(0, img_query.rows),
          Point2f(img_query.cols - 1, img_scene.rows - 1),
          Scalar(255, 240, 240), CV_FILLED);
    } else if (img_scene.rows < img_query.rows) {
      rectangle(img_matches, Point2f(img_query.cols, img_scene.rows),
          Point2f(img_query.cols + img_scene.cols - 1, img_query.rows - 1),
          Scalar(255, 240, 240), CV_FILLED);
    }
    if (res) {
      cout << "Object found" << endl;
      drawProjection(img_matches, img_query, scene_corners);
    } else {
      cout << "Object not found" << endl;
    }
    // Write result to a file
    cv::imwrite(output_file, img_matches);*/
}
// return 0;

void cropBottom(Mat &img_scene_full, Mat &img_scene, float crop_fraction)
{
    // Crop the lower part of the scene
    cv::Rect crop;
    crop.x = 0;
    crop.y = 0;
    crop.width = img_scene_full.size().width;
    crop.height = img_scene_full.size().height * crop_fraction;
    img_scene = img_scene_full(crop);
}

bool alignPerspective(vector<Point2f> &query, vector<Point2f> &scene,
                      Mat &img_query, Mat &img_scene, vector<Point2f> &scene_corners)
{
    Mat H = findHomography(query, scene, RANSAC);
    if (H.rows == 0 && H.cols == 0)
    {
        cout << "Failed rule0: Empty homography" << endl;
        return false;
    }

    vector<Point2f> query_corners(4);
    query_corners[0] = cvPoint(0, 0);
    query_corners[1] = cvPoint(img_query.cols, 0);
    query_corners[2] = cvPoint(img_query.cols, img_query.rows);
    query_corners[3] = cvPoint(0, img_query.rows);

    perspectiveTransform(query_corners, scene_corners, H);

    float min_area = 32.0 * 32.0;
    double max_area = img_scene.rows * img_scene.cols;
    float ratio_inside = 0.75;
    float min_angle_sin = 0.173; // Minimum 10 degree angle required

    // Geometric verification heuristics
    // Rule 1: Must be a convex hull.
    // Rule 2: Area can’t be less than 32x32
    // Rule 3: The detected projection can’t have more than 100% area
    // Rule 4: Projection can't contain very small angle < 10 degree
    // Rule 5: More than 75% of the area of the detected projection should have
    // to be within image bounds

    // Rule 1: Must be a convex hull.
    vector<Point2f> sc_vec(4);
    // Generate 4 vectors from the 4 scene corners
    for (int i = 0; i < 4; i++)
    {
        sc_vec[i] = scene_corners[(i + 1) % 4] - scene_corners[i];
    }
    vector<float> sc_cross(4);
    // Calculate cross product of pairwise vectors
    for (int i = 0; i < 4; i++)
    {
        sc_cross[i] = sc_vec[i].cross(sc_vec[(i + 1) % 4]);
    }

    // Check for convex hull
    if (!(sc_cross[0] < 0 && sc_cross[1] < 0 && sc_cross[2] < 0 && sc_cross[3] < 0) && !(sc_cross[0] > 0 && sc_cross[1] > 0 && sc_cross[2] > 0 && sc_cross[3] > 0))
    {
        cout << "Failed rule1: Not a convex hull" << endl;
        return false;
    }

    // Rule 2: Area can’t be less than 32x32
    // Rule 3: The detected projection can’t have more than 100% area
    float area = (sc_cross[0] + sc_cross[2]) / 2.0;
    if (fabs(area) < min_area)
    {
        cout << "Failed rule2: Projection too small" << endl;
        return false;
    }
    else if (fabs(area) > max_area)
    {
        cout << "Failed rule3: Projection too large" << endl;
        return false;
    }

    // Rule 4: Can't contain very small angle < 10 degree inside projection.
    // Check for angles
    vector<float> sc_norm(4);
    for (int i = 0; i < 4; i++)
    {
        sc_norm[i] = norm(sc_vec[i]);
    }
    for (int i = 0; i < 4; i++)
    {
        float sint = sc_cross[i] / (sc_norm[i] * sc_norm[(i + 1) % 4]);
        if (fabs(sint) < min_angle_sin)
        {
            cout << "Failed rule4: Contains very small angle" << endl;
            return false;
        }
    }

    // Rule 5: More than 75% of the area of the detected projection should
    // have to be within image bounds.
    // Approximate mechanism by determining the bounding rectangle.
    cv::Rect bound = boundingRect(scene_corners);
    cv::Rect scene_rect(0.0, 0.0, img_scene.cols, img_scene.rows);
    cv::Rect isect = bound & scene_rect;
    if (isect.width * isect.height < ratio_inside * bound.width * bound.height)
    {
        cout << "Failed rule5: Large proportion outside scene" << endl;
        return false;
    }
    return true;
}

// Show the projection
void drawProjection(Mat &img_matches, Mat &img_query,
                    vector<Point2f> &scene_corners)
{
    line(img_matches, scene_corners[0] + Point2f(img_query.cols, 0),
         scene_corners[1] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
    line(img_matches, scene_corners[1] + Point2f(img_query.cols, 0),
         scene_corners[2] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
    line(img_matches, scene_corners[2] + Point2f(img_query.cols, 0),
         scene_corners[3] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
    line(img_matches, scene_corners[3] + Point2f(img_query.cols, 0),
         scene_corners[0] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
}

string type2str(int type)
{
    std::string r;
    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth)
    {
    case CV_8U:
        r = "8U";
        break;
    case CV_8S:
        r = "8S";
        break;
    case CV_16U:
        r = "16U";
        break;
    case CV_16S:
        r = "16S";
        break;
    case CV_32S:
        r = "32S";
        break;
    case CV_32F:
        r = "32F";
        break;
    case CV_64F:
        r = "64F";
        break;
    default:
        r = "User";
        break;
    }

    r += "C";
    r += (chans + '0');
    return r;
}
