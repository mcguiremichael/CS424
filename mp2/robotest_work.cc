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

using std::chrono::duration;
using std::chrono::steady_clock;
using std::cout;
using std::endl;
using std::string;
using std::vector;

using namespace iRobot;
using namespace LibSerial;
using namespace std;

vector<Mat> query_image;
bool alignPerspective(vector<Point2f>& query, vector<Point2f>& scene,
    Mat& img_query, Mat& img_scene, vector<Point2f>& scene_corners);
void cropBottom(Mat& img_scene_full, Mat& img_scene, float crop_fraction);
void drawProjection(Mat& img_matches, Mat& img_query,
    vector<Point2f>& scene_corners);
string type2str(int type);
void usage();
bool object_indetification(Mat img_query, Mat img_scene_full, float keep_top_fraction);
void getFiles (string dir, vector<Mat> &files)
{
    files.clear();
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return ;
    }

    while ((dirp = readdir(dp)) != NULL) {
        string file_name(string(dirp->d_name));
        string end = ".jpg";
        if(file_name.find(end) != string::npos){
        Mat imgfile = imread(dir+file_name,IMREAD_GRAYSCALE);
        files.push_back(imgfile);
    }
       //files.push_back(string(dirp->d_name));
    }
    closedir(dp);
    return ;
}

void init_Image(){
    getFiles("object_identification/query-image/",query_image);

}



int main()
{
    //char serial_loc[] = "/dev/ttyUSB0";

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
        cv::Mat img_capture,img_scene;
         Camera.grab();
         Camera.retrieve(img_capture);
         cvtColor( img_capture, img_scene, CV_BGR2GRAY );
         cv::imwrite("irobot_capture.jpg",img_scene );
         getFiles("object_identification/query-image/",query_image);
         for(int i = 0; i<query_image.size(); i++){
              bool res = object_indetification(query_image[i],img_scene,0.85);
              if(res){
                  cout<<"Object Matched !!"<<endl;
                  cv::imwrite("irobot_image.jpg",img_scene );
                  //delete the mathced element from the query_image list
              }
              else {
                cout << "Image not matched !!"<<endl;
              }
         }
        
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

bool object_indetification(Mat img_query, Mat img_scene_full, float keep_top_fraction) {
  
    if(!img_query.data || !img_scene_full.data) {
      cout<< "Error reading images" << endl;
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
      << (duration <double>(steady_clock::now() - sttime)).count()
      << " sec" << endl;

    sttime = steady_clock::now();
    detector->detectAndCompute(
        img_scene, Mat(), keypoints_scene, descriptors_scene);
    cout << "Feature extraction scene image "
      << (duration <double>(steady_clock::now() - sttime)).count()
      << " sec" << endl;
    sttime = steady_clock::now();

    // Matching descriptor vectors using Brute Force matcher
    BFMatcher matcher(NORM_L2);
    vector<vector<DMatch>> matches;
    matcher.knnMatch(descriptors_query, descriptors_scene, matches, 2);

    vector<DMatch> good_matches;
    for(int i = 0; i < descriptors_query.rows; i++) {
      if (matches[i][0].distance < 0.75 * matches[i][1].distance)
        good_matches.push_back(matches[i][0]);
    }

    // Find the location of the query in the scene
    vector<Point2f> query;
    vector<Point2f> scene;
    for(size_t i = 0; i < good_matches.size(); i++) {
      query.push_back(keypoints_query[good_matches[i].queryIdx].pt);
      scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
    }

    vector<Point2f> scene_corners(4);
    bool res = alignPerspective(
        query, scene, img_query, img_scene, scene_corners);

    if(res){
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






void cropBottom(Mat& img_scene_full, Mat& img_scene, float crop_fraction) {
  // Crop the lower part of the scene
  cv::Rect crop;
  crop.x = 0;
  crop.y = 0;
  crop.width = img_scene_full.size().width;
  crop.height = img_scene_full.size().height * crop_fraction;
  img_scene = img_scene_full(crop);
}


bool alignPerspective(vector<Point2f>& query, vector<Point2f>& scene,
    Mat& img_query, Mat& img_scene, vector<Point2f>& scene_corners) {
  Mat H = findHomography(query, scene, RANSAC);
  if (H.rows == 0 && H.cols == 0) {
    cout << "Failed rule0: Empty homography" << endl;
    return false;
  }

  vector<Point2f> query_corners(4);
  query_corners[0] = cvPoint(0,0);
  query_corners[1] = cvPoint(img_query.cols, 0);
  query_corners[2] = cvPoint(img_query.cols, img_query.rows);
  query_corners[3] = cvPoint(0, img_query.rows );

  perspectiveTransform(query_corners, scene_corners, H);

  float min_area = 32.0 * 32.0;
  double max_area = img_scene.rows * img_scene.cols;
  float ratio_inside = 0.75;
  float min_angle_sin =  0.173; // Minimum 10 degree angle required

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
  for(int i = 0; i < 4; i++) {
    sc_vec[i] = scene_corners[(i + 1) % 4] - scene_corners[i];
  }
  vector<float> sc_cross(4);
  // Calculate cross product of pairwise vectors
  for(int i = 0; i < 4; i++) {
    sc_cross[i] = sc_vec[i].cross(sc_vec[(i+1) % 4]);
  }

  // Check for convex hull
  if (!(sc_cross[0] < 0 && sc_cross[1] < 0 && sc_cross[2] < 0 && sc_cross[3] < 0)
      && !(sc_cross[0] > 0 && sc_cross[1] > 0 && sc_cross[2] > 0 && sc_cross[3] > 0)) {
    cout << "Failed rule1: Not a convex hull" << endl;
    return false;
  }

  // Rule 2: Area can’t be less than 32x32
  // Rule 3: The detected projection can’t have more than 100% area
  float area = (sc_cross[0] + sc_cross[2]) / 2.0;
  if (fabs(area) < min_area) {
    cout << "Failed rule2: Projection too small" << endl;
    return false;
  } else if (fabs(area) > max_area) {
    cout << "Failed rule3: Projection too large" << endl;
    return false;
  }

  // Rule 4: Can't contain very small angle < 10 degree inside projection.
  // Check for angles
  vector<float> sc_norm(4);
  for (int i = 0; i < 4; i++) {
    sc_norm[i] = norm(sc_vec[i]);
  }
  for (int i = 0; i < 4; i++) {
    float sint = sc_cross[i] / (sc_norm[i] * sc_norm[(i + 1) % 4]);
    if (fabs(sint) < min_angle_sin) {
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
  if (isect.width * isect.height <  ratio_inside * bound.width * bound.height ) {
    cout << "Failed rule5: Large proportion outside scene" << endl;
    return false;
  }
  return true;
}

// Show the projection
void drawProjection(Mat& img_matches, Mat& img_query,
    vector<Point2f>& scene_corners) {
  line(img_matches, scene_corners[0] + Point2f(img_query.cols, 0),
      scene_corners[1] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
  line(img_matches, scene_corners[1] + Point2f(img_query.cols, 0),
      scene_corners[2] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
  line(img_matches, scene_corners[2] + Point2f(img_query.cols, 0),
      scene_corners[3] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
  line(img_matches, scene_corners[3] + Point2f(img_query.cols, 0),
      scene_corners[0] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
}

string type2str(int type) {
  std::string r;
  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans + '0');
  return r;
}

void usage() {
  cout << " Usage: ./robovision <query-image>  <crop-bottom>" << endl;
}

