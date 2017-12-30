#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/photo.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "math.h"

class ImageConverter
{
  ros::NodeHandle n;
  image_transport::ImageTransport it;
  image_transport::Subscriber rgb_sub;
  image_transport::Subscriber depth_sub;
  ros::Publisher turn_pub = n.advertise<std_msgs::Float32>("/target_turner", 1);
  ros::Publisher dist_pub = n.advertise<std_msgs::Float32>("/object_distance", 2);
  ros::Publisher color_pub = n.advertise<std_msgs::String>("/object_color", 2);
  ros::Publisher angle_pub = n.advertise<std_msgs::Float32>("/object_angle", 2);

  public:
  ImageConverter()
    : it(n)
  {
    depth_sub = it.subscribe("/camera/depth/image_raw", 1, &ImageConverter::depthCb, this);
    rgb_sub = it.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);
  }

  // Keeping track of the objects location
  unsigned int xR = 0;
  unsigned int yR = 0;

  unsigned int xY = 0;
  unsigned int yY = 0;

  // Tracking location of the pixel a certain color was last seen
  // Ending with R is for red, while Y is for yellow
  int lastXR = -1;
  int lastYR = -1;
  int lastXY = -1;
  int lastYY = -1;

  //These are for keeping track of which colors are seen
  bool red = false;
  bool yellow = false;

  //To see later on, which color to prioritize
  bool redTarget = false;
  bool yellowTarget = false;

  //These are for finding out whether the program should still track a color
  //When true, it will no longer send the location of the color
  bool redDone = false;
  bool yellowDone = false;

  //Mat-variables are being declared for use
  cv::Mat image, imageLines, HSVImage, ThreshImageR, ThreshImageY, depthImage;

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //A cv_pointer is created, and it's made to point to the new image
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    //The image from the cv_pointer is put into image
    image = cv_ptr->image;

    //The color is converted from BGR to HSV
    cvtColor(image, HSVImage, CV_BGR2HSV);

    //The pixels with values between scalar1 and scalar2 is set to 255 (white), others are set to 0 (black)
    //Red
    //inRange(HSVImage, cv::Scalar(150, 140, 100), cv::Scalar(200, 200, 255), ThreshImageR);

    //Yellow
    inRange(HSVImage, cv::Scalar(15, 100, 120), cv::Scalar(50, 255, 255), ThreshImageY);

    // For seeing a red box in gazebo
    inRange(HSVImage, cv::Scalar(0, 220, 80), cv::Scalar(10, 255, 255), ThreshImageR);

    //This variable is set to have the same size as image for later use
    imageLines = cv::Mat::zeros(image.size(), CV_8UC3);

    //Moments from red threshImage are found and extracted into double variables
    cv::Moments MomentsR = moments(ThreshImageR);
    double m10R = MomentsR.m10;
    double m01R = MomentsR.m01;
    double AreaR = MomentsR.m00;

    //red is set to false in order to avoid an ealier positive
    red = false;

    //If not enough pixels has been seen, the if-statement won't run
    if(AreaR > 30000){
      //x- and y-values are found fro moments
      xR = m10R / AreaR;
      yR = m01R / AreaR;

      if(xR >= 0 && yR >= 0){
        bottom('r');

        //A blue line is written to x and y from earlier positive
        cv::line(imageLines, cv::Point(xR, yR), cv::Point(lastXR, lastYR), cv::Scalar(255, 0, 0), 4);
      }

      //Saving location of positive
      lastXR = xR;
      lastYR = yR;
      //Setting red to true, since it has been seen
      red = true;
    }

    //Below code is the same as for red, but with yellow
    cv::Moments MomentsY = moments(ThreshImageY);
    double m10Y = MomentsY.m10;
    double m01Y = MomentsY.m01;
    double AreaY = MomentsY.m00;
    yellow = false;

    if(AreaY > 100000){
      xY = m10Y / AreaY;
      yY = m01Y / AreaY;

      if(xY >= 0 && yY >= 0){
        bottom('y');
        cv::line(imageLines, cv::Point(xY, yY), cv::Point(lastXY, lastYY), cv::Scalar(0, 0, 255), 2);
      }

      lastXY = xY;
      lastYY = yY;
      yellow = true;
    }

    //A constant has been found for focal point of camera
    float Fc = 577.295;
    //Float to hold value for publishing how much to turn in order to see object
    std_msgs::Float32 turn;
    turn.data = 10;

    //If red has been seen, and hasn't been measured before, it will now
    //otherwise, the same will be seen for yellow
    if(red && !redDone){
      redTarget = true;
      turn.data = atan(lastXR / Fc) * 180 / 3.14591 - 29;
      turn_pub.publish(turn);
    } else if(yellow && !yellowDone){
      yellowTarget = true;
      turn.data = atan(lastXY / Fc) * 180 / 3.14591 - 29;
      turn_pub.publish(turn);
    }

    //If the robot is looking at the object to measure, and it has seen a color, it will call depth image
    if(turn.data > -1 && turn.data < 1)
      pubInfo();

    //The lines are added to the image
    image += imageLines;

    //Reset
    redTarget = false;
    yellowTarget = false;

    //For viewing on screen
    cv::imshow("Image", image);
    // cv::imshow("Y", ThreshImageY);
    // cv::imshow("R", ThreshImageR);

    cv::waitKey(3);
  }

  void depthCb(const sensor_msgs::ImageConstPtr &msg)
  {
    //Same story as before, pointer and image
    cv_bridge::CvImagePtr depth_ptr;
    depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    depthImage = depth_ptr->image;
  }

  void pubInfo()
  {
    //Variables for publishing are declared
    std_msgs::Float32 dist;
    std_msgs::String color;
    std_msgs::Float32 angle;

    float Fc = 430.222;
    float hyp = 0;

    //If red is the target
    if(redTarget){
      // The hypotenuse for the triangle going from:
      // robot_floor_position -> robot -> object -> robot_floor_position is being found
      hyp = depthImage.at<float>(depthImage.cols / 2, lastYR);
      // Angle between hyp and robot_floor_position -> robot is being found
      angle.data = atan(lastXY / Fc) * 180 / 3.14591 - 22.5;
      //dist is found using sin, hyp and angle
      dist.data = sin(angle.data) * hyp;
      //color is getting the color red
      color.data = "red";
      // red will no longer be looked for
      redDone = true;
      //The variables are published
      dist_pub.publish(dist);
      color_pub.publish(color);
      angle_pub.publish(angle);
    }

    //Same story as with red
    if(yellowTarget){
      hyp = depthImage.at<float>(depthImage.cols / 2, lastYY);
      angle.data = atan(lastXY / Fc) * 180 / 3.14591 - 22.5;
      dist.data = sin(angle.data) * hyp;
      color.data = "yellow";
      yellowDone = true;
      dist_pub.publish(dist);
      color_pub.publish(color);
      angle_pub.publish(angle);
    }
  }

  void bottom(unsigned char colorb){
    cv::Mat inp;

    // Creating variables to store input
    unsigned int inpX = 0;
    unsigned int inpY = 0;

    // Deciding whether the stored inut should be from yellow or red
    if(colorb == 'r'){
      inp = ThreshImageR;
      inpX = xR;
      inpY = yR;
    } else {
      inp = ThreshImageY;
      inpX = xY;
      inpY = yY;
    }

    // Creating a variabke for lower threshold of required found pixels in order to count object as seen
    unsigned int threshold = 0;

    for(unsigned int ix = 0; ix < 640; ix++)
      if(inp.at<unsigned char>(cv::Point(ix, inpY)) != 0)
        threshold++;

    // Lowering threshold a little due to noise when seeing object
    threshold *= 0.95;

    // Creating variables for the changed x,y coordinates
    unsigned int botX = inpX;
    unsigned int botY = inpY;

    // Creating a counter to count non-zero pixels
    unsigned int x_counter = 0;

    // Creating variables to store the moments of x and y
    unsigned int moment_x = 0;
    unsigned int moment_y = 0;

    // Variable for keeping track of layers with acceptable color underneath, so that botY can move down
    unsigned int diff_count = 0;

    for(unsigned int iy = inpY; iy > 0; iy++){
      x_counter = 0;
      moment_x = 0;
      moment_y = 0;

      for(unsigned int ix = 0; ix < 640; ix++){
        if(inp.at<unsigned char>(cv::Point(ix, iy)) != 0){
          x_counter++;
          moment_x += ix;
        }
      }

      // Checking whether there is insufficient non-zero pixels on row
      if(x_counter < threshold)
        break;

      // Calculating moment_x for current row
      botX = moment_x / x_counter;

      diff_count++;

      if(diff_count > 3){
        botY++;
        diff_count--;
      }
    }

    if(colorb == 'r'){
      xR = botX;
      yR = botY;
    } else {
      xY = botX;
      yY = botY;
    }
  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "read");

  ImageConverter ic;

  ros::spin();
  return 0;
}
