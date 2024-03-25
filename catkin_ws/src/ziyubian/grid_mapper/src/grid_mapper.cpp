#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
//#include <algorithm>//needed to get a min or max elements among two element;
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <tf/transform_listener.h>


using namespace boost::posix_time;


class GridMapper {
public:
  // Construst a new occupancy grid mapper  object and hook up
  // this ROS node to the simulated robot's pose, velocity control,
  // and laser topics
  GridMapper(ros::NodeHandle& nh, int width, int height) :
      fsm(FSM_MOVE_FORWARD),
      rotateStartTime(ros::Time::now()),
      rotateDuration(0.f),
      canvas(height, width, CV_8UC1) {
    // Initialize random time generator
    srand(time(NULL));

    // Advertise a new publisher for the current simulated robot's
    // velocity command topic (the second argument indicates that
    // if multiple command messages are in the queue to be sent,
    // only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Subscribe to the current simulated robot's laser scan topic and
    // tell ROS to call this->laserCallback() whenever a new message
    // is published on that topic base_scan
    //modify the parameters
    laserSub = nh.subscribe("scan", 1, \
      &GridMapper::laserCallback, this);
    
    // Subscribe to the current simulated robot' ground truth pose topic
    // and tell ROS to call this->poseCallback(...) whenever a new
    // message is published on that topic "base_pose_ground_truth"
    poseSub = nh.subscribe("odom", 1, \
      &GridMapper::poseCallback, this);
      
    // Create resizeable named window
    cv::namedWindow("Occupancy Grid Canvas", \
      CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  };
  
  
  // Save a snapshot of the occupancy grid canvas
  // NOTE: image is saved to same folder where code was executed
  void saveSnapshot() {
    std::string filename = "grid_" + to_iso_string(second_clock::local_time()) + ".jpg";
    canvasMutex.lock();
    cv::imwrite(filename, canvas);
    canvasMutex.unlock();
  };
  
  
  // Update grayscale intensity on canvas pixel (x, y) (in robot coordinate frame)
  void plot(int x, int y, char value) {
    canvasMutex.lock();
    x+=canvas.rows/2;
    y+=canvas.cols/2;
    if (x >= 0 && x < canvas.rows && y >= 0 && y < canvas.cols) {
      canvas.at<char>(x, y) = value;
    }
    canvasMutex.unlock();
  };

  // Update grayscale intensity on canvas pixel (x, y) (in image coordinate frame)
  void plotImg(int x, int y, char value) {
    canvasMutex.lock();
    if (x >= 0 && x < canvas.cols && y >= 0 && y < canvas.rows) {
      canvas.at<char>(y, x) = value;
    }
    canvasMutex.unlock();
  };

  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  };


  // Process incoming laser scan message
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // TODO: parse laser data and update occupancy grid canvas
    //       (use CELL_OCCUPIED, CELL_UNKNOWN, CELL_FREE, and CELL_ROBOT values)
    // (see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html)
    //update occupancy grid canvas
    //ROS_INFO_STREAM("ROBOT_x1: " << x);
    //ROS_INFO_STREAM("ROBOT_y1: " << y);
    //plot(x,y,CELL_ROBOT);
    unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD -msg->angle_min) / msg->angle_increment);
    unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD -msg->angle_min) / msg->angle_increment);
    
    plot(x,y,CELL_ROBOT);
    if (fsm== FSM_MOVE_FORWARD){
        float closestRange = msg->ranges[minIndex];
        float currentRange;
        double x1;
        double y1;
        double xmin;
        double xmax;
        double ymin;
        double ymax;
        double angle = msg->angle_min;
        //ROS_INFO_STREAM("Range: " << closestRange);
        for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
          if (msg->ranges[currIndex] <closestRange) {
           closestRange = msg->ranges[currIndex];
          }
          ROS_INFO_STREAM("Range: " << msg->ranges[currIndex]);
          currentRange = msg->ranges[currIndex];
          angle += currIndex *(msg->angle_increment);
          x1 = x - currentRange * sin(angle/M_PI*180+heading/M_PI*180);
          y1 = y + currentRange * cos(angle/M_PI*180+heading/M_PI*180);
          //plot(x1,y1,CELL_OCCUPIED);
          if(x1<=x){
            xmin = x1;
            xmax = x;
          }else{
            xmin = x;
            xmax = x1;
          }
          
          if(y1<=y){
            ymin = y1;
            ymax = y;
          }else{
            ymin = y;
            ymax = y1;
          }

          for (int i = xmin; i < xmax; i++){
            for (int j = ymin; j < ymax; j++){
                plot(i,j,CELL_FREE);
            }
          }
          
          plot(x1,y1,CELL_OCCUPIED);
          

        }
        //ROS_INFO_STREAM("ROBOT_COORDINATE: " << x);
	//ROS_INFO_STREAM("ROBOT_COORDINATE: " << y);
        if(closestRange < PROXIMITY_RANGE_M){
          fsm = FSM_ROTATE;
          float a;
          a = rand() % 100;
          
          //a = 1.0;
          ros::Time rotateStartTime(ros::Time::now());
          ros::Duration rotateDuration(a);
      
        }
        
    }
    
    
    //ROS_INFO_STREAM("ROBOT_x2: " << x);
    //ROS_INFO_STREAM("ROBOT_y2: " << y);
    //ROS_INFO_STREAM("ROBOT_heading: " << heading/M_PI*180);
    //plot(x,y,CELL_ROBOT);
    
  };
  
  
  // Process incoming ground truth robot pose message
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    x = msg->pose.pose.position.y;
    y = msg->pose.pose.position.x;
    heading=tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO_STREAM("ROBOT_POSE: " << heading);
  };
  
  
  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    int key = 0;
    
    // Initialize all pixel values in canvas to CELL_UNKNOWN
    canvasMutex.lock();
    canvas = cv::Scalar(CELL_UNKNOWN);
    canvasMutex.unlock();
    
    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
      // TODO: remove following demo code and make robot move around the environment
      //plot(x, y, CELL_ROBOT); // Demo code: plot robot's current position on canvas
      //plotImg(0, 0, CELL_OCCUPIED); // Demo code: plot different colors at 4 canvas corners
      //plotImg(0, canvas.rows-1, CELL_UNKNOWN);
      //plotImg(canvas.cols-1, 0, CELL_FREE);
      //plotImg(canvas.cols-1, canvas.rows-1, CELL_ROBOT);
      //ROS_INFO_STREAM("Range: " << closestRange);
      //plot(x,y,CELL_OCCUPIED);
      //move(FORWARD_SPEED_MPS,0);
      if(fsm==FSM_MOVE_FORWARD){
        move(FORWARD_SPEED_MPS,0);
      }else{
        move(0, ROTATE_SPEED_RADPS);
      }
      
      if(ros::Time::now()>=rotateStartTime+rotateDuration&&fsm == FSM_ROTATE){
        fsm = FSM_MOVE_FORWARD;
      }
      
      // NOTE: DO NOT REMOVE CODE BELOW THIS LINE
      cv::imshow("Occupancy Grid Canvas", canvas);
      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      key = cv::waitKey(1000/SPIN_RATE_HZ); // Obtain keypress from user; wait at most N milliseconds
      if (key == 'x' || key == 'X') {
        break;
      } else if (key == ' ') {
        saveSnapshot();
      }
    }
    
    ros::shutdown(); // Ensure that this ROS node shuts down properly
  };
  
  enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE};

  // Tunable motion controller parameters
  constexpr static double FORWARD_SPEED_MPS = 1.0;
  constexpr static double ROTATE_SPEED_RADPS = M_PI/2;
  
  // add extra parameters of laserscan
  constexpr static double MIN_SCAN_ANGLE_RAD = -10.0/180*M_PI;
  constexpr static double MAX_SCAN_ANGLE_RAD = +10.0/180*M_PI;
  constexpr static float PROXIMITY_RANGE_M = 3.0;
  
  const static int SPIN_RATE_HZ = 30;
  
  const static char CELL_OCCUPIED = 0;
  const static char CELL_UNKNOWN = 86;
  const static char CELL_FREE = 172;
  const static char CELL_ROBOT = 255;


protected:
  ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the current robot's laser scan topic
  ros::Subscriber poseSub; // Subscriber to the current robot's ground truth pose topic
  
  // add extra parameters
  enum FSM fsm; // Finite state machine for the random walk algorithm
  ros::Time rotateStartTime; // Start time of the rotation
  ros::Duration rotateDuration;

  double x; // in simulated Stage units, + = East/right
  double y; // in simulated Stage units, + = North/up
  double heading; // in radians, 0 = East (+x dir.), pi/2 = North (+y dir.)
  
  cv::Mat canvas; // Occupancy grid canvas
  boost::mutex canvasMutex; // Mutex for occupancy grid canvas object
};


int main(int argc, char **argv) {
  int width, height;
  bool printUsage = false;
  
  // Parse and validate input arguments
  if (argc <= 2) {
    printUsage = true;
  } else {
    try {
      width = boost::lexical_cast<int>(argv[1]);
      height = boost::lexical_cast<int>(argv[2]);

      if (width <= 0) { printUsage = true; }
      else if (height <= 0) { printUsage = true; }
    } catch (std::exception err) {
      printUsage = true;
    }
  }
  if (printUsage) {
    std::cout << "Usage: " << argv[0] << " [CANVAS_WIDTH] [CANVAS_HEIGHT]" << std::endl;
    return EXIT_FAILURE;
  }
  
  ros::init(argc, argv, "grid_mapper"); // Initiate ROS node
  ros::NodeHandle n; // Create default handle
  GridMapper robbie(n, width, height); // Create new grid mapper object
  robbie.spin(); // Execute FSM loop

  return EXIT_SUCCESS;
};
