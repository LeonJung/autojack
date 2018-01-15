#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>

using namespace cv ;

int main(int argc, char* argv[] ) {
  ros::init( argc, argv, "image_to_ground" ) ;

  int board_w = atoi(argv[1]);
  int board_h = atoi(argv[2]);

// //   ros::init(argc, argv, "key_to_vel"); 
//   ros::NodeHandle n;
// //   ros::Publisher pub_vel = n.advertise<key_to_vel_pkg::VelMsg>("k_v_vel", 1000); 
//   ros::Rate loop_rate(10);

//   while (ros::ok())
//   { 
//     // key_to_vel_pkg::VelMsg vel_msg;

//     // vel_msg.vel_linear = 1.012;
//     // vel_msg.vel_angular = 2.451;

//     ROS_INFO("Message");

//     // pub_vel.publish(vel_msg);

//     ros::spinOnce();
//     loop_rate.sleep();
//   }

//   return 0;

  cv::Size board_sz( board_w, board_h );
  cv::Mat image = cv::imread( "/home/leon/catkin_ws/src/autojack/turtlebot3_auto/turtlebot3_auto_camera/src/image.jpg" ) ;
  cv::Mat gray_image, tmp, H , birds_image;
  cv::Point2f objPts[4], imgPts[4] ;
  std::vector<cv::Point2f> corners ;
  float Z = 1 ; //have experimented from values as low as .1 and as high as 100
  int key = 0 ;
  int found = cv::findChessboardCorners( image, board_sz, corners ) ;

  ROS_INFO("ssssssss");

  if (found) {
    cv::drawChessboardCorners(image, board_sz, corners, 1) ;
    cvtColor( image, gray_image, CV_RGB2GRAY ) ;
    cornerSubPix( gray_image, corners, Size(11, 11), Size(-1, -1),
                  TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1) ) ;
    cv::resize( image, tmp, Size(), .5, .5 ) ;


    ROS_INFO("bbbbb");

    namedWindow( "IMAGE" ) ;
    cv::imshow( "IMAGE" , tmp ) ;
    waitKey(0) ;
  }


  ROS_INFO("aaaaaaaa");

    objPts[0].x = 0 ; 
    objPts[0].y = 0 ; objPts[1].x = board_w-1 ; 
    objPts[1].y = 0 ; 
    objPts[2].x = 0 ; objPts[2].y = board_h-1 ; 
    objPts[3].x = board_w-1 ; 
    objPts[3].y = board_h-1 ;

  imgPts[0] = corners.at(0) ;
  imgPts[1] = corners.at(board_w-1) ;
  imgPts[2] = corners.at((board_h-1) * board_w) ;
  imgPts[3] = corners.at((board_h-1) * board_w + board_w-1) ;
  
  ROS_INFO("ssssssss");

  H = cv::getPerspectiveTransform( objPts, imgPts ) ;

  birds_image = image ;



  while ( key != 27 ) {

    ROS_INFO("aaa");
    H.at<double>(2,2) = Z ;

    cv::warpPerspective( image, birds_image, H, Size( 2 * image.cols, 2 * tmp.rows ) ,
                         CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS ) ;

    cv::imshow( "IMAGE", birds_image ) ;
    cv::waitKey(0) ;
  }

  return 0;
}