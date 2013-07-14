/*#include "ros/ros.h"
#include <stdio.h>

using namespace std;


bool getObjectsLoc(nao_msgs::ObjectLocations::Request  &req,
         nao_msgs::ObjectLocations::Response &res){
	return true;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_object_locs");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("ObjectLocations", getObjectsLoc);
  ROS_INFO("Retrieving information from vision");
  ros::spin();

  return 0;
}
*/


#include <ros/ros.h>
#include <sstream>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <iostream>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <visualization_msgs/Marker.h>
#include "nao_world_msgs/ObjectLocations.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/lexical_cast.hpp>
#include <math.h> 
#include <nao_world_msgs/GridCoordinate.h>

using namespace std;

//ros::Publisher marker_pub;
ros::Publisher pub;
//ros::Publisher pub_ball;
struct coordinate
{
      float x;
      float y;
      float z;
};

void calculateGridCellID(std::vector<float> & x_locs, std::vector<float> & y_locs)
{
    for (int i=0; (int) i<x_locs.size(); i++) {
        
    }
}

bool
color_detection (nao_world_msgs::ObjectLocations::Request  &req,
         nao_world_msgs::ObjectLocations::Response &res)//const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud_p;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl_unfiltered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZHSV> cloud_pcl_hsv;
 // sensor_msgs::PointCloud2 output;
 // sensor_msgs::PointCloud2 output_ball;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl_boxes (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl_balls (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(req.cloud,*cloud_pcl_unfiltered);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm=0.01f
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (cloud_pcl_unfiltered);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_pcl);

  pcl::PointCloudXYZRGBtoXYZHSV (*cloud_pcl, cloud_pcl_hsv);


  //uint32_t shape = visualization_msgs::Marker::ARROW;
  //visualization_msgs::Marker box_marker[2],ball_marker[2];
  for(size_t i = 0; i < cloud_pcl_hsv.size(); i++)
  {  //                             12                              253                            0.75
      if(((cloud_pcl_hsv.at(i).h < 12) || (cloud_pcl_hsv.at(i).h > 253) )  && (cloud_pcl_hsv.at(i).s > 0.75) && (cloud_pcl_hsv.at(i).v > 30))
      {
          //std::cout<<"Punkt"<<std::endl;
          cloud_pcl_boxes->points.push_back(cloud_pcl->points[i]);
      }
      if(((cloud_pcl_hsv.at(i).h < 90) && (cloud_pcl_hsv.at(i).h > 50) )  && (cloud_pcl_hsv.at(i).s > 0.5) && (cloud_pcl_hsv.at(i).s < 1) && (cloud_pcl_hsv.at(i).v > 50))
      {
          cloud_pcl_balls->points.push_back(cloud_pcl->points[i]);
      }
  }

  //Clustering Boxes
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

  ec.setClusterTolerance (0.05); //0.01 kisten getrennt
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (3500); //3500
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_pcl_boxes);
  ec.extract (cluster_indices);
  std::vector<pcl::PointIndices>::const_iterator it;

  std::vector<int>::const_iterator pit;
  //uint8_t r = 255, g = 255, b = 255;

  //uint32_t rgb;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > cluster_box;
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_boxes (new pcl::PointCloud<pcl::PointXYZRGB>);
  for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {

      //rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl_box (new pcl::PointCloud<pcl::PointXYZRGB>);
          for(pit = it->indices.begin(); pit != it->indices.end(); pit++)
          {
          //push_back: add a point to the end of the existing vector
                  cloud_pcl_box->points.push_back(cloud_pcl_boxes->points[*pit]);

                  //cluster_boxes->points.push_back(cloud_pcl_boxes->points[*pit]);
                  //cluster_boxes->back().rgb=*reinterpret_cast<float*>(&rgb);
          }

          //Merge current clusters to whole point cloud
          cluster_box.push_back(cloud_pcl_box);



  }

  //Clustering Balls
  std::vector<pcl::PointIndices> cluster_indices_balls;
  ec.setClusterTolerance (0.02); //0.01 kisten getrennt
  ec.setMinClusterSize (30);
  ec.setMaxClusterSize (1000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_pcl_balls);
  ec.extract (cluster_indices_balls);


  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > cluster_ball;
 // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_balls (new pcl::PointCloud<pcl::PointXYZRGB>);

  for(it = cluster_indices_balls.begin(); it != cluster_indices_balls.end(); ++it)
  {

     // rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl_ball (new pcl::PointCloud<pcl::PointXYZRGB>);
          for(pit = it->indices.begin(); pit != it->indices.end(); pit++)
          {
          //push_back: add a point to the end of the existing vector
                  cloud_pcl_ball->points.push_back(cloud_pcl_balls->points[*pit]);
          }

          //Merge current clusters to whole point cloud
          cluster_ball.push_back(cloud_pcl_ball);

  }

  float all_x=0;
  float all_z=0;
  std::vector<float> average_x;
  std::vector<float> average_z;
  unsigned int i=0,j=0;

  for(j=0;cluster_box.size()>j;j++)
  {
      all_x=0;
      all_z=0;
      for(i = 0; i < cluster_box.at(j)->points.size(); i++)
      {
          all_x=all_x+cluster_box.at(j)->points.at(i).x;
          all_z=all_z+cluster_box.at(j)->points.at(i).z;

      }
      //std::cout<<"x"<<all_x<<"   y"<<all_z<< std::endl;
      all_x=all_x/i;
      all_z=all_z/i;

      //Coordinate of the BOXES
      average_x.push_back(all_x);
      average_z.push_back(all_z);
  }

  std::vector<float> ball_x;
  std::vector<float> ball_z;
  //std::cout<<"Balls"<<std::endl;
  for(j=0;cluster_ball.size()>j;j++)
  {
    all_x=0;
    all_z=0;
    for(i = 0; i < cluster_ball.at(j)->points.size(); i++)
    {
     all_x=all_x+cluster_ball.at(j)->points.at(i).x;
     all_z=all_z+cluster_ball.at(j)->points.at(i).z;
    }
    all_x=all_x/i;
    all_z=all_z/i;

    //Coordinate of the Ball
    ball_x.push_back(all_x);
    ball_z.push_back(all_z);
  }
   
  std::vector<float> x_loc_boxes = average_x;
  std::vector<float> y_loc_boxes = average_z;
  std::vector<float> x_loc_balls = ball_x;
  std::vector<float> y_loc_balls = ball_z;
  
  // get cell size
  ros::NodeHandle nhPriv("~");
  double cell_size = 0.23;
  //nhPriv.getParam("cell_size", cell_size);
  
  // get robot position
  tf::TransformListener listener;
  ros::Rate rate(10.0);
  ros::NodeHandle node;
  tf::StampedTransform map_to_robot;
  while (node.ok()){
    try{
      listener.lookupTransform("/map", "/base_link",  
                               ros::Time(0), map_to_robot);
      break;
    }
    catch (tf::TransformException ex){
      //ROS_ERROR("%s",ex.what());
    }

    rate.sleep();
  }
  
  std::cout << map_to_robot.getOrigin().getX() << " - " << map_to_robot.getOrigin().getY() << ", #boxes: " << x_loc_boxes.size() <<  ", #balls:" << x_loc_balls.size() << std::endl;
  
  // get transformations to boxes
  double map_origin = 1.84; // TODO: set properly
  std::vector<nao_world_msgs::GridCoordinate> box_coordinates;
  for (int i=0; i<x_loc_boxes.size(); i++) {
    tf::Transform robot_to_box;
    robot_to_box.setOrigin(tf::Vector3(y_loc_boxes[i], -1 * x_loc_boxes[i], 0));
    
    tf::Transform map_to_box;
    //map_to_box = robot_to_box * map_to_robot;
    map_to_box = map_to_robot * robot_to_box;
    /*visualization_msgs::Marker boxGridMarker;
    boxGridMarker.header.frame_id= "/map";
    boxGridMarker.header.stamp= ros::Time::now();
    boxGridMarker.id= 0;*/
    stringstream ss2;
    ss2 << "grid-box" << i+1;
    /*boxGridMarker.ns= ss2.str();
    boxGridMarker.type= visualization_msgs::Marker::ARROW;//CUBE;
    boxGridMarker.action= visualization_msgs::Marker::ADD;
    boxGridMarker.color.b= 1.0;
    boxGridMarker.color.a= 1.0;
    boxGridMarker.pose.position.y= map_to_box.getOrigin().getX();
    boxGridMarker.pose.position.x= map_to_box.getOrigin().getY();
    boxGridMarker.pose.position.z= 0;
    boxGridMarker.pose.orientation.x = 0;
    boxGridMarker.pose.orientation.y = 1;
    boxGridMarker.pose.orientation.z = 0;
    boxGridMarker.pose.orientation.w = -3.14/2;
    boxGridMarker.scale.x= 0.15;
    boxGridMarker.scale.y= 0.15;
    boxGridMarker.scale.z= 0.5;
    boxGridMarker.lifetime= ros::Duration();
    res.boxLocs.markers.push_back(boxGridMarker);*/
    
    
    // calculate grid coordinate
    //tf::Transform map_to_box;
    //map_to_box = map_to_robot * robot_to_box;
    double delta_x = map_to_box.getOrigin().getX();
    double delta_y = map_to_box.getOrigin().getY();
    nao_world_msgs::GridCoordinate box;
    int x_coord = -floor(delta_x / cell_size);
    int y_coord = floor((delta_y + map_origin) / cell_size) + 1;
    box.x = x_coord;
    box.y = y_coord;
    box_coordinates.push_back(box);
    
    std::cout << "robot to: " << ss2.str() << ": [" << robot_to_box.getOrigin().getX() << ", " << robot_to_box.getOrigin().getY() << "], in cell: [" << x_coord << ", " << y_coord << "]\n";
    
    /*// box in calculated grid cell
    tf::Transform origin_to_cell;
    double origin_to_cell_X = -1 * ((x_coord * cell_size) + 0.5 * cell_size);
    double origin_to_cell_Y = (y_coord * cell_size) + 0.5 * cell_size;
    origin_to_cell.setOrigin(tf::Vector3(origin_to_cell_X, origin_to_cell_Y, 0));
    tf::Transform map_to_origin;
    map_to_origin.setOrigin(tf::Vector3(0, -1 * map_origin, 0));
    tf::Transform map_to_grid_cell;
    map_to_grid_cell = map_to_origin * origin_to_cell;
    std::cout << "coordX =" << map_to_grid_cell.getOrigin().getX() << ", coordY = " << map_to_grid_cell.getOrigin().getY() << std::endl;
    visualization_msgs::Marker boxGridMarker;
    boxGridMarker.header.frame_id= "/map";
    boxGridMarker.header.stamp= ros::Time::now();
    boxGridMarker.id= 0;
    stringstream ss2;
    ss2 << "grid-box" << i+1;
    boxGridMarker.ns= ss2.str();
    boxGridMarker.type= visualization_msgs::Marker::CUBE;
    boxGridMarker.action= visualization_msgs::Marker::ADD;
    boxGridMarker.color.b= 1.0;
    boxGridMarker.color.a= 1.0;
    boxGridMarker.pose.position.y= map_to_grid_cell.getOrigin().getX();
    boxGridMarker.pose.position.x= map_to_grid_cell.getOrigin().getY();
    boxGridMarker.pose.position.z= 0;
    boxGridMarker.scale.x= 0.15;
    boxGridMarker.scale.y= 0.15;
    boxGridMarker.scale.z= 0.15;
    boxGridMarker.lifetime= ros::Duration();
    res.boxLocs.markers.push_back(boxGridMarker);*/
    
    // box as seen from robot
    visualization_msgs::Marker boxMarker;
    boxMarker.header.frame_id= "/base_link";
    boxMarker.header.stamp= ros::Time::now();
    boxMarker.id= 0;
    stringstream ss;
    ss << "box" << i+1;
    boxMarker.ns= ss.str();
    boxMarker.type= visualization_msgs::Marker::CUBE;
    boxMarker.action= visualization_msgs::Marker::ADD;
    boxMarker.color.r= 1.0;
    boxMarker.color.a= 1.0;
    boxMarker.pose.position.y= -x_loc_boxes[i];
    boxMarker.pose.position.x= y_loc_boxes[i];
    boxMarker.pose.position.z= -0.2;
    boxMarker.scale.x= 0.15;
    boxMarker.scale.y= 0.15;
    boxMarker.scale.z= 0.15;
    boxMarker.lifetime= ros::Duration();
    res.boxLocs.markers.push_back(boxMarker);
  }
  
  // get transformations to balls
  std::vector<nao_world_msgs::GridCoordinate> ball_coordinates;
  for (int i=0; i<x_loc_balls.size(); i++) {
    tf::Transform robot_to_ball;
    robot_to_ball.setOrigin(tf::Vector3(y_loc_balls[i], -1 * x_loc_balls[i], 0));
    
    tf::Transform map_to_ball;
    map_to_ball = map_to_robot * robot_to_ball;
    double delta_x = map_to_ball.getOrigin().getX();
    double delta_y = map_to_ball.getOrigin().getY();
    nao_world_msgs::GridCoordinate ball;
    int x_coord = floor(delta_x / cell_size);
    int y_coord = floor((delta_y + map_origin) / cell_size) + 1;   
    ball.x = x_coord;
    ball.y = y_coord;
    ball_coordinates.push_back(ball);
    
    visualization_msgs::Marker ballMarker;
    
    ballMarker.id= 0;
    stringstream ss;
    ss << "ball" << i+1;
    ballMarker.ns= ss.str();
	  ballMarker.pose.position.y= -x_loc_balls[i];
	  ballMarker.pose.position.x= y_loc_balls[i];
	  ballMarker.pose.position.z= -0.2;  
	  ballMarker.header.frame_id= "/base_link";
	  ballMarker.header.stamp= ros::Time::now();
	  ballMarker.type= visualization_msgs::Marker::SPHERE;
	  ballMarker.action= visualization_msgs::Marker::ADD;
	  ballMarker.scale.x= 0.05;
	  ballMarker.scale.y= 0.05;
	  ballMarker.scale.z= 0.05;
	  ballMarker.color.g= 1.0;
	  ballMarker.color.a= 1.0;
	  ballMarker.lifetime= ros::Duration();
	  res.ballLocs.markers.push_back(ballMarker);
	  
	  std::cout << "robot to: " << ss.str() << ": [" << robot_to_ball.getOrigin().getX() << ", " << robot_to_ball.getOrigin().getY() << "], in cell: [" << x_coord << ", " << y_coord << "]\n";
  }
  
  res.boxes = box_coordinates;
  res.balls = ball_coordinates;
  
  

  return true;
}

int main (int argc, char** argv)
{
  // In..itialize ROS
  ros::init (argc, argv, "get_object_locs");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, color_detection);
  //ros::Subscriber sub = nh.subscribe ("/xtion/depth_registered/points", 1, color_detection);

  //ros::spin ();

  ros::ServiceServer service = nh.advertiseService("ObjectLocations", color_detection);
  ROS_INFO("Retrieving information from vision");
  ros::spin();

  return 0;

}

