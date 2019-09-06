#include "takeoff_server.h"

takeoff_server::takeoff_server():
  rate_(kDefaultRate)
{
  ros::NodeHandle pnh("~");

  std::string gps_sub_topic;
  std::string local_pose_sub_topic;
  std::string takeoff_service_topic;
  double  rate;
  // Get params if given
  pnh.param("GpsSubTopic",gps_sub_topic,kDefaultGpsSubTopic);
  pnh.param("LocalPoseSubTopic",local_pose_sub_topic,kDefaulLocalPoseSubTopic);
  pnh.param("TakeoffServiceTopic",takeoff_service_topic,kDefaulLocalPoseSubTopic);
  pnh.param("DistanceThreshold",distance_threshold_,kDefaultDistanceThreshold);
  pnh.param("publish_frequency",rate,kDefaultRate);
  //Subscribe to topics
  //nh_.subscribe<sensor_msgs::NavSatFix>(gps_sub_topic,10,boost::bind(&takeoff_server::GpsCallback,this,_1));
  nh_.subscribe<geometry_msgs::PoseStamped>(gps_sub_topic,10,boost::bind(&takeoff_server::PoseCallback,this,_1));
  //nh_.subscribe<geometry_msgs::PoseStamped>(local_pose_sub_topic,10,&takeoff_server::PoseCallback);
  //advertise services
  nh_.advertiseService(takeoff_service_topic,&takeoff_server::CheckTakeoff,this);

  t_prev_gps_.sec=0;
  t_prev_local_.sec=0;
  prev_alt_gps_=0;
  prev_alt_local_=0;
  request_.goal_altitude=10*distance_threshold_;
  request_.local_frame=false;
  request_.uav_ref.x=0;
  request_.uav_ref.y=0;
  request_.uav_ref.z=0;
  distance_to_goal_gps_=10*distance_threshold_;
  distance_to_goal_local_=10*distance_threshold_;

}

takeoff_server::~takeoff_server(){

}

void takeoff_server::GpsCallback(const sensor_msgs::NavSatFixConstPtr& msg){
  //t_now=ros::Time::now();
  t_prev_gps_=ros::Time::now() - ros::Duration(5.0);
  if(t_prev_local_.sec>0){
    prev_gps=curr_gps;
  }
  curr_gps=*msg;

  //double DesiredAltitude=request_.goal_altitude+request_.uav_ref.z;
  /*ROS_INFO("Current altitude over sea level: ");std::cout<<CurrentPose.altitude<<std::endl;
  std::cout<<"Reference altitude: "<<goal_.uav_ref.altitude<<std::endl;
  std::cout<<"Goal altitude: "<<DesiredAltitude<<std::endl;
  std::cout<<"Altitude relative to reference: "<<CurrentPose.altitude-goal_.uav_ref.altitude<<std::endl;
  std::cout<<"Remaining distance: "<<DesiredAltitude-(CurrentPose.altitude)<<std::endl;*/
  /*if(fabs(DesiredAltitude-(msg->altitude))<distance_threshold_ && fabs(msg->altitude-prev_alt_gps_)<distance_threshold_) {
    success_gps_=true;
  }
  else{
    success_gps_=false;
  }
  distance_to_goal_gps_=DesiredAltitude-msg->altitude;
  if(t_prev_gps_.sec>0){
    prev_alt_gps_=msg->altitude;
  }*/
}
void takeoff_server::PoseCallback(const geometry_msgs::PoseStampedPtr& msg){
  t_prev_local_=ros::Time::now() - ros::Duration(5.0);
  if(t_prev_local_.sec>0){
    prev_local_pose=curr_local_pose;
  }
  curr_local_pose=msg->pose;

  //double DesiredAltitude=request_.goal_altitude+request_.uav_ref.z;

  /*ROS_INFO("Current altitude over sea level: ");std::cout<<CurrentPose.altitude<<std::endl;
  std::cout<<"Reference altitude: "<<goal_.uav_ref.altitude<<std::endl;
  std::cout<<"Goal altitude: "<<DesiredAltitude<<std::endl;
  std::cout<<"Altitude relative to reference: "<<CurrentPose.altitude-goal_.uav_ref.altitude<<std::endl;
  std::cout<<"Remaining distance: "<<DesiredAltitude-(CurrentPose.altitude)<<std::endl;*/
  /*distance_to_goal_local_=DesiredAltitude-msg->pose.position.z;
  if(fabs(DesiredAltitude-(msg->pose.position.z))<distance_threshold_ && fabs(msg->pose.position.z-prev_alt_local_)<distance_threshold_) {

    success_local_=true;
  }
  else{
    success_local_=false;
  }
  if(t_prev_local_.sec>0){
    prev_alt_local_=msg->pose.position.z;
  }*/
}
bool takeoff_server::CheckTakeoff(mavros_offboard_control::Takeoff::Request &req,
                  mavros_offboard_control::Takeoff::Response &res){
  request_=req;
  if(req.local_frame){
    if(fabs(req.goal_altitude+req.uav_ref.z-curr_local_pose.position.z)<distance_threshold_ &&
       fabs(curr_local_pose.position.z-prev_local_pose.position.z)<distance_threshold_){
      res.takeoff_completed=true;
    }
    else{
      res.takeoff_completed=false;
    }
    //req.goal_altitude+req.uav_ref.z;
    /*if(success_local_){

    }
    else{
      res.takeoff_completed=false;
    }*/
    res.distance_to_goal=distance_to_goal_local_;
  }
  else {
    if(fabs(req.goal_altitude+req.uav_ref.z-curr_gps.altitude)<distance_threshold_ &&
       fabs(curr_gps.altitude-prev_gps.altitude)<distance_threshold_){
      res.takeoff_completed=true;
    }
    else{
      res.takeoff_completed=false;
    }


    /*if(success_gps_){
      res.takeoff_completed=true;
    }
    else{
      res.takeoff_completed=false;
    }*/
    res.distance_to_goal=distance_to_goal_gps_;
  }
}
void takeoff_server::Main_task(){
  while (ros::ok()) {
    ros::spinOnce();
    rate_.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "takeoff_server");
  takeoff_server  to_server;;
  //ros::spin();
  to_server.Main_task();
  return 0;
}
