#include "check_takeoff_server.h"
void  PoseCb(const geometry_msgs::PoseStampedPtr& msg){

}

check_takeoff_server::check_takeoff_server()
  :rate_(kDefaultRate)
{
    ros::NodeHandle pnh("~");

    std::string gps_sub_topic;
    std::string local_pose_sub_topic;
    std::string takeoff_service_topic;
    double  rate;
    // Get params if given
    pnh.param("GpsSubTopic",gps_sub_topic,kDefaultGpsSubTopic);
    pnh.param("LocalPoseSubTopic",local_pose_sub_topic,kDefaultLocalPoseSubTopic);
    pnh.param("TakeoffServiceTopic",takeoff_service_topic,kDefaultTakeoffServerTopic);
    pnh.param("DistanceThreshold",distance_threshold_,kDefaultDistanceThreshold);
    pnh.param("publish_frequency",rate,kDefaultRate);
    //Subscribe to topics
    GpsSubscriber_=nh_.subscribe<sensor_msgs::NavSatFix>(gps_sub_topic,10,boost::bind(&check_takeoff_server::GpsCallback,this,_1));
    Pose_Subscriber_=nh_.subscribe<geometry_msgs::PoseStamped>(local_pose_sub_topic,20,boost::bind(&check_takeoff_server::PoseCallback,this,_1));
    //Pose_Subscriber_=nh_.subscribe<sensor_msgs::NavSatFix>(local_pose_sub_topic,10,boost::bind(&check_takeoff_server::GpsCallback,this,_1));
    //advertise services
    Takeoff_server_= nh_.advertiseService(takeoff_service_topic,&check_takeoff_server::CheckTakeoff,this);

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

check_takeoff_server::~check_takeoff_server(){

}

void check_takeoff_server::GpsCallback(const sensor_msgs::NavSatFixConstPtr& msg){
  //t_now=ros::Time::now();
  t_prev_gps_=ros::Time::now() - ros::Duration(5.0);
  if(t_prev_local_.sec>0){
    prev_gps=curr_gps;
  }
  curr_gps=*msg;
}

void check_takeoff_server::PoseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
  t_prev_local_=ros::Time::now() - ros::Duration(5.0);
  if(t_prev_local_.sec>0){
    prev_local_pose=curr_local_pose;
  }
  curr_local_pose=msg->pose;
}
bool check_takeoff_server::CheckTakeoff(mavros_offboard_control::Takeoff::Request &req,
                mavros_offboard_control::Takeoff::Response &res){

  request_=req;
  if(req.local_frame){
    res.distance_to_goal=req.goal_altitude+req.uav_ref.z-curr_local_pose.position.z;
    //if(fabs(req.goal_altitude+req.uav_ref.z-curr_local_pose.position.z)<distance_threshold_ &&
    if(fabs(res.distance_to_goal)<distance_threshold_ &&
       fabs(curr_local_pose.position.z-prev_local_pose.position.z)<distance_threshold_){
      res.takeoff_completed=true;
    }
    else{
      res.takeoff_completed=false;
    }
  }
  else {
    res.distance_to_goal=req.goal_altitude+req.uav_ref.z-curr_gps.altitude;
    //if(fabs(req.goal_altitude-curr_gps.altitude)<distance_threshold_ &&
    if(fabs(res.distance_to_goal)<distance_threshold_ &&
       fabs(curr_gps.altitude-prev_gps.altitude)<distance_threshold_){
      res.takeoff_completed=true;
    }
    else{
      res.takeoff_completed=false;
    }
  }
}
void check_takeoff_server::Main_task(){
  #ifdef DEBUG
  ROS_INFO_ONCE("Ready to check takeoff");
  #endif
  while (ros::ok()) {
    ros::spinOnce();
    rate_.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "takeoff_server");
  check_takeoff_server  to_server=check_takeoff_server();
  //ros::spin();
  to_server.Main_task();
  return 0;
}
