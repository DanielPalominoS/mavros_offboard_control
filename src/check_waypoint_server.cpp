#include "check_waypoint_server.h"

check_waypoint_server::check_waypoint_server()
  :rate_(10)
{
  ros::NodeHandle pnh("~");
  std::string wp_service_topic;
  double  rate;
  // Get params if given
  pnh.param("WpServiceTopic",wp_service_topic,kDefaultWpServerTopic);
  pnh.param("publish_frequency",rate,kDefaultRate);

  //advertise services
  //nh_.advertiseService(wp_service_topic,boost::bind(&check_waypoint_server::check_wp_cb,this,_1),this);
  waypoint_controller_= nh_.advertiseService(wp_service_topic,&check_waypoint_server::check_wp_cb,this);

}
check_waypoint_server::~check_waypoint_server(){

}
void check_waypoint_server::Main_task(){
  ROS_INFO_ONCE("ready to check waypoints");
  while (ros::ok()) {
    ros::spinOnce();
    rate_.sleep();
  }
}

bool  check_waypoint_server::check_wp_cb(mavros_offboard_control::Check_waypoint::Request  &req,
                   mavros_offboard_control::Check_waypoint::Response &res){
  bool  success;
  double  lat1,lat2,lon1,lon2,h1,h2;
  double  azi1,azi2,s12;
  //double  dist;
  double  factor;
  //Realizar operaciones
  const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
  ros::Rate r(1);
  success=false;

  factor=1.0;
  if(!req.are_degrees){factor=180.0/M_PI;}
  /*lat1=(double) req.uav_current_gps.x*factor;
  lon1=(double) req.uav_current_gps.y*factor;
  lat2=(double) req.uav_desired_gps.x*factor;
  lon2=(double) req.uav_desired_gps.y*factor;*/
  h1=(double) req.uav_current_gps.z;
  h2=(double) req.uav_desired_gps.z;
  geod.Inverse(req.uav_current_gps.x*factor,req.uav_current_gps.y*factor,
               req.uav_desired_gps.x*factor,req.uav_desired_gps.y*factor,
               s12,azi1,azi2);

  std::cout<<"\ndistxy [m]: "<<s12<<std::endl;
  std::cout<<"dist [m]: "<<res.distance<<std::endl;
  //std::cout<<"Azi1 [deg]: "<<azi1<<std::endl;
  //std::cout<<"Azi2 [deg]: "<<azi2<<std::endl;
  std::cout<<"current heading(YAW) [deg]: "<<req.current_heading*factor<<std::endl;
  std::cout<<"desired heading(YAW) [deg]: "<<req.desired_heading*factor<<std::endl;
  //std::cout<<"Steering angle [deg]: "<<res.remaining_angle<<std::endl;
  if(req.current_heading>180/factor){
    req.current_heading-=360/factor;
  }else if (req.current_heading<-180/factor){
    req.current_heading+=360/factor;
  }

  if(req.desired_heading>180/factor){
    req.desired_heading-=360*factor;
  }else if (req.desired_heading<-180/factor){
    req.desired_heading+=360/factor;
  }
  res.distance=sqrt(pow(h2-h1,2)+pow(s12,2));
  //res.remaining_angle=azi1-req.current_heading*factor;
  res.remaining_angle=req.desired_heading-req.current_heading;

  //Condición para girar con el menor ángulo
  if(res.remaining_angle>180/factor){
    res.remaining_angle-=360/factor;
  }else if (res.remaining_angle<-180/factor){
    res.remaining_angle+=360/factor;
  }
  //res.azimuth=azi1/factor;
  //
  std::cout<<"Factor : "<<factor<<" are_degrees : "<<req.are_degrees<<std::endl;
  std::cout<<"current heading(YAW) [deg]: "<<req.current_heading*factor<<std::endl;
  std::cout<<"desired heading(YAW) [deg]: "<<req.desired_heading*factor<<std::endl;
  std::cout<<"Steering angle [deg]: "<<res.remaining_angle*factor<<std::endl;
  //if(dist<req.distance_threshold&& res.remaining_angle<req.heading_angle_threshold*factor){
  if(res.distance<req.distance_threshold && fabs(res.remaining_angle*factor)<req.heading_angle_threshold*factor){
    ROS_WARN("Waypoint reached\n");
    res.waypoint_reached=true;
  }
  else{
    res.waypoint_reached=false;
  }
  //res.remaining_angle/=factor;
  success=true;
  return  success;
}

int main(int argc, char** argv)
{
  std::setprecision (10);
  ros::init(argc, argv, "wp_server");
  check_waypoint_server waypoint_srvr=check_waypoint_server();
  waypoint_srvr.Main_task();
  //ros::spin();
  return 0;
}
