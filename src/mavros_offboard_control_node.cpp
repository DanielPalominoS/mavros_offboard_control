#include "mavros_offboard_control_node.h"

mavros_offboard_control_node::mavros_offboard_control_node():
  rate_(kDefaultFreq)
{
  ros::NodeHandle pnh("~");

  std::string gps_sub_topic;
  std::string local_sub_topic;
  std::string imu_sub_topic;
  std::string heading_sub_topic;
  std::string state_sub_topic;

  std::string sp_global_pub_topic;
  std::string sp_local_pub_topic;

  std::string arming_srv_topic;
  std::string set_mode_srv_topic;
  std::string take_pic_srv_topic;
  std::string process_image_srv_topic;
  std::string plan_flight_srv_topic;
  std::string wp_ctrl_srv_topic;
  std::string takeoff_srv_topic;

  double publish_frequency;
  /*****  Initializtion of those variable whose values are passed as params *****/
  //subscription topics initialization
  pnh.param("GpsSubTopic",gps_sub_topic,kDefaultGpsSubTopic);
  pnh.param("LocalPoseSubTopic",local_sub_topic,kDefaultLocalPoseSubTopic);
  pnh.param("ImuSubpTopic",imu_sub_topic,kDefaultImuSubTopic);
  pnh.param("HeadingSubTopic",heading_sub_topic,kDefaultHeadingSubTopic);
  pnh.param("StateSubTopic",state_sub_topic,kDefaultStateSubTopic);

  pnh.param("SetpointGlobalTopic",sp_global_pub_topic,kDefaultSetpointGlobalPubTopic);
  pnh.param("SetpointLocalTopic",sp_local_pub_topic,kDefaultSetpointLocalPubTopic);

  pnh.param("ArmingTopic",arming_srv_topic,kDefaultCmdArmingbTopic);
  pnh.param("SetModeTopic",set_mode_srv_topic,kDefaultSetModeTopic);
  pnh.param("TakePicTopic",take_pic_srv_topic,kDefaultTakePicTopic);
  pnh.param("ProcessImgTopic",process_image_srv_topic,kDefaultProcessImgTopic);
  pnh.param("PlanFlightTopic",plan_flight_srv_topic,kDefaultPlanFlightTopic);
  pnh.param("WpCtrlTopic",wp_ctrl_srv_topic,kDefaultWpCtrlTopic);
  pnh.param("TakeoffTopic",takeoff_srv_topic,kDefaultTakeoffTopic);


  pnh.param("PublishFrequency",publish_frequency,kDefaultFreq);
  pnh.param("TakeoffHieght",takeoff_height_,kDefaultTakeoffHeight);

  pnh.param("DesiredGsd",desired_gsd_,kDefaultDesiredGsd);
  pnh.param("DesiredEndlap",desired_endlap_,kDefaultDesiredEndlap);
  pnh.param("DesiredSidelap",desired_sidelap_,kDefaultDesiredSidelap);
  pnh.param("CamOri",s_width_,kDefaultCamOri);

  pnh.param("ImgWidth",i_width_,kDefaultImgWidth);
  pnh.param("ImgHeight",i_height_,kDefaultImgHeight);
  pnh.param("Fx",fx_,kDefaultFx);
  pnh.param("Fy",fy_,kDefaultFy);
  pnh.param("Cx",cx_,kDefaultCx);
  pnh.param("Cy",cy_,kDefaultCy);
  pnh.param("Skew",skew_,kDefaultSkew);
  pnh.param("FocalLength",focal_lenght_,kDefaultFocalLenght);
  pnh.param("SensorWidth",s_width_,kDefaultSensorWidth);
  pnh.param("SensorHeight",s_heigth_,kDefaultSensorHeight);

  rate_=ros::Rate(publish_frequency);
  //Considerar dejar las frecuencias de cada nodo de manera independiente
  //Subscribers' initialization
  global_pos_sub_=nh_.subscribe<sensor_msgs::NavSatFix>
      (gps_sub_topic,5,boost::bind(&mavros_offboard_control_node::gps_cb,this,_1));
  local_pos_sub_=nh_.subscribe<geometry_msgs::PoseStamped>
      (local_sub_topic,5,boost::bind(&mavros_offboard_control_node::local_pose_cb,this,_1));
  imu_sub_=nh_.subscribe<sensor_msgs::Imu>
      (imu_sub_topic,10,boost::bind(&mavros_offboard_control_node::imu_cb,this,_1));
  heading_sub_=nh_.subscribe<std_msgs::Float64>
      (heading_sub_topic,5,boost::bind(&mavros_offboard_control_node::heading_cb,this,_1));
  state_sub_ = nh_.subscribe<mavros_msgs::State>
          (state_sub_topic, 5,boost::bind(&mavros_offboard_control_node::state_cb,this,_1));

  //Publishers' initialization
  /*local_pos_pub_ = nh_.advertise<mavros_msgs::PositionTarget>
      (sp_local_pub_topic, 20);*/
  local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
          ("mavros/setpoint_position/local", 10);
  global_pos_pub_ = nh_.advertise<mavros_msgs::GlobalPositionTarget>
              (sp_global_pub_topic, 20);

  //Services' initialization
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
      (arming_srv_topic);
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>
      (set_mode_srv_topic);
  take_picture_client_=nh_.serviceClient<crop_image_processing::Take_save_picture>
      (take_pic_srv_topic);
  process_image_client_=nh_.serviceClient<crop_image_processing::Find_crop_features>
      (process_image_srv_topic);
  plan_flight_client_=nh_.serviceClient<flight_planning::generate_plan>
      (plan_flight_srv_topic);
  wp_control_client_=nh_.serviceClient<mavros_offboard_control::Check_waypoint>
      (wp_ctrl_srv_topic);
  takeoff_client_=nh_.serviceClient<mavros_offboard_control::Takeoff>
      (takeoff_srv_topic);
  //take_picture_client_ = nh_.serviceClient<crop_image_processing::Take_save_picture>
  //    ()

  fcu_connected_=false;
  start_flag_=false;
  debug_flag_=true;
  plan_available_flag_=false;
  plan_executed_flag_=false;
  crop_found_flag_=false;
  are_degrees_flag_=true;
  current_wp_index_=0;
  takeoff_done_flag_=false;
  waypoint_reached_=false;

  //find_crop

  /*desired_endlap_=60;
  desired_sidelap_=50;
  desired_gsd_=0.04;

  i_width_=1280;
  i_height_=960;
  fx_=1464;
  fy_=1464;
  cx_=640;
  cy_=480;
  skew_=0;
  focal_lenght_=0.0055;
  cam_ori_=90;
  s_width_=0.0048;
  s_heigth_=0.0036;*/
}

mavros_offboard_control_node::~mavros_offboard_control_node(){

}

void mavros_offboard_control_node::main_task(){

  //mavros_offboard_control_node::wait_for_servers();
  // wait for FCU connection
  while(ros::ok() && !current_state_.connected){
    ros::spinOnce();
    rate_.sleep();
  }
  ROS_INFO_ONCE("Fcu connected");
  fcu_connected_=true;
  takeoff_done_flag_=false;
  bool  t1_measured=false;
  ros::Time t1,t2;
  mavros_msgs::PositionTarget TargetPose;
  TargetPose.coordinate_frame=mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  TargetPose.position.x=0;
  TargetPose.position.y=0;
  TargetPose.position.z=takeoff_height_;
  TargetPose.yaw=0*M_PI/2;
  TargetPose.yaw_rate=2;
  TargetPose.velocity.x=10;
  TargetPose.velocity.y=0;
  TargetPose.velocity.z=20;
  TargetPose.type_mask=960;

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 120;

  //send a few setpoints before starting
  for(int i = 100; ros::ok() && i > 0; --i){
      TargetPose.header.stamp = ros::Time::now();
      local_pos_pub_.publish(pose);
      //local_pos_pub_.publish(TargetPose);
      ros::spinOnce();
      rate_.sleep();
  }

  // Update takeoff request
  takeoff_srv_.request.uav_ref.x=0;//initial_gps_pose_.latitude;
  takeoff_srv_.request.uav_ref.y=0;//initial_gps_pose_.longitude;
  takeoff_srv_.request.uav_ref.z=0;
  takeoff_srv_.request.local_frame=true;
  takeoff_srv_.request.goal_altitude=takeoff_height_;

  //update Find crop features request
  find_crop_srv_.request.name="crop_features";
  //find_crop_srv_.request.storage_path="/home/daniel/Pictures/planning/";
  find_crop_srv_.request.storage_path="/home/nvidia/Pictures/planning/";
  find_crop_srv_.request.save=true;

  //Update Take picture request
  //save_pict_srv_.request.storage_path="/home/daniel/Pictures/planning/mosaicking/";
  save_pict_srv_.request.storage_path="/home/nvidia/Pictures/planning/mosaicking/";

  bool  wp_request_configured=false;
  mavros_msgs::GlobalPositionTarget Global_target_pose;
  ROS_INFO_ONCE("Waiting until offboard mode enabling");
  while(ros::ok()){
    if(current_state_.mode=="OFFBOARD"){
      if(!current_state_.armed){
        arming_cmd_srv_.request.value=true;
        arming_client_.call(arming_cmd_srv_);
      }
      if(!takeoff_done_flag_&&!plan_available_flag_){
        takeoff_client_.call(takeoff_srv_);
        ROS_INFO_ONCE("None takeoff done nor plan available");
        local_pos_pub_.publish(pose);
        //local_pos_pub_.publish(TargetPose);
        if(takeoff_srv_.response.takeoff_completed){
          local_pos_pub_.publish(pose);
          //local_pos_pub_.publish(TargetPose);
          if(!crop_found_flag_){
            local_pos_pub_.publish(pose);
            //local_pos_pub_.publish(TargetPose);
            if(!t1_measured){
              t1_measured=true;
              t1=ros::Time::now();
              t2=t1;
            }
            local_pos_pub_.publish(pose);
            //local_pos_pub_.publish(TargetPose);
            process_image_client_.call(find_crop_srv_);
            crop_found_flag_=find_crop_srv_.response.success;
            local_pos_pub_.publish(pose);
            //local_pos_pub_.publish(TargetPose);
            if( crop_found_flag_){
              update_generate_plan_request();
              t1_measured=false;
            }else{
               t2=ros::Time::now();
               if(t2-t1>ros::Duration(timeout_)){
                 takeoff_done_flag_=true;
                 plan_available_flag_=false;
                 ROS_WARN("Takeoff was done but none crop was identified");
               }
            }
            local_pos_pub_.publish(pose);
            //local_pos_pub_.publish(TargetPose);

          }

          else{
            if(!t1_measured){
              t1_measured=true;
              t1=ros::Time::now();
              t2=t1;
            }
            plan_flight_client_.call(generate_plan_srv_);
            plan_available_flag_=generate_plan_srv_.response.success;
            if( plan_available_flag_ ){
              ROS_INFO("Plan generated succesfully.\nStarting execution");
              takeoff_done_flag_=true;
              wp_request_configured=false;
              current_wp_index_=0;
            }else {
              t2=ros::Time::now();
              if(t2-t1>ros::Duration(timeout_)){
                takeoff_done_flag_=true;
                plan_available_flag_=false;
                ROS_WARN("Takeoff was done but none plan could be generated");
              }
            }
          }
        }
        TargetPose.header.stamp = ros::Time::now();
        local_pos_pub_.publish(pose);
        //local_pos_pub_.publish(TargetPose);
      }
      else if(plan_available_flag_){
        ROS_INFO_ONCE("Takeoff done and plan available");
        //Update request
        double  factor=180.0/M_PI;
        if(!are_degrees_flag_){factor=1.0;}
        double  des_yaw=generate_plan_srv_.response.uav_heading[current_wp_index_];
        if(des_yaw>M_PI*factor){
          des_yaw-=2*M_PI*factor;
        }else if (des_yaw<-M_PI*factor){
          des_yaw+=2*M_PI*factor;
        }

        if(!wp_request_configured){
          if(are_degrees_flag_){
            Global_target_pose.yaw=(90-des_yaw)/factor;//generate_plan_srv_.response.uav_heading[current_wp_index_];
            ROS_INFO("Degrees");
          }else{
            ROS_INFO("no degrees");
            Global_target_pose.yaw=M_PI/2-des_yaw;//generate_plan_srv_.response.uav_heading[current_wp_index_];
          }
          
          //Global_target_pose.yaw=0;
          Global_target_pose.altitude=generate_plan_srv_.response.plan_gps_coors[current_wp_index_].z;
          Global_target_pose.latitude=generate_plan_srv_.response.plan_gps_coors[current_wp_index_].x;
          Global_target_pose.longitude=generate_plan_srv_.response.plan_gps_coors[current_wp_index_].y;
          Global_target_pose.coordinate_frame=0;
          Global_target_pose.velocity.x=20;
          Global_target_pose.velocity.z=20;
          Global_target_pose.velocity.y=10;
          Global_target_pose.type_mask=Global_target_pose.IGNORE_AFX+
              Global_target_pose.IGNORE_AFY+Global_target_pose.IGNORE_AFZ;

          ROS_INFO("\n");
          std::cout<<"current heading(YAW) [deg]: "<<current_heding_.data<<std::endl;
          std::cout<<"desired heading(YAW) [deg]: "<<des_yaw<<std::endl;

          check_wp_srv_.request.uav_desired_gps=generate_plan_srv_.response.plan_gps_coors[current_wp_index_];
          check_wp_srv_.request.desired_heading=des_yaw;//generate_plan_srv_.response.uav_heading[current_wp_index_]*factor;// El angulo calculado por el planificador esta en radianes
          check_wp_srv_.request.are_degrees=are_degrees_flag_;
          check_wp_srv_.request.distance_threshold=2.0;
          check_wp_srv_.request.heading_angle_threshold=2.0;
          wp_request_configured=true;
        }
        check_wp_srv_.request.current_heading=current_heding_.data;
        check_wp_srv_.request.uav_current_gps=current_gps_;
        //ROS_INFO("\n");std::cout<<"Response: "<<check_wp_srv_.response<<std::endl;
        Global_target_pose.header.stamp = ros::Time::now();
        global_pos_pub_.publish(Global_target_pose);
        wp_control_client_.call(check_wp_srv_);
        waypoint_reached_=check_wp_srv_.response.waypoint_reached;
        if(waypoint_reached_){
          t1=ros::Time::now();
          t2=t1;
          while(t2-t1<ros::Duration(3.0)){
            Global_target_pose.header.stamp = ros::Time::now();
            global_pos_pub_.publish(Global_target_pose);
            t2=ros::Time::now();
          }
          save_pict_srv_.request.name="plan_"+std::to_string(current_wp_index_);
          take_picture_client_.call(save_pict_srv_);
          photo_taken_flag_=save_pict_srv_.response.success;
          if(photo_taken_flag_){
            current_wp_index_++;

            if(current_wp_index_>=generate_plan_srv_.response.plan_gps_coors.size()){
              plan_executed_flag_=true;
              plan_available_flag_=false;
            }
            else{
              plan_executed_flag_=false;
              plan_available_flag_=true;

              /*Global_target_pose.yaw=des_yaw;//generate_plan_srv_.response.uav_heading[current_wp_index_];
              Global_target_pose.altitude=generate_plan_srv_.response.plan_gps_coors[current_wp_index_].z;
              Global_target_pose.latitude=generate_plan_srv_.response.plan_gps_coors[current_wp_index_].x;
              Global_target_pose.longitude=generate_plan_srv_.response.plan_gps_coors[current_wp_index_].y;
              //Global_target_pose.coordinate_frame=Global_target_pose.FRAME_GLOBAL_REL_ALT;
              Global_target_pose.coordinate_frame=0;
              Global_target_pose.velocity.x=20;
              Global_target_pose.velocity.z=20;
              Global_target_pose.velocity.y=10;
              //Global_target_pose.type_mask=Global_target_pose.IGNORE_AFX+Global_target_pose.IGNORE_AFY+Global_target_pose.IGNORE_AFZ;*/
            }
            wp_request_configured=false;
            photo_taken_flag_=false;
            waypoint_reached_=false;
          }
        }
        /*else {
          Global_target_pose.yaw=generate_plan_srv_.response.uav_heading[current_wp_index_];
          Global_target_pose.altitude=generate_plan_srv_.response.plan_gps_coors[current_wp_index_].z;
          Global_target_pose.latitude=generate_plan_srv_.response.plan_gps_coors[current_wp_index_].x;
          Global_target_pose.longitude=generate_plan_srv_.response.plan_gps_coors[current_wp_index_].y;
          //Global_target_pose.coordinate_frame=Global_target_pose.FRAME_GLOBAL_REL_ALT;
          Global_target_pose.coordinate_frame=0;
          Global_target_pose.velocity.x=10;
          Global_target_pose.velocity.z=10;
          Global_target_pose.velocity.y=5;
          Global_target_pose.type_mask=Global_target_pose.IGNORE_AFX+Global_target_pose.IGNORE_AFY+Global_target_pose.IGNORE_AFZ;
        }*/
        Global_target_pose.header.stamp = ros::Time::now();
        global_pos_pub_.publish(Global_target_pose);

      }
      else if(plan_executed_flag_){
        /*Global_target_pose.yaw=0;
        Global_target_pose.altitude=initial_gps_pose_.altitude+2;
        Global_target_pose.latitude=initial_gps_pose_.latitude;
        Global_target_pose.longitude=initial_gps_pose_.longitude;
        //Global_target_pose.coordinate_frame=Global_target_pose.FRAME_GLOBAL_REL_ALT;
        Global_target_pose.coordinate_frame=0;
        Global_target_pose.velocity.x=10;
        Global_target_pose.velocity.z=10;
        Global_target_pose.velocity.y=5;
        Global_target_pose.type_mask=Global_target_pose.IGNORE_AFX+Global_target_pose.IGNORE_AFY+Global_target_pose.IGNORE_AFZ;

        check_wp_srv_.request.uav_desired_gps.x=initial_gps_pose_.latitude;
        check_wp_srv_.request.uav_desired_gps.y=initial_gps_pose_.longitude;
        check_wp_srv_.request.uav_desired_gps.z=initial_gps_pose_.altitude+2;
        check_wp_srv_.request.desired_heading=0;
        check_wp_srv_.request.are_degrees=are_degrees_flag_;

        global_pos_pub_.publish(Global_target_pose);

        check_wp_srv_.request.current_heading=current_heding_.data;
        check_wp_srv_.request.uav_current_gps=current_gps_;
        wp_control_client_.call(check_wp_srv_);
        waypoint_reached_=check_wp_srv_.response.waypoint_reached;

        //set_mode_srv_.request.custom_mode="AUTO.RTL";
        if(waypoint_reached_){
          ROS_INFO("Plan executed succesfully, returnning to rc control in position mode");
          set_mode_srv_.request.custom_mode="POSCTL";
          set_mode_client_.call(set_mode_srv_);
        }*/
        set_mode_srv_.request.custom_mode="AUTO.RTL";
        set_mode_client_.call(set_mode_srv_);
      }
      else{
        ROS_WARN("Changing to return home mode");
        set_mode_srv_.request.custom_mode="AUTO.RTL";
        set_mode_client_.call(set_mode_srv_);        
      }

    }
    else {
      if(!plan_available_flag_){
        local_pos_pub_.publish(pose);
        //local_pos_pub_.publish(TargetPose);
      }

    }
    ros::spinOnce();
    rate_.sleep();
  }

}

void  mavros_offboard_control_node::gps_cb(const sensor_msgs::NavSatFixConstPtr& msg){
  if(start_flag_){
    current_gps_pose_=*msg;
    current_gps_.x=current_gps_pose_.latitude;
    current_gps_.y=current_gps_pose_.longitude;
    current_gps_.z=current_gps_pose_.altitude;
    //ROS entrega los datos de lat lon en grados
    if(!are_degrees_flag_){
      current_gps_.x*=M_PI/180.0;
      current_gps_.y*=M_PI/180.0;
    }
  }
  else{
    initial_gps_pose_=*msg;
    start_flag_=true;
    //takeoff_goal_.uav_ref=initial_gps_pose_;
  }
  //if(debug_flag_){ROS_INFO("Initial Pose:\t");std::cout<<initial_gps_pose_<<std::endl<<"current pose:\t"<<current_gps_pose_<<std::endl;}
  return;
}
void  mavros_offboard_control_node::local_pose_cb(const geometry_msgs::PoseStampedConstPtr& msg){
  current_local_pose_=*msg;
}
void  mavros_offboard_control_node::imu_cb(const sensor_msgs::ImuConstPtr& msg){
  uav_attitude_=*msg;
  tf::Quaternion  q(uav_attitude_.orientation.x,uav_attitude_.orientation.y,
                    uav_attitude_.orientation.z,uav_attitude_.orientation.w);

  tf::Matrix3x3 rm(q);
  rm.getRPY(uav_ypr_.z,uav_ypr_.y,uav_ypr_.x);
  //if(debug_flag_){std::cout<<"UAV Yaw pitch Roll:"<<"\nyaw:\t"<<uav_ypr_.x*180/M_PI<<"\npitch:\t"<<uav_ypr_.y*180/M_PI<<"\nroll:\t"<<uav_ypr_.z*180/M_PI<<std::endl;}
  geometry_msgs::Vector3 uav_ypr_euler_;
  //Los datos entregados por la conversión se encuentran en radianes
  if(are_degrees_flag_){
    uav_ypr_.x*=180.0/M_PI;
    uav_ypr_.y*=180.0/M_PI;
    uav_ypr_.z*=180.0/M_PI;}
  //Se multiplica por -1 porque la transformación de la libreria TF usa la notación
  //North East Up, en donde un giro en sentido positivo es opuesto al de la notación NED
  uav_ypr_.x*=-1;

  rm.getEulerYPR(uav_ypr_euler_.x,uav_ypr_euler_.y,uav_ypr_euler_.z);
  //if(debug_flag_){ROS_INFO("UAV attitude:\t");std::cout<<uav_attitude_.orientation<<std::endl;}
  return;
}

void  mavros_offboard_control_node::heading_cb(const  std_msgs::Float64ConstPtr& msg){

  current_heding_=*msg;
  if(current_heding_.data>180){
    current_heding_.data-=360;
  }else if(current_heding_.data<-180){
    current_heding_.data+=360;
  }
  if(!are_degrees_flag_){
    current_heding_.data*=M_PI/180;
  }
  //if(debug_flag_){ROS_INFO("Heading: "); std::cout<<current_heding_.data<<std::endl;}
}

void  mavros_offboard_control_node::state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state_=*msg;
  return;
}


double mavros_offboard_control_node::calculate_height(double fl,double gsd,double sw,double iw){
  /****
   * f: focal lenght
   *gsd: ground sample distance
   * iw: Image width
   * sw: Sensor width
   * Fh: flight height
   *
   *FH=gsd*f*iw/sw;
   *
   * */
  double  height;
  height=gsd*fl*iw/sw;
  return  height;
}

void  mavros_offboard_control_node::update_generate_plan_request(){
  generate_plan_srv_.request.cam_ori=cam_ori_;
  generate_plan_srv_.request.cx=cx_;
  generate_plan_srv_.request.cy=cy_;
  generate_plan_srv_.request.fx=fx_;
  generate_plan_srv_.request.fy=fy_;
  generate_plan_srv_.request.skew=skew_;
  generate_plan_srv_.request.i_width=i_width_;
  generate_plan_srv_.request.i_height=i_height_;
  generate_plan_srv_.request.fl=focal_lenght_;
  generate_plan_srv_.request.endlap=desired_endlap_;
  generate_plan_srv_.request.sidelap=desired_sidelap_;
  generate_plan_srv_.request.gsd=desired_gsd_;
  generate_plan_srv_.request.are_degrees=are_degrees_flag_;
  generate_plan_srv_.request.s_width=s_width_;
  generate_plan_srv_.request.s_heigth=s_heigth_;

  generate_plan_srv_.request.uav_gps.x=current_gps_pose_.latitude;
  generate_plan_srv_.request.uav_gps.y=current_gps_pose_.longitude;
  generate_plan_srv_.request.uav_gps.z=current_gps_pose_.altitude;
  generate_plan_srv_.request.ref_gps.x=initial_gps_pose_.latitude;
  generate_plan_srv_.request.ref_gps.y=initial_gps_pose_.longitude;
  generate_plan_srv_.request.ref_gps.z=initial_gps_pose_.altitude;
  generate_plan_srv_.request.landing_gps=generate_plan_srv_.request.ref_gps;
  generate_plan_srv_.request.crop=find_crop_srv_.response.crop;
  generate_plan_srv_.request.are_degrees=true;
  generate_plan_srv_.request.max_speed=2;
  generate_plan_srv_.request.frame_rate=2;
}

bool  mavros_offboard_control_node::wait_for_servers(){
  bool success=false;
  if(!takeoff_client_.waitForExistence(ros::Duration(5.0))){
    if(debug_flag_){ROS_WARN(" ");std::cout<<takeoff_client_.getService() <<"."<<std::endl;}
    return false;
  }
  if(debug_flag_){ROS_INFO_ONCE("takeoff server started.");}

  if (!wp_control_client_.waitForExistence(ros::Duration(5.0))){
    ROS_WARN("wp_control service not found");
    return false;
  }

  if (!wp_control_client_.waitForExistence(ros::Duration(5.0))){
    ROS_WARN("wp_control service not found");
    return false;
  }
  if(debug_flag_){ROS_INFO_ONCE("Enable motor server started.");}

  if(!process_image_client_.waitForExistence(ros::Duration(5.0))){
    ROS_WARN("Process image service not found");
    return false;
  }
  if(debug_flag_){ROS_INFO_ONCE("Process image server started.");}

  if (!take_picture_client_.waitForExistence(ros::Duration(5.0))){
    ROS_WARN("Take Photo service not found");
    return false;
  }

  if (!arming_client_.waitForExistence(ros::Duration(5.0))){
    ROS_WARN("Uav arming service not found");
    return false;
  }
  if(debug_flag_){ROS_INFO_ONCE("Arming server started.");}

  success=true;

  return success;
}

int main(int argc, char **argv)
{
  //std::setprecision (10);
  ros::init(argc, argv, "mavros_offboard_control");
  ros::Time::init();
  mavros_offboard_control_node controller;
  while(ros::ok() && !controller.wait_for_servers()){
    ROS_WARN_ONCE("Plese verify that all servers have been initialized");
  }
  controller.main_task();
  return 0;
}

