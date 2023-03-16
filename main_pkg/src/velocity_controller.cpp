// node that controls the joint velocities to keep a joint configuration
// subscribe to joint_poses
// publish to velocity controller

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <map>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "custom_interfaces/srv/rob_conf.hpp"

//#include <dmp/Trajectory.hpp>

#include <iostream>
#include <fstream>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

//#include <eigen3/Eigen/Eigen>
//#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/Geometry>


//#include <dmp/Trajectory.hpp>

using namespace std::chrono_literals;
//using namespace DmpBbo;
using namespace std::placeholders;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

//################# Main parameters #############################

int control_rate = 500;
int control_rate_ms = (float)1/control_rate * 1000;

int rate = 1000;
bool activate_stabilization = true;
bool got_initial_config = false;
//double threshold = 0.01;
double vel_max = M_PI;
double scaler_cubic = 50000;
int scaler_square = 10000;
double scaler_lin = 400;
int counter = 0;

//Six UR joints plus one for the gripper (must be the last)
int n_joints = 12;
//Five passive gripper joints that move according to the one controlled gripper joint
int n_passive_joints = 5;
double vel;
std::vector<double> vels;
bool get_time_stamp = false;
double difference;
rclcpp::Clock clk;
rclcpp::Time stamp;
rclcpp::Time eef_vc_stamp;
double time_pose_received;
std::vector<double> time_stamps;
int secs;
bool logging = false;

std::ofstream myfile;
std::string filename;
std::vector<std::vector<double>> history;

std::string reference_frame = "base_link";
geometry_msgs::msg::TransformStamped transform_ur_eef;
std::vector<double> tcp_position;
std::vector<double> ik_offsets = {0.025,-0.002,0.055};
//std::vector<double> ik_offsets = {0.0,-0.0,0.0};
std::vector<double> multipliers = {-1,-1,1};
bool trigger = false;
bool save_params_to_csv = false;
std::string dir_path = "docs/data/push_translation/";
//bool set_ur_joint_id = true;


//std::vector<std::string> ur_joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
/*std::map<std::string, int> map_joints_to_joint_states_id = {
    { "shoulder_pan_joint", 0 },
    { "shoulder_lift_joint", 13 },
    { "elbow_joint", 3 },
    { "wrist_1_joint", 8 },
    { "wrist_2_joint", 12 },
    { "wrist_3_joint", 11 },
    { "gripper_right_driver_joint", 9 }
};*/

std::map<std::string, int> map_joints_to_joint_states_id = {
    { "shoulder_pan_joint", 0 },
    { "shoulder_lift_joint", 1 },
    { "elbow_joint", 8 },
    { "wrist_1_joint", 2 },
    { "wrist_2_joint", 3 },
    { "wrist_3_joint", 4 },
    { "gripper_right_driver_joint", 5 },
    { "gripper_left_driver_joint", 7 },
    { "gripper_right_spring_link_joint", 6 },
    { "gripper_left_spring_link_joint", 11 },
    { "gripper_right_follower_joint", 9 },
    { "gripper_left_follower_joint", 10 }
};

std::vector<double> current_joint_configuration(n_joints,0.0);
std::vector<double> desired_joint_configuration(n_joints,0.0);
std::vector<double> diffs(n_joints,0.0);
//std::vector<double> set_joint_velocities(n_joints,0.0);
double dev2vel2(double deviation){
  double vel;
  if(deviation<0.0){
    vel = -scaler_square*pow(deviation,2);//std::exp()
  }
  else {
    vel = scaler_square*pow(deviation,2);
  }

  if(vel > vel_max){vel = vel_max;}
  else if(vel < -vel_max){vel = -vel_max;}
  return vel;
}

/*double dev2vel(double deviation){
  double vel = scaler_cubic*pow(deviation,3);//std::exp()
  if(vel > vel_max){vel = vel_max;}
  else if(vel < -vel_max){vel = -vel_max;}
  return vel;
}

double dev2vel3(double deviation){
  double vel;
  vel = scaler_lin*deviation;//std::exp()

  if(vel > vel_max){vel = vel_max;}
  else if(vel < -vel_max){vel = -vel_max;}
  return vel;
}*/


class VelocityPublisher : public rclcpp::Node
{
  public:
    VelocityPublisher() : Node("velocity_publisher")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rate, std::bind(&VelocityPublisher::topic_callback, this, _1));

      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_velocity_controller/commands", control_rate);
      timer_ = this->create_wall_timer(
      std::chrono::milliseconds(control_rate_ms), std::bind(&VelocityPublisher::timer_callback, this));

      service_ = this->create_service<custom_interfaces::srv::RobConf>("/velocity_controller/set_desired_joint_config", std::bind(&VelocityPublisher::set_desired_joint_config, this, _1, _2));

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }


  private:
    void topic_callback(const sensor_msgs::msg::JointState & msg) const
    {


      current_joint_configuration[0] = msg.position[map_joints_to_joint_states_id["shoulder_pan_joint"]];
      current_joint_configuration[1] = msg.position[map_joints_to_joint_states_id["shoulder_lift_joint"]];
      current_joint_configuration[2] = msg.position[map_joints_to_joint_states_id["elbow_joint"]];
      current_joint_configuration[3] = msg.position[map_joints_to_joint_states_id["wrist_1_joint"]];
      current_joint_configuration[4] = msg.position[map_joints_to_joint_states_id["wrist_2_joint"]];
      current_joint_configuration[5] = msg.position[map_joints_to_joint_states_id["wrist_3_joint"]];
      current_joint_configuration[6] = msg.position[map_joints_to_joint_states_id["gripper_right_driver_joint"]];
      current_joint_configuration[7] = msg.position[map_joints_to_joint_states_id["gripper_left_driver_joint"]];
      current_joint_configuration[8] = msg.position[map_joints_to_joint_states_id["gripper_right_spring_link_joint"]];
      current_joint_configuration[9] = msg.position[map_joints_to_joint_states_id["gripper_left_spring_link_joint"]];
      current_joint_configuration[10] = msg.position[map_joints_to_joint_states_id["gripper_right_follower_joint"]];
      current_joint_configuration[11] = msg.position[map_joints_to_joint_states_id["gripper_left_follower_joint"]];
      //RCLCPP_INFO(this->get_logger(), "Current joint configuration: '(%f, %f, %f, %f, %f, %f)'", current_joint_configuration[0], current_joint_configuration[1], current_joint_configuration[2], current_joint_configuration[3], current_joint_configuration[4], current_joint_configuration[5]);
      //RCLCPP_INFO(this->get_logger(), "Message size: %ld", msg.position.size());
      try {
        transform_ur_eef = tf_buffer_->lookupTransform(
          reference_frame, "wrist_3_link",
          tf2::TimePointZero);
        //time_eef = clk.now();
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          reference_frame.c_str(), "wrist_3_link", ex.what());
        return;
      }
      eef_vc_stamp = clk.now();
      tcp_position = {(transform_ur_eef.transform.translation.x*multipliers[0])-ik_offsets[0],(transform_ur_eef.transform.translation.y*multipliers[1])-ik_offsets[1],(transform_ur_eef.transform.translation.z*multipliers[2])-ik_offsets[2]};
    }

    void set_desired_joint_config(std::shared_ptr<custom_interfaces::srv::RobConf::Request>  req,
             std::shared_ptr<custom_interfaces::srv::RobConf::Response> res)
    {
      //scaler_square = req->vc_scaler;
      for(int i = 0; i < n_joints; i++)
      {
        if(i<7){
          desired_joint_configuration[i] = req->conf[i];
        }
        else if(i>=10){
          desired_joint_configuration[i] = -req->conf[6];
        }
        else{
          desired_joint_configuration[i] = req->conf[6];
        }
      }
      time_stamps = req->time_stamps;

      if(req->start){
        save_params_to_csv = true;
        filename = dir_path + std::to_string((int)time_stamps[1]) + "_times_" + std::to_string(scaler_square) + "vc_" + std::to_string(control_rate_ms)+ "ms.csv";
        myfile.open(filename);
        myfile << "trigger,secs_vc,secs_eef_vc,secs_eef,secs_sensed,secs_delayed,secs_ik,eef_vc_x,eef_vc_y,eef_vc_z,vels[0],vels[1],vels[2],vels[3],vels[4],vels[5],desired_conf[0],desired_conf[1],desired_conf[2],desired_conf[3],desired_conf[4],desired_conf[5],current_conf[0],current_conf[1],current_conf[2],current_conf[3],current_conf[4],current_conf[5]\n";
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "started logging...");
      }
      trigger = req->event_trigger;
      if(req->event_trigger){

        //RCLCPP_INFO(this->get_logger(), "Receiving event trigger with time stamp %f -> ready to get time stamp", time_pose_received);
        //save_params_to_csv = false;
        get_time_stamp = true;
      }
      if(save_params_to_csv){
        secs = time_stamps[1];
        //history.push_back({(float)trigger,(int)secs,stamp.seconds()-secs, eef_vc_stamp.seconds()-secs,time_stamps[2]-secs,time_stamps[3]-secs,time_stamps[4]-secs,time_stamps[5],tcp_position[0],tcp_position[1],tcp_position[2]});
        history.push_back({(float)trigger,stamp.seconds()-secs, eef_vc_stamp.seconds()-secs,time_stamps[2]-secs,time_stamps[3]-secs,time_stamps[4]-secs,time_stamps[5],tcp_position[0],tcp_position[1],tcp_position[2],vels[0],vels[1],vels[2],vels[3],vels[4],vels[5],desired_joint_configuration[0],desired_joint_configuration[1],desired_joint_configuration[2],desired_joint_configuration[3],desired_joint_configuration[4],desired_joint_configuration[5],current_joint_configuration[0],current_joint_configuration[1],current_joint_configuration[2],current_joint_configuration[3],current_joint_configuration[4],current_joint_configuration[5]});
      }
      /*if(req->dmp_mode){
        //call service offered by python node with client (in other thread?)
      }*/
      res->success = true;
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "new desired configuration is set to (%f, %f, %f, %f, %f, %f, %f)", req->conf[0], req->conf[1], req->conf[2], req->conf[3], req->conf[4], req->conf[5], req->conf[6]);

    }

    void timer_callback()
    {
      auto message = std_msgs::msg::Float64MultiArray();
      if(!got_initial_config)
      {
        desired_joint_configuration = current_joint_configuration;
        got_initial_config = true;
        RCLCPP_INFO(this->get_logger(), "Saving as desired configuration: '(%f, %f, %f, %f, %f, %f, %f)'", desired_joint_configuration[0], desired_joint_configuration[1], desired_joint_configuration[2], desired_joint_configuration[3], desired_joint_configuration[4], desired_joint_configuration[5], desired_joint_configuration[6]);
        /*int trajpoints = 100;
        double start_time = 0.0;
        double cycletime = 5.0;
        Eigen::VectorXd ts = Eigen::VectorXd::LinSpaced(trajpoints,start_time,start_time+cycletime);
        Eigen::VectorXd goal_pose_(n_goal_elements_);
        Eigen::VectorXd curr_pose_(n_goal_elements_);
        curr_pose_ << 0,0,0,0,0,0;
        goal_pose_ << 0.1,0,0.5,-1,0,0;


        Trajectory traj = Trajectory::generateMinJerkTrajectory(ts, curr_pose_, goal_pose_);
        if(save_traj_to_file)
        {
          traj.saveToFile("/","test.txt", true);
        }*/



      }

      // Implement herre different ways of reaching the desired configuration
      // 1. linear trajectory with set max_velocity
      // 2. along a DMP trajectory





      for(int i = 0; i<n_joints; i++)
      {
        //RCLCPP_INFO(this->get_logger(), "Saving as desired configuration: '(%f, %f, %f, %f, %f, %f)'", desired_joint_configuration[0], desired_joint_configuration[1], desired_joint_configuration[2], desired_joint_configuration[3], desired_joint_configuration[4], desired_joint_configuration[5]);
        diffs[i] = desired_joint_configuration[i] - current_joint_configuration[i];
        //RCLCPP_INFO(this->get_logger(), "Difference of joint %d: %f -> results in velocity: %f", i, difference, dev2vel(difference))

        vel = dev2vel2(diffs[i]);



        message.data.push_back(vel);


      }
      vels = message.data;




      //message.data = "Hello, world! " + std::to_string(count_++);

      //RCLCPP_INFO(this->get_logger(), "Publishing: '(%f, %f, %f, %f, %f, %f)'", message.data[0], message.data[1], message.data[2], message.data[3], message.data[4], message.data[5]);

      //if(get_time_stamp){
        stamp = clk.now();
        //RCLCPP_INFO(this->get_logger(), "Event trigger received! Sending to velocity controller at time %f -> elapsed time since receiving pose: %f", stamp.seconds(), stamp.seconds()-time_pose_received[1]);


      publisher_->publish(message);
      if(get_time_stamp){
        //time_pose_received.push_back(stamp.seconds());
        if(counter>=30){
          if(save_params_to_csv){
            for (int i = 0; i<(int)history.size();i++){
              for(int j=0;j<(int)history[0].size();j++){
                myfile << history[i][j] << ",";
              }
              myfile << "\n";
            }
            myfile.close();
            save_params_to_csv = false;
            RCLCPP_INFO(this->get_logger(), "Finished writing times to file to %s: %d, history size: %d",filename.c_str(),control_rate_ms,(int)history.size());
          }

          counter = 0;

          history.clear();
        }
        secs = time_stamps[1];
        history.push_back({(float)trigger,stamp.seconds()-secs, eef_vc_stamp.seconds()-secs,time_stamps[2]-secs,time_stamps[3]-secs,time_stamps[4]-secs,time_stamps[5],tcp_position[0],tcp_position[1],tcp_position[2],vels[0],vels[1],vels[2],vels[3],vels[4],vels[5],desired_joint_configuration[0],desired_joint_configuration[1],desired_joint_configuration[2],desired_joint_configuration[3],desired_joint_configuration[4],desired_joint_configuration[5],current_joint_configuration[0],current_joint_configuration[1],current_joint_configuration[2],current_joint_configuration[3],current_joint_configuration[4],current_joint_configuration[5]});
        get_time_stamp = false;
        counter ++;
        //RCLCPP_INFO(this->get_logger(), "Finished writing times to file: %d",secs);
      }
    }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Service<custom_interfaces::srv::RobConf>::SharedPtr service_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ {nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

/*
class JointStateSubscriber : public rclcpp::Node
{
  public:
    JointStateSubscriber()
    : Node("joint_state_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 100, std::bind(&JointStateSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::JointState & msg) const
    {
      for(int i = 0; i < n_joints; i++)
      {
        current_joint_configuration[i] = msg.position[i];
      }
      RCLCPP_INFO(this->get_logger(), "Current joint configuration: '(%f, %f, %f, %f, %f, %f)'", current_joint_configuration[0], current_joint_configuration[1], current_joint_configuration[2], current_joint_configuration[3], current_joint_configuration[4], current_joint_configuration[5]);

    }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);



  rclcpp::spin(std::make_shared<VelocityPublisher>());

  rclcpp::shutdown();
  return 0;
}
