#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose.hpp>
//#include <gazebo_msgs/msg/entity_state.hpp>
//#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


#include <custom_interfaces/srv/set_obj_id.hpp>
#include <custom_interfaces/msg/pose_array.hpp>


#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <iostream>
#include <string.h>

//#include <tf2/convert.h>
//#include <tf2/LinearMath/Transform.h>
//#include <geometry_msgs/msg/transform_stamped.h>



using namespace std::chrono_literals;
using namespace std::placeholders;

bool gazebo_model_id_set = false;

/*std::map<std::string, int> map_models_to_gazebo_id = {
    { "cube_tag0_grey", 9 },
    { "cube_tag1_violet", 5 },
    { "cube_tag2_indigo", 6 },
    { "cube_tag3_blue", 1 },
    { "cube_tag4_green", 7 },
    { "cube_tag5_yellow", 2 },
    { "cube_tag6_orange", 3 },
    { "cube_tag7_red", 4 }
};*/

class ObjectInfo {
private:
    std::string objPath;
    unsigned int id;
    std::string frame_id;
    std::string name;
    geometry_msgs::msg::Pose pose;
    double r;
    double g;
    double b;
    double scale_x;
    double scale_y;
    double scale_z;

    bool use_embedded_materials;
public:
    inline void setObjPath(std::string p) {objPath=p;}
    inline std::string getObjPath() {return objPath;}
    inline unsigned int getTagID() {return id;}
    inline unsigned int getGazeboModelID() {return id;}
    inline std::string get_frame_id() {return frame_id;}
    inline std::string getname() {return name;}
    inline double getx() {return pose.position.x;}
    inline double gety() {return pose.position.y;}
    inline double getz() {return pose.position.z;}
    inline double getqx() {return pose.orientation.x;}
    inline double getqy() {return pose.orientation.y;}
    inline double getqz() {return pose.orientation.z;}
    inline double getqw() {return pose.orientation.w;}
    inline bool get_use_embeded_materials() {return use_embedded_materials;}
    inline double getr() {return r;}
    inline double getg() {return g;}
    inline double getb() {return b;}
    inline double getscale_x() {return scale_x;}
    inline double getscale_y() {return scale_y;}
    inline double getscale_z() {return scale_z;}

    inline geometry_msgs::msg::Pose getPose() {return pose;}

    inline void setTagID(unsigned int i) {id=i;}
    inline void setGazeboModelID(unsigned int i) {id=i;}
    inline void set_frame_id(std::string s) {frame_id=s;}
    inline void setname(std::string s) {name=s;}
    inline void setPose(geometry_msgs::msg::Pose g) {pose=g;}
    inline void setx(double v) {pose.position.x=v;}
    inline void sety(double v) {pose.position.y=v;}
    inline void setz(double v) {pose.position.z=v;}
    inline void setqx(double v) {pose.orientation.x=v;}
    inline void setqy(double v) {pose.orientation.y=v;}
    inline void setqz(double v) {pose.orientation.z=v;}
    inline void setqw(double v) {pose.orientation.w=v;}
    inline void setr(double v) {r=v;}
    inline void setg(double v) {g=v;}
    inline void setb(double v) {b=v;}
    inline void setscale_x(double v) {scale_x=v;}
    inline void setscale_y(double v) {scale_y=v;}
    inline void setscale_z(double v) {scale_z=v;}

    inline void set_use_embedded_materials(bool t) {use_embedded_materials = t;}
};

std::vector<ObjectInfo> objects;

void SetWorld()
{
    ObjectInfo obj;

    objects.clear();



    //pawn dimension 0.03x0.03x0.04
    //pawnB1ntityState
    obj.setObjPath("package://main_pkg/meshes/cube_tag0_grey.dae");
    obj.setTagID(0);
    obj.setGazeboModelID(0);
    obj.setname("cube_tag0_grey");
    obj.set_frame_id("base_link");
    obj.setx(0.5);
    obj.sety(0.2);
    obj.setz(0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(0);
    obj.setscale_x(0.025);//3.0/5.0
    obj.setscale_y(0.025);//3.0/5.0
    obj.setscale_z(0.025);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag1_violet.dae");
    obj.setTagID(1);
    obj.setGazeboModelID(1);
    obj.setname("cube_tag1_violet");
    obj.set_frame_id("base_link");
    obj.setx(0.4);
    obj.sety(0.2);
    obj.setz(0.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(0);
    obj.setscale_x(0.025);//3.0/5.0
    obj.setscale_y(0.025);//3.0/5.0
    obj.setscale_z(0.025);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag2_indigo.dae");
    obj.setTagID(2);
    obj.setGazeboModelID(2);
    obj.setname("cube_tag2_indigo");
    obj.set_frame_id("base_link");
    obj.setx(0.3);
    obj.sety(0.2);
    obj.setz(0.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(0);
    obj.setscale_x(0.025);//3.0/5.0cube_tag4_green
    obj.setscale_y(0.025);//3.0/5.0
    obj.setscale_z(0.025);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag3_blue.dae");
    obj.setTagID(3);
    obj.setGazeboModelID(3);
    obj.setname("cube_tag3_blue");
    obj.set_frame_id("base_link");
    obj.setx(0.5);
    obj.sety(0.1);
    obj.setz(0.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(0);
    obj.setscale_x(0.025);//3.0/5.0
    obj.setscale_y(0.025);//3.0/5.0
    obj.setscale_z(0.025);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag4_green.dae");
    obj.setTagID(4);
    obj.setGazeboModelID(4);
    obj.setname("cube_tag4_green");
    obj.set_frame_id("base_link");
    obj.setx(0.4);
    obj.sety(0.1);
    obj.setz(0.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(0);
    obj.setscale_x(0.025);//3.0/5.0
    obj.setscale_y(0.025);//3.0/5.0
    obj.setscale_z(0.025);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag5_yellow.dae");
    obj.setTagID(5);
    obj.setGazeboModelID(5);
    obj.setname("cube_tag5_yellow");
    obj.set_frame_id("base_link");
    obj.setx(0.3);
    obj.sety(0.1);
    obj.setz(0.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(0);
    obj.setscale_x(0.025);//3.0/5.0
    obj.setscale_y(0.025);//3.0/5.0
    obj.setscale_z(0.025);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag6_orange.dae");
    obj.setTagID(6);
    obj.setGazeboModelID(6);
    obj.setname("cube_tag6_orange");
    obj.set_frame_id("base_link");
    obj.setx(0.5);
    obj.sety(0.0);
    obj.setz(0.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(0);
    obj.setscale_x(0.025);//3.0/5.0
    obj.setscale_y(0.025);//3.0/5.0
    obj.setscale_z(0.025);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.9);
    obj.setg(0.9);
    obj.setb(0.9);
    objects.push_back(obj);

    obj.setObjPath("package://main_pkg/meshes/cube_tag7_red.dae");
    obj.setTagID(7);
    obj.setGazeboModelID(7);
    obj.setname("cube_tag7_red");
    obj.set_frame_id("base_link");
    obj.setx(0.4);
    obj.sety(0.0);
    obj.setz(0.0);//reference frame defined in the middle point
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(0);
    obj.setscale_x(0.025);//3.0/5.0
    obj.setscale_y(0.025);//3.0/5.0
    obj.setscale_z(0.025);//4.0/5.0
    obj.set_use_embedded_materials(true); //colors set in the dae fil
    obj.setr(0.0);
    obj.setg(0.0);
    obj.setb(0.9);
    objects.push_back(obj);



    //set the index maps
    //kautham indices are automtically set according to the order in which they are written in the kautham problem xml file
    //rviz indices are set in the order they are loaded in the objects vector
    //rviz indices have been made coincident to kautham indices, although they could differ
    //the following maps relate the aruco markers with the rviz and kautham indices
/*
    for(int i=0; i<objects.size();i++){
        //mymap.insert ( std::pair<char,int>('a',100) );
        aruco2kautham_map.insert(std::pair<unsigned int,std::string>(objects[i].getArucoID(), objects[i].getKauthamName()));
        kautham2aruco_map.insert(std::pair<std::string,unsigned int>(objects[i].getKauthamName(), objects[i].getArucoID()));
        aruco2rviz_map.insert(std::pair<unsigned int,unsigned int>(objects[i].getArucoID(), i));
        rviz2aruco_map.insert(std::pair<unsigned int,unsigned int>(i, objects[i].getArucoID()));
        aruco2ff_map.insert(std::pair<unsigned int,std::string>(objects[i].getArucoID(), objects[i].getname()));
        ff2aruco_map.insert(std::pair<std::string,unsigned int>(objects[i].getname(),objects[i].getArucoID()));
    }
    */
}






class WorldSetup : public rclcpp::Node
{


  public:
    WorldSetup() : Node("world_setup")//, state_service_ready_(false), state_received_(false)
    {
      SetWorld();
      RCLCPP_INFO(this->get_logger(), "World is setup! Loaded %d object(s)", (int)objects.size());
      //subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      //"/joint_states", rate, std::bind(&WorldSetup::topic__callback, this, _1));
      topic_ = "visualization_marker_array";
      rate_ = 1;
      attached_  = -1;


      publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_, rate_);
      //timer_ = this->create_wall_timer(
      //std::chrono::milliseconds(1000/rate_), std::bind(&WorldSetup::update_marker_array, this));

      //state_subscriber_ = this->create_subscription<custom_interfaces::msg::PoseArray>(
      //"/gazebo_poses", rate_, std::bind(&WorldSetup::update_marker_array, this, _1));

      state_subscriber_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
      "/gazebo_state/model_states", rate_, std::bind(&WorldSetup::update_marker_array, this, _1));

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      //auto publisher_ = node->create_publisher<visualization_msgs::msg::Marker>(topic_, 1000);
      /*while (!state_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          exit(1);
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }*/

      //attach_service_ = this->create_service<custom_interfaces::srv::SetObjId>("/world_setup/attach_obj", std::bind(&WorldSetup::attach_obj, this, _1, _2));
      //detach_service_ = this->create_service<custom_interfaces::srv::SetObjId>("/world_setup/detach_obj", std::bind(&WorldSetup::detach_obj, this, _1, _2));

      /*RCLCPP_INFO(this->get_logger(), "Waiting for Rviz to load...");

      while(this->get_node_graph_interface()->count_subscribers(topic_) == 0) {
          rclcpp::sleep_for(200ms);
      }*/

      //auto result = std::make_shared<gazebo_msgs::srv::GetEntityState::Response>();


    }


  private:

    void update_marker_array(gazebo_msgs::msg::ModelStates::SharedPtr msg)
    {
      if(!gazebo_model_id_set){
        std::vector<std::string> names = msg->name;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "names length: %s!", names[0].c_str());
        for (int i = 0; i<(int)names.size();i++){
          for (int j=0; j<(int)objects.size();j++){

            if(objects[j].getname().compare(names[i])==0){
              objects[j].setGazeboModelID(i);
              //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Found an equal name! -> (i:%d, j:%d)", i, j);
            }
          }
        }

        gazebo_model_id_set = true;

      }

        visualization_msgs::msg::Marker marker;
        visualization_msgs::msg::MarkerArray markers;

        for(int i=0; i<(int)objects.size(); i++)
        {


          objects[i].setPose(msg->pose[objects[i].getGazeboModelID()]);
                //objects[i].setx(result->state.pose.position.x);
                //objects[i].sety(result->state.pose.position.y);
                //objects[i].setz(result->state.pose.position.z);
                //objects[i].setqx(result->state.pose.orientation.x);
                //objects[i].setqy(result->state.pose.orientation.y);
                //objects[i].setqz(result->state.pose.orientation.z);
                //objects[i].setqw(result->state.pose.orientation.w);


            //marker.header.frame_id = "chess_frame";
            marker.header.frame_id = objects[i].get_frame_id();
            //marker.header.stamp = node->now();
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "cubes";
            marker.id = objects[i].getTagID();
            marker.mesh_resource = objects[i].getObjPath();

            marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = objects[i].getPose();

            marker.scale.x = objects[i].getscale_x();
            marker.scale.y = objects[i].getscale_y();
            marker.scale.z = objects[i].getscale_z();
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = objects[i].getr();
            marker.color.g = objects[i].getg();
            marker.color.b = objects[i].getb();
            marker.mesh_use_embedded_materials = objects[i].get_use_embeded_materials();//if true the r,g,b peviously defined are overriden

            markers.markers.push_back(marker);
        }
        //double x = objects[3].getx();
        //double y = objects[3].gety();
        //RCLCPP_INFO(this->get_logger(), "Updating cube poses in rviz at %d Hz (%f,%f)", rate_, x,y);

        publisher_->publish(markers);


    }
    //rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    bool state_service_ready_;
    bool state_received_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::Service<custom_interfaces::srv::SetObjId>::SharedPtr attach_service_;
    rclcpp::Service<custom_interfaces::srv::SetObjId>::SharedPtr detach_service_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr state_subscriber_;
    //rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr state_client_;
    std::string topic_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;//{nullptr}
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    int rate_;
    int attached_;
    geometry_msgs::msg::Pose pose_response_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<WorldSetup>());

  rclcpp::shutdown();
  return 0;
}
