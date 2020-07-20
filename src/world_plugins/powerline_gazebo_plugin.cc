#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h> 
#include <thread>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/pose.pb.h>
#include <gazebo/msgs/poses_stamped.pb.h>
#include "gazebo/physics/physics.hh"
#include "WGS84toCartesian.hpp"
namespace gazebo
{

  typedef boost::shared_ptr<const msgs::Pose>         GazeboPoseConstPtr;
  typedef boost::shared_ptr<const msgs::PosesStamped> GazeboPosesStampedConstPtr;

  class PowerlinePlugin : public WorldPlugin
  {

    public:
      void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    private:
      const unsigned int TILE_SIZE = 256;    
      const int _spawn_distance = 4;
      std::vector<ignition::math::Vector2d> _loaded_pylon_positions;
      std::vector<ignition::math::Vector2d> _spawned_pylon_positions;
      ignition::math::Vector2d center;
      int _prev_pylon_index = 0;
      physics::WorldPtr _world = NULL;
      ignition::math::Vector3d uav_position_;
      // Gazebo subscribers
      std::string file_path = "/home/mrs/workspace/src/aerialcore_simulation/gps/v223-v224_gps.txt";      
      
      transport::NodePtr       gz_node_;
      transport::SubscriberPtr gz_uav_sub_;
      transport::NodePtr gz_node_pub_;
      transport::PublisherPtr gz_msg_pub_;
      transport::PublisherPtr factory_pub_;
      // parameters
      bool exists = false;
      std::string _uav_name_;
      // callbacks
      void CallbackGazeboUavPose(const GazeboPosesStampedConstPtr& msg);

      void SpawnModel(std::string modelName, double lat, double lon, double rotation_rad);

      std::string getFileName(const std::string& s);

  };

  void PowerlinePlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    this->_world = _world;
    // get model that will be tracked
    if (!_sdf->HasElement("uav_name")) {
      _uav_name_ = "uav1";
      ROS_INFO("Use default reference frame: %s",_uav_name_.c_str());
    }else{
      _uav_name_ = _sdf->Get<std::string>("uav_name");
      ROS_INFO("Use set reference frame: %s",_uav_name_.c_str());
    }
    if (_sdf->HasElement("file_path"))
      file_path = _sdf->Get<std::string>("file_path");

  if (!_sdf->HasElement("center"))
  {
    gzerr << "Please specify latitude and longitude coordinates of map center"
          << std::endl;
    return;
  }

  this->center =
      _sdf->Get<ignition::math::Vector2d>("center");

    gz_node_ = transport::NodePtr(new transport::Node());
    gz_node_->Init();

    gz_node_pub_ = transport::NodePtr(new transport::Node());
    gz_node_pub_->Init();

    gz_msg_pub_ = gz_node_->Advertise<gazebo::msgs::Request>("~/request");
    factory_pub_ = gz_node_->Advertise<gazebo::msgs::Factory>("~/factory");
    gz_uav_sub_ = gz_node_->Subscribe("~/pose/info",&PowerlinePlugin::CallbackGazeboUavPose, this, 1);

    
// Create an input filestream
    std::ifstream csv_file(file_path);

    // Make sure the file is open
    if(!csv_file.is_open()) throw std::runtime_error("Could not open file");

    if(!csv_file.good()) throw std::runtime_error("I/O error encountered");
    
    std::string line;
    double lat, lon;

    std::array<double, 2> WGS84Reference{this->center.X(), this->center.Y()};
    std::array<double, 2> WGS84Position;
    //std::string file_name = this->getFileName(file_path);
    std::string file_name = _sdf->GetName();
    while(std::getline(csv_file, line))
    {
      std::stringstream line_stream(line);
      int index;
      line_stream >> index;
      line_stream >> lat;
      line_stream >> lon;
      WGS84Position = {lat,lon};
      std::array<double, 2> result{wgs84::toCartesian(WGS84Reference, WGS84Position)};

      SpawnModel(file_name, result[0], result[1], 0);
    }
    csv_file.close();
    
  }          

  void PowerlinePlugin::CallbackGazeboUavPose(const GazeboPosesStampedConstPtr& msg) {
    // here comes the spawning and despawning logic
    return;
  }

  void PowerlinePlugin::SpawnModel(std::string model_name, double x, double y, double rotation_rad)
  {
    
      sdf::SDF newSDF;
      std::stringstream newStream;

      newStream << "<sdf version ='1.5'>\n" <<
         "   <model name='"<< model_name <<"_"<<x<<"_"<<y<<"'>\n" <<
         "     <static>true</static>\n" <<
         "     <link name='link'>\n" <<
         "       <pose>"<<x<<" "<<y<<" 0 0 0 "<<rotation_rad<<"</pose>\n" <<
         "       <visual name='visual'>\n" <<
         "         <cast_shadows>false</cast_shadows>\n" <<
         "         <geometry>\n" <<
         "           <mesh>\n" <<
         "             <uri>model://power_tower_danube/meshes/power_tower_danube.dae</uri>\n" <<
         "           </mesh>\n" <<
         "         </geometry>\n" <<
         "       </visual>\n" <<
         "       <collision name='collision'>\n" <<
         "         <geometry>\n" <<
         "           <mesh>\n" <<
         "             <uri>model://power_tower_danube/meshes/power_tower_danube_lowpoly.dae</uri>\n" <<
         "           </mesh>\n" <<
         "         </geometry>\n" <<
         "       </collision>\n" <<
         "     </link>\n" <<
         "   </model>\n" <<
         "</sdf>";
      gazebo::msgs::Factory msgfac;
      gazebo::msgs::Init(msgfac, "spawn_model");
      msgfac.set_sdf(newStream.str());
      factory_pub_->Publish(msgfac);
  }

  std::string getFileName(const std::string& s) {

     char sep = '/';
     size_t i = s.rfind(sep, s.length());
     if (i != std::string::npos) {
        return(s.substr(i+1, s.length() - i));
     }

     return("");
  }

  GZ_REGISTER_WORLD_PLUGIN(PowerlinePlugin)
}

