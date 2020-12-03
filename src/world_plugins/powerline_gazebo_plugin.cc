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
      const int spawn_distance = 500;
      std::set<ignition::math::Vector3d> loaded_pylon_positions;
      std::set<ignition::math::Vector3d> spawned_pylon_positions;
      ignition::math::Vector2d center;
      int _prev_pylon_index = 0;
      int refresh_rate = 5;
      int prev_second = 0;
      physics::WorldPtr _world = NULL;
      ignition::math::Vector3d uav_position_;
      // Gazebo subscribers
      std::string file_path;      
      std::string model_name;
      std::string spawn_name;
      transport::NodePtr       gz_node_;
      transport::SubscriberPtr gz_uav_sub_;
      transport::NodePtr gz_node_pub_;
      transport::PublisherPtr gz_msg_pub_;
      transport::PublisherPtr factory_pub_;
      // parameters
      bool exists = false;
      std::string uav_name;
      // callbacks
      void CallbackGazeboUavPose(const GazeboPosesStampedConstPtr& msg);

      void SpawnModel(ignition::math::Vector3d position);
      void DespawnModel(ignition::math::Vector3d position);

      void CheckDistances(ignition::math::Vector3d uavPosition);
      std::string GetFilename(const std::string& s);

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
      uav_name = "uav1";
      ROS_INFO("Use default reference frame: %s",uav_name.c_str());
    }else{
      uav_name = _sdf->Get<std::string>("uav_name");
      ROS_INFO("Use set reference frame: %s",uav_name.c_str());
    }
    if (_sdf->HasElement("file_path"))
      file_path = _sdf->Get<std::string>("file_path");
    if (_sdf->HasElement("model_name"))
      model_name = _sdf->Get<std::string>("model_name");
    if (_sdf->HasElement("refresh_rate"))
      refresh_rate = _sdf->Get<int>("refresh_rate");

  if (!_sdf->HasElement("center"))
  {
    gzerr << "Please specify latitude and longitude coordinates of map center"
          << std::endl;
    return;
  }

  this->spawn_name = this->GetFilename(file_path);

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
    if(!csv_file.is_open()) throw std::runtime_error("Could not open file: "+file_path);

    if(!csv_file.good()) throw std::runtime_error("I/O error encountered");
    
    std::string line;
    double lat, lon;

    std::array<double, 2> WGS84Reference{this->center.X(), this->center.Y()};
    std::array<double, 2> WGS84Position;
    //std::string file_name = this->getFileName(file_path);
    std::string file_name = _sdf->GetName();
    
    std::vector<ignition::math::Vector3d> pylonsFromFile;

    while(std::getline(csv_file, line))
    {
      std::stringstream line_stream(line);
      std::string index;
      line_stream >> index;
      line_stream >> lat;
      line_stream >> lon;
      WGS84Position = {lat,lon};
      std::array<double, 2> result{wgs84::toCartesian(WGS84Reference, WGS84Position)};
      pylonsFromFile.push_back(ignition::math::Vector3d(result[0],result[1],0));
    }
    csv_file.close();
    int totalSize = pylonsFromFile.size();
 gzerr << "Loaded " << totalSize << "positions"; 
    ignition::math::Vector3d previousPylon;
    ignition::math::Vector3d currentPylon = pylonsFromFile.back();
    pylonsFromFile.pop_back();
    ignition::math::Vector3d nextPylon = pylonsFromFile.back();
    pylonsFromFile.pop_back();
 
    double rotation1 = std::atan2(nextPylon.Y()-currentPylon.Y(), nextPylon.X()-currentPylon.X());
    double rotation2;
    currentPylon.Z(rotation1);
    loaded_pylon_positions.insert(currentPylon);

      for (int i = 0; i <= totalSize-3; i++) {

        previousPylon = currentPylon;
        currentPylon = nextPylon;
        nextPylon = pylonsFromFile.back();
        pylonsFromFile.pop_back();
        rotation1 = std::atan2(nextPylon.Y()-currentPylon.Y(), nextPylon.X()-currentPylon.X());
        rotation2 = std::atan2(currentPylon.Y()-previousPylon.Y(), currentPylon.X()-previousPylon.X());
        currentPylon.Z((rotation1+rotation2)/2);
        loaded_pylon_positions.insert(currentPylon);
      }
      previousPylon = currentPylon;
      currentPylon = nextPylon;
      rotation2 = std::atan2(currentPylon.Y()-previousPylon.Y(), currentPylon.X()-previousPylon.X());
      currentPylon.Z(rotation2);
      loaded_pylon_positions.insert(currentPylon);
 gzerr << "Loaded all positions"; 

  }

  void PowerlinePlugin::CheckDistances(ignition::math::Vector3d uavPosition) {
    std::vector<ignition::math::Vector3d> addedPylons;   
    std::vector<ignition::math::Vector3d> removedPylons;
    double distance;
    ignition::math::Vector3d currentPylon;

  for (auto it = loaded_pylon_positions.begin(); it != loaded_pylon_positions.end(); ) {
    distance = std::sqrt(std::pow((uavPosition.X()-it->X()),2)+std::pow(uavPosition.X()-it->Y(),2));
    if(distance <= spawn_distance) {
    std::cout << "Spawning"<<std::endl;
      addedPylons.push_back(*it);
      this->SpawnModel(*it);
      it = loaded_pylon_positions.erase(it);
    }else{
      ++it;
      }
    }

  for (auto it = spawned_pylon_positions.begin(); it != spawned_pylon_positions.end(); ) {
    distance = std::sqrt(std::pow((uavPosition.X()-it->X()),2)+std::pow(uavPosition.X()-it->Y(),2));
    if(distance > spawn_distance) {
    std::cout << "Despawning"<<std::endl;
      removedPylons.push_back(*it);
      this->DespawnModel(*it);
      it = spawned_pylon_positions.erase(it);
    }else{
      ++it;
      }
    }
    for (auto pylon : addedPylons){
      spawned_pylon_positions.insert(pylon); 
    }
    for (auto pylon : removedPylons){
      loaded_pylon_positions.insert(pylon); 
    }
  }

  void PowerlinePlugin::CallbackGazeboUavPose(const GazeboPosesStampedConstPtr& msg) {
    // here comes the spawning and despawning logic
    
    int time = msg->time().sec(); 
    //std::cout << time << " " << time % refresh_rate << std::endl; 
    if (prev_second == time || time % refresh_rate != 0) {
     return; 
    }
    prev_second = time;

      bool exists = false;
      ignition::math::Vector3d uav_position_;
      for (int i = 0; i < msg->pose_size(); i++) {
        if (msg->pose(i).name() == this->uav_name) {
  /* std::cout << "found uav"<< std::endl; */
    exists = true;
          uav_position_.X() = msg->pose(i).position().x();
          uav_position_.Y() = msg->pose(i).position().y();
          uav_position_.Z() = msg->pose(i).position().z(); 
          break;
        }
      }

    if (exists)
      this->CheckDistances(uav_position_);


  }

  void PowerlinePlugin::SpawnModel(ignition::math::Vector3d position)
  {
    
      sdf::SDF newSDF;
      std::stringstream newStream;

      newStream << "<sdf version ='1.5'>\n" <<
         "   <model name='"<< this->spawn_name <<"_"<<position[0]<<"_"<<position[1]<<"'>\n" <<
         "     <static>true</static>\n" <<
         "     <link name='link'>\n" <<
         "       <pose>"<<position[0]<<" "<<position[1]<<" 0 0 0 "<<position[2]<<"</pose>\n" <<
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

      void PowerlinePlugin::DespawnModel(ignition::math::Vector3d position) {
      std::stringstream newStream; 
        newStream << this->model_name <<"_"<<position[0]<<"_"<<position[1];
msgs::Request *msg = gazebo::msgs::CreateRequest("entity_delete", newStream.str());
      this->gz_msg_pub_->Publish(*msg,true);
      delete msg;

      }
  std::string PowerlinePlugin::GetFilename(const std::string& s) {

     char sep = '/';
     size_t i = s.rfind(sep, s.length());
     if (i != std::string::npos) {
        return(s.substr(i+1, s.length() - i));
     }

     return("");
  }

  GZ_REGISTER_WORLD_PLUGIN(PowerlinePlugin)
}

