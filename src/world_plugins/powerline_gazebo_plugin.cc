#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h> 
#include <thread>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/pose.pb.h>
#include <gazebo/msgs/poses_stamped.pb.h>
#include "gazebo/physics/physics.hh"

namespace gazebo
{

  typedef boost::shared_ptr<const msgs::Pose>         GazeboPoseConstPtr;
  typedef boost::shared_ptr<const msgs::PosesStamped> GazeboPosesStampedConstPtr;

  class PowerlinePlugin : public WorldPlugin
  {

    public:
      void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    private:
      const int _spawn_distance = 4;
      const int _despawn_distance = 5;
      const int _pylon_distancing = 40;
      std::vector<int> _spawned_pylon_indexes;
      int _prev_pylon_index = 0;
      physics::WorldPtr _world = NULL;
      ignition::math::Vector3d uav_position_;
      // Gazebo subscribers
      std::string file_name = "V223-224 ISPD Stozary pro vyjadrovani 2020-06-25-14-00-59.csv";      
      
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

    gz_node_ = transport::NodePtr(new transport::Node());
    gz_node_->Init();

    gz_node_pub_ = transport::NodePtr(new transport::Node());
    gz_node_pub_->Init();

    gz_msg_pub_ = gz_node_->Advertise<gazebo::msgs::Request>("~/request");
    factory_pub_ = gz_node_->Advertise<gazebo::msgs::Factory>("~/factory");
    gz_uav_sub_ = gz_node_->Subscribe("~/pose/info",&PowerlinePlugin::CallbackGazeboUavPose, this, 1);

    for(int i=-_spawn_distance; i<= _spawn_distance; i++) {
      sdf::SDF pylonSDF;
      pylonSDF.SetFromString(
          "<sdf version ='1.5'>\
            <model name='power_tower_danube"+std::to_string(i)+"'>\
              <static>true</static>\
              <link name='link'>\
                <pose>"+std::to_string(i*_pylon_distancing)+" 0 0 0 0 0</pose>\
                <visual name='visual'>\
                  <cast_shadows>false</cast_shadows>\
                  <geometry>\
                    <mesh>\
                      <uri>model://power_tower_danube/meshes/power_tower_danube.dae</uri>\
                    </mesh>\
                  </geometry>\
                </visual>\
                <collision name='collision'>\
                  <geometry>\
                    <mesh>\
                      <uri>model://power_tower_danube/meshes/power_tower_danube_lowpoly.dae</uri>\
                    </mesh>\
                  </geometry>\
                </collision>\
              </link>\
            </model>\
          </sdf>");
      _world->InsertModelSDF(pylonSDF);

      sdf::SDF grassSDF;
      grassSDF.SetFromString(
  "<sdf version ='1.5'>\
    <model name='ground_plane"+std::to_string(i)+"'>\
      <static>true</static>\
      <link name='link'>\
        <collision name='collision'>\
          <pose>"+std::to_string(i*_pylon_distancing)+" 0 0 0 0 0</pose>\
          <geometry>\
            <plane>\
              <normal>0 0 1</normal>\
              <size>"+std::to_string(_pylon_distancing/2)+" "+std::to_string(_pylon_distancing/2)+"</size>\
            </plane>\
          </geometry>\
          <surface>\
            <friction>\
              <ode>\
                <mu>1</mu>\
                <mu2>1</mu2>\
              </ode>\
            </friction>\
          </surface>\
        </collision>\
        <visual name='grass'>\
          <pose frame=''>"+std::to_string(i*_pylon_distancing)+" 0 0 0 0 0</pose>\
          <cast_shadows>false</cast_shadows>\
          <geometry>\
            <mesh>\
              <uri>file://grass_plane/meshes/grass_plane.dae</uri>\
              <scale>"+std::to_string(0.004*_pylon_distancing)+" "+std::to_string(0.004*_pylon_distancing)+" 1</scale>\
            </mesh>\
          </geometry>\
        </visual>\
      </link>\
    </model>\
  </sdf>");
      _world->InsertModelSDF(grassSDF);
 
      sdf::SDF wireSDF;
      wireSDF.SetFromString(
  "<sdf version ='1.5'>\
    <model name='power_tower_danube_wires"+std::to_string(i)+"'>\
      <static>true</static>\
      <link name='link'>\
        <pose>"+std::to_string(i*_pylon_distancing)+" 0 0 0 0 0</pose>\
        <visual name='visual'>\
          <cast_shadows>false</cast_shadows>\
          <geometry>\
            <mesh>\
              <uri>model://power_tower_danube_wires/meshes/power_tower_danube_wires.dae</uri>\
              <scale>1 1 1</scale>\
            </mesh>\
          </geometry>\
        </visual>\
        <collision name='collision'>\
          <geometry>\
            <mesh>\
              <uri>model://power_tower_danube_wires/meshes/power_tower_danube_wires.dae</uri>\
              <scale>1 1 1</scale>\
            </mesh>\
          </geometry>\
        </collision>\
      </link>\
    </model>\
  </sdf>");
      _world->InsertModelSDF(wireSDF);
    }
  }          

  void PowerlinePlugin::CallbackGazeboUavPose(const GazeboPosesStampedConstPtr& msg) {
exists = false;
    for (int i = 0; i < msg->pose_size(); i++) {
      if (msg->pose(i).name() == _uav_name_) {
/* std::cout << "found uav"<< std::endl; */
exists = true;
        uav_position_.X() = msg->pose(i).position().x();
        uav_position_.Y() = msg->pose(i).position().y();
        uav_position_.Z() = msg->pose(i).position().z(); 
        break;
      }
    }

if (!exists)
  return;

    // here comes the spawning and despawning logic

    int pylonIndex = (int)uav_position_.X() / _pylon_distancing;
    if(pylonIndex != _prev_pylon_index) {
      int newSpawnIndex = 0;
      int newDespawnIndex = 0;
      if(pylonIndex>_prev_pylon_index){
        newSpawnIndex = pylonIndex + _spawn_distance;
        newDespawnIndex = pylonIndex - _despawn_distance;
      }else{
        newSpawnIndex = pylonIndex - _spawn_distance;
        newDespawnIndex = pylonIndex + _despawn_distance;
      }
      sdf::SDF newSDF;

      newSDF.SetFromString(
          "<sdf version ='1.5'>\
            <model name='power_tower_danube"+std::to_string(newSpawnIndex)+"'>\
              <static>true</static>\
              <link name='link'>\
                <pose>"+std::to_string(newSpawnIndex*_pylon_distancing)+" 0 0 0 0 0</pose>\
                <visual name='visual'>\
                  <cast_shadows>false</cast_shadows>\
                  <geometry>\
                    <mesh>\
                      <uri>model://power_tower_danube/meshes/power_tower_danube.dae</uri>\
                    </mesh>\
                  </geometry>\
                </visual>\
                <collision name='collision'>\
                  <geometry>\
                    <mesh>\
                      <uri>model://power_tower_danube/meshes/power_tower_danube_lowpoly.dae</uri>\
                    </mesh>\
                  </geometry>\
                </collision>\
              </link>\
            </model>\
          </sdf>");
      //this->_world->InsertModelSDF(newSDF);
      gazebo::msgs::Factory msgfac;
      gazebo::msgs::Init(msgfac, "spawn_model");
      msgfac.set_sdf(newSDF.ToString());
      factory_pub_->Publish(msgfac);

      sdf::SDF grassSDF;
      grassSDF.SetFromString(
        "<sdf version ='1.5'>\
          <model name='ground_plane"+std::to_string(newSpawnIndex)+"'>\
            <static>true</static>\
            <link name='link'>\
              <collision name='collision'>\
                <pose>"+std::to_string(newSpawnIndex*_pylon_distancing)+" 0 0 0 0 0</pose>\
                <geometry>\
                  <plane>\
                    <normal>0 0 1</normal>\
                    <size>"+std::to_string(_pylon_distancing/2)+" "+std::to_string(_pylon_distancing/2)+"</size>\
                  </plane>\
                </geometry>\
                <surface>\
                  <friction>\
                    <ode>\
                      <mu>1</mu>\
                      <mu2>1</mu2>\
                    </ode>\
                  </friction>\
                </surface>\
              </collision>\
              <visual name='grass'>\
                <pose frame=''>"+std::to_string(newSpawnIndex*_pylon_distancing)+" 0 0 0 0 0</pose>\
                <cast_shadows>false</cast_shadows>\
                <geometry>\
                  <mesh>\
                    <uri>file://grass_plane/meshes/grass_plane.dae</uri>\
                    <scale>"+std::to_string(0.004*_pylon_distancing)+" "+std::to_string(0.004*_pylon_distancing)+" 1</scale>\
                  </mesh>\
                </geometry>\
              </visual>\
            </link>\
          </model>\
        </sdf>");
      
      gazebo::msgs::Factory grassfac;
      gazebo::msgs::Init(grassfac, "spawn_model");
      grassfac.set_sdf(grassSDF.ToString());
      factory_pub_->Publish(grassfac);

      sdf::SDF wireSDF;
      wireSDF.SetFromString(
  "<sdf version ='1.5'>\
    <model name='power_tower_danube_wires"+std::to_string(newSpawnIndex)+"'>\
      <static>true</static>\
      <link name='link'>\
        <pose>"+std::to_string(newSpawnIndex*_pylon_distancing)+" 0 0 0 0 0</pose>\
        <visual name='visual'>\
          <cast_shadows>false</cast_shadows>\
          <geometry>\
            <mesh>\
              <uri>model://power_tower_danube_wires/meshes/power_tower_danube_wires.dae</uri>\
              <scale>1 1 1</scale>\
            </mesh>\
          </geometry>\
        </visual>\
        <collision name='collision'>\
          <geometry>\
            <mesh>\
              <uri>model://power_tower_danube_wires/meshes/power_tower_danube_wires.dae</uri>\
              <scale>1 1 1</scale>\
            </mesh>\
          </geometry>\
        </collision>\
      </link>\
    </model>\
  </sdf>");

      gazebo::msgs::Factory wirefac;
      gazebo::msgs::Init(wirefac, "spawn_model");
      wirefac.set_sdf(wireSDF.ToString());
      factory_pub_->Publish(wirefac);
      
      gazebo::msgs::Request *delete_pylon_msg = gazebo::msgs::CreateRequest("entity_delete", "power_tower_danube"+std::to_string(newDespawnIndex));
      gz_msg_pub_->Publish(*delete_pylon_msg,true);
      delete delete_pylon_msg;

      gazebo::msgs::Request *delete_grass_msg = gazebo::msgs::CreateRequest("entity_delete", "ground_plane"+std::to_string(newDespawnIndex));
      gz_msg_pub_->Publish(*delete_grass_msg,true);
      delete delete_grass_msg;

      gazebo::msgs::Request *delete_wire_msg = gazebo::msgs::CreateRequest("entity_delete", "power_tower_danube_wires"+std::to_string(newDespawnIndex));
      gz_msg_pub_->Publish(*delete_grass_msg,true);
      delete delete_wire_msg;

      _prev_pylon_index = pylonIndex;
    }
  }

  void PowerlinePlugin::SpawnModel(std::string modelName, double x, double y, double rotation_rad)
  {
    
      sdf::SDF newSDF;
      std::stringstream newStream;

      newStream << "<sdf version ='1.5'>" <<
         "   <model name='"<< modelName <<"_"<<x<<"_"<<y<<"'>" <<
         "     <static>true</static>" <<
         "     <link name='link'>" <<
         "       <pose>"<<x<<" "<<y<<" 0 0 0 "<<rotation_rad<<"</pose>" <<
         "       <visual name='visual'>" <<
         "         <cast_shadows>false</cast_shadows>" <<
         "         <geometry>" <<
         "           <mesh>" <<
         "             <uri>model://power_tower_danube/meshes/power_tower_danube.dae</uri>" <<
         "           </mesh>" <<
         "         </geometry>" <<
         "       </visual>" <<
         "       <collision name='collision'>" <<
         "         <geometry>" <<
         "           <mesh>" <<
         "             <uri>model://power_tower_danube/meshes/power_tower_danube_lowpoly.dae</uri>" <<
         "           </mesh>" <<
         "         </geometry>" <<
         "       </collision>" <<
         "     </link>" <<
         "   </model>" <<
         "</sdf>";
      gazebo::msgs::Factory msgfac;
      gazebo::msgs::Init(msgfac, "spawn_model");
      msgfac.set_sdf(newStream.str());
      factory_pub_->Publish(msgfac);
  }
  GZ_REGISTER_WORLD_PLUGIN(PowerlinePlugin)
}


