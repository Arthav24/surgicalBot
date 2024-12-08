#include <ignition/gazebo6/PhysicsEnginePlugin.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/common/Events.hh>

namespace surgicalBot
{
  class EndEffectorPositionPlugin : public ignition::gazebo::PhysicsEnginePlugin
  {
  public:
    EndEffectorPositionPlugin() : node_(nullptr), publisher_(nullptr) {}

    ~EndEffectorPositionPlugin() {}

    // Load function for the plugin
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;
      this->node_ = std::make_shared<rclcpp::Node>("end_effector_position_publisher");

      // Create a ROS2 publisher to publish the end-effector position
      this->publisher_ = node_->create_publisher<geometry_msgs::msg::Point>("end_effector_position", 10);

      // Set up a Gazebo update connection
      this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&EndEffectorPositionPlugin::OnUpdate, this));

      // Start ROS2 spinning in a separate thread
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node_);
      std::thread([&executor]() { executor.spin(); }).detach();
    }

  private:
    void OnUpdate()
    {
      // Get the end-effector position from the model (for simplicity, assume it's the last link)
      gazebo::physics::LinkPtr end_effector_link = this->model->GetLink("gripper_right_link");

      if (end_effector_link)
      {
        // Get the position of the end-effector (world frame)
        ignition::math::Pose3d pose = end_effector_link->WorldPose();
        
        // Convert the Gazebo pose to ROS2 geometry_msgs::msg::Point
        geometry_msgs::msg::Point end_effector_position;
        end_effector_position.x = pose.Pos().X();
        end_effector_position.y = pose.Pos().Y();
        end_effector_position.z = pose.Pos().Z();

        // Publish the end-effector position
        publisher_->publish(end_effector_position);
      }
    }

  private:
    gazebo::physics::ModelPtr model;
    gazebo::event::ConnectionPtr update_connection_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
  };

} // namespace surgicalBot

// Register the plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(surgicalBot::EndEffectorPositionPlugin)
