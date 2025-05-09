#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>

class MoveToPoseNode : public rclcpp::Node
{
public:
  MoveToPoseNode() : Node("move_to_pose_node")
  {
    // El constructor ya no crea el MoveGroup
  }

  void init()
  {
    // Ahora sí, ya puedes usar shared_from_this()
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "brazo_igus");

    // Suscribirse al topic donde se publican las poses
    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pose_target", 10,
      std::bind(&MoveToPoseNode::poseCallback, this, std::placeholders::_1));

    // Publisher de JointState
    pub_joints_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "📥 Recibida nueva pose objetivo");

    move_group_->setPoseTarget(*msg);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "✅ Planificación exitosa, ejecutando...");

      const std::vector<double>& joint_positions = plan.trajectory_.joint_trajectory.points.back().positions;
      const std::vector<std::string>& joint_names = plan.trajectory_.joint_trajectory.joint_names;

      sensor_msgs::msg::JointState joint_state_msg;
      joint_state_msg.header.stamp = this->now();
      joint_state_msg.name = joint_names;
      joint_state_msg.position = joint_positions;

      pub_joints_->publish(joint_state_msg);

      move_group_->execute(plan);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "⚠️ No se pudo planificar hacia la pose objetivo");
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Creamos el nodo con make_shared, y luego llamamos a init()
  auto node = std::make_shared<MoveToPoseNode>();
  node->init();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

