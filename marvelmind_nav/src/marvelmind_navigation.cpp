#include "marvelmind_nav/marvelmind_navigation.hpp"

uint32_t MarvelmindNavigation::hedge_timestamp_prev = 0;
sem_t *MarvelmindNavigation::sem = nullptr;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MarvelmindNavigation::on_activate(const rclcpp_lifecycle::State &)
{
 return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MarvelmindNavigation::on_deactivate(const rclcpp_lifecycle::State &)
{
return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MarvelmindNavigation::on_error(const rclcpp_lifecycle::State &)
{
return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MarvelmindNavigation::on_cleanup(const rclcpp_lifecycle::State &)
{
return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MarvelmindNavigation::on_configure(const rclcpp_lifecycle::State &)
{
return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MarvelmindNavigation::on_shutdown(const rclcpp_lifecycle::State &)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void MarvelmindNavigation::semCallback()
{
  sem_post(sem);
}



