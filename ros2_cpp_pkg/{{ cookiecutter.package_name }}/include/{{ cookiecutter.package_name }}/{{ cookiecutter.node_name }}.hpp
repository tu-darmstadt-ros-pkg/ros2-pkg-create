#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <template_interfaces_pkg/action/template_action.hpp>
#include <template_interfaces_pkg/msg/template_message.hpp>
#include <template_interfaces_pkg/srv/template_service.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


namespace {{ cookiecutter.package_name }} {


class {{ cookiecutter.node_cpp_class_name }} : public rclcpp::Node {

 public:
  {{ cookiecutter.node_cpp_class_name }}();

 protected:
  // constants
  static const std::string kInputTopic;
  static const std::string kOutputTopic;

  // parameter names
  static const std::string kPeriodParam;

 private:
  void loadParameters();

  void setup();

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters);

  void publishTimerCallback();

  void topicCallback(const template_interfaces_pkg::msg::TemplateMessage& msg);

  void serviceCallback(
    const std::shared_ptr<
      template_interfaces_pkg::srv::TemplateService::Request>
      request,
    std::shared_ptr<template_interfaces_pkg::srv::TemplateService::Response>
      response);

{% if cookiecutter.action %}
  rclcpp_action::GoalResponse actionHandleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const {{ cookiecutter.action }}::Goal>
      goal);

  rclcpp_action::CancelResponse actionHandleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
      {{ cookiecutter.action }}>>
      goal_handle);

  void actionHandleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
      {{ cookiecutter.action }}>>
      goal_handle);

  void actionExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                       {{ cookiecutter.action }}>>
                       goal_handle);
{% endif %}

 private:
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  rclcpp::Publisher<template_interfaces_pkg::msg::TemplateMessage>::SharedPtr
    publisher_;

  rclcpp::Subscription<template_interfaces_pkg::msg::TemplateMessage>::SharedPtr
    subscriber_;

  rclcpp::Service<template_interfaces_pkg::srv::TemplateService>::SharedPtr
    service_server_;

{% if cookiecutter.action %}
  rclcpp_action::Server<
    {{ cookiecutter.action }}>::SharedPtr action_server_;
{% endif %}

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  double period_ = 1.0;

  int count_ = 0;
};


}  // namespace {{ cookiecutter.package_name }}