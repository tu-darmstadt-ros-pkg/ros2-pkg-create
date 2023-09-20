#include <rclcpp/rclcpp.hpp>
{% if cookiecutter.has_action_server %}
#include <rclcpp_action/rclcpp_action.hpp>
{% endif %}


namespace {{ cookiecutter.package_name }} {


class {{ cookiecutter.node_cpp_class_name }} : public rclcpp::Node {

 public:

  {{ cookiecutter.node_cpp_class_name }}();

 private:

{% if cookiecutter.has_subscriber %}
  static const std::string kInputTopic;
{% endif %}
{% if cookiecutter.has_publisher %}
  static const std::string kOutputTopic;
{% endif %}
{% if cookiecutter.has_params %}
  static const std::string kParam;
{% endif %}

 private:

{% if cookiecutter.has_params %}
  void loadParameters();

{% endif %}
  void setup();

{% if cookiecutter.has_params %}
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

{% endif %}
{% if cookiecutter.has_subscriber %}
  void topicCallback(const template_interfaces_pkg::msg::TemplateMessage& msg);

{% endif %}
{% if cookiecutter.has_service_server %}
  void serviceCallback(const std::shared_ptr<template_interfaces_pkg::srv::TemplateService::Request>request, std::shared_ptr<template_interfaces_pkg::srv::TemplateService::Response> response);

{% endif %}
{% if cookiecutter.has_action_server %}
  rclcpp_action::GoalResponse actionHandleGoal(const rclcpp_action::GoalUUID& uuid,std::shared_ptr<const template_interfaces_pkg::action::TemplateAction::Goal> goal);
  rclcpp_action::CancelResponse actionHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<template_interfaces_pkg::action::TemplateAction>> goal_handle);
  void actionHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<template_interfaces_pkg::action::TemplateAction>> goal_handle);
  void actionExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<template_interfaces_pkg::action::TemplateAction>> goal_handle);

{% endif %}
{% if cookiecutter.has_timer %}
  void publishTimerCallback();

{% endif %}
 private:

{% if cookiecutter.has_params %}
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

{% endif %}
{% if cookiecutter.has_subscriber %}
  rclcpp::Subscription<template_interfaces_pkg::msg::TemplateMessage>::SharedPtr subscriber_;

{% endif %}
{% if cookiecutter.has_publisher %}
  rclcpp::Publisher<template_interfaces_pkg::msg::TemplateMessage>::SharedPtr publisher_;

{% endif %}
{% if cookiecutter.has_service_server %}
  rclcpp::Service<template_interfaces_pkg::srv::TemplateService>::SharedPtr service_server_;

{% endif %}
{% if cookiecutter.has_action_server %}
  rclcpp_action::Server<template_interfaces_pkg::action::TemplateAction>::SharedPtr action_server_;

{% endif %}
{% if cookiecutter.has_timer %}
  rclcpp::TimerBase::SharedPtr publish_timer_;

{% endif %}
  double period_ = 1.0;

  int count_ = 0;
};


}
