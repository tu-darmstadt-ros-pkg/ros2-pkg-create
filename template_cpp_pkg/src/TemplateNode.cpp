#include <math.h>

#include <chrono>
#include <functional>
#include <thread>

#include <template_cpp_pkg/TemplateNode.hpp>


/**
 * @brief Namespace for C++ template node
 * 
 */
namespace template_cpp_pkg {


// parameter names
const std::string TemplateNode::kPeriodParam = "period";

// constants
const std::string TemplateNode::kInputTopic = "~/topic";
const std::string TemplateNode::kOutputTopic = "~/topic";


/**
 * @brief Creates the template node inheriting from the Node class.
 * 
 */
TemplateNode::TemplateNode() : Node("template_node") {

  this->loadParameters();
  this->setup();
}

/**
 * @brief Loads ROS parameters used in the node.
 * 
 */
void TemplateNode::loadParameters() {

  // set parameter description
  rcl_interfaces::msg::ParameterDescriptor period_desc;
  period_desc.description = "Period between published messages";

  // set allowed parameter range
  rcl_interfaces::msg::FloatingPointRange period_range;
  period_range.set__from_value(0.1).set__to_value(10.0).set__step(0.1);
  period_desc.floating_point_range = {period_range};

  // declare parameter
  this->declare_parameter(kPeriodParam, rclcpp::ParameterType::PARAMETER_DOUBLE, period_desc);

  // load parameter
  try {
    period_ = this->get_parameter(kPeriodParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kPeriodParam.c_str());
    exit(EXIT_FAILURE);
  }
}

/**
 * @brief Sets up subscribers, publishers, and more.
 * 
 */
void TemplateNode::setup() {

  // create a callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
    std::bind(&TemplateNode::parametersCallback, this, std::placeholders::_1));

  // create a transform broadcaster and listener
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // create a subscriber for handling incoming messages
  subscriber_ =
    this->create_subscription<template_interfaces_pkg::msg::TemplateMessage>(
      kInputTopic, 10,
      std::bind(&TemplateNode::topicCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscriber_->get_topic_name());

  // create a publisher for publishing messages
  publisher_ = this->create_publisher<template_interfaces_pkg::msg::TemplateMessage>(
    kOutputTopic, 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_->get_topic_name());

  // create a timer for repeatedly invoking a callback to publish messages
  publish_timer_ =
    this->create_wall_timer(std::chrono::duration<double>(period_),
                            std::bind(&TemplateNode::publishTimerCallback,
                            this));

  // create a service server for handling service calls
  service_server_ =
    this->create_service<template_interfaces_pkg::srv::TemplateService>(
      "service", std::bind(&TemplateNode::serviceCallback, this,
                           std::placeholders::_1, std::placeholders::_2));

  // create an action server for handling action goal requests
  action_server_ = rclcpp_action::create_server<
    template_interfaces_pkg::action::TemplateAction>(
    this, "action",
    std::bind(&TemplateNode::actionHandleGoal, this, std::placeholders::_1,
              std::placeholders::_2),
    std::bind(&TemplateNode::actionHandleCancel, this, std::placeholders::_1),
    std::bind(&TemplateNode::actionHandleAccepted, this,
              std::placeholders::_1));
}

/**
 * @brief This callback is invoked when a parameter value has changed
 * 
 * @param[in] parameters                                  input
 * 
 * @return    rcl_interfaces::msg::SetParametersResult    output  
 */
rcl_interfaces::msg::SetParametersResult TemplateNode::parametersCallback(
  const std::vector<rclcpp::Parameter> &parameters) {

  // update timer with newly configured period parameter value
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto &param : parameters) {
    if (param.get_name() == "period") {
      period_ = param.as_double();
      publish_timer_ =
        this->create_wall_timer(std::chrono::duration<double>(period_),
                                std::bind(&TemplateNode::publishTimerCallback,
                                this));
    }
  }

  // mark parameter change successful
  result.successful = true;
  result.reason = "success";

  return result;
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 * 
 * @param[in] msg   input
 */
void TemplateNode::topicCallback(
  const template_interfaces_pkg::msg::TemplateMessage &msg) {

  // echo received message
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.s.c_str());

  // lookup and echo a dynamic transform
  geometry_msgs::msg::TransformStamped tf;
  std::string to_frame = "base_link";
  std::string from_frame = "map";
  try {
    tf = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform '%s' to '%s': '%s'",
                to_frame.c_str(), from_frame.c_str(), ex.what());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "map -> base_link: [%.2f,%.2f,%.2f]",
              tf.transform.translation.x, tf.transform.translation.y,
              tf.transform.translation.z);
}

/**
 * @brief This callback is invoked every period seconds by the timer
 * 
 */
void TemplateNode::publishTimerCallback() {

  // skip callback if the published topic has no subscription
  if (publisher_->get_subscription_count() == 0) return;

  // create and publish a new message
  template_interfaces_pkg::msg::TemplateMessage msg;
  msg.i = count_;
  msg.s = "Hello ROS 2! (" + std::to_string(count_) + ")";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.s.c_str());
  publisher_->publish(msg);

  // create and broadcast a new dynamic transform
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = get_clock()->now();
  tf.header.frame_id = "map";
  tf.child_frame_id = "base_link";
  tf.transform.translation.x = count_ % 11;
  tf.transform.translation.y = count_ % 11;
  tf.transform.translation.z = count_ % 11;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, count_ % 37 * 10 / 180.0 * M_PI);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(tf);

  count_++;
}

/**
 * @brief This callback is invoked when the service is called
 * 
 * @param[in] request   input1
 * @param[in] response  input2
 */
void TemplateNode::serviceCallback(
  const template_interfaces_pkg::srv::TemplateService::Request::SharedPtr
    request,
  template_interfaces_pkg::srv::TemplateService::Response::SharedPtr
    response) {

  RCLCPP_INFO(this->get_logger(), "Received service request");

  // compute service response (sum of two integers)
  response->sum = request->a + request->b;
}


/**
 * @brief This callback is invoked when an action goal is requested
 * 
 * @param[in] uuid                          input1
 * @param[in] goal                          input2
 * 
 * @return    rclcpp_action::GoalResponse   output
 */
rclcpp_action::GoalResponse TemplateNode::actionHandleGoal(
  const rclcpp_action::GoalUUID &uuid,
  template_interfaces_pkg::action::TemplateAction::Goal::ConstSharedPtr
    goal) {

  RCLCPP_INFO(this->get_logger(), "Received action goal request");

  // accept action goal request
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief This callback is invoked when a running action is requested to cancel
 * 
 * @param[in] goal_handle                     input
 * 
 * @return    rclcpp_action::CancelResponse   output
 */
rclcpp_action::CancelResponse TemplateNode::actionHandleCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<
    template_interfaces_pkg::action::TemplateAction>>
    goal_handle) {

  RCLCPP_INFO(this->get_logger(), "Received request to cancel action goal");

  // accept action cancel request
  return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief This callback is invoked when an action goal request is accepted
 * 
 * @param[in] goal_handle     input
 */
void TemplateNode::actionHandleAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<
    template_interfaces_pkg::action::TemplateAction>>
    goal_handle) {

  // execute the action in a separate thread to avoid blocking
  std::thread{
    std::bind(&TemplateNode::actionExecute, this, std::placeholders::_1),
    goal_handle}
    .detach();
}


/**
 * @brief Execution of action
 * 
 * @param[in] goal_handle     input
 */
void TemplateNode::actionExecute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<
    template_interfaces_pkg::action::TemplateAction>>
    goal_handle) {

  RCLCPP_INFO(this->get_logger(), "Executing action goal");

  // define a sleeping rate between computing individual Fibonacci numbers
  rclcpp::Rate loop_rate(1);

  // create handy accessors for the action goal, feedback, and result
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<
    template_interfaces_pkg::action::TemplateAction::Feedback>();
  auto result =
    std::make_shared<template_interfaces_pkg::action::TemplateAction::Result>();

  // initialize the Fibonacci sequence
  auto &partial_fibonacci = feedback->partial_fibonacci;
  partial_fibonacci.push_back(0);
  partial_fibonacci.push_back(1);

  // compute the Fibonacci sequence up to the requested order n
  for (int i = 1; i < goal->n && rclcpp::ok(); ++i) {

    // cancel, if requested
    if (goal_handle->is_canceling()) {
      result->fibonacci = feedback->partial_fibonacci;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Action goal canceled");
      return;
    }

    // compute the next Fibonacci number
    partial_fibonacci.push_back(partial_fibonacci[i] +
                                partial_fibonacci[i - 1]);

    // publish the current sequence as action feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Publishing action feedback");

    // sleep before computing the next Fibonacci number
    loop_rate.sleep();
  }

  // finish by publishing the action result
  if (rclcpp::ok()) {
    result->fibonacci = partial_fibonacci;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}


}  // namespace template_cpp_pkg


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<template_cpp_pkg::TemplateNode>());
  rclcpp::shutdown();

  return 0;
}
