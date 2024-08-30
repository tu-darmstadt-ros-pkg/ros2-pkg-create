#pragma once

#include <memory>
#include <string>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/int32.hpp>


namespace ros2_cpp_lifecycle_pkg {

template <typename C> struct is_vector : std::false_type {};    
template <typename T,typename A> struct is_vector< std::vector<T,A> > : std::true_type {};    
template <typename C> inline constexpr bool is_vector_v = is_vector<C>::value;


class Ros2CppLifecycleNode : public rclcpp_lifecycle::LifecycleNode {

 public:

  Ros2CppLifecycleNode();

 protected:

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

 private:

  template <typename T>
  void declareAndLoadParameter(const std::string &name,
                               T &param,
                               const std::string &description,
                               const bool add_to_auto_reconfigurable_params = true,
                               const bool is_required = false,
                               const bool read_only = false,
                               const std::optional<double> &from_value = std::nullopt,
                               const std::optional<double> &to_value = std::nullopt,
                               const std::optional<double> &step_value = std::nullopt,
                               const std::string &additional_constraints = "");

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  void setup();

  void cleanUp();

  void topicCallback(const std_msgs::msg::Int32& msg);

 private:

  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter &)>>> auto_reconfigurable_params_;

  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr publisher_;

  double param_ = 1.0;
};


}
