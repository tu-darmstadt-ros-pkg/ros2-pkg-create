#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>


namespace test_pkg {



class TestPkg : public rclcpp::Node {

 public:

  TestPkg();

 private:
  /**
  * @brief Sets up subscribers, publishers, etc. to configure the node
  */
  void setup();

 private:
};


}
