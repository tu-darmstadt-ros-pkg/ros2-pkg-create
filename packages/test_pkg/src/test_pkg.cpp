#include <functional>

#include <test_pkg/test_pkg.hpp>


namespace test_pkg {


TestPkg::TestPkg() : Node("test_pkg") {

  this->setup();
}


void TestPkg::setup() {
}


}


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<test_pkg::TestPkg>());
  rclcpp::shutdown();

  return 0;
}
