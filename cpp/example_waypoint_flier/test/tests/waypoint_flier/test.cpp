#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {
    sch_start_ = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "/uav1/waypoint_flier/start_waypoints_following");
  }

  bool test(void);

private:
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sch_start_;
};

bool Tester::test(void) {

  const std::string uav_name = "uav1";

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh;

  {
    auto [uhopt, message] = getUAVHandler(uav_name);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", uav_name.c_str(), message.c_str());
      return false;
    }

    uh = uhopt.value();
  }

  {
    auto [success, message] = uh->activateMidAir();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "midair activation failed with message: '%s'", message.c_str());
      return false;
    }
  }

  this->sleep(2.0);

  {
    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    {
      auto response = sch_start_.callSync(request);

      if (!response || !response.value()->success) {
        RCLCPP_ERROR(node_->get_logger(), "start service failed");
        return false;
      }
    }
  }

  this->sleep(2.0);

  while (true) {

    if (!rclcpp::ok()) {
      return false;
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: waiting for the landing to finish", name_.c_str());

    if (!uh->isOutputEnabled()) {

      return true;
    }

    sleep(1.0);
  }
}

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  bool test_result = true;

  Tester tester;

  test_result &= tester.test();

  tester.sleep(2.0);

  std::cout << "Test: reporting test results" << std::endl;

  tester.reportTestResult(test_result);

  tester.join();
}
