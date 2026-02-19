/* 
Adapted from: https://github.com/PlanSys2/ros2_planning_system_examples/blob/rolling/plansys2_simple_example/src/ask_charge_action_node.cpp
*/
#include <memory>
#include <algorithm>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

# define ACTION "unhook" // Name of the action

using namespace std::chrono_literals;

class Action : public plansys2::ActionExecutorClient
{
public:
  Action()
  : plansys2::ActionExecutorClient(ACTION, 100ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) {
      progress_ += 0.05;
      send_feedback(progress_, std::string(ACTION) + " running");
    } else {
      finish(true, 1.0, std::string(ACTION) + " completed");

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << ACTION << " progress ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
      std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Action>();

  node->set_parameter(rclcpp::Parameter("action_name", ACTION));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}