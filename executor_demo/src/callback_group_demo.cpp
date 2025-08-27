#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/executors/static_single_threaded_executor.hpp>

using namespace std::chrono_literals;

class CallbackGroupDemo : public rclcpp::Node
{
public:
  CallbackGroupDemo() : Node("callback_group_demo")
  {
    auto exclusive_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto reentrant_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::TimerOptions exclusive_options;
    exclusive_options.callback_group = exclusive_group;
    timer_exclusive_1_ = this->create_wall_timer(1s,
      std::bind(&CallbackGroupDemo::exclusive_timer1, this), exclusive_options);
    timer_exclusive_2_ = this->create_wall_timer(1s,
      std::bind(&CallbackGroupDemo::exclusive_timer2, this), exclusive_options);

    rclcpp::TimerOptions reentrant_options;
    reentrant_options.callback_group = reentrant_group;
    timer_reentrant_1_ = this->create_wall_timer(1s,
      std::bind(&CallbackGroupDemo::reentrant_timer1, this), reentrant_options);
    timer_reentrant_2_ = this->create_wall_timer(1s,
      std::bind(&CallbackGroupDemo::reentrant_timer2, this), reentrant_options);
  }

private:
  void exclusive_timer1()
  {
    RCLCPP_INFO(this->get_logger(), "Mutually exclusive timer 1 start");
    std::this_thread::sleep_for(500ms);
    RCLCPP_INFO(this->get_logger(), "Mutually exclusive timer 1 end");
  }

  void exclusive_timer2()
  {
    RCLCPP_INFO(this->get_logger(), "Mutually exclusive timer 2 start");
    std::this_thread::sleep_for(500ms);
    RCLCPP_INFO(this->get_logger(), "Mutually exclusive timer 2 end");
  }

  void reentrant_timer1()
  {
    RCLCPP_INFO(this->get_logger(), "Reentrant timer 1 start");
    std::this_thread::sleep_for(500ms);
    RCLCPP_INFO(this->get_logger(), "Reentrant timer 1 end");
  }

  void reentrant_timer2()
  {
    RCLCPP_INFO(this->get_logger(), "Reentrant timer 2 start");
    std::this_thread::sleep_for(500ms);
    RCLCPP_INFO(this->get_logger(), "Reentrant timer 2 end");
  }

  rclcpp::TimerBase::SharedPtr timer_exclusive_1_;
  rclcpp::TimerBase::SharedPtr timer_exclusive_2_;
  rclcpp::TimerBase::SharedPtr timer_reentrant_1_;
  rclcpp::TimerBase::SharedPtr timer_reentrant_2_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CallbackGroupDemo>();

  std::string executor_type = "single";
  if (argc > 1) {
    executor_type = argv[1];
  }

  if (executor_type == "multi") {
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  } else if (executor_type == "static") {
    rclcpp::executors::StaticSingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  } else {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  }

  rclcpp::shutdown();
  return 0;
}
