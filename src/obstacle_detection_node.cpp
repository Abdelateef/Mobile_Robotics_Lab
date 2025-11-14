#include "datmo.hpp"

//UPDATE
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void Datmo::update() 
{

    // we wait for new data of the laser and of the robot_moving_node to perform laser processing
    if ( new_laser_ ) 
    {

        RCLCPP_INFO(this->get_logger(), "\n");
        RCLCPP_INFO(this->get_logger(), "New data of laser received");

        find_closest_obstacle();
    }
    else
        RCLCPP_WARN(this->get_logger(), "waiting for laser data: run a rosbag");

}// update


int main(int argc, char ** argv)
{
  // 1) Initialise ROS2 context
  rclcpp::init(argc, argv);

  // 2) Create node based on the Datmo class. Pass node name character string.
  auto node = std::make_shared<Datmo>("obstacle_detection_node");

  // 3) Waiting log
  RCLCPP_INFO(node->get_logger(), "waiting for laser data");

  // 4) Loop / Cycle at 10Hz
  rclcpp::Rate rate(10);

  // 5) Main loop: execute callbacks and update function
  while (rclcpp::ok()) {
    // Non-blocking callback execution
    rclcpp::spin_some(node);
    // node logic
    node->update();
    rate.sleep();
  }

  // 6) Cleanup and shutdown
  rclcpp::shutdown();
  return 0;
}