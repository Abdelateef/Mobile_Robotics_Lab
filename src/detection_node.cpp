#include "datmo.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>

//UPDATE
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void Datmo::update() 
{
    // YOU DO NOT HAVE TO CHANGE ANYTHING IN THIS FILE. Just read and understand
    // Keep previous and current_robot_moving at true until we have received a laser data (to ensure we do store_background on the first scan)
    if (!init_laser_) {
        previous_robot_moving_ = true;
        current_robot_moving_ = true;
    }


    // we wait for new data of the laser and of the robot_moving_node to perform laser processing
    if ( new_laser_ && new_robot_ ) 
    {

        RCLCPP_INFO(this->get_logger(), "\n");
        RCLCPP_INFO(this->get_logger(), "New data of laser received");
        RCLCPP_INFO(this->get_logger(), "New data of robot_moving received");

        // display field of view
        display_field_of_view();

        //detection of motion
        detect_motion();
        display_motion();

        // clustering
        perform_clustering(); // to perform clustering
        display_clustering();

        // detection of legs
        detect_legs(); // to detect legs using cluster
        display_legs();

        // detection of persons
        detect_persons(); // to detect persons using legs detected
        display_persons();
       
        detect_a_moving_person();  
        display_a_moving_detected_person();

        update_state_variables();      
        
    }
    else
    {
        if ( !init_laser_ )
            RCLCPP_WARN(this->get_logger(), "waiting for laser data: run a rosbag or connect to a RobAIR");
        else
            if ( !init_robot_ )
            {
                RCLCPP_WARN(this->get_logger(), "waiting for robot_moving_node: ros2 run follow_me robot_moving_node");
                display_field_of_view();
            }
    }

    update_state_variables();

}// update

int main(int argc, char ** argv)
{
  // 1) Initialise ROS2 context
  rclcpp::init(argc, argv);

  // 2) Create node based on the Datmo class. Pass node name character string.
  auto node = std::make_shared<Datmo>("detection_node");

  // 3) Waiting log
  RCLCPP_INFO(node->get_logger(), "waiting for detection of a moving person");

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
