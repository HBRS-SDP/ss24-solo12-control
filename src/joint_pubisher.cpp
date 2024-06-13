#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <cmath>
#include <chrono> 

class JointPublisher : public rclcpp::Node {
public:
    JointPublisher() 
    : Node("joint_publisher"),count_(0) 
    {
         // Initialize publishers, subscribers, and other variables here
         publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
             "joint_trajectory_controller/joint_trajectory", 10);

         timer_ = this->create_wall_timer(
             std::chrono::milliseconds(100), std::bind(&JointPublisher::timer_callback, this));
    }

private:
    // Declare ROS 2 publishers, subscribers, and other variables
    void timer_callback()
    {
        auto message = trajectory_msgs::msg::JointTrajectory();
        // Add names of the joints you want to control here
        message.joint_names.push_back("FL_HAA");
        message.joint_names.push_back("FL_HFE");
        message.joint_names.push_back("FL_KFE");
        message.joint_names.push_back("FR_HAA");
        message.joint_names.push_back("FR_HFE");
        message.joint_names.push_back("FR_KFE");
        message.joint_names.push_back("HL_HAA");
        message.joint_names.push_back("HL_HFE");
        message.joint_names.push_back("HL_KFE");
        message.joint_names.push_back("HR_HAA");
        message.joint_names.push_back("HR_HFE");
        message.joint_names.push_back("HR_KFE");    

        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        double position1    = 1.5*(1-cos(count_*0.1));
        double position2    = -position1;
        double position3    = 1.5*(1-cos(count_*0.1));
        double position4    = -position3;
        double position5    = 1.5*(1-cos(count_*0.1));
        double position6    = -position5;
        double position7    = 1.5*(1-cos(count_*0.1));
        double position8    = -position7;
        double position9    = 1.5*(1-cos(count_*0.1));
        double position10   = -position9;
        double position11   = 1.5*(1-cos(count_*0.1));
        double position12   = -position11;
    
        point.positions.push_back(position1);
        point.positions.push_back(position2);
        point.positions.push_back(position3);
        point.positions.push_back(position4);
        point.positions.push_back(position5);
        point.positions.push_back(position6);
        point.positions.push_back(position7);
        point.positions.push_back(position8);
        point.positions.push_back(position9);
        point.positions.push_back(position10);
        point.positions.push_back(position11);
        point.positions.push_back(position12);
    
        point.time_from_start.sec = rclpp::Duration::from_seconds(0.1);
        message.points.push_back(point);
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Publishing: '%f', %f', '%f', %f', '%f', %f', '%f', %f', '%f', %f', '%f', %f',", position1, position2, position3, position4, position5, position6, position7, position8, position9, position10, position11, position12);
        count_++;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    size_t count_;
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
