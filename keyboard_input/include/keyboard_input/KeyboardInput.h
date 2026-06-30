//
// Created by biao on 24-9-11.
//


#ifndef KEYBOARDINPUT_H
#define KEYBOARDINPUT_H

#include <quadruped_controller_msgs/msg/inputs.hpp>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

class KeyboardInput final : public rclcpp::Node
{
  public:
    KeyboardInput();

    ~KeyboardInput() override
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
    }

  private:
    void timer_callback();

    void check_command(char key);
    void check_value(char key);

    static bool kbhit();

    quadruped_controller_msgs::msg::Inputs inputs_;
    rclcpp::Publisher<quadruped_controller_msgs::msg::Inputs>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool just_published_ = false;
    int reset_count_ = 0;

    float sensitivity_left_ = 0.05;
    float sensitivity_right_ = 0.05;
    termios old_tio_{}, new_tio_{};
};

#endif // KEYBOARDINPUT_H
