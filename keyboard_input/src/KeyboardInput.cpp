//
// Created by biao on 24-9-11.
//

#include <keyboard_input/KeyboardInput.h>

#include <algorithm>
#include <cstdio>
#include <sys/select.h>

KeyboardInput::KeyboardInput() : Node("keyboard_input_node")
{
    publisher_ = create_publisher<quadruped_controller_msgs::msg::Inputs>("/control_input", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&KeyboardInput::timer_callback, this));
    inputs_ = quadruped_controller_msgs::msg::Inputs();

    tcgetattr(STDIN_FILENO, &old_tio_);
    new_tio_ = old_tio_;
    new_tio_.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);
    RCLCPP_INFO(get_logger(), "Keyboard input node started.");
    RCLCPP_INFO(get_logger(),
                "Mode keys: 1 PASSIVE, 2 FIXEDSTAND, 4 FREESTAND, 6 MPC_TROTTING, 7 MPC_WBC_TROTTING.");
    RCLCPP_INFO(get_logger(), "FreeStand keys: A/D roll, W/S pitch, J/L yaw, I/K height.");
    RCLCPP_INFO(get_logger(), "MPC keys: A/D lx, W/S ly, J/L yaw rate, I/K height.");
    RCLCPP_INFO(get_logger(), "Space resets command axes.");
    RCLCPP_INFO(get_logger(), "Please input keys, press Ctrl+C to quit.");
}

void KeyboardInput::timer_callback()
{
    if (kbhit())
    {
        char key = getchar();
        check_command(key);
        if (inputs_.command == 0)
        {
            check_value(key);
        }
        else
        {
            inputs_.lx = 0;
            inputs_.ly = 0;
            inputs_.rx = 0;
            inputs_.ry = 0;
            reset_count_ = 100;
        }
        publisher_->publish(inputs_);
        RCLCPP_INFO(get_logger(), "Published key='%c': command=%d lx=%.2f ly=%.2f rx=%.2f ry=%.2f", key,
                    inputs_.command, inputs_.lx, inputs_.ly, inputs_.rx, inputs_.ry);
        just_published_ = true;
    }
    else
    {
        if (just_published_)
        {
            reset_count_ -= 1;
            if (reset_count_ == 0)
            {
                just_published_ = false;
                if (inputs_.command != 0)
                {
                    inputs_.command = 0;
                    publisher_->publish(inputs_);
                    RCLCPP_INFO(get_logger(), "Command pulse reset: command=%d lx=%.2f ly=%.2f rx=%.2f ry=%.2f",
                                inputs_.command, inputs_.lx, inputs_.ly, inputs_.rx, inputs_.ry);
                }
            }
        }
    }
}

void KeyboardInput::check_command(const char key)
{
    switch (key)
    {
    case '1':
        inputs_.command = 1;
        break;
    case '2':
        inputs_.command = 2;
        break;
    case '4':
        inputs_.command = 4;
        break;
    case '6':
        inputs_.command = 6;
        break;
    case '7':
        inputs_.command = 7;
        break;
    case ' ':
        inputs_.lx = 0;
        inputs_.ly = 0;
        inputs_.rx = 0;
        inputs_.ry = 0;
        inputs_.command = 0;
        break;
    default:
        inputs_.command = 0;
        break;
    }
}

void KeyboardInput::check_value(char key)
{
    switch (key)
    {
    case 'w':
    case 'W':
        inputs_.ly = std::clamp(inputs_.ly + sensitivity_left_, -1.0F, 1.0F);
        break;
    case 's':
    case 'S':
        inputs_.ly = std::clamp(inputs_.ly - sensitivity_left_, -1.0F, 1.0F);
        break;
    case 'd':
    case 'D':
        inputs_.lx = std::clamp(inputs_.lx + sensitivity_left_, -1.0F, 1.0F);
        break;
    case 'a':
    case 'A':
        inputs_.lx = std::clamp(inputs_.lx - sensitivity_left_, -1.0F, 1.0F);
        break;

    case 'i':
    case 'I':
        inputs_.ry = std::clamp(inputs_.ry + sensitivity_right_, -1.0F, 1.0F);
        break;
    case 'k':
    case 'K':
        inputs_.ry = std::clamp(inputs_.ry - sensitivity_right_, -1.0F, 1.0F);
        break;
    case 'l':
    case 'L':
        inputs_.rx = std::clamp(inputs_.rx + sensitivity_right_, -1.0F, 1.0F);
        break;
    case 'j':
    case 'J':
        inputs_.rx = std::clamp(inputs_.rx - sensitivity_right_, -1.0F, 1.0F);
        break;
    default:
        break;
    }
}

bool KeyboardInput::kbhit()
{
    timeval tv = {0L, 0L};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardInput>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
