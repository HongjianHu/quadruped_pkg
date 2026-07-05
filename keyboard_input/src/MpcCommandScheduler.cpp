#include <quadruped_controller_msgs/msg/inputs.hpp>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

namespace
{

struct CommandPhase
{
    double start{0.0};
    double end{0.0};
    float lx{0.0F};
    float ly{0.0F};
    float rx{0.0F};
    float ry{0.0F};
    const char *label{""};
};

float clampCommand(const double value)
{
    return static_cast<float>(std::clamp(value, -1.0, 1.0));
}

} // namespace

class MpcCommandScheduler final : public rclcpp::Node
{
  public:
    MpcCommandScheduler() : Node("mpc_command_scheduler")
    {
        fixedstand_duration_ = declare_parameter("fixedstand_duration", 3.0);
        switch_duration_ = declare_parameter("switch_duration", 1.0);
        publish_hz_ = declare_parameter("publish_hz", 50.0);
        speed_scale_ = declare_parameter("speed_scale", 1.0);
        stop_after_schedule_ = declare_parameter("stop_after_schedule", true);

        phases_ = {
            {0.0, 1.5, scaled(0.85), 0.0F, 0.0F, 0.0F, "forward"},
            {1.5, 2.0, 0.0F, 0.0F, 0.0F, 0.0F, "stop"},
            {2.0, 3.5, 0.0F, scaled(0.65), 0.0F, 0.0F, "sidestep"},
            {3.5, 4.0, 0.0F, 0.0F, 0.0F, 0.0F, "stop"},
            {4.0, 6.0, 0.0F, 0.0F, scaled(0.65), 0.0F, "turn"},
            {6.0, 6.5, 0.0F, 0.0F, 0.0F, 0.0F, "stop"},
            {6.5, 8.0, scaled(0.80), 0.0F, scaled(0.55), 0.0F, "forward_turn"},
            {8.0, 9.5, scaled(1.00), 0.0F, 0.0F, 0.0F, "fast_forward"},
            {9.5, 10.5, 0.0F, 0.0F, 0.0F, 0.0F, "stop"},
        };

        publisher_ = create_publisher<quadruped_controller_msgs::msg::Inputs>("/control_input", 10);
        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_hz_));
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                   std::bind(&MpcCommandScheduler::timerCallback, this));

        RCLCPP_INFO(get_logger(),
                    "MPC command scheduler started: fixedstand %.2fs, switch %.2fs, schedule %.2fs, speed_scale %.2f",
                    fixedstand_duration_, switch_duration_, scheduleDuration(), speed_scale_);
        RCLCPP_INFO(get_logger(), "Normalized commands map inside StateMPCTrotting to max vx=0.50 m/s, vy=0.32 m/s, yaw=0.80 rad/s.");
    }

  private:
    float scaled(const double value) const
    {
        return clampCommand(value * speed_scale_);
    }

    double nowSeconds()
    {
        return get_clock()->now().seconds();
    }

    double elapsedSeconds()
    {
        const double now = nowSeconds();
        if (!started_)
        {
            start_time_ = now;
            started_ = true;
        }
        return now - start_time_;
    }

    double scheduleDuration() const
    {
        if (phases_.empty())
        {
            return 0.0;
        }
        return phases_.back().end;
    }

    void timerCallback()
    {
        const double t = elapsedSeconds();
        auto msg = quadruped_controller_msgs::msg::Inputs();

        if (t < fixedstand_duration_)
        {
            msg.command = 2;
            publishIfPhaseChanged("FIXEDSTAND", t);
        }
        else if (t < fixedstand_duration_ + switch_duration_)
        {
            msg.command = 6;
            publishIfPhaseChanged("MPC_TROTTING", t);
        }
        else
        {
            msg.command = 0;
            const double demo_t = t - fixedstand_duration_ - switch_duration_;
            const CommandPhase *phase = phaseAt(demo_t);
            if (phase != nullptr)
            {
                msg.lx = phase->lx;
                msg.ly = phase->ly;
                msg.rx = phase->rx;
                msg.ry = phase->ry;
                publishIfPhaseChanged(phase->label, t);
            }
            else
            {
                publishIfPhaseChanged("DONE", t);
                if (stop_after_schedule_)
                {
                    publisher_->publish(msg);
                    rclcpp::shutdown();
                    return;
                }
            }
        }

        publisher_->publish(msg);
    }

    const CommandPhase *phaseAt(const double demo_t) const
    {
        for (const auto &phase : phases_)
        {
            if (demo_t >= phase.start && demo_t < phase.end)
            {
                return &phase;
            }
        }
        return nullptr;
    }

    void publishIfPhaseChanged(const std::string &label, const double elapsed)
    {
        if (label == last_label_)
        {
            return;
        }
        last_label_ = label;
        RCLCPP_INFO(get_logger(), "phase=%s elapsed=%.2f", label.c_str(), elapsed);
    }

    rclcpp::Publisher<quadruped_controller_msgs::msg::Inputs>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<CommandPhase> phases_;
    std::string last_label_;
    double fixedstand_duration_{3.0};
    double switch_duration_{1.0};
    double publish_hz_{50.0};
    double speed_scale_{1.0};
    double start_time_{0.0};
    bool started_{false};
    bool stop_after_schedule_{true};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MpcCommandScheduler>();
    rclcpp::spin(node);
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    return 0;
}
