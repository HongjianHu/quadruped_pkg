#include "quadruped_controller/gait/Gait.h"

#include <iostream>

int main()
{
    using quadruped_controller::Gait;

    const double period = 0.45;
    const double frequency_hz = 1.0 / period;
    const double duty = 0.5;

    Gait gait(frequency_hz, duty);

    std::cout << "period: " << gait.period() << "\n";
    std::cout << "stance_time: " << gait.stanceTime() << "\n";
    std::cout << "swing_time: " << gait.swingTime() << "\n";

    const auto mask = gait.computeCurrentMask(0.0);
    std::cout << "mask t=0 [FL FR RL RR]: " << mask.transpose() << "\n";

    const int horizon = 8;
    const double dt = gait.period() / static_cast<double>(horizon);
    const auto table = gait.computeContactTable(0.0, dt, horizon);

    quadruped_controller::Vec3 p0;
    p0 << 0.0, 0.0, 0.0;

    quadruped_controller::Vec3 pf;
    pf << 0.2, 0.0, 0.0;

    const auto swing_start = Gait::evaluateSwingTrajectory(p0, pf, 0.0, gait.swingTime());
    const auto swing_mid = Gait::evaluateSwingTrajectory(p0, pf, 0.5 * gait.swingTime(), gait.swingTime());
    const auto swing_end = Gait::evaluateSwingTrajectory(p0, pf, gait.swingTime(), gait.swingTime());

    std::cout << "swing start p/v/a: " << swing_start.position.transpose() << " | " << swing_start.velocity.transpose()
              << " | " << swing_start.acceleration.transpose() << "\n";

    std::cout << "swing mid p/v/a: " << swing_mid.position.transpose() << " | " << swing_mid.velocity.transpose()
              << " | " << swing_mid.acceleration.transpose() << "\n";

    std::cout << "swing end p/v/a: " << swing_end.position.transpose() << " | " << swing_end.velocity.transpose()
              << " | " << swing_end.acceleration.transpose() << "\n";

    quadruped_controller::Vec34 current_feet = quadruped_controller::Vec34::Zero();
    current_feet.col(0) << 0.2, 0.1, 0.0;   // FL
    current_feet.col(1) << 0.2, -0.1, 0.0;  // FR
    current_feet.col(2) << -0.2, 0.1, 0.0;  // RL
    current_feet.col(3) << -0.2, -0.1, 0.0; // RR

    quadruped_controller::Vec34 touchdown_feet = current_feet;
    touchdown_feet.col(1).x() += 0.2; // FR swing target
    touchdown_feet.col(2).x() += 0.2; // RL swing target

    quadruped_controller::Vec34 target_pos = quadruped_controller::Vec34::Zero();
    quadruped_controller::Vec34 target_vel = quadruped_controller::Vec34::Zero();
    quadruped_controller::Vec34 target_acc = quadruped_controller::Vec34::Zero();

    gait.resetSwingState(current_feet);
    gait.updateSwingTrajectory(0.0, current_feet, touchdown_feet, target_pos, target_vel, target_acc);
    gait.updateSwingTrajectory(0.5 * gait.swingTime(), current_feet, touchdown_feet, target_pos, target_vel,
                               target_acc);

    std::cout << "managed swing target pos at mid, rows=[x y z], cols=[FL FR RL RR]:\n";
    std::cout << target_pos << "\n";

    Gait::TouchdownInput touchdown_input;
    touchdown_input.base_pos_world << 0.0, 0.0, 0.27;
    touchdown_input.com_pos_world << 0.0, 0.0, 0.27;
    touchdown_input.com_vel_world << 0.2, 0.0, 0.0;
    touchdown_input.desired_velocity_world << 0.2, 0.0, 0.0;
    touchdown_input.desired_position_world << 0.0, 0.0, 0.27;
    touchdown_input.yaw_rotation_body_to_world.setIdentity();
    touchdown_input.hip_offset_body << 0.2, -0.1, 0.0;
    touchdown_input.yaw_rate_des_world = 0.5;

    const auto touchdown = gait.computeTouchdownWorld(touchdown_input);
    std::cout << "touchdown world sample: " << touchdown.transpose() << "\n";

    quadruped_controller::Vec34 hip_offsets_body = quadruped_controller::Vec34::Zero();
    hip_offsets_body.col(0) << 0.2, 0.1, 0.0;   // FL
    hip_offsets_body.col(1) << 0.2, -0.1, 0.0;  // FR
    hip_offsets_body.col(2) << -0.2, 0.1, 0.0;  // RL
    hip_offsets_body.col(3) << -0.2, -0.1, 0.0; // RR

    const auto touchdown_all = gait.computeTouchdownWorlds(touchdown_input, hip_offsets_body);
    std::cout << "touchdown all, rows=[x y z], cols=[FL FR RL RR]:\n";
    std::cout << touchdown_all << "\n";

    gait.resetSwingState(current_feet);
    gait.updateSwingTrajectory(0.0, current_feet, touchdown_all, target_pos, target_vel, target_acc);
    gait.updateSwingTrajectory(0.5 * gait.swingTime(), current_feet, touchdown_all, target_pos, target_vel, target_acc);

    std::cout << "managed swing with computed touchdown at mid:\n";
    std::cout << target_pos << "\n";

    std::cout << "contact table rows=[FL FR RL RR], cols=horizon:\n";
    std::cout << table << "\n";

    return 0;
}