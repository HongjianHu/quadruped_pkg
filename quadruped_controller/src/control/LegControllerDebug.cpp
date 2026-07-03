#include "quadruped_controller/control/LegController.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

std::string readFile(const std::string &path)
{
    std::ifstream file(path);
    if (!file)
    {
        throw std::runtime_error("Failed to open file: " + path);
    }

    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

std::vector<Eigen::VectorXd> makeStandJointPositions()
{
    std::vector<Eigen::VectorXd> joint_pos(4, Eigen::VectorXd::Zero(3));
    for (auto &leg_pos : joint_pos)
    {
        leg_pos << 0.0, 0.9, -1.8;
    }
    return joint_pos;
}

std::vector<Eigen::VectorXd> makeZeroJointVelocities()
{
    return std::vector<Eigen::VectorXd>(4, Eigen::VectorXd::Zero(3));
}

void printOutput(const std::string &label, const quadruped_controller::LegOutput &output)
{
    std::cout << label << "\n";
    std::cout << "  tau:     " << output.tau.transpose() << "\n";
    std::cout << "  pos_des: " << output.pos_des.transpose() << "\n";
    std::cout << "  pos_now: " << output.pos_now.transpose() << "\n";
    std::cout << "  vel_des: " << output.vel_des.transpose() << "\n";
    std::cout << "  vel_now: " << output.vel_now.transpose() << "\n";
}

} // namespace

int main()
{
    using quadruped_controller::Gait;
    using quadruped_controller::LegController;
    using quadruped_controller::Vec3;
    using quadruped_controller::Vec34;
    using quadruped_controller::go2_robot_data::PinGo2Model;

    const std::string description_share = ament_index_cpp::get_package_share_directory("go2_description");
    const std::string robot_description = readFile(description_share + "/urdf/robot.urdf");

    const std::vector<std::string> feet_names = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
    PinGo2Model go2(robot_description, feet_names, "base");
    go2.updateModel(makeStandJointPositions(), makeZeroJointVelocities());

    Gait gait(1.0 / 0.45, 0.5);
    LegController leg_controller;

    Vec34 touchdown_positions = Vec34::Zero();
    for (int leg = 0; leg < 4; ++leg)
    {
        touchdown_positions.col(leg) = go2.footPositionWorld(leg);
    }
    touchdown_positions.col(LegController::legIndex("FR")).x() += 0.08;
    touchdown_positions.col(LegController::legIndex("RL")).x() += 0.08;
    leg_controller.setTouchdownPositionsWorld(touchdown_positions);

    std::cout << "mass: " << go2.mass() << "\n";
    std::cout << "gait mask t=0 [FL FR RL RR]: " << gait.computeCurrentMask(0.0).transpose() << "\n";

    for (int leg = 0; leg < 4; ++leg)
    {
        const auto state = go2.footStateWorld(leg);
        std::cout << "foot " << leg << " pos/vel world: " << state.position.transpose() << " | "
                  << state.velocity.transpose() << "\n";
    }

    Vec3 stance_force_world;
    stance_force_world << 0.0, 0.0, 70.0;

    const auto fl_stance = leg_controller.computeLegTorque("FL", go2, gait, stance_force_world, 0.0);
    printOutput("FL stance sample", fl_stance);

    Vec3 zero_force = Vec3::Zero();
    const auto fr_swing_start = leg_controller.computeLegTorque("FR", go2, gait, zero_force, 0.0);
    const auto fr_swing_mid = leg_controller.computeLegTorque("FR", go2, gait, zero_force, 0.5 * gait.swingTime());

    printOutput("FR swing start sample", fr_swing_start);
    printOutput("FR swing mid sample", fr_swing_mid);

    std::cout << "FR swing displacement at mid: " << (fr_swing_mid.pos_des - fr_swing_mid.pos_now).transpose() << "\n";
    std::cout << "FR swing tau norm at mid: " << fr_swing_mid.tau.norm() << "\n";

    return 0;
}
