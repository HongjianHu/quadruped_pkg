#include "quadruped_controller/control/ContactStateManager.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace quadruped_controller
{

ContactStateManager::ContactStateManager(const ContactStateConfig &config) : config_(config)
{
    if (!std::isfinite(config_.contact_force_enter_threshold) ||
        !std::isfinite(config_.contact_force_exit_threshold) ||
        config_.contact_force_exit_threshold < 0.0 ||
        config_.contact_force_enter_threshold <= config_.contact_force_exit_threshold)
    {
        throw std::invalid_argument("Contact-state force thresholds are invalid");
    }

    const std::array<double, 10> positive_durations = {
        config_.force_filter_time_constant,
        config_.touchdown_confirm_duration,
        config_.contact_loss_confirm_duration,
        config_.touchdown_force_ramp_duration,
        config_.touchdown_assumption_duration,
        config_.assumed_contact_grace_duration,
        config_.liftoff_force_ramp_duration,
        config_.liftoff_force_release_duration,
        config_.max_unconfirmed_support_duration,
        config_.max_update_dt};

    for (const double duration : positive_durations)
    {
        if (!std::isfinite(duration) || duration <= 0.0)
        {
            throw std::invalid_argument("Contact-state durations must be finite and positive");
        }
    }

    if (!std::isfinite(config_.support_transfer_min_activation) ||
        config_.support_transfer_min_activation <= 0.0 || config_.support_transfer_min_activation > 1.0)
    {
        throw std::invalid_argument("Support-transfer minimum activation must be in (0, 1]");
    }
    if (config_.minimum_confirmed_support_legs <= 0 || config_.minimum_confirmed_support_legs > 4)
    {
        throw std::invalid_argument("Minimum confirmed support legs must be in [1, 4]");
    }
    reset();
}

void ContactStateManager::reset()
{
    states_.fill(FootContactState::SWING);
    state_elapsed_time_.setZero();
    contact_confirmation_time_.setZero();
    contact_loss_time_.setZero();
    activation_.setZero();
    filtered_normal_force_.setZero();
    force_confirmed_.setZero();
    filter_initialized_ = false;
    unsupported_time_ = 0.0;
}

ContactStateOutput ContactStateManager::update(const VecInt4 &planned_contact, const Vec4 &raw_normal_force,
                                               const double dt)
{
    for (int leg = 0; leg < 4; ++leg)
    {
        if (planned_contact[leg] != 0 && planned_contact[leg] != 1)
        {
            throw std::invalid_argument("Planned contact mask must contain only 0 or 1");
        }
    }

    const double safe_dt = std::clamp(std::isfinite(dt) ? dt : 0.0, 0.0, config_.max_update_dt);
    Vec4 sanitized_force = Vec4::Zero();
    for (int leg = 0; leg < 4; ++leg)
    {
        sanitized_force[leg] = sanitizeNormalForce(raw_normal_force[leg]);
    }

    if (!filter_initialized_)
    {
        filtered_normal_force_ = sanitized_force;
        filter_initialized_ = true;
    }
    else
    {
        const double filter_gain = safe_dt / (config_.force_filter_time_constant + safe_dt);
        filtered_normal_force_ += filter_gain * (sanitized_force - filtered_normal_force_);
    }

    for (int leg = 0; leg < 4; ++leg)
    {
        state_elapsed_time_[leg] += safe_dt;
        const bool planned_stance = planned_contact[leg] == 1;
        const double normal_force = filtered_normal_force_[leg];

        switch (states_[leg])
        {
        case FootContactState::SWING:
            activation_[leg] = 0.0;
            if (planned_stance)
            {
                transitionTo(leg, FootContactState::TOUCHDOWN_PENDING);
            }
            break;

        case FootContactState::TOUCHDOWN_PENDING:
            activation_[leg] = 0.0;
            if (!planned_stance)
            {
                transitionTo(leg, FootContactState::SWING);
                break;
            }

            if (normal_force >= config_.contact_force_enter_threshold)
            {
                contact_confirmation_time_[leg] += safe_dt;
            }
            else
            {
                contact_confirmation_time_[leg] = 0.0;
            }

            if (contact_confirmation_time_[leg] >= config_.touchdown_confirm_duration)
            {
                force_confirmed_[leg] = 1;
                transitionTo(leg, FootContactState::STANCE);
            }
            else if (state_elapsed_time_[leg] >= config_.touchdown_assumption_duration)
            {
                // 仅把新腿加入运动学约束并逐步增力；force_confirmed仍为0，
                // 因此协调转移逻辑不会过早释放旧支撑腿。
                transitionTo(leg, FootContactState::STANCE);
            }
            break;

        case FootContactState::STANCE:
            if (!planned_stance)
            {
                transitionTo(leg, FootContactState::LIFTOFF);
                break;
            }

            if (force_confirmed_[leg] == 0)
            {
                if (normal_force >= config_.contact_force_enter_threshold)
                {
                    contact_confirmation_time_[leg] += safe_dt;
                }
                else
                {
                    contact_confirmation_time_[leg] = 0.0;
                }

                if (contact_confirmation_time_[leg] >= config_.touchdown_confirm_duration)
                {
                    force_confirmed_[leg] = 1;
                }
                else if (state_elapsed_time_[leg] >= config_.assumed_contact_grace_duration)
                {
                    // 临时硬约束仍未产生真实接触，撤回并重新执行触地搜索。
                    transitionTo(leg, FootContactState::TOUCHDOWN_PENDING);
                }
                break;
            }

            if (normal_force < config_.contact_force_exit_threshold)
            {
                contact_loss_time_[leg] += safe_dt;
            }
            else
            {
                contact_loss_time_[leg] = 0.0;
            }

            if (contact_loss_time_[leg] >= config_.contact_loss_confirm_duration)
            {
                // 计划仍在支撑但真实接触已经持续丢失，重新进入触地搜索。
                transitionTo(leg, FootContactState::TOUCHDOWN_PENDING);
            }
            break;

        case FootContactState::LIFTOFF:
            if (planned_stance)
            {
                // 步态计划反转时恢复支撑，并从当前激活比例继续增加接触力。
                transitionTo(leg, FootContactState::STANCE);
                break;
            }

            if (normal_force < config_.contact_force_exit_threshold)
            {
                contact_loss_time_[leg] += safe_dt;
            }
            else
            {
                contact_loss_time_[leg] = 0.0;
            }
            break;
        }

        if (states_[leg] == FootContactState::STANCE)
        {
            activation_[leg] = std::min(1.0, activation_[leg] + safe_dt / config_.touchdown_force_ramp_duration);
        }
    }

    // 接触释放必须跨腿协调，而不能让四条腿各自独立决定。
    // 对角步态切换时，新支撑腿通常仍处于TOUCHDOWN_PENDING；若旧支撑腿此时已经
    // 独立完成LIFTOFF，WBC会短暂得到[0 0 0 0]，从而只能给出自由落体解。
    int confirmed_planned_support_count = 0;
    for (int leg = 0; leg < 4; ++leg)
    {
        if (planned_contact[leg] == 1 && states_[leg] == FootContactState::STANCE && force_confirmed_[leg] == 1)
        {
            ++confirmed_planned_support_count;
        }
    }

    const int required_confirmed_support_count =
        std::min(config_.minimum_confirmed_support_legs, planned_contact.sum());
    const bool support_transfer_confirmed =
        required_confirmed_support_count == 0 ||
        confirmed_planned_support_count >= required_confirmed_support_count;

    for (int leg = 0; leg < 4; ++leg)
    {
        if (states_[leg] != FootContactState::LIFTOFF)
        {
            continue;
        }

        const double next_activation =
            activation_[leg] - safe_dt / config_.liftoff_force_ramp_duration;
        if (!support_transfer_confirmed)
        {
            // 新支撑集合没有建立前，只卸载到安全下限，并继续保留硬接触约束。
            activation_[leg] = std::max(config_.support_transfer_min_activation, next_activation);
            continue;
        }

        activation_[leg] = std::max(0.0, next_activation);
        const bool force_released =
            contact_loss_time_[leg] >= config_.liftoff_force_release_duration;
        if (activation_[leg] <= 0.0 || force_released)
        {
            transitionTo(leg, FootContactState::SWING);
        }
    }

    ContactStateOutput output;
    output.activation = activation_;
    output.filtered_normal_force = filtered_normal_force_;
    output.force_confirmed = force_confirmed_;

    for (int leg = 0; leg < 4; ++leg)
    {
        output.hard_contact[leg] =
            states_[leg] == FootContactState::STANCE || states_[leg] == FootContactState::LIFTOFF ? 1 : 0;
    }

    if (planned_contact.sum() > 0 && !support_transfer_confirmed)
    {
        unsupported_time_ += safe_dt;
    }
    else
    {
        unsupported_time_ = 0.0;
    }
    output.healthy = unsupported_time_ <= config_.max_unconfirmed_support_duration;

    return output;
}

FootContactState ContactStateManager::state(const int leg) const
{
    if (leg < 0 || leg >= 4)
    {
        throw std::out_of_range("Contact-state leg index is out of range");
    }
    return states_[leg];
}

double ContactStateManager::stateElapsedTime(const int leg) const
{
    if (leg < 0 || leg >= 4)
    {
        throw std::out_of_range("Contact-state leg index is out of range");
    }
    return state_elapsed_time_[leg];
}

const char *ContactStateManager::stateName(const FootContactState state)
{
    switch (state)
    {
    case FootContactState::SWING:
        return "SW";
    case FootContactState::TOUCHDOWN_PENDING:
        return "TD";
    case FootContactState::STANCE:
        return "ST";
    case FootContactState::LIFTOFF:
        return "LO";
    }
    return "??";
}

void ContactStateManager::transitionTo(const int leg, const FootContactState next_state)
{
    const FootContactState previous_state = states_[leg];
    states_[leg] = next_state;
    state_elapsed_time_[leg] = 0.0;
    contact_confirmation_time_[leg] = 0.0;
    contact_loss_time_[leg] = 0.0;

    if (next_state == FootContactState::SWING || next_state == FootContactState::TOUCHDOWN_PENDING)
    {
        activation_[leg] = 0.0;
        force_confirmed_[leg] = 0;
    }
    else if (next_state == FootContactState::STANCE && previous_state != FootContactState::LIFTOFF)
    {
        // 新触地从零接触力开始爬升；从LIFTOFF撤销则保留当前比例。
        activation_[leg] = 0.0;
    }
}

double ContactStateManager::sanitizeNormalForce(const double force)
{
    return std::isfinite(force) ? std::max(0.0, force) : 0.0;
}

} // namespace quadruped_controller
