// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rm_utils/math/trajectory_compensator.hpp"

namespace fyt {
bool TrajectoryCompensator::compensate(const Eigen::Vector3d &target_position,
                                       double &pitch) const noexcept {
  double target_height = target_position(2);
  // The iterative_height is used to calculate angle in each iteration
  double iterative_height = target_height;
  double impact_height = 0;
  double distance =
    std::sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
  double angle = std::atan2(target_height, distance);
  double dh = 0;
  // Iterate to find the right angle, which makes the impact height equal to the
  // target height
  for (int i = 0; i < iteration_times; ++i) {
    angle = std::atan2(iterative_height, distance);
    if (std::abs(angle) > M_PI / 2.5) {
      break;
    }
    impact_height = calculateTrajectory(distance, angle);
    dh = target_height - impact_height;
    if (std::abs(dh) < 0.01) {
      break;
    }
    iterative_height += dh;
  }
  if (std::abs(dh) > 0.01 || std::abs(angle) > M_PI / 2.5) {
    return false;
  }
  pitch = angle;
  return true;
}

std::vector<std::pair<double, double>> TrajectoryCompensator::getTrajectory(
  double distance, double angle) const noexcept {
  std::vector<std::pair<double, double>> trajectory;

  if (distance < 0) {
    return trajectory;
  }

  for (double x = 0; x < distance; x += 0.03) {
    trajectory.emplace_back(x, calculateTrajectory(x, angle));
  }
  return trajectory;
}

double IdealCompensator::calculateTrajectory(const double x, const double angle) const noexcept {
  double t = x / (velocity * cos(angle));
  double y = velocity * sin(angle) * t - 0.5 * gravity * t * t;
  return y;
}

double IdealCompensator::getFlyingTime(const Eigen::Vector3d &target_position) const noexcept {
  double distance =
    sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
  double angle = atan2(target_position(2), distance);
  double t = distance / (velocity * cos(angle));
  return t;
}

double ResistanceCompensator::calculateTrajectory(double x, double angle) const noexcept {
    constexpr double epsilon = 1e-8;
    const double cd = 0.47; // 阻力系数
    const double k_quad = 0.5 * this->air_density * cd * M_PI * pow(this->diameter / 2, 2); // 使用 this-> 访问成员变量

    // 校验输入有效性
    if (x <= epsilon || std::abs(angle) >= M_PI / 2 - epsilon) return NAN;

    // 初始速度分量
    const double vx0 = this->velocity * std::cos(angle);
    const double vy0 = this->velocity * std::sin(angle);

    // 使用 RK4 解算弹道
    State s{0, 0, vx0, vy0, 0};
    double dt = this->dt_base;

    for (int i = 0; i < this->max_steps; ++i) {
        const auto k1 = this->derivative(s, k_quad);
        const auto k2 = this->derivative(this->applyState(s, k1, dt / 2), k_quad);
        const auto k3 = this->derivative(this->applyState(s, k2, dt / 2), k_quad);
        const auto k4 = this->derivative(this->applyState(s, k3, dt), k_quad);

        s = {
            s.x + dt * (k1.dx + 2 * k2.dx + 2 * k3.dx + k4.dx) / 6,
            s.y + dt * (k1.dy + 2 * k2.dy + 2 * k3.dy + k4.dy) / 6,
            s.vx + dt * (k1.dvx + 2 * k2.dvx + 2 * k3.dvx + k4.dvx) / 6,
            s.vy + dt * (k1.dvy + 2 * k2.dvy + 2 * k3.dvy + k4.dvy) / 6,
            s.t + dt
        };

        // 当弹丸到达目标X坐标时
        if (s.x >= x) {
            const double alpha = (x - (s.x - dt * s.vx)) / (s.vx * dt);
            return (s.y - dt * s.vy) + alpha * dt * s.vy - 0.5 * 9.81 * pow((s.t - dt) + alpha * dt, 2);
        }
    }

    return NAN; // 未命中
}

double ResistanceCompensator::getFlyingTime(const Eigen::Vector3d& target_position) const noexcept {
    constexpr double epsilon = 1e-8;
    const double cd = 0.47; // 阻力系数
    const double k_quad = 0.5 * this->air_density * cd * M_PI * pow(this->diameter / 2, 2); // 使用 this-> 访问成员变量

    // 计算水平距离和仰角
    const double distance = target_position.head<2>().norm();
    const double vertical = target_position.z();
    if (distance <= epsilon) return NAN;

    // 计算理论发射角（需迭代优化，此处简化）
    const double angle = std::atan2(vertical, distance);

    // 使用 RK4 解算弹道
    State s{0, 0, this->velocity * std::cos(angle), this->velocity * std::sin(angle), 0};
    double dt = this->dt_base;

    for (int i = 0; i < this->max_steps; ++i) {
        const auto k1 = this->derivative(s, k_quad);
        const auto k2 = this->derivative(this->applyState(s, k1, dt / 2), k_quad);
        const auto k3 = this->derivative(this->applyState(s, k2, dt / 2), k_quad);
        const auto k4 = this->derivative(this->applyState(s, k3, dt), k_quad);

        s = {
            s.x + dt * (k1.dx + 2 * k2.dx + 2 * k3.dx + k4.dx) / 6,
            s.y + dt * (k1.dy + 2 * k2.dy + 2 * k3.dy + k4.dy) / 6,
            s.vx + dt * (k1.dvx + 2 * k2.dvx + 2 * k3.dvx + k4.dvx) / 6,
            s.vy + dt * (k1.dvy + 2 * k2.dvy + 2 * k3.dvy + k4.dvy) / 6,
            s.t + dt
        };

        // 当弹丸到达目标X坐标时
        if (s.x >= distance) {
            return s.t;
        }
    }

    return NAN; // 未命中
}
ResistanceCompensator::Derivative ResistanceCompensator::derivative(
    const State& s, double k_quad) const 
{
    const double v = std::max(hypot(s.vx, s.vy), 1e-5);
    return {
        s.vx,
        s.vy,
        -k_quad * v * s.vx / this->mass,
        -9.81 - k_quad * v * s.vy / this->mass
    };
}

ResistanceCompensator::State ResistanceCompensator::applyState(
    const State& s, const Derivative& d, double dt) const 
{
    return {
        s.x + d.dx * dt,
        s.y + d.dy * dt,
        s.vx + d.dvx * dt,
        s.vy + d.dvy * dt,
        s.t + dt
    };
}

};// namespace fyt
