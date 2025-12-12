#pragma once

#include <deque>
#include <mutex>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

class OdometryBuffer {
public:
    struct OdomData {
        double timestamp;
        Eigen::Matrix4f pose;
        Eigen::Vector3f vel_linear;
        Eigen::Vector3f vel_angular;
    };

    OdometryBuffer() = default;

    void addMeasurement(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mMutex);
        
        OdomData data;
        data.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        
        Eigen::Quaternionf q(
            static_cast<float>(msg->pose.pose.orientation.w),
            static_cast<float>(msg->pose.pose.orientation.x),
            static_cast<float>(msg->pose.pose.orientation.y),
            static_cast<float>(msg->pose.pose.orientation.z)
        );
        q.normalize();
        
        Eigen::Vector3f t(
            static_cast<float>(msg->pose.pose.position.x),
            static_cast<float>(msg->pose.pose.position.y),
            static_cast<float>(msg->pose.pose.position.z)
        );
        
        data.pose = Eigen::Matrix4f::Identity();
        data.pose.block<3,3>(0,0) = q.toRotationMatrix();
        data.pose.block<3,1>(0,3) = t;
        
        data.vel_linear = Eigen::Vector3f(
            static_cast<float>(msg->twist.twist.linear.x),
            static_cast<float>(msg->twist.twist.linear.y),
            static_cast<float>(msg->twist.twist.linear.z)
        );
        data.vel_angular = Eigen::Vector3f(
            static_cast<float>(msg->twist.twist.angular.x),
            static_cast<float>(msg->twist.twist.angular.y),
            static_cast<float>(msg->twist.twist.angular.z)
        );
        
        mBuffer.push_back(data);
        
        while (mBuffer.size() > 400) {
            mBuffer.pop_front();
        }
    }

    bool getDeltaPose(double t1, double t2, 
                      Eigen::Matrix4f& delta_pose, 
                      float& translation_norm,
                      float& rotation_norm) {
        std::lock_guard<std::mutex> lock(mMutex);
        
        if (mBuffer.size() < 2) return false;
        
        OdomData odom1, odom2;
        if (!interpolate(t1, odom1) || !interpolate(t2, odom2)) {
            return false;
        }
        
        Eigen::Matrix4f T1_inv = Eigen::Matrix4f::Identity();
        T1_inv.block<3,3>(0,0) = odom1.pose.block<3,3>(0,0).transpose();
        T1_inv.block<3,1>(0,3) = -T1_inv.block<3,3>(0,0) * odom1.pose.block<3,1>(0,3);
        
        delta_pose = T1_inv * odom2.pose;
        translation_norm = delta_pose.block<3,1>(0,3).norm();
        
        Eigen::Matrix3f R_delta = delta_pose.block<3,3>(0,0);
        Eigen::AngleAxisf aa(R_delta);
        rotation_norm = std::abs(aa.angle());
        
        return true;
    }

    size_t size() {
        std::lock_guard<std::mutex> lock(mMutex);
        return mBuffer.size();
    }

private:
    bool interpolate(double t, OdomData& result) {
        if (mBuffer.empty()) return false;
        
        if (t <= mBuffer.front().timestamp) {
            result = mBuffer.front();
            return true;
        }
        if (t >= mBuffer.back().timestamp) {
            result = mBuffer.back();
            return true;
        }
        
        auto it = mBuffer.begin();
        while (it != mBuffer.end() && it->timestamp < t) ++it;
        
        if (it == mBuffer.begin()) {
            result = *it;
            return true;
        }
        
        auto prev = std::prev(it);
        double dt = it->timestamp - prev->timestamp;
        if (dt < 1e-9) {
            result = *prev;
            return true;
        }
        
        float alpha = static_cast<float>((t - prev->timestamp) / dt);
        alpha = std::max(0.0f, std::min(1.0f, alpha));
        
        Eigen::Vector3f t_interp = (1.0f - alpha) * prev->pose.block<3,1>(0,3) 
                                   + alpha * it->pose.block<3,1>(0,3);
        
        Eigen::Quaternionf q1(prev->pose.block<3,3>(0,0));
        Eigen::Quaternionf q2(it->pose.block<3,3>(0,0));
        Eigen::Quaternionf q_interp = q1.slerp(alpha, q2);
        
        result.timestamp = t;
        result.pose = Eigen::Matrix4f::Identity();
        result.pose.block<3,3>(0,0) = q_interp.toRotationMatrix();
        result.pose.block<3,1>(0,3) = t_interp;
        result.vel_linear = (1.0f - alpha) * prev->vel_linear + alpha * it->vel_linear;
        result.vel_angular = (1.0f - alpha) * prev->vel_angular + alpha * it->vel_angular;
        
        return true;
    }

    std::deque<OdomData> mBuffer;
    std::mutex mMutex;
};
