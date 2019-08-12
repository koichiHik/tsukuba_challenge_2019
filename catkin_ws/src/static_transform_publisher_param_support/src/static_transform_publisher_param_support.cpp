/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include <tf/transform_broadcaster.h>

class TransformSender {
public:
  ros::NodeHandle node_;
  // constructor
  TransformSender(double x, double y, double z, double yaw, double pitch,
                  double roll, ros::Time time, const std::string &frame_id,
                  const std::string &child_frame_id) {
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    transform_ = tf::StampedTransform(tf::Transform(q, tf::Vector3(x, y, z)),
                                      time, frame_id, child_frame_id);
  };
  TransformSender(double x, double y, double z, double qx, double qy, double qz,
                  double qw, ros::Time time, const std::string &frame_id,
                  const std::string &child_frame_id)
      : transform_(
            tf::Transform(tf::Quaternion(qx, qy, qz, qw), tf::Vector3(x, y, z)),
            time, frame_id, child_frame_id){};
  // Clean up ros connections
  ~TransformSender() {}

  // A pointer to the rosTFServer class
  tf::TransformBroadcaster broadcaster;

  // A function to call to send data periodically
  void send(ros::Time time) {
    transform_.stamp_ = time;
    broadcaster.sendTransform(transform_);
  };

private:
  tf::StampedTransform transform_;
};

template <typename T>
void ReadParamAndWarnIfFail(const ros::NodeHandle &nh,
                            const std::string &param_name, T &param) {
  if (!nh.getParam(param_name, param)) {
    ROS_ERROR("Param read failuer : %s ", param_name.c_str());
  }
}

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "static_transform_publisher_param_support",
            ros::init_options::AnonymousName);

  // Node Handle
  ros::NodeHandle nh("~");

  // Read Params.
  double x, y, z, yaw, pitch, roll, period_in_ms;
  std::string frame_id, child_frame_id;

  ReadParamAndWarnIfFail(nh, "x", x);
  ReadParamAndWarnIfFail(nh, "y", y);
  ReadParamAndWarnIfFail(nh, "z", z);
  ReadParamAndWarnIfFail(nh, "yaw", yaw);
  ReadParamAndWarnIfFail(nh, "pitch", pitch);
  ReadParamAndWarnIfFail(nh, "roll", roll);
  ReadParamAndWarnIfFail(nh, "period_in_ms", period_in_ms);
  ReadParamAndWarnIfFail(nh, "frame_id", frame_id);
  ReadParamAndWarnIfFail(nh, "child_frame_id", child_frame_id);

  {
    ros::Duration sleeper(period_in_ms / 1000.0);

    if (strcmp(frame_id.c_str(), child_frame_id.c_str()) == 0)
      ROS_FATAL("target_frame and source frame are the same (%s, %s) this "
                "cannot work",
                frame_id.c_str(), child_frame_id.c_str());

    TransformSender tf_sender(
        x, y, z, yaw, pitch, roll,
        ros::Time() +
            sleeper, // Future dating to allow slower sending w/o timeout
        frame_id,
        child_frame_id);

    while (tf_sender.node_.ok()) {
      tf_sender.send(ros::Time::now() + sleeper);
      ROS_DEBUG("Sending transform from %s with parent %s\n", frame_id.c_str(),
                child_frame_id.c_str());
      sleeper.sleep();
    }
    return 0;
  }
};
