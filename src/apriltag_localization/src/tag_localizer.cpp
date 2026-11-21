# include <cmath>
# include <memory>
# include <stdexcept>
# include <string>
# include <unordered_map>
# include <vector>
# include "rclcpp/rclcpp.hpp"
# include "perception/msg/tag_observation_array.hpp"
# include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
# include "tf2/LinearMath/Quaternion.h"
# include "tf2/LinearMath/Transform.h"
# include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class TagLocalizer : public rclcpp::Node{
public:
    TagLocalizer() : Node("tag_localizer"){
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        if (base_frame_.empty()) {
            throw std::invalid_argument("Set base_frame to the expected tag observation frame.");
        }
        

        const double pos_std = declare_parameter<double>("position_stddev", 0.05);  // meters sd
        const double ang_std = declare_parameter<double>("angle_stddev", 0.05);     // radians sd
        move_threshold_ = declare_parameter<double>("move_threshold", 0.1); // meters

        pos_var_ = pos_std * pos_std; // variance
        ang_var_ = ang_std * ang_std;

        const auto ids     = declare_parameter<std::vector<int64_t>>("tag_ids",   std::vector<int64_t>{});
        const auto xs      = declare_parameter<std::vector<double>>("tag_x",      std::vector<double>{});
        const auto ys      = declare_parameter<std::vector<double>>("tag_y",      std::vector<double>{});
        const auto zs      = declare_parameter<std::vector<double>>("tag_z",      std::vector<double>{});
        const auto rolls   = declare_parameter<std::vector<double>>("tag_roll",   std::vector<double>{});
        const auto pitches = declare_parameter<std::vector<double>>("tag_pitch",  std::vector<double>{});
        const auto yaws    = declare_parameter<std::vector<double>>("tag_yaw",    std::vector<double>{});

        const size_t n = ids.size();

        // check that every list has same length
        if (xs.size() != n || ys.size() != n || zs.size() != n || rolls.size() != n || pitches.size() != n || yaws.size() != n) {
            RCLCPP_WARN(get_logger(), "All tag_* lists in the YAML must have the same length.");
            throw std::runtime_error("bad tag map");  // stops the node
            }

        // Turn each tag's 6 numbers into one tf2::Transform and store it.
        for (size_t i = 0; i < n; ++i) {
            tf2::Quaternion q;
            q.setRPY(rolls[i], pitches[i], yaws[i]);
            tag_in_map_[ids[i]] = tf2::Transform(q, tf2::Vector3(xs[i], ys[i], zs[i]));
        }
        RCLCPP_INFO(get_logger(), "Loaded %zu tag(s) into the map.", n);  // %zu = print a size

        sub_ = create_subscription<perception::msg::TagObservationArray>(
            "tag/observations", 10,
            [this](perception::msg::TagObservationArray::ConstSharedPtr msg) {on_observations(msg);
            });
        pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped> ("tag_pose",10);
    }
private:
    void on_observations(const perception::msg::TagObservationArray::ConstSharedPtr msg){
        if (msg->header.frame_id != base_frame_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Ignoring tag observations in frame %s; expected %s.",
                msg->header.frame_id.c_str(), base_frame_.c_str());
            return;
        }
        for (const auto & observation:msg->observations){
            const auto it = tag_in_map_.find(observation.id); // find id
            if (it == tag_in_map_.end()) {
                continue;  // to the next detection
                }
            
            const tf2::Transform & map_to_tag = it->second; // first is id itself, second is stored value
            tf2::Transform base_to_tag;
            tf2::fromMsg(observation.transform, base_to_tag);
            
            const tf2::Transform map_to_base = map_to_tag * base_to_tag.inverse();

            geometry_msgs::msg::PoseWithCovarianceStamped out;

            out.header.stamp = msg->header.stamp; // Same timestamp as the image
            out.header.frame_id = map_frame_; // position measured in world frame

            tf2::toMsg(map_to_base, out.pose.pose);
            double move_distance_squared = (out.pose.pose.position.x-last_pose_.pose.pose.position.x) * (out.pose.pose.position.x-last_pose_.pose.pose.position.x) +
                                                   (out.pose.pose.position.y-last_pose_.pose.pose.position.y) * (out.pose.pose.position.y-last_pose_.pose.pose.position.y) +
                                                   (out.pose.pose.position.z-last_pose_.pose.pose.position.z) * (out.pose.pose.position.z-last_pose_.pose.pose.position.z);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Robot moved %.2f meters", sqrt(move_distance_squared));
            if (has_last_pose_ &&
                move_distance_squared > move_threshold_ * move_threshold_) {
                // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Robot moved %.2f meters, more than %.2f meters limit from the last pose estimate. Ignoring this pose.", sqrt(move_distance_squared), move_threshold_);
                last_pose_ = out;
                continue; // Skip publishing this pose
            }

            out.pose.covariance[0]  = pos_var_; // x
            out.pose.covariance[7]  = pos_var_; // y
            out.pose.covariance[14] = pos_var_; // z
            out.pose.covariance[21] = ang_var_; // roll
            out.pose.covariance[28] = ang_var_; // pitch
            out.pose.covariance[35] = ang_var_; // yaw
            last_pose_ = out; // Store the last published pose
            has_last_pose_ = true;
            pub_->publish(out);
        }
    }

    std::string map_frame_, base_frame_; // take readings from .yaml
    double pos_var_{0.0}, ang_var_{0.0};

    std::unordered_map<int64_t, tf2::Transform> tag_in_map_;
    rclcpp::Subscription<perception::msg::TagObservationArray>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
    double move_threshold_; // meters
    geometry_msgs::msg::PoseWithCovarianceStamped last_pose_; // Store the last published pose
    bool has_last_pose_{false};

};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TagLocalizer>());
    rclcpp::shutdown();

    return 0;
}
