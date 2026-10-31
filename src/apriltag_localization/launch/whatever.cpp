// =============================================================================
// tag_localizer.cpp
//
// WHAT THIS NODE DOES
//   Figures out WHERE THE ROBOT IS IN THE WORLD by looking at AprilTags whose
//   world positions we already know.
//
// THE IDEA (one tag)
//   We know:  map -> tag        (we measured where we put the tag; from YAML)
//   We see:   base_link -> tag  (tf2 gives this: camera mount + what the
//                                camera saw, chained together)
//   We want:  map -> base_link  (where the robot is in the world)
//
//   map -> base_link  =  (map -> tag)  *  (base_link -> tag)^-1
//
//   In words: go from the map to the tag, then walk BACKWARDS from the tag to
//   the robot. "^-1" (inverse) means "flip the direction": if the tag is 2 m
//   in front of the robot, the robot is 2 m behind the tag.
//
// DATA FLOW
//   apriltag_ros --/detections--> [ tag_localizer ] --/tag_pose--> (EKF later)
//                                        ^
//                          tf2 lookup: base_link -> tag
//                          YAML: where each tag is in the map
// =============================================================================

// ---------------------------- Includes ---------------------------------------
// Standard C++ libraries
#include <memory>          // smart pointers: std::shared_ptr, std::unique_ptr
#include <string>          // std::string (text)
#include <unordered_map>   // lookup table (like a dictionary): ID -> value
#include <vector>          // resizable lists: std::vector

// ROS 2 core library (nodes, publishers, subscribers, parameters, logging)
#include "rclcpp/rclcpp.hpp"

// INPUT message type: the list of tags apriltag_ros detected in one image
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"

// OUTPUT message type: a pose (position + orientation) PLUS how much to trust it
// (covariance). This is the format the robot_localization EKF accepts.
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// tf2 "math form" objects. Unlike messages (which are just data), these can
// be multiplied (chained) and inverted (flipped).
#include "tf2/LinearMath/Quaternion.h"   // rotations
#include "tf2/LinearMath/Transform.h"    // rotation + position together

// Converts between "message form" and "math form" (fromMsg / toMsg)
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// tf2 machinery for receiving transforms from other nodes
#include "tf2_ros/buffer.h"              // storage for recent transforms
#include "tf2_ros/transform_listener.h"  // fills the storage from /tf, /tf_static


// ============================ The node =======================================
// "class TagLocalizer : public rclcpp::Node" means TagLocalizer IS a ROS node,
// so it automatically gets node abilities (logging, parameters, topics, clock).
class TagLocalizer : public rclcpp::Node
{
public:
  // ---------------------------------------------------------------------------
  // CONSTRUCTOR: runs ONCE when the node is created. This is all setup/wiring;
  // no detections are processed here.
  // ": Node("tag_localizer")" gives the node its name. That name must match the
  // top line of tag_map.yaml ("tag_localizer:") or the settings are ignored.
  // ---------------------------------------------------------------------------
  TagLocalizer() : Node("tag_localizer")
  {
    // ===== a. Simple settings ================================================
    // declare_parameter<TYPE>("name", default) means:
    //   "I have a setting called 'name'. If the YAML gives a value, use it;
    //    otherwise use the default."

    // Name of the world frame our output pose is measured in
    map_frame_ = declare_parameter<std::string>("map_frame", "map");

    // Name of the robot's body frame (its center)
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    // apriltag_ros names each tag's TF frame like "tag36h11:0", "tag36h11:3".
    // We store the part before the number and add the ID later.
    tag_prefix_ = declare_parameter<std::string>("tag_frame_prefix", "tag36h11:");

    // How much error we EXPECT in our answer (standard deviation).
    //   position_stddev 0.05 -> "usually within about 5 cm"
    //   angle_stddev    0.05 -> "usually within about 3 degrees" (0.05 rad)
    const double pos_std = declare_parameter<double>("position_stddev", 0.05);  // meters
    const double ang_std = declare_parameter<double>("angle_stddev", 0.05);     // radians

    // Covariance needs the VARIANCE, which is the standard deviation SQUARED.
    // We compute it once here instead of every frame.
    pos_var_ = pos_std * pos_std;
    ang_var_ = ang_std * ang_std;

    // ===== b. The tag map: where each tag is in the world ====================
    // The YAML stores seven PARALLEL lists. Position i in every list describes
    // the SAME tag. Example with two tags:
    //   tag_ids: [0,   3  ]
    //   tag_x:   [0.0, 2.0]   <- tag 0 at x = 0, tag 3 at x = 2
    //   tag_y:   [1.0, 1.0]
    //   ...
    // std::vector<double>{} is an empty list, used as the default if missing.
    const auto ids     = declare_parameter<std::vector<int64_t>>("tag_ids",   std::vector<int64_t>{});
    const auto xs      = declare_parameter<std::vector<double>>("tag_x",      std::vector<double>{});
    const auto ys      = declare_parameter<std::vector<double>>("tag_y",      std::vector<double>{});
    const auto zs      = declare_parameter<std::vector<double>>("tag_z",      std::vector<double>{});
    const auto rolls   = declare_parameter<std::vector<double>>("tag_roll",   std::vector<double>{});
    const auto pitches = declare_parameter<std::vector<double>>("tag_pitch",  std::vector<double>{});
    const auto yaws    = declare_parameter<std::vector<double>>("tag_yaw",    std::vector<double>{});

    // Safety check: every list must have the same length. If someone forgets a
    // number in one list, values would get paired with the wrong tag. Better to
    // stop with a clear error than to silently give wrong positions.
    // ("||" means "or"; ".size()" is how many items a list has.)
    const size_t n = ids.size();
    if (xs.size() != n || ys.size() != n || zs.size() != n ||
      rolls.size() != n || pitches.size() != n || yaws.size() != n)
    {
      RCLCPP_WARN(get_logger(), "All tag_* lists in the YAML must have the same length.");
      throw std::runtime_error("bad tag map");  // stops the node
    }

    // Turn each tag's 6 numbers into one tf2::Transform and store it.
    for (size_t i = 0; i < n; ++i) {
      // Convert human-friendly angles (roll, pitch, yaw in radians) into a
      // quaternion, which is how tf2 stores rotations internally.
      tf2::Quaternion q;
      q.setRPY(rolls[i], pitches[i], yaws[i]);

      // A Transform = rotation + position, i.e. "where the tag is AND which
      // way it faces". Store it in the lookup table under its ID, so later
      // tag_in_map_[0] answers "where is tag 0 in the world?"
      tag_in_map_[ids[i]] = tf2::Transform(q, tf2::Vector3(xs[i], ys[i], zs[i]));
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu tag(s) into the map.", n);  // %zu = print a size

    // ===== c. tf2: "where is X relative to Y?" ===============================
    // BUFFER:   storage that remembers recent transforms (about the last 10 s).
    // LISTENER: quietly subscribes to /tf and /tf_static in the background and
    //           fills the buffer. It includes your camera mount (static TF),
    //           the RealSense's internal frames, and camera -> tag from
    //           apriltag_ros. tf2 can then chain any of them for us.
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ===== d. Input and output ===============================================
    // INPUT: listen to apriltag_ros's "detections" topic. Queue size 10 =
    // hold up to 10 waiting messages if we fall behind.
    // The [this](...) { ... } part is a lambda (tiny nameless function) that
    // ROS calls for every message; it just hands the message to on_detections.
    sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "detections", 10,
      [this](const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
        on_detections(msg);
      });

    // OUTPUT: we'll publish the robot's world pose on "tag_pose".
    pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("tag_pose", 10);
  }

private:
  // ---------------------------------------------------------------------------
  // CALLBACK: runs EVERY time apriltag_ros sends detections (every camera frame).
  // One message can contain several tags, so we handle each one.
  // ---------------------------------------------------------------------------
  void on_detections(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    // "for each detection in the message"
    //   auto = let C++ figure out the type
    //   &    = use the original, don't copy it (faster)
    //   const = promise not to change it
    for (const auto & det : msg->detections) {

      // ===== Step 0: is this a tag we know? ==================================
      // find() searches the lookup table. If the ID isn't there, it returns
      // end() ("not found"), e.g. a random tag someone else put up.
      const auto it = tag_in_map_.find(det.id);
      if (it == tag_in_map_.end()) {
        continue;  // skip to the next detection
      }
      // Found it. it->second is the stored value: this tag's world transform.
      // (it->first would be the ID itself.)
      const tf2::Transform & map_to_tag = it->second;

      // ===== Step 1: where is the tag relative to the ROBOT? =================
      // Build the frame name apriltag_ros uses, e.g. "tag36h11:" + "0".
      const std::string tag_frame = tag_prefix_ + std::to_string(det.id);

      geometry_msgs::msg::TransformStamped base_to_tag_msg;  // will hold the answer

      // lookupTransform(target, source, time, wait) asks tf2:
      //   "Where is tag_frame relative to base_link, at the moment this image
      //    was taken? Wait up to 0.1 s for the data if it isn't here yet."
      // Using the IMAGE's timestamp matters: if the robot moves, the answer
      // must match the instant the photo was taken, not a moment later.
      //
      // lookupTransform THROWS an error if it can't answer (e.g. at startup
      // before TF data has arrived, or a frame name is wrong). try/catch
      // handles that instead of crashing the whole node.
      try {
        base_to_tag_msg = tf_buffer_->lookupTransform(
          base_frame_, tag_frame, rclcpp::Time(msg->header.stamp),
          rclcpp::Duration::from_seconds(0.1));
      } catch (const tf2::TransformException & ex) {
        // _THROTTLE + 2000 = print this at most once every 2000 ms, so the
        // terminal isn't flooded at 30 frames per second.
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Can't look up %s -> %s: %s", base_frame_.c_str(), tag_frame.c_str(), ex.what());
        continue;  // skip this tag, try the next
      }

      // The answer arrives in "message form" (plain data). Convert it into
      // "math form" so we can invert and multiply it.
      tf2::Transform base_to_tag;
      tf2::fromMsg(base_to_tag_msg.transform, base_to_tag);

      // ===== Step 2: THE localization formula ================================
      //   map_to_tag            : map  -> tag   (we measured it)
      //   base_to_tag.inverse() : tag  -> robot (flip what the camera saw)
      //   *                     : "do this, then that" (chain them)
      // Result: map -> robot = where the robot is in the world.
      // Note: "*" here is NOT normal multiplication; for transforms it means
      // following one set of directions after another.
      const tf2::Transform map_to_base = map_to_tag * base_to_tag.inverse();

      // ===== Step 3: package the answer and publish it =======================
      geometry_msgs::msg::PoseWithCovarianceStamped out;

      // Same timestamp as the image, so others know WHEN this was true.
      out.header.stamp = msg->header.stamp;

      // The position is measured in the "map" frame (the world).
      out.header.frame_id = map_frame_;

      // Math form -> message form, written into the pose's position/orientation.
      tf2::toMsg(map_to_base, out.pose.pose);

      // ----- Covariance: how much to trust this pose -----
      // A pose has 6 values (x, y, z, roll, pitch, yaw), so covariance is a
      // 6x6 grid = 36 numbers, stored row by row ("row major"):
      //
      //          x    y    z    roll pitch yaw
      //   x    [ 0    1    2    3    4    5  ]
      //   y    [ 6    7    8    9    10   11 ]
      //   z    [ 12   13   14   15   16   17 ]
      //   roll [ 18   19   20   21   22   23 ]
      //   pitch[ 24   25   26   27   28   29 ]
      //   yaw  [ 30   31   32   33   34   35 ]
      //
      // We only fill the DIAGONAL (0, 7, 14, 21, 28, 35): each value's own
      // uncertainty. The rest stay 0 ("errors in different values aren't
      // related"). The EKF uses these to weigh tags vs. other sensors:
      // smaller number = trusted more.
      out.pose.covariance[0]  = pos_var_;   // x
      out.pose.covariance[7]  = pos_var_;   // y
      out.pose.covariance[14] = pos_var_;   // z
      out.pose.covariance[21] = ang_var_;   // roll
      out.pose.covariance[28] = ang_var_;   // pitch
      out.pose.covariance[35] = ang_var_;   // yaw

      // Send it out on /tag_pose. One message per recognized tag.
      pub_->publish(out);
    }
  }

  // ---------------------------------------------------------------------------
  // MEMBER VARIABLES: the node's memory. They live as long as the node does,
  // unlike variables inside a function, which vanish when it ends.
  // The trailing underscore (map_frame_) means "belongs to this class".
  // ---------------------------------------------------------------------------

  // Settings read from the YAML
  std::string map_frame_, base_frame_, tag_prefix_;
  double pos_var_{0.0}, ang_var_{0.0};

  // Lookup table: tag ID -> that tag's position/orientation in the world
  std::unordered_map<int64_t, tf2::Transform> tag_in_map_;

  // tf2 storage + background receiver. These must stay alive for the whole
  // run, or the listener stops receiving transforms.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Input (detections) and output (robot pose)
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};


// ============================ main ===========================================
// The "on switch". Same three steps as every node:
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                          // 1. start ROS 2
  rclcpp::spin(std::make_shared<TagLocalizer>());    // 2. create node (runs constructor),
                                                     //    then wait for messages forever
  rclcpp::shutdown();                                // 3. clean up after Ctrl+C
  return 0;
}