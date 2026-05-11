#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <ctime>

class FKNode : public rclcpp::Node
{
public:
    FKNode() : Node("fk_node")
    {
        // Get robot_description from robot_state_publisher
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
        this, "/robot_state_publisher");

        if (!parameters_client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_logger(), "Could not connect to robot_state_publisher");
            return;
        }

        std::string robot_desc_string =
        parameters_client->get_parameter<std::string>("robot_description");

        // Get kinematics specific configuration
        urdf::Model robot_model;

        // Build a kinematic chain of the robot
        if (!robot_model.initString(robot_desc_string)) {
            RCLCPP_ERROR(get_logger(), "Failed to parse URDF from robot_description");
            return;
        }
        if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree)) {
            RCLCPP_ERROR(get_logger(), "Failed to parse KDL tree from urdf model");
            return;
        }
        if (!robot_tree.getChain("base_link", "tool0", main_chain)) {
            RCLCPP_ERROR(get_logger(), "Failed to parse robot chain to tool0 from URDF model.");
            return;
        }

        // Create FK solvers using full tree for position (can query any link)
        fk_pos_solver = std::make_shared<KDL::TreeFkSolverPos_recursive>(robot_tree);

        // Resize joint arrays (use size from main chain)
        joint_positions.resize(main_chain.getNrOfJoints());
        joint_velocities.resize(main_chain.getNrOfJoints());

        // Build joint name to index mapping (only count real joints, skip fixed joints)
        size_t idx = 0;
        for (const auto &seg : main_chain.segments) {
            if (seg.getJoint().getType() != KDL::Joint::None) {
                kdl_joint_map[seg.getJoint().getName()] = idx++;
            }
        }

        // Create publishers and subscriber
        rclcpp::QoS qos(3);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        ee_pub = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
            create_publisher<geometry_msgs::msg::PoseStamped>("tool0_pos", qos));

        joint_sub = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", qos,
        std::bind(&FKNode::jointCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "FK node initialized, listening to /joint_states");
    }

private:
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < main_chain.getNrOfJoints()) {
            RCLCPP_WARN(get_logger(), "Not enough joint positions received");
            return;
        }

        // Update joint positions and velocities (direct index mapping)
        for (size_t i = 0; i < msg->name.size(); ++i) {
            auto it = kdl_joint_map.find(msg->name[i]);
            if (it != kdl_joint_map.end()) {
                joint_positions(it->second) = msg->position[i];
                joint_velocities(it->second) = msg->velocity[i];
            }
        }
        
        bool success = (fk_pos_solver->JntToCart(joint_positions, current_ee, "tool0") >= 0);

        if (!success) {
            RCLCPP_WARN(get_logger(), "FK computation failed");
            return;
        }

        // Publish ee
        publishPose(ee_pub, current_ee);
    }

    void publishPose(const std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>& pub,
                     const KDL::Frame& frame)
    {
        if (pub->trylock()) {
            auto& msg = pub->msg_;
            msg.header.stamp = now();
            msg.header.frame_id = "base_link";

            msg.pose.position.x = frame.p.x();
            msg.pose.position.y = frame.p.y();
            msg.pose.position.z = frame.p.z();

            double x, y, z, w;
            frame.M.GetQuaternion(x, y, z, w);
            msg.pose.orientation.x = x;
            msg.pose.orientation.y = y;
            msg.pose.orientation.z = z;
            msg.pose.orientation.w = w;

            pub->unlockAndPublish();
        }
    }
    
    KDL::Tree robot_tree;
    KDL::Chain main_chain;
    KDL::JntArray joint_positions;
    KDL::JntArray joint_velocities;
    std::shared_ptr<KDL::TreeFkSolverPos_recursive> fk_pos_solver;
    std::map<std::string, size_t> kdl_joint_map;

    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>> ee_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FKNode>());
    rclcpp::shutdown();
    return 0;
}
