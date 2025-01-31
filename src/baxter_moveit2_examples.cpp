#include <chrono>
#include <cmath>
#include <control_msgs/action/detail/follow_joint_trajectory__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <ignition/msgs/empty.pb.h>
#include <ignition/msgs/link.pb.h>
#include <ignition/msgs/model.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/visual.pb.h>
#include <memory>
#include <moveit/utils/moveit_error_code.h>
#include <moveit_msgs/msg/detail/collision_object__struct.hpp>
#include <moveit_msgs/msg/detail/planning_scene__struct.hpp>
#include <moveit_msgs/msg/detail/robot_trajectory__struct.hpp>
#include <ostream>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <shape_msgs/msg/detail/solid_primitive__struct.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h> 
#include <moveit/move_group_interface/move_group_interface.h> 
#include <stdexcept>
#include <string>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <trajectory_msgs/msg/detail/joint_trajectory_point__struct.hpp>
#include <utility>

#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/scene.pb.h> 

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <tuple>

#include "rviz_gazebo_bridge/rviz_gazebo_bridge.h"

class BaxterMoveit2Examples
{
public:
    BaxterMoveit2Examples(const rclcpp::NodeOptions& options, const std::string& robot_name);
    void createObstacles();
    void run();
    void rviz2gazebo_example();
    void move_left_arm();
    void move_part(moveit::planning_interface::MoveGroupInterface& mgi, bool async_execute);
    void sleep();

    void gazebo2rviz_example();

    void move_both_arms();
    void move_both_arms2();

    rclcpp::Node::SharedPtr node_;

private:
    moveit::planning_interface::MoveGroupInterface move_group_interface_left_arm;
    moveit::planning_interface::MoveGroupInterface move_group_interface_right_arm;
    moveit::planning_interface::MoveGroupInterface move_group_interface_both_arms;

    rclcpp::Publisher<control_msgs::action::FollowJointTrajectory::Goal>::SharedPtr left_arm_goal_publisher; //for controlling real baxter
    rclcpp::Publisher<control_msgs::action::FollowJointTrajectory::Goal>::SharedPtr right_arm_goal_publisher; //for controlling real baxter
    std::shared_ptr<RVizGazeboBridge> rviz_gazebo_bridge;
};

static rclcpp::Logger LOGGER = rclcpp::get_logger("baxter_moveit2_examples_node");

BaxterMoveit2Examples::BaxterMoveit2Examples(const rclcpp::NodeOptions& options, const std::string& robot_name="baxter")
                                                                    :
                                                                    node_(std::make_shared<rclcpp::Node>("baxter_moveit2_examples_node", options)),
                                                                        move_group_interface_left_arm(this->node_, "left_arm"), //group is from .srdf-a
                                                                    move_group_interface_right_arm(this->node_, "right_arm"),
                                                                    move_group_interface_both_arms(this->node_, "both_arms"),
                                                                    rviz_gazebo_bridge(std::make_shared<RVizGazeboBridge>(robot_name))
                                                                    
{
    this->left_arm_goal_publisher = this->node_->create_publisher<control_msgs::action::FollowJointTrajectory::Goal>("/robot/limb/left/follow_joint_trajectory_bridge", 10);
    this->right_arm_goal_publisher = this->node_->create_publisher<control_msgs::action::FollowJointTrajectory::Goal>("/robot/limb/right/follow_joint_trajectory_bridge", 10);

}

void BaxterMoveit2Examples::rviz2gazebo_example() //for creating collision objects, loading them from planning scene and spawning them into gazebo
{
    this->createObstacles();
    this->rviz_gazebo_bridge->load_from_rviz_to_gazebo();
}

void BaxterMoveit2Examples::gazebo2rviz_example()
{
    this->rviz_gazebo_bridge->load_from_gazebo_to_rviz();
    
    moveit::planning_interface::PlanningSceneInterface psi;
    const auto &collision_objects = psi.getObjects();
    RCLCPP_INFO_STREAM(LOGGER, "added collision object in planning scene:");
    for (const auto &coll_obj : collision_objects)
    {
        RCLCPP_INFO_STREAM(LOGGER, "  " << coll_obj.first);
    }
}

void BaxterMoveit2Examples::sleep()
{
    int seconds = 4;
    const std::chrono::nanoseconds sleep_time = std::chrono::seconds(seconds);
    RCLCPP_INFO_STREAM(LOGGER, "sleeping for " << seconds << " seconds");
    rclcpp::sleep_for(sleep_time);
    RCLCPP_INFO_STREAM(LOGGER, "Resuming");
}

std::tuple<control_msgs::action::FollowJointTrajectory::Goal, control_msgs::action::FollowJointTrajectory::Goal>
construct_goals(const moveit_msgs::msg::RobotTrajectory::_joint_trajectory_type& traj)
{
    auto goal_left = control_msgs::action::FollowJointTrajectory::Goal();
    auto goal_right = control_msgs::action::FollowJointTrajectory::Goal();

    //header
    goal_left.trajectory.header = goal_right.trajectory.header = traj.header;

    //joint_names
    std::copy(traj.joint_names.begin(), traj.joint_names.begin() + 7, std::back_inserter(goal_left.trajectory.joint_names)); 
    std::copy(traj.joint_names.begin() + 7, traj.joint_names.end(), std::back_inserter(goal_right.trajectory.joint_names)); 

    //points
    for (auto &point : traj.points)
    {
        //left_point
        auto left_point = trajectory_msgs::msg::JointTrajectoryPoint();
        //right point
        auto right_point = trajectory_msgs::msg::JointTrajectoryPoint();
        
        if (point.positions.size() == 14)
        {
            std::copy(point.positions.begin(), point.positions.begin() + 7, std::back_inserter(left_point.positions));
            std::copy(point.positions.begin() + 7, point.positions.end(), std::back_inserter(right_point.positions));
        }

        if (point.velocities.size() == 14)
        {
            std::copy(point.velocities.begin(), point.velocities.begin() + 7, std::back_inserter(left_point.velocities));
            std::copy(point.velocities.begin() + 7, point.velocities.end(), std::back_inserter(right_point.velocities));
        }

        if (point.accelerations.size() == 14)
        {
            std::copy(point.accelerations.begin(), point.accelerations.begin() + 7, std::back_inserter(left_point.accelerations));
            std::copy(point.accelerations.begin() + 7, point.accelerations.end(), std::back_inserter(right_point.accelerations));
        }

        if (point.effort.size() == 14)
        {
            std::copy(point.effort.begin(), point.effort.begin() + 7, std::back_inserter(left_point.effort)); 
            std::copy(point.effort.begin() + 7, point.effort.end(), std::back_inserter(right_point.effort));
        }

        left_point.time_from_start = point.time_from_start;
        goal_left.trajectory.points.push_back(left_point);

        right_point.time_from_start = point.time_from_start;
        goal_right.trajectory.points.push_back(right_point);
    }
    //goal time tolerance
    goal_left.goal_time_tolerance = goal_right.goal_time_tolerance = rclcpp::Duration::from_seconds(0.1);

    return {goal_left, goal_right};

};

void BaxterMoveit2Examples::move_part(moveit::planning_interface::MoveGroupInterface& mgi, bool async_execute = false)
{
    mgi.setMaxAccelerationScalingFactor(0.8); 
    mgi.setMaxVelocityScalingFactor(0.8); 
    mgi.setNumPlanningAttempts(10);
    mgi.setPlanningTime(10);
    mgi.setGoalOrientationTolerance(0.01);

    auto const [success, plan] = [&mgi]()
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto ok = mgi.plan(msg);
        
        return std::make_pair(ok, msg);
    }();

    if (success == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(LOGGER, "[MOJE] BaxterMoveit2Examples::move_part(), group that this move group interface operates on: %s", mgi.getName().c_str());
        auto mgi_name = mgi.getName();
        if (mgi_name.find("both") != std::string::npos) //for both arms
        {
            auto [goal_left, goal_right] = construct_goals(plan.trajectory_.joint_trajectory);

            auto left_pub_thread = std::make_unique<std::thread>(
                [this, goal_left]()
                {
                    this->left_arm_goal_publisher->publish(goal_left);
                    RCLCPP_INFO(LOGGER, "[MOJE] BaxterMoveit2Examples::move_part(), goal message published to topic: %s", this->left_arm_goal_publisher->get_topic_name());
                }
            );

            auto right_pub_thread = std::make_unique<std::thread>(
                [this, goal_right]()
                {
                    this->right_arm_goal_publisher->publish(goal_right);
                    RCLCPP_INFO(LOGGER, "[MOJE] BaxterMoveit2Examples::move_part(), goal message published to topic: %s", this->right_arm_goal_publisher->get_topic_name());
                }
            );

            left_pub_thread->detach();
            right_pub_thread->detach();
        } 
        else
        {
            rclcpp::Publisher<control_msgs::action::FollowJointTrajectory::Goal>::SharedPtr pub;
            if (mgi_name.find("left") != std::string::npos)
            {
                pub = this->left_arm_goal_publisher;
            }
            else
            {
                pub = this->right_arm_goal_publisher;
            }
            //publish
            auto pub_thread = std::make_unique<std::thread>( 
            [plan, &pub]()
                    {
                        auto goal = control_msgs::action::FollowJointTrajectory::Goal();
                        goal.trajectory = plan.trajectory_.joint_trajectory;
                        goal.goal_time_tolerance = rclcpp::Duration::from_seconds(0.1);
                        pub->publish(goal);
                        RCLCPP_INFO(LOGGER, "[MOJE] BaxterMoveit2Examples::move_part(), goal message published to topic: %s", pub->get_topic_name());
                    }
                );
            pub_thread->detach();
        }

        if (async_execute)
            mgi.asyncExecute(plan);
        else
            mgi.execute(plan);
    } 
    else
    {
        RCLCPP_ERROR(LOGGER, "Planning failed!");
        throw std::runtime_error("Trajectory execution failed, stopping execution.");
    }
}

void BaxterMoveit2Examples::move_left_arm()
{
    this->move_group_interface_left_arm.setPositionTarget(0.8, 0.86, 0.8);
    this->move_group_interface_left_arm.setPlannerId("RRTConnectkConfigDefault");
    
    this->move_part(this->move_group_interface_left_arm);
    this->sleep();
    this->move_group_interface_left_arm.setPositionTarget(0.9, 0.86-0.8, 0.8);
    this->move_part(this->move_group_interface_left_arm);
}

void print_pose(const std::string& pre_text, const geometry_msgs::msg::PoseStamped& pose)
{
    RCLCPP_INFO(LOGGER, "%s", pre_text.c_str());
    RCLCPP_INFO(LOGGER, "Position - x: %.2f, y: %.2f, z: %.2f",
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z);

    // Log the orientation
    RCLCPP_INFO(LOGGER, "Orientation - x: %.2f, y: %.2f, z: %.2f, w: %.2f",
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w);
}

void print_joint_values_target(const std::string& pre_text, const std::vector<double>& joint_vals)
{
    std::string str = "";
    for (const auto &val : joint_vals)
    {
        str += std::to_string(val) + " ";
    }
    RCLCPP_INFO(LOGGER, "%s: %s", pre_text.c_str(), str.c_str());   
}

void BaxterMoveit2Examples::move_both_arms()
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.8;
    pose.position.y = 0.86;
    pose.position.z = 0.8;
    pose.orientation.x = -0.05;
    pose.orientation.y = 0.87;
    pose.orientation.z = 0.02;
    pose.orientation.w = 0.49;

    geometry_msgs::msg::Pose right_arm_goal_pose;
    right_arm_goal_pose.position.x = 0.9;
    right_arm_goal_pose.position.y = 0.06;
    right_arm_goal_pose.position.z = 0.8;
    right_arm_goal_pose.orientation.x = -0.51;
    right_arm_goal_pose.orientation.y = 0.58;
    right_arm_goal_pose.orientation.z = 0.18;
    right_arm_goal_pose.orientation.w = 0.61;

    RCLCPP_INFO(LOGGER, "left arm end-effector link: %s", this->move_group_interface_left_arm.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "right arm end-effector link: %s", this->move_group_interface_right_arm.getEndEffectorLink().c_str());
    this->move_group_interface_both_arms.setPoseTarget(pose, this->move_group_interface_left_arm.getEndEffectorLink()); 
    this->move_group_interface_both_arms.setPoseTarget(right_arm_goal_pose, this->move_group_interface_right_arm.getEndEffectorLink()); 
    this->move_part(this->move_group_interface_both_arms);
}

void BaxterMoveit2Examples::move_both_arms2() //kao prekrizene ruke,, radi :)
{
    auto left_arm_goal_pose = [](){
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.603;
        pose.position.y = -0.231;
        pose.position.z = 0.711;
        pose.orientation.x = 0.737;
        pose.orientation.y = 0.302;
        pose.orientation.z = -0.178;
        pose.orientation.w = 0.578;

        return pose;
    }();

    auto right_arm_goal_pose = [](){
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.788;
        pose.position.y = 0.298;
        pose.position.z = 1.048;
        pose.orientation.x = -0.718;
        pose.orientation.y = -0.419;
        pose.orientation.z = -0.201;
        pose.orientation.w = 0.518;

        return pose;
    }();

    this->move_group_interface_both_arms.setPoseTarget(left_arm_goal_pose, this->move_group_interface_left_arm.getEndEffectorLink()); 
    this->move_group_interface_both_arms.setPoseTarget(right_arm_goal_pose, this->move_group_interface_right_arm.getEndEffectorLink()); 
    this->move_part(this->move_group_interface_both_arms);
}

void BaxterMoveit2Examples::run()
{
    createObstacles();
    this->rviz_gazebo_bridge->load_from_rviz_to_gazebo(); 
    move_left_arm();
}

void BaxterMoveit2Examples::createObstacles()
{
    RCLCPP_INFO_STREAM(LOGGER, "in createObstacles");
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    
    moveit_msgs::msg::CollisionObject object;
    object.id = "box0";
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = {0.8, 1.5, 0.7};

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.91;
    pose.position.y = 0.51;
    pose.position.z = 0.35;
    object.pose = pose;
    collision_objects.emplace_back(object);

    //create cylinder
    object.id = "cylinder0";
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions.resize(2);
    object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.35;
    object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 0.75;
    pose.position.x = 1.3;
    pose.position.y = -1.3;
    pose.position.z = 0.35;
    object.pose = pose;
    collision_objects.emplace_back(object);

    //create sphere
    object.id = "sphere0";
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
    object.primitives[0].dimensions = {0.7}; //radius
    pose.position.x = 2.5;
    pose.position.y = -0.7;
    pose.position.z = 0.7;
    object.pose = pose;
    collision_objects.emplace_back(object);



    //create separator
    object.id = "separator";
    object.primitives[0].dimensions = {0.8, 0.07, 0.25}; 
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    pose.position.x = collision_objects[0].pose.position.x;
    pose.position.y = collision_objects[0].pose.position.y;
    pose.position.z = object.primitives[0].dimensions[2]/2 + collision_objects[0].primitives[0].dimensions[2];
    object.pose = pose;
    collision_objects.emplace_back(object);

    moveit::planning_interface::PlanningSceneInterface psi; 
    RCLCPP_INFO_STREAM(LOGGER, "applied collision obstacle");
    psi.applyCollisionObjects(collision_objects);
}


int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<BaxterMoveit2Examples>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto spin_thread = std::make_unique<std::thread>(
        [&executor, &node]()
        {
            executor.add_node(node->node_->get_node_base_interface());
            executor.spin();
            executor.remove_node(node->node_->get_node_base_interface());
        }
    );

    
    /* EXAMPLES */

    //ex1: load from rviz to gazebo, move left arm from one side of the "separator" to another
    node->run(); 
    
    //ex2: same as ex1 just without moving the arm
    // node->rviz2gazebo_example();
    
    //ex3: loading objects from gazebo to rviz (without moving the arm)
    // node->gazebo2rviz_example();
    
    //ex4-6: moving the arm(s), can be combined with loading obstacles to rviz from gazebo and vice versa
    // node->move_left_arm(); //sometimes fails when used with gazebo
    // node->move_both_arms();
    // node->move_both_arms2();
    
    //-----//
    spin_thread->join();
    rclcpp::shutdown();

    return 0;
}