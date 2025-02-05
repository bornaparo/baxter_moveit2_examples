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

#include "baxter_moveit2_adapter/baxter_moveit2_adapter.h"

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
    std::shared_ptr<BaxterMoveit2Adapter> baxter_moveit2_adapter;
};

static rclcpp::Logger LOGGER = rclcpp::get_logger("baxter_moveit2_examples_node");

BaxterMoveit2Examples::BaxterMoveit2Examples(const rclcpp::NodeOptions& options, const std::string& robot_name="baxter")
                                                                    :
                                                                    node_(std::make_shared<rclcpp::Node>("baxter_moveit2_examples_node", options)),
                                                                        move_group_interface_left_arm(this->node_, "left_arm"), //group is from .srdf-a
                                                                    move_group_interface_right_arm(this->node_, "right_arm"),
                                                                    move_group_interface_both_arms(this->node_, "both_arms"),
                                                                    rviz_gazebo_bridge(std::make_shared<RVizGazeboBridge>(robot_name)),
                                                                    baxter_moveit2_adapter(std::make_shared<BaxterMoveit2Adapter>(node_))
                                                                    
{
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

void BaxterMoveit2Examples::move_part(moveit::planning_interface::MoveGroupInterface& mgi, bool async_execute = false)
{
    mgi.setMaxAccelerationScalingFactor(0.8); 
    mgi.setMaxVelocityScalingFactor(0.8); 
    mgi.setNumPlanningAttempts(1); //ovo ti je koliko ti valid plannova mora pronac, ako mora pronac n planova a on je pronasao m gdje je m < n on ce rec da planiranje nije uspjelo (nisam provjerio pa nez 100%)
    mgi.setPlanningTime(30);
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

        const std::string mgi_name = mgi.getName();
        auto real_baxter_thread = std::make_unique<std::thread>(
            [this, mgi_name, plan]()
            {
                this->baxter_moveit2_adapter->send_to_real_baxter(mgi_name, plan.trajectory_.joint_trajectory);
            }
        );

        if (async_execute)
            mgi.asyncExecute(plan); 
        else
            mgi.execute(plan);

        real_baxter_thread->join();
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
    // this->move_group_interface_left_arm.setPlannerId("RRTConnectkConfigDefault");
    
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
    node->run();  //sometimes can fail to find path, i think it happens if the /joint_states is not published/received fast enough
    
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