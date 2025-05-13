#include <functional>
#include <memory>
#include <thread>

#include "messages/action/moveit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "std_msgs/msg/string.hpp"

namespace move_it_ur5e
{
class MoveItActionServer : public rclcpp::Node//, public std::enable_shared_from_this<MoveItActionServer>
{
public:
    using MoveItAction = messages::action::Moveit;
    using GoalHandleMoveItAction = rclcpp_action::ServerGoalHandle<MoveItAction>;

    // CUSTOM_ACTION_CPP_PUBLIC
    explicit MoveItActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("move_it_action_server", options), robot_description_received_(false)
    {
        using namespace std::placeholders;
        RCLCPP_INFO(this->get_logger(), "MoveItActionServer node has been started");

        // Subscribe to the /robot_description topic
        robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description",
            10,
            std::bind(&MoveItActionServer::robot_description_callback, this, _1));

        auto handle_goal = [this](
                               const rclcpp_action::GoalUUID &uuid,
                               std::shared_ptr<const MoveItAction::Goal> goal)
        {
            float w = goal->pose[0];
            float x = goal->pose[1];
            float y = goal->pose[2];
            float z = goal->pose[3];
            RCLCPP_INFO(this->get_logger(), "Received goal request with pose: [%f, %f, %f, %f]", w, x, y, z);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };

        auto handle_cancel = [this](
                                 const std::shared_ptr<GoalHandleMoveItAction> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        auto handle_accepted = [this](
                                   const std::shared_ptr<GoalHandleMoveItAction> goal_handle)
        {
            auto execute_in_thread = [this, goal_handle]()
            { return this->execute(goal_handle); };
            std::thread{execute_in_thread}.detach();
        };

        this->action_server_ = rclcpp_action::create_server<MoveItAction>(
            this,
            "robot_moveit",
            handle_goal,
            handle_cancel,
            handle_accepted);
    }

private:
    rclcpp_action::Server<MoveItAction>::SharedPtr action_server_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
    std::string robot_description_;
    bool robot_description_received_;

    void robot_description_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        robot_description_ = msg->data;
        robot_description_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received robot description from /robot_description topic");
    }

    void execute(const std::shared_ptr<GoalHandleMoveItAction> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        // Wait for the robot description to be received
        if (!robot_description_received_)
        {
            RCLCPP_ERROR(this->get_logger(), "Robot description not received yet. Aborting goal.");
            goal_handle->abort(std::make_shared<MoveItAction::Result>());
            return;
        }

        // Create the MoveIt MoveGroup Interface
        using moveit::planning_interface::MoveGroupInterface;
        auto move_group_interface = MoveGroupInterface(this->shared_from_this(), "ur_manipulator");

        // Set the target pose
        const auto goal = goal_handle->get_goal();
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.w = goal->pose[0];
        target_pose.position.x = goal->pose[1];
        target_pose.position.y = goal->pose[2];
        target_pose.position.z = goal->pose[3];
        move_group_interface.setPoseTarget(target_pose);

        // Plan and execute
        auto [success, plan] = [&move_group_interface] {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();

        if (success)
        {
            move_group_interface.execute(plan);
            auto result = std::make_shared<MoveItAction::Result>();
            result->status = "Completed";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
            goal_handle->abort(std::make_shared<MoveItAction::Result>());
        }
    }
}; // class MoveItActionServer
} // namespace move_it_ur5e

RCLCPP_COMPONENTS_REGISTER_NODE(move_it_ur5e::MoveItActionServer);