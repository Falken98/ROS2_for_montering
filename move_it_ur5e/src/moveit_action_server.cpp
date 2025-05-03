#include <functional>
#include <memory>
#include <thread>

#include "move_it_ur5e/visibility_control.h"

#include "messages/action/moveit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"

namespace move_it_ur5e
{
class MoveItActionServer : public rclcpp::Node//, public std::enable_shared_from_this<MoveItActionServer> 
{
    public:
    using MoveItAction = messages::action::Moveit;
    using GoalHandleMoveItAction = rclcpp_action::ServerGoalHandle<MoveItAction>;

    CUSTOM_ACTION_CPP_PUBLIC
    explicit MoveItActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("move_it_action_server", options)
    {
        using namespace std::placeholders;

        auto handle_goal = [this](
        const rclcpp_action::GoalUUID & uuid,
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
            // this needs to return quickly to avoid blocking the executor,
            // so we declare a lambda function to be called inside a new thread
            auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);};
            std::thread{execute_in_thread}.detach();
        };

        this->action_server_ = rclcpp_action::create_server<MoveItAction>(
            this,
            // "move_it_action",
            "robot_moveit",
            handle_goal,
            handle_cancel,
            handle_accepted
        );
    }

    private:
    rclcpp_action::Server<MoveItAction>::SharedPtr action_server_;

    void execute(const std::shared_ptr<GoalHandleMoveItAction> goal_handle) {
        // rclcpp::init();
        // auto const node = make_shared<rclcpp::Node>(
        //     "moveit_node",
        //     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        // );
    
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveItAction::Feedback>();
        auto & status = feedback->partial_status;
        status.push_back(0);
        status.push_back(1);
        auto result = std::make_shared<MoveItAction::Result>();
        

        // Create the MoveIt MoveGroup Interface
        using moveit::planning_interface::MoveGroupInterface;
        auto move_group_interface = MoveGroupInterface(std::enable_shared_from_this<rclcpp::Node>::shared_from_this(), "ur_manipulator");

        // Set the target pose
        geometry_msgs::msg::Pose target_pose = [goal]{
            geometry_msgs::msg::Pose msg;
            msg.orientation.w = goal->pose[0];
            msg.position.x = goal->pose[1];
            msg.position.y = goal->pose[2];
            msg.position.z = goal->pose[3];
            return msg;
        }();
        move_group_interface.setPoseTarget(target_pose);

        // Create a plan to that target pose
        auto const [success, plan] = [&move_group_interface]{
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();

        // Execute the plan
        if(success) {
            move_group_interface.execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
    
        // Check if goal is done
        if (rclcpp::ok()) {
            result->status = status;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }; // void execute
}; // class MoveItActionServer
} // namespace move_it_ur5e

RCLCPP_COMPONENTS_REGISTER_NODE(move_it_ur5e::MoveItActionServer);