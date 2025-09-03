#include "collaboration_panel.h"
#include <QApplication>
#include <QLabel>
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <spdlog/spdlog.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <nav_core/base_global_planner.h>
#include <pluginlib/class_loader.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace my_rviz_panel {

CollaborationPanel::CollaborationPanel(QWidget* parent)
    : rviz::Panel(parent)
    // , tf_broadcaster_()
    , nh_("~")
{
    QVBoxLayout* layout = new QVBoxLayout;
    spdlog::info("Load params");
    loadParameters();

    spdlog::info("Creating GUI elements");
    // Create mir fields horizontally aligned
    QGridLayout* mirLayout = new QGridLayout;
    int mirColumn = 0;
    for (const auto& coordName : { "mir_x", "mir_y", "mir_yaw" }) {
        QLabel* label = new QLabel(QString::fromStdString(coordName));
        QLineEdit* edit = new QLineEdit;
        mirBaseCoordinates.push_back(edit);
        mirLayout->addWidget(label, 0, mirColumn);
        mirLayout->addWidget(edit, 1, mirColumn);
        mirColumn++;
    }
    layout->addLayout(mirLayout);

    // Create rbkairos fields horizontally aligned
    QGridLayout* rbkLayout = new QGridLayout;
    int rbkColumn = 0;
    // Create RB-Kairos base coordinate input fields (x, y, yaw)
    for (const auto& coordName : { "rbkairos_x", "rbkairos_y", "rbkairos_yaw" }) {
        QLabel* label = new QLabel(QString::fromStdString(coordName));
        QLineEdit* edit = new QLineEdit;
        rbkBaseCoordinates.push_back(edit);
        rbkLayout->addWidget(label, 0, rbkColumn);
        rbkLayout->addWidget(edit, 1, rbkColumn);
        rbkColumn++;
    }
    layout->addLayout(rbkLayout);

    // Create Buttons for move_base
    QHBoxLayout* mbButtonLayout = new QHBoxLayout;
    mirBaseGoalButton = new QPushButton("Publish MiR Base Goal");
    rbkBaseGoalButton = new QPushButton("Publish RB-Kairos Base Goal");
    moveAllBasesButton = new QPushButton("Publish all Base Goals");  
    mbButtonLayout->addWidget(mirBaseGoalButton);
    mbButtonLayout->addWidget(rbkBaseGoalButton);
    mbButtonLayout->addWidget(moveAllBasesButton);
    layout->addLayout(mbButtonLayout);

    // Create joint sliders
    QVBoxLayout* leftHalf = new QVBoxLayout;
    for (const auto& jointName : jointNames) {
        // Retrieve min and max range
        double minRange = jointRanges[jointName].first;
        double maxRange = jointRanges[jointName].second;

        QSlider* slider = new QSlider(Qt::Horizontal);
        QDoubleSpinBox* spinBox = new QDoubleSpinBox;

        slider->setRange(static_cast<int>(minRange * 100), static_cast<int>(maxRange * 100));  // Using integer range, but scaling for floating point
        
        spinBox->setRange(minRange, maxRange);  // Same range as slider, but in floating-point
        spinBox->setDecimals(1);           // Allow up to 1 decimal places
        spinBox->setSingleStep(0.5);       // Small step for fine control

        // Set sliders to robots ready & spawning position
        slider->setValue(static_cast<int>(jointReadyAngle[jointName] * 100));
        spinBox->setValue(jointReadyAngle[jointName]);

        // Synchronize slider and spin box
        connect(slider, &QSlider::valueChanged, [spinBox](int value) {
            spinBox->setValue(value / 100.0);  // Scale the slider value to float
        });
        connect(spinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [slider](double value) {
            slider->setValue(static_cast<int>(value * 100));  // Scale the float value to integer
        });

        jointSliders.push_back(slider);

        leftHalf->addWidget(new QLabel(QString::fromStdString(jointName)));
        QHBoxLayout* sliderLayout = new QHBoxLayout;  // Create a horizontal layout to hold the slider and spin box
        sliderLayout->addWidget(slider);
        sliderLayout->addWidget(spinBox);
        leftHalf->addLayout(sliderLayout);
    }

    // Create buttons
    QHBoxLayout* jointButtonLayout = new QHBoxLayout;
   
    // previewJointGoalButton = new QPushButton("Preview Joint States");
    // jointButtonLayout->addWidget(previewJointGoalButton);

    armGoalButton = new QPushButton("Publish Joint Goal");
    jointButtonLayout->addWidget(armGoalButton);
    leftHalf->addLayout(jointButtonLayout);


    // Create Gripper Slider & Buttons
    QSlider* gSlider = new QSlider(Qt::Horizontal);
    gSlider->setRange(0, 35);
    QDoubleSpinBox* gripperBox = new QDoubleSpinBox;
    gripperBox->setRange(0, 0.035);
    gripperBox->setDecimals(3);
    gripperBox->setSingleStep(0.005);

    // synchronize sliders and spinBoxes
    connect(gSlider, &QSlider::valueChanged, [gripperBox](int value) {
        gripperBox->setValue(value / 1000.0);
    });
    connect(gripperBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [gSlider](double value) {
        gSlider->setValue(static_cast<int>(value * 1000));
    });

    gripperSlider = gSlider;
    leftHalf->addWidget(new QLabel(QString::fromStdString(gripperJointName)));
    QBoxLayout* gripperSliderLayout = new QHBoxLayout;
    gripperSliderLayout->addWidget(gSlider);
    gripperSliderLayout->addWidget(gripperBox);
    leftHalf->addLayout(gripperSliderLayout);

    QHBoxLayout* gripperButtonLayout = new QHBoxLayout;
    gripperGoalButton = new QPushButton("Publish Gripper Goal");
    gripperButtonLayout->addWidget(gripperGoalButton);
    leftHalf->addLayout(gripperButtonLayout);

    // Create Center panel to make the sliders take up only half
    QBoxLayout* centerPanel = new QHBoxLayout;
    centerPanel->addLayout(leftHalf, 1);
    
    // Callback connect
    connect(mirBaseGoalButton, SIGNAL(clicked()), this, SLOT(publishMirBaseGoal()));
    connect(rbkBaseGoalButton, SIGNAL(clicked()), this, SLOT(publishRbkBaseGoal()));
    connect(moveAllBasesButton, SIGNAL(clicked()), this, SLOT(publishMoveBaseGoals()));
    connect(armGoalButton, SIGNAL(clicked()), this, SLOT(publishArmGoal()));
    connect(gripperGoalButton, SIGNAL(clicked()), this, SLOT(publishGripperGoal()));
    // connect(previewJointGoalButton, SIGNAL(clicked()), this, SLOT(previewJointGoal()));
    

    armPlannerComboBox = new QComboBox;
    basePlannerComboBox = new QComboBox;
    // Add available planners here
    armPlannerComboBox->addItems({ "RRTConnect", "RRTstar", "PRMstar" });
    basePlannerComboBox->addItems({ "GlobalPlanner", "NavfnROS", "CarrotPlanner" });

    // Add widget for choosing planner
    QBoxLayout* rightHalf = new QVBoxLayout;
    rightHalf->addWidget(new QLabel("Arm Planner:"));
    rightHalf->addWidget(armPlannerComboBox);
    rightHalf->addWidget(new QLabel("Base Planner:"));
    rightHalf->addWidget(basePlannerComboBox);

    // Add a spacer that will expand and take up the remaining space
    QSpacerItem* verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);
    rightHalf->addSpacerItem(verticalSpacer);

    // Add rightHalf to centerPanel and centerPanel to layout
    centerPanel->addLayout(rightHalf, 1);
    layout->addLayout(centerPanel);

    setLayout(layout);
    spdlog::info("Layout set");
    // Layout Done

    // ROS Communication Setup
    spdlog::info("Create Action Clients!");
    move_group_client_ = new actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>(
        nh_, "/" + RBK_PREFIX + "move_group", true);
    mir_move_base_client_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
        nh_, "/" + MIR_PREFIX + "move_base", true);
    rbk_move_base_client_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
    nh_, "/" + RBK_PREFIX + "move_base", true);

    // Set the timeout duration
    ros::Duration timeout(5.0);
    // Wait for the servers with timeout
    if (!move_group_client_->waitForServer(timeout)) {
        spdlog::error("Failed to connect to move_group server within timeout!");
        // Disable buttons if server unavailable
        armGoalButton->setEnabled(false);
        gripperGoalButton->setEnabled(false);
    } else {
        spdlog::info("Connected to move_group server!");
    }

    if (!mir_move_base_client_->waitForServer(timeout)) {
        spdlog::error("Failed to connect to move_base server within timeout!");
        // Disable buttons if server unavailable
        mirBaseGoalButton->setEnabled(false);
        moveAllBasesButton->setEnabled(false);
    } else {
        spdlog::info("Connected to MiR move_base server!");
    }

    if (!rbk_move_base_client_->waitForServer(timeout)) {
        spdlog::error("Failed to connect to move_base server within timeout!");
        // Disable buttons if server unavailable
        rbkBaseGoalButton->setEnabled(false);
        moveAllBasesButton->setEnabled(false);
    } else {
        spdlog::info("Connected to RB-Kairos move_base server!");
    }

    // Initialize the mutex
    pthread_mutex_init(&arm_mutex, NULL);
    pthread_mutex_init(&gripper_mutex, NULL);
    pthread_mutex_init(&mir_base_mutex, NULL);
    pthread_mutex_init(&rbk_base_mutex, NULL);

    // Used for joint angle preview
    // planning_scene_monitor_.reset(
    //     new planning_scene_monitor::PlanningSceneMonitor(RBK_PREFIX + "robot_description"));
    // planning_scene_monitor_->startSceneMonitor();
    // planning_scene_monitor_->startStateMonitor();
    // planning_scene_monitor_->startWorldGeometryMonitor();
}

// Preview the selected joint configuration
// void CollaborationPanel::previewJointGoal()
// {
//     if (!move_group_client_) {
//         spdlog::error("MoveGroup action client not initialized!");
//         return;
//     }

//     // Create and start the thread
//     pthread_create(&plannerThread, NULL, &CollaborationPanel::previewJointGoalThread, this);
// }

// void* CollaborationPanel::previewJointGoalThread(void* arg)
// {
//     CollaborationPanel* panel = static_cast<CollaborationPanel*>(arg);

//     // Create a MoveGroupGoal message for previewing
//     moveit_msgs::MoveGroupGoal goal;
//     goal.request.group_name = panel->planningGroupName;
//     goal.request.allowed_planning_time = 1.0;
//     goal.request.num_planning_attempts = 5;
//     goal.request.planner_id = panel->armPlannerComboBox->currentText().toStdString();

//     // Set the planning options to plan only (preview mode)
//     goal.planning_options.plan_only = true;
//     goal.planning_options.planning_scene_diff.is_diff = true;
//     goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

//     // Set the target joint values based on the current sliders
//     goal.request.goal_constraints.resize(1);
//     moveit_msgs::JointConstraint joint_constraint;
//     for (size_t i = 0; i < panel->jointNames.size(); ++i) {
//         joint_constraint.joint_name = panel->jointNames[i];
//         joint_constraint.position = panel->jointSliders[i]->value() * M_PI / 180.0;  // Convert to radians
//         joint_constraint.tolerance_above = 0.01;
//         joint_constraint.tolerance_below = 0.01;
//         joint_constraint.weight = 1.0;
//         goal.request.goal_constraints[0].joint_constraints.push_back(joint_constraint);
//     }

//     // Send the goal and wait for the result
//     panel->move_group_client_->sendGoal(goal);
//     bool success = panel->move_group_client_->waitForResult();

//     if (success) {
//         moveit_msgs::MoveGroupResultConstPtr result = panel->move_group_client_->getResult();
//         if (result->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
//             // **Preview Mode**: Show the planned trajectory in RViz without execution
//             spdlog::info("Preview successful! Displaying in RViz!");
            
//             // Create a MoveIt Visual Tools object to help visualize the plan
//             moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC, panel->planning_scene_monitor_);

//             // Access the planned trajectory from the response of MoveGroupResult
//             const moveit_msgs::RobotTrajectory& trajectory = result->planned_trajectory;

//             // Get the end-effector link model (replace "end_effector" with your robot's end-effector link name if needed)
//             const moveit::core::LinkModel* ee_link = panel->planning_scene_monitor_->getRobotModel()->getLinkModel("panda_link8");
//             spdlog::info("T1");
//             // Get the joint model group (replace "panda_arm" with your planning group)
//             const moveit::core::JointModelGroup* joint_model_group = panel->planning_scene_monitor_->getRobotModel()->getJointModelGroup("panda_arm");

//             // Publish the planned trajectory (line) in RViz
//             // Load the planning scene monitor into the visual tools
//             spdlog::info("T2");
//             visual_tools.publishTrajectoryLine(trajectory, ee_link, joint_model_group, rviz_visual_tools::BLUE);
//             visual_tools.trigger();  // Trigger the visualization

//             spdlog::info("Preview trajectory visualized.");
//         } else {
//             spdlog::warn("Planning for preview failed with error code: {}", result->error_code.val);
//         }
//     } else {
//         spdlog::warn("Planning for preview timed out!");
//     }

//     return nullptr;
// }


void CollaborationPanel::publishMirBaseGoal()
{
    if (!mir_move_base_client_) {
        spdlog::error("MiR MoveBase action client not initialized!");
        return;
    }
    pthread_create(&plannerThread, NULL, &CollaborationPanel::planMirBaseGoalThread, this);
}

void* CollaborationPanel::planMirBaseGoalThread(void* arg)
{
    CollaborationPanel* panel = static_cast<CollaborationPanel*>(arg);
    ROS_INFO_STREAM("Initializing MiR goal!");
    geometry_msgs::PoseStamped basePose;
    basePose.header.frame_id = panel->mapFrameId;
    basePose.header.stamp = ros::Time::now();
    basePose.pose.position.x = panel->mirBaseCoordinates[0]->text().toDouble();
    basePose.pose.position.y = panel->mirBaseCoordinates[1]->text().toDouble();
    basePose.pose.orientation = tf::createQuaternionMsgFromYaw(panel->mirBaseCoordinates[2]->text().toDouble() * M_PI / 180.0);

    panel->nh_.setParam(
        "/" + panel->MIR_PREFIX + "move_base/base_global_planner",
        panel->basePlannerComboBox->currentText().toStdString());
    spdlog::info("Using base planner: {}", panel->basePlannerComboBox->currentText().toStdString());

    pthread_mutex_lock(&(panel->mir_base_mutex));
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = basePose;
    ROS_INFO_STREAM("Send MiR goal!");
    panel->mir_move_base_client_->sendGoal(goal);
    ROS_INFO_STREAM("Waiting for result!");
    bool success = panel->mir_move_base_client_->waitForResult();
    ROS_INFO_STREAM("Execution: " << success ? "Success!" : "Failed!");

    pthread_mutex_unlock(&(panel->mir_base_mutex));
    ROS_INFO_STREAM("MiR Base Goal: Done!");
    return NULL;
}

void CollaborationPanel::publishRbkBaseGoal()
{
    if (!rbk_move_base_client_) {
        spdlog::error("RB-Kairos MoveBase action client not initialized!");
        return;
    }
    pthread_create(&plannerThread, NULL, &CollaborationPanel::planRbkBaseGoalThread, this);
}

void* CollaborationPanel::planRbkBaseGoalThread(void* arg)
{
    CollaborationPanel* panel = static_cast<CollaborationPanel*>(arg);
    ROS_INFO_STREAM("Initializing RB-Kairos goal!");
    geometry_msgs::PoseStamped basePose;
    basePose.header.frame_id = panel->mapFrameId;
    basePose.header.stamp = ros::Time::now();
    basePose.pose.position.x = panel->rbkBaseCoordinates[0]->text().toDouble();
    basePose.pose.position.y = panel->rbkBaseCoordinates[1]->text().toDouble();
    basePose.pose.orientation = tf::createQuaternionMsgFromYaw(panel->rbkBaseCoordinates[2]->text().toDouble() * M_PI / 180.0);

    panel->nh_.setParam(
        "/" + panel->RBK_PREFIX + "move_base/base_global_planner",
        panel->basePlannerComboBox->currentText().toStdString());
    spdlog::info("Using base planner: {}", panel->basePlannerComboBox->currentText().toStdString());

    pthread_mutex_lock(&(panel->rbk_base_mutex));
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = basePose;
    ROS_INFO_STREAM("Send RB-Kairos goal!");
    panel->rbk_move_base_client_->sendGoal(goal);
    ROS_INFO_STREAM("Waiting for result!");
    bool success = panel->rbk_move_base_client_->waitForResult();
    ROS_INFO_STREAM("Execution: " << success ? "Success!" : "Failed!");

    pthread_mutex_unlock(&(panel->rbk_base_mutex));
    ROS_INFO_STREAM("RB-Kairos Base Goal: Done!");
    return NULL;
}


void CollaborationPanel::publishMoveBaseGoals()
{
    if (!mir_move_base_client_) {
        spdlog::error("MiR move base action client not initialized!");
        return;
    }
    if (!rbk_move_base_client_) {
        spdlog::error("RB-Kairos move base action client not initialized!");
        return;
    }

    // Create and start the threads
    pthread_create(&plannerThread, NULL, &CollaborationPanel::planMirBaseGoalThread, this);
    pthread_create(&plannerThread, NULL, &CollaborationPanel::planRbkBaseGoalThread, this);
}

void CollaborationPanel::publishArmGoal()
{
    if (!move_group_client_) {
        spdlog::error("MoveGroup action client not initialized!");
        return;
    }

    // Create and start the thread
    pthread_create(&plannerThread, NULL, &CollaborationPanel::planArmGoalThread, this);
}

void* CollaborationPanel::planArmGoalThread(void* arg)
{
    CollaborationPanel* panel = static_cast<CollaborationPanel*>(arg);

    // Create a MoveGroupGoal message
    moveit_msgs::MoveGroupGoal goal;
    goal.request.group_name = panel->planningGroupName;
    goal.request.planner_id = panel->armPlannerComboBox->currentText().toStdString();
    goal.request.num_planning_attempts = 5;
    goal.request.allowed_planning_time = 1;
    spdlog::info("Using arm planner: {}", goal.request.planner_id);

    // planning request
    goal.planning_options.plan_only = true;
    goal.planning_options.planning_scene_diff.is_diff = true; 
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    // Fill in joint values for the goal message
    goal.request.goal_constraints.resize(1);
    moveit_msgs::JointConstraint joint_constraint;
    for (size_t i = 0; i < panel->jointNames.size(); ++i) {
        joint_constraint.joint_name = panel->jointNames[i];
        joint_constraint.position = panel->jointSliders[i]->value() * M_PI / (180.0 * 100.0); // Radians conversion & 100 adjusts for scaling above
        spdlog::info("Goal arm joint values: {}", joint_constraint.position);
        joint_constraint.weight = 1.0;
        joint_constraint.tolerance_above = 0.01; 
        joint_constraint.tolerance_below = 0.01;
        goal.request.goal_constraints[0].joint_constraints.push_back(joint_constraint);
    }

    // Send the goal and wait for a result
    spdlog::info("Planning Arm goal!");
    panel->move_group_client_->sendGoal(goal);
    bool success = panel->move_group_client_->waitForResult(); 

    if (success) {
        moveit_msgs::MoveGroupResultConstPtr result = panel->move_group_client_->getResult();

        if (result->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            spdlog::info("Succes! Starting execution!");
            // Execute the goal with the goal from above
            goal.planning_options.plan_only = false;
            panel->move_group_client_->sendGoal(goal);
            success = panel->move_group_client_->waitForResult(
                ros::Duration(10.0)); 

            if (success) {
                spdlog::info("Successfully executed!");
            } else {
                spdlog::warn("Failed or timed out execution!");
            }
        } else {
            // Planning failed
            spdlog::warn("Planning failed with error code: {}", result->error_code.val);
        }
    } else {
        // Action timed out
        spdlog::warn("Planning action timed out!");
    }

    return NULL;
}

void CollaborationPanel::publishGripperGoal()
{
    if (!move_group_client_) {
        spdlog::error("MoveGroup action client not initialized!");
        return;
    }

    // Create and start the thread
    pthread_create(&plannerThread, NULL, &CollaborationPanel::planGripperGoalThread, this);
}

void* CollaborationPanel::planGripperGoalThread(void* arg)
{
    CollaborationPanel* panel = static_cast<CollaborationPanel*>(arg);

    // Create goal message for move_group
    moveit_msgs::MoveGroupGoal goal;
    goal.request.group_name = panel->gripperGroupName;
    goal.request.planner_id = panel->armPlannerComboBox->currentText().toStdString();
    goal.request.num_planning_attempts = 5;
    goal.request.allowed_planning_time = 1; 
    spdlog::info("Using gripper planner: {}", goal.request.planner_id);

    // planning request
    goal.planning_options.plan_only = true;
    goal.planning_options.planning_scene_diff.is_diff = true; 
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    // Fill in joint values for the goal message
    goal.request.goal_constraints.resize(1);
    moveit_msgs::JointConstraint joint_constraint;
    joint_constraint.joint_name = panel->gripperJointName;
    joint_constraint.position = panel->gripperSlider->value() / 1000.0; // Adjust for scaling from above
    joint_constraint.weight = 1.0;
    joint_constraint.tolerance_above = 0.01; 
    joint_constraint.tolerance_below = 0.01;
    goal.request.goal_constraints[0].joint_constraints.push_back(joint_constraint);

    // Send the goal and wait for a result
    spdlog::info("Planning gripper goal!");
    panel->move_group_client_->sendGoal(goal);
    bool success = panel->move_group_client_->waitForResult(); 

    if (success) {
        moveit_msgs::MoveGroupResultConstPtr result = panel->move_group_client_->getResult();

        if (result->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            spdlog::info("Succes! Starting execution!");
            // Execute the motion with constructed goal
            goal.planning_options.plan_only = false;
            panel->move_group_client_->sendGoal(goal);
            success = panel->move_group_client_->waitForResult(
                ros::Duration(10.0)); // Set a timeout for execution

            if (success) {
                spdlog::info("Successfully executed!");
            } else {
                spdlog::warn("Failed or timed out execution!");
            }
        } else {
            // Planning failed
            spdlog::warn("Planning failed with error code: {}", result->error_code.val);
        }
    } else {
        // Action timed out
        spdlog::warn("Planning action timed out!");
    }

    return NULL;
}


void CollaborationPanel::loadParameters()
{
    // Read the prefix parameter 
    if (!nh_.getParam("mir_prefix", MIR_PREFIX)) {
        ROS_WARN_STREAM("MiR-Prefix parameter not found. Using default mir/ prefix.");
        MIR_PREFIX = "mir/"; // Default value
    } else {
        ROS_INFO_STREAM("Using prefix: " << MIR_PREFIX);
    }
    if (!nh_.getParam("rbk_prefix", RBK_PREFIX)) {
        ROS_WARN_STREAM("Prefix parameter not found. Using default rbkairos/ prefix.");
        RBK_PREFIX = "rbkairos/"; // Default value
    } else {
        ROS_INFO_STREAM("Using prefix: " << RBK_PREFIX);
    }
    if (!nh_.getParam("map_frame_id", mapFrameId)) {
        ROS_WARN_STREAM("Map frame parameter not found. Using default map.");
        mapFrameId = "map"; // Default value
    } else {
        ROS_INFO_STREAM("Using map frame ID: " << mapFrameId);
    }
    if (!nh_.getParam("panda_group_name", planningGroupName)) {
        ROS_WARN_STREAM("Group name parameter not found. Using default panda_arm.");
        planningGroupName = "panda_arm"; // Default value
    } else {
        ROS_INFO_STREAM("Using Panda planning group name: " << planningGroupName);
    }
    if (!nh_.getParam("gripper_group_name", gripperGroupName)) {
        ROS_WARN_STREAM("Gripper Group name parameter not found. Using default panda_hand.");
        gripperGroupName = "panda_hand"; // Default value
    } else {
        ROS_INFO_STREAM("Using Gripper planning group name: " << gripperGroupName);
    }
}

} // namespace my_rviz_panel

// Pluginlib Macro
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_rviz_panel::CollaborationPanel, rviz::Panel)