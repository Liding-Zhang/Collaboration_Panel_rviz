#ifndef COLLABORATION_PANEL_H
#define COLLABORATION_PANEL_H

#include <QComboBox>
#include <QPushButton>
#include <QSlider>
#include <rviz/panel.h>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h> 
#include <moveit_msgs/MoveGroupAction.h>
#include <pthread.h>
#include <qlineedit.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

namespace my_rviz_panel {

class CollaborationPanel : public rviz::Panel {
    Q_OBJECT

public:
    CollaborationPanel(QWidget* parent = 0);
    ros::NodeHandle nh_;
    // PARAMETERS
    std::string MIR_PREFIX = "";
    std::string RBK_PREFIX = "";
    std::string mapFrameId = "map";
    std::string planningGroupName = "panda_arm";
    std::string gripperGroupName = "panda_hand";

    // unused currently
    // std::vector<double> currentArmValues;
    // std::vector<double> currentGripperValues;

    // action clients for bases and moveit
    actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>* move_group_client_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* mir_move_base_client_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* rbk_move_base_client_;

    // Values Fields for move_base navigation
    std::vector<QLineEdit*> mirBaseCoordinates;
    std::vector<QLineEdit*> rbkBaseCoordinates;

    

// Callbacks
private Q_SLOTS:
    void publishMirBaseGoal();
    void publishRbkBaseGoal();
    void publishMoveBaseGoals();
    void publishArmGoal();
    void publishGripperGoal();
    // void previewJointGoal();


private:
    // GUI Elements
    std::vector<QSlider*> jointSliders;
    QSlider* gripperSlider;
    QPushButton* armGoalButton;
    QPushButton* gripperGoalButton;
    QPushButton* mirBaseGoalButton;
    QPushButton* rbkBaseGoalButton;
    QPushButton* moveAllBasesButton;
    QComboBox* armPlannerComboBox;
    QComboBox* basePlannerComboBox;
    // QPushButton* previewJointGoalButton; 

    // Configuration
    std::vector<std::string> jointNames = { "panda_joint1", "panda_joint2", "panda_joint3",
                                            "panda_joint4", "panda_joint5", "panda_joint6",
                                            "panda_joint7" };
    std::string gripperJointName = "panda_finger_joint1";
    // Define the joint ranges: joint name -> (min, max) values
    std::map<std::string, std::pair<double, double>> jointRanges = {
        {"panda_joint1", {-166.0, 166.0}},
        {"panda_joint2", {-101.0, 101.0}},
        {"panda_joint3", {-166.0, 166.0}},
        {"panda_joint4", {-176.0, -4.0}},
        {"panda_joint5", {-166.0, 166.0}},
        {"panda_joint6", {-1.0, 215.0}},
        {"panda_joint7", {-166.0, 166.0}}
    };
    // Define initial joint angles for the ready position
    std::map<std::string, double> jointReadyAngle = {
        {"panda_joint1", 0.0},
        {"panda_joint2", -45.0},
        {"panda_joint3", 0.0},
        {"panda_joint4", -135.0},
        {"panda_joint5", 0.0},
        {"panda_joint6", 90.0},
        {"panda_joint7", 45.0}
    };

    // Add a thread to handle planning
    pthread_t plannerThread;
    pthread_mutex_t arm_mutex;
    pthread_mutex_t gripper_mutex;
    pthread_mutex_t mir_base_mutex;
    pthread_mutex_t rbk_base_mutex;

    // Function that are called by Button Callbacks and started in a thread
    static void* planArmGoalThread(void* arg);
    static void* planGripperGoalThread(void* arg);
    static void* planMirBaseGoalThread(void* arg);
    static void* planRbkBaseGoalThread(void* arg);
    // Load Parameter
    void loadParameters();


    // Functionality for previewing joint slider values
    // std::unique_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
    // static void* previewJointGoalThread(void* arg);

    
};

} // namespace my_rviz_panel

#endif // COLLABORATION_PANEL_H