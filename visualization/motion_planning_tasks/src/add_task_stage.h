/******************************************************************
dialog of add task or stage

Features:
- add task or stage  ux logic

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-09-07: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <QDialog>
#include <QPointer>

#include <memory>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_interface/planning_interface.h>
#include <geometry_msgs/Pose.h>

#include "task_display.h"

// forward declaration
class QLayout;
class QTableWidget;
class QComboBox;
class QTableWidget;
class QRadioButton;
class QCheckBox;
class QLineEdit;
class QDoubleSpinBox;

namespace moveit::task_constructor
{
class Task;
}

namespace moveit_rviz_plugin
{
	class DlgAddTaskStage : public QDialog
	{
		Q_OBJECT
	public:
		enum Type { TYPE_TASK = 0, TYPE_STAGE };

	public:
		DlgAddTaskStage(std::shared_ptr<moveit::task_constructor::Task> Task, TaskDisplay* Display, int Type, QWidget* Parent = nullptr);
		~DlgAddTaskStage();

	private:
		void initLayout();
		QWidget* initGroupMoveTo();
		QWidget* initGroupMoveRelative();
		QWidget* initGroupWaypoints();
		QLayout* initMoveToPoseGroup();
		QLayout* initMoveToTcp();
		QLayout* initMoveRelativeTcp();
		QLayout* initMoveRelativeJoint();		
		void removeLayout(QWidget* Group, int Index);
		void mousePressEvent(QMouseEvent* Event) override;
		void closeEvent(QCloseEvent* Event) override;

	private:
		void onPlanningGroupTextChanged(const QString& Text);
		void onModeIndexChanged(int Index, QWidget* Group);
		void onMoveToTargetChecked(bool Checked, const QString& Name, QWidget* Group);
		void onMoveToTcpCurrentClicked();
		void onMoveRelativeTargetChecked(bool Checked, const QString& Name, QWidget* Group);
		void onWaypointsAddClicked(QCheckBox* CheckCurrent);
		void onWaypointsInsertClicked(QCheckBox* CheckCurrent);
		void onWaypointsRemoveClicked(QCheckBox* CheckCurrent);

	private:
		void fillRowWith(QTableWidget* Table, int Row, const std::shared_ptr<QStringList> Data = nullptr);
		geometry_msgs::Pose queryCurrent();
		void retrieveWaypoints(QTableWidget* Table, std::vector<geometry_msgs::Pose>& Waypoints);
		void retrieveWaypoints(QTableWidget* Table, std::vector<std::vector<double>>& Waypoints);
		void visualizeWaypoints(QTableWidget* Table, int Row);
		void interactiveMarkerProcessFeedback(int Index, visualization_msgs::InteractiveMarkerFeedback& Feedback);
		std::string getPlanningGroupOfTask();
		void add();

	private:
		int type_{ TYPE_TASK };
		QLineEdit* line_task_name_{ nullptr };
		QComboBox* combx_mode_{ nullptr };
		QRadioButton* radio_joint_space_{ nullptr };
		QRadioButton* radio_cartesian_space_{ nullptr };
		QDoubleSpinBox* spin_box_duration_{ nullptr };
		QPointer<QComboBox> combx_pose_group_;
		QPointer<QTableWidget> table_tcp_;
		QPointer<QTableWidget> table_joint_;
		QPointer<QTableWidget> table_waypoints_;
		QPointer<QLineEdit> line_fixed_frame_;
		std::string joint_model_group_name_;
		planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_{ nullptr };
		const moveit::core::JointModelGroup* joint_model_group_{ nullptr };
		std::shared_ptr<moveit::task_constructor::Task> task_{ nullptr };
		TaskDisplay* display_{ nullptr };
		geometry_msgs::Pose frame_reference_;
		std::unique_ptr<geometry_msgs::Pose> current_goal_{ nullptr };
		enum TargetType { TARGET_TYPE_POSE_GROUP = 0, TARGET_TYPE_TCP };
		int target_type_{ TARGET_TYPE_POSE_GROUP };
		std::unique_ptr<ros::NodeHandle> node_handle_{ nullptr };
		std::unique_ptr<ros::Subscriber> sub_interactive_goal_{ nullptr };
	};
}  // namespace moveit_rviz_plugin


