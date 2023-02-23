/******************************************************************
dialog of add task or stage

Features:
- add task or stage ux logic

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "add_task_stage.h"

#include <iostream>
#include <boost/any.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QRadioButton>
#include <QPushButton>
#include <QComboBox>
#include <QTableWidget>
#include <QHeaderView>
#include <QCheckBox>
#include <QMenu>
#include <QLabel>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QSpacerItem>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/container.h>

namespace moveit_rviz_plugin
{
	DlgAddTaskStage::DlgAddTaskStage(std::shared_ptr<moveit::task_constructor::Task> Task,
		TaskDisplay* Display, int Type, QWidget* Parent/* = nullptr*/)
		: type_(Type), task_(Task), display_(Display), QDialog(Parent)
		, node_handle_(std::make_unique<ros::NodeHandle>())
	{
		sub_interactive_goal_ = std::make_unique<ros::Subscriber>(
			node_handle_->subscribe<visualization_msgs::InteractiveMarkerFeedback>(
            "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",
			1, [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr& Feedback)
			{
				if (!current_goal_)
				{
					current_goal_ = std::make_unique<geometry_msgs::Pose>(Feedback->pose);
				}
				else
				{
					*current_goal_ = Feedback->pose;
				}
            }));

		QString title = Type == TYPE_TASK ? "Add Task with Stage" : "Add Stage to Task";

		setWindowTitle(title);
		setMinimumWidth(400);

		initLayout();
	}

	DlgAddTaskStage::~DlgAddTaskStage()
	{
		display_->clearInteractiveMarkers();
	}

	void DlgAddTaskStage::initLayout()
	{
		QVBoxLayout* layoutMain = new QVBoxLayout(this);

		/// group: type
		QGroupBox* groupBoxType = new QGroupBox("Type");

		QHBoxLayout* hBox = new QHBoxLayout();
		QLabel* labelType = new QLabel("Task");
		line_task_name_ = new QLineEdit(task_->name().empty() ? "Task pipeline" : task_->name().c_str());
		line_task_name_->setEnabled(type_ == TYPE_TASK);	
		hBox->addWidget(labelType);
		hBox->addWidget(line_task_name_);
		hBox->addStretch();
		groupBoxType->setLayout(hBox);

		layoutMain->addWidget(groupBoxType);

		/// group: mode
		// get the joint model
		planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
		const robot_model::RobotModelConstPtr& robotModel = planning_scene_monitor_->getRobotModel();
		std::vector<std::string> groupNames = robotModel->getJointModelGroupNames();
		joint_model_group_name_ = getPlanningGroupOfTask();
		if (joint_model_group_name_.empty())
		{
			joint_model_group_name_ = groupNames.front();
		}
		joint_model_group_ = robotModel->getJointModelGroup(joint_model_group_name_);
		
		// line 1: planning group
		QGroupBox* groupBoxMode = new QGroupBox("Mode");
		QVBoxLayout* vBox = new QVBoxLayout();
		hBox = new QHBoxLayout();
		QLabel* labelPlanningGroup = new QLabel("Planning group");
		hBox->addWidget(labelPlanningGroup);
		QComboBox* combxPlanningGroup = new QComboBox();
		for (const std::string& groupName : groupNames)
		{
			combxPlanningGroup->addItem(QString::fromStdString(groupName));
		}
		combxPlanningGroup->setCurrentText(QString::fromStdString(joint_model_group_name_));
		getPlanningGroupOfTask();
		hBox->addWidget(combxPlanningGroup);
		hBox->addStretch();
		vBox->addLayout(hBox);
		// line 2: motion options
		hBox = new QHBoxLayout();
		combx_mode_ = new QComboBox();
		combx_mode_->addItems(QStringList({ "Move to", "Move relative"/*, "Waypoints"*/ }));
		hBox->addWidget(combx_mode_);
		radio_joint_space_ = new QRadioButton("Joint space");
		radio_cartesian_space_ = new QRadioButton("Cartesian space");
		radio_joint_space_->setChecked(true);
		hBox->addWidget(radio_joint_space_);
		hBox->addWidget(radio_cartesian_space_);
		vBox->addLayout(hBox);
		// line3: duration from previous
		hBox = new QHBoxLayout();
		QLabel* labelDuration = new QLabel("duration from previous(s)");
		hBox->addWidget(labelDuration);
		spin_box_duration_ = new QDoubleSpinBox();
		spin_box_duration_->setRange(0.0, 3600.0); // 1 hour
		spin_box_duration_->setValue(0.0);
		spin_box_duration_->setSingleStep(0.1);
		hBox->addWidget(spin_box_duration_);
		QSpacerItem* horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
		hBox->addItem(horizontalSpacer);
		vBox->addLayout(hBox);

		groupBoxMode->setLayout(vBox);

		connect(combxPlanningGroup, &QComboBox::currentTextChanged, this,
			[=](const QString& Text) { onPlanningGroupTextChanged(Text); });
		connect(combx_mode_, QOverload<int>::of(&QComboBox::activated), this,
			[=](int Index) { onModeIndexChanged(Index, this); });

		layoutMain->addWidget(groupBoxMode);

		/// group: variable group according to the active motion option
		layoutMain->addWidget(initGroupMoveTo());
		layoutMain->addStretch();

		/// buttons
		hBox = new QHBoxLayout();
		hBox->addStretch();
		QPushButton* buttonAdd = new QPushButton("Add");
		hBox->addWidget(buttonAdd);
		QPushButton* buttonCancel = new QPushButton("Cancel");
		hBox->addWidget(buttonCancel);

		connect(buttonAdd, &QPushButton::clicked, this, [=]() { add(); done(0); });
		connect(buttonCancel, &QPushButton::clicked, this, [=]() { done(1); });

		layoutMain->addLayout(hBox);
	}

	QWidget* DlgAddTaskStage::initGroupMoveTo()
	{
		QGroupBox* groupMoveTo = new QGroupBox("Move To");

		// pose type
		QVBoxLayout* layoutMain = new QVBoxLayout();
		QHBoxLayout* hBox = new QHBoxLayout();
		QRadioButton* radioPoseGroup = new QRadioButton("Pose Group");
		QRadioButton* radioTcp = new QRadioButton("TCP");
		hBox->addWidget(radioPoseGroup);
		hBox->addWidget(radioTcp);
		hBox->addStretch();
		layoutMain->addLayout(hBox);

		connect(radioPoseGroup, &QRadioButton::clicked, this,
			[=](bool Checked) { onMoveToTargetChecked(Checked, radioPoseGroup->text(), groupMoveTo); });
		connect(radioTcp, &QRadioButton::clicked, this,
			[=](bool Checked) { onMoveToTargetChecked(Checked, radioTcp->text(), groupMoveTo); });

		radioPoseGroup->setChecked(true);

		// type variant, default is pose group
		layoutMain->addLayout(initMoveToPoseGroup());

		groupMoveTo->setLayout(layoutMain);

		return groupMoveTo;
	}

	QWidget* DlgAddTaskStage::initGroupMoveRelative()
	{
		QGroupBox* groupMoveRelative = new QGroupBox("Move Relative");

		// pose type
		QVBoxLayout* layoutMain = new QVBoxLayout();
		QHBoxLayout* hBox = new QHBoxLayout();
		QRadioButton* radioTcp = new QRadioButton("TCP");
		QRadioButton* radioJoint = new QRadioButton("Joint");
		hBox->addWidget(radioTcp);
		hBox->addWidget(radioJoint);
		hBox->addStretch();
		layoutMain->addLayout(hBox);

		connect(radioTcp, &QRadioButton::clicked, this,
			[=](bool Checked) { onMoveRelativeTargetChecked(Checked, radioTcp->text(), groupMoveRelative); });
		connect(radioJoint, &QRadioButton::clicked, this,
			[=](bool Checked) { onMoveRelativeTargetChecked(Checked, radioJoint->text(), groupMoveRelative); });

		radioTcp->setChecked(true);

		// type variant, default is pose tcp
		layoutMain->addLayout(initMoveRelativeTcp());

		groupMoveRelative->setLayout(layoutMain);

		return groupMoveRelative;
	}

	QWidget* DlgAddTaskStage::initGroupWaypoints()
	{
		QGroupBox* groupWaypoints = new QGroupBox("Waypoints");

		QVBoxLayout* layoutMain = new QVBoxLayout();
		if (table_waypoints_.isNull())
		{
			table_waypoints_ = new QTableWidget();
		}
		QStringList header = { "x", "y", "z", "roll", "pitch", "yaw" };
		table_waypoints_.data()->setColumnCount(header.size());
		table_waypoints_.data()->setHorizontalHeaderLabels(header);
		table_waypoints_.data()->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
		table_waypoints_.data()->setContextMenuPolicy(Qt::CustomContextMenu);
		layoutMain->addWidget(table_waypoints_.data());

		QHBoxLayout* hBox = new QHBoxLayout();
		QCheckBox* current = new QCheckBox("Current");
		hBox->addWidget(current);
		hBox->addStretch();
		QPushButton* buttonAdd = new QPushButton("Add");
		hBox->addWidget(buttonAdd);
		QPushButton* buttonInsert = new QPushButton("Insert");
		hBox->addWidget(buttonInsert);
		QPushButton* buttonRemove = new QPushButton("Remove");
		hBox->addWidget(buttonRemove);
		layoutMain->addLayout(hBox);

		connect(buttonAdd, &QPushButton::clicked, this, [=]() { onWaypointsAddClicked(current); });
		connect(buttonInsert, &QPushButton::clicked, this, [=]() { onWaypointsInsertClicked(current); });
		connect(buttonRemove, &QPushButton::clicked, this, [=]() { onWaypointsRemoveClicked(current); });
		connect(table_waypoints_.data(), &QTableWidget::cellChanged, this,
			[=](int Row, int Column) { visualizeWaypoints(table_waypoints_.data(), Row); });
    	connect(table_waypoints_.data(), &QTableWidget::currentCellChanged, this,
			[=](int Row, int Column) { visualizeWaypoints(table_waypoints_.data(), Row); });
		connect(table_waypoints_.data(), &QTableWidget::customContextMenuRequested, this, [=](const QPoint& Pos)
		{
        	QMenu* menu = new QMenu;
        	auto action = menu->addAction("Current");
        	QObject::connect(action, &QAction::triggered, this, [=]()
			{
				QTableWidgetItem* item = table_waypoints_.data()->itemAt(Pos);

				geometry_msgs::Pose currentPose = queryCurrent();
				tf::Quaternion quat(currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z,
					currentPose.orientation.w);
  				double roll = 0.0, pitch = 0.0, yaw = 0.0;
  				tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

				std::shared_ptr<QStringList> data = std::make_shared<QStringList>(
					QStringList({ QString::number(currentPose.position.x), QString::number(currentPose.position.y),
					QString::number(currentPose.position.z), QString::number(angles::to_degrees(roll)),
					QString::number(angles::to_degrees(pitch)), QString::number(angles::to_degrees(yaw)) }));
				fillRowWith(table_waypoints_.data(), item->row(), data);

				action->deleteLater();
            	menu->deleteLater();
        	});
        	menu->exec(QCursor::pos());
        	menu->clear();
    	});

		groupWaypoints->setLayout(layoutMain);

		geometry_msgs::PoseStamped pose;
		pose.pose.orientation.w = 1.0;
		frame_reference_ = pose.pose;

		return groupWaypoints;
	}

	QLayout* DlgAddTaskStage::initMoveToPoseGroup()
	{
		QHBoxLayout* hBox = new QHBoxLayout();
		if (combx_pose_group_.isNull())
		{
			combx_pose_group_ = new QComboBox();
		}
		hBox->addWidget(combx_pose_group_.data());
		hBox->addStretch();

		const robot_model::RobotModelConstPtr& robotModel = planning_scene_monitor_->getRobotModel();
		joint_model_group_ = robotModel->getJointModelGroup(joint_model_group_name_);
		for (const std::string& state : joint_model_group_->getDefaultStateNames())
		{
			combx_pose_group_.data()->addItem(QString::fromStdString(state));
		}
		
		return hBox;
	}

	QLayout* DlgAddTaskStage::initMoveToTcp()
	{
		QVBoxLayout* vBox = new QVBoxLayout();
		if (table_tcp_.isNull())
		{
			table_tcp_ = new QTableWidget();
		}
		table_tcp_.data()->setRowCount(0);

		QStringList header = { "x", "y", "z", "roll", "pitch", "yaw" };
		table_tcp_.data()->setColumnCount(header.size());
		table_tcp_.data()->setHorizontalHeaderLabels(header);
		table_tcp_.data()->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
		table_tcp_.data()->insertRow(table_tcp_.data()->rowCount());
		fillRowWith(table_tcp_.data(), 0);
		vBox->addWidget(table_tcp_.data());

		QHBoxLayout* hBox = new QHBoxLayout();
		hBox->addStretch();
		QPushButton* buttonCurrent = new QPushButton("Current");
		hBox->addWidget(buttonCurrent);
		vBox->addLayout(hBox);

		connect(buttonCurrent, &QPushButton::clicked, this, [=]() { onMoveToTcpCurrentClicked(); });
		connect(table_tcp_.data(), &QTableWidget::cellChanged, this,
			[=](int Row, int Column) { visualizeWaypoints(table_tcp_.data(), Row);  });

		// interactive marker
		geometry_msgs::PoseStamped pose;
		pose.pose.orientation.w = 1.0;
		frame_reference_ = pose.pose;
		visualizeWaypoints(table_tcp_.data(), 0);

		return vBox;
	}

	QLayout* DlgAddTaskStage::initMoveRelativeTcp()
	{
		QVBoxLayout* vBox = new QVBoxLayout();
		if (table_tcp_.isNull())
		{
			table_tcp_ = new QTableWidget();
		}
		table_tcp_.data()->setRowCount(0);

		QStringList header = { "x", "y", "z", "roll", "pitch", "yaw" };
		table_tcp_.data()->setColumnCount(header.size());
		table_tcp_.data()->setHorizontalHeaderLabels(header);
		table_tcp_.data()->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
		table_tcp_.data()->insertRow(table_tcp_.data()->rowCount());
		fillRowWith(table_tcp_.data(), 0);
		vBox->addWidget(table_tcp_.data());

		connect(table_tcp_.data(), &QTableWidget::cellChanged, this, [=](int Row, int Column)
		{
			std::vector<geometry_msgs::Pose> waypoints;
			retrieveWaypoints(table_tcp_.data(), waypoints);

			tf::Quaternion quatTable(waypoints.front().orientation.x, waypoints.front().orientation.y,
				waypoints.front().orientation.z, waypoints.front().orientation.w);
			double rollTable = 0.0, pitchTable = 0.0, yawTable = 0.0;
			tf::Matrix3x3(quatTable).getRPY(rollTable, pitchTable, yawTable);

			tf::Quaternion quatReference(frame_reference_.orientation.x, frame_reference_.orientation.y,
				frame_reference_.orientation.z, frame_reference_.orientation.w);
			double rollReference = 0.0, pitchReference = 0.0, yawReference = 0.0;
			tf::Matrix3x3(quatReference).getRPY(rollReference, pitchReference, yawReference);

			geometry_msgs::PoseStamped pose;
			pose.pose.position.x = frame_reference_.position.x + waypoints.front().position.x;
			pose.pose.position.y = frame_reference_.position.y + waypoints.front().position.y;
			pose.pose.position.z = frame_reference_.position.z + waypoints.front().position.z;
			tf2::Quaternion orientation;
			orientation.setRPY(angles::from_degrees(rollReference + rollTable),
				angles::from_degrees(pitchReference + pitchTable),
				angles::from_degrees(yawReference + yawTable));
			pose.pose.orientation = tf2::toMsg(orientation);

			std::vector<geometry_msgs::PoseStamped> poses;
			poses.push_back(pose);
			display_->visualizeInteractiveMarkers(0, poses,
				std::bind(&DlgAddTaskStage::interactiveMarkerProcessFeedback,
				this, std::placeholders::_1, std::placeholders::_2));
		});

		// TCP only runs in Cartesian space
		radio_joint_space_->setEnabled(false);
		radio_cartesian_space_->setChecked(true);

		// interactive marker
		frame_reference_ = queryCurrent();
		geometry_msgs::PoseStamped pose;
		pose.pose = frame_reference_;
		std::vector<geometry_msgs::PoseStamped> poses;
		poses.push_back(pose);
		display_->visualizeInteractiveMarkers(0, poses,
			std::bind(&DlgAddTaskStage::interactiveMarkerProcessFeedback,
			this, std::placeholders::_1, std::placeholders::_2));

		return vBox;
	}

	QLayout* DlgAddTaskStage::initMoveRelativeJoint()
	{
		QVBoxLayout* vBox = new QVBoxLayout();
		if (table_joint_.isNull())
		{
			table_joint_ = new QTableWidget();
		}
		table_joint_.data()->setRowCount(0);

		QStringList header;
		for (std::size_t i = 0; i < joint_model_group_->getVariableNames().size(); ++i)
		{
			header.push_back("J" + QString::number(i + 1));
		}
		table_joint_.data()->setColumnCount(header.size());
		table_joint_.data()->setHorizontalHeaderLabels(header);
		table_joint_.data()->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
		table_joint_.data()->insertRow(table_joint_.data()->rowCount());
		fillRowWith(table_joint_.data(), 0);
		vBox->addWidget(table_joint_.data());

		connect(table_joint_.data(), &QTableWidget::cellChanged, this,
			[=](int Row, int Column)
			{ std::cout << "cell changed row " << std::to_string(Row) << " col " << std::to_string(Column) << std::endl; });

		return vBox;
	}

	void DlgAddTaskStage::removeLayout(QWidget* Group, int Index)
	{
		QLayout* main = Group->layout();
		QLayoutItem* removing = main->itemAt(Index);
		if (removing->widget())
		{
			if (removing->widget()->layout())
			{
				for (int i = 0; i < removing->widget()->layout()->count(); ++i)
				{
					removeLayout(removing->widget(), i);
				}
			}
			delete removing->widget();
		}
		else
		{		
			QLayoutItem* child = nullptr;
			while ((child = removing->layout()->takeAt(0)) != nullptr)
			{
				if (child->widget())
				{
					child->widget()->setParent(nullptr);
					delete child->widget();
				}
				delete child;
			}
		}
		main->removeItem(removing);
	}

	void DlgAddTaskStage::mousePressEvent(QMouseEvent* Event)
	{
		if (Event->button() == Qt::LeftButton)
		{
			if (!table_waypoints_.isNull() && table_waypoints_.data()->itemAt(Event->pos()) == nullptr)
			{
				visualizeWaypoints(table_waypoints_.data(), -1);
			}
    	}
	}

	void DlgAddTaskStage::closeEvent(QCloseEvent* Event)
	{
		done(1);
	}

	void DlgAddTaskStage::onPlanningGroupTextChanged(const QString& Text)
	{
		joint_model_group_name_ = Text.toStdString();
		const robot_model::RobotModelConstPtr& robotModel = planning_scene_monitor_->getRobotModel();
		joint_model_group_ = robotModel->getJointModelGroup(joint_model_group_name_);

		if (!combx_pose_group_.isNull())
		{
			combx_pose_group_.data()->clear();

			for (const std::string& state : joint_model_group_->getDefaultStateNames())
			{
				combx_pose_group_.data()->addItem(QString::fromStdString(state));
			}
		}
		if (!table_joint_.isNull())
		{
			table_joint_.data()->setRowCount(0);

			QStringList header;
			for (std::size_t i = 0; i < joint_model_group_->getVariableNames().size(); ++i)
			{
				header.push_back("J" + QString::number(i + 1));
			}
			table_joint_.data()->setColumnCount(header.size());
			table_joint_.data()->setHorizontalHeaderLabels(header);
			table_joint_.data()->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
			table_joint_.data()->insertRow(table_joint_.data()->rowCount());
			fillRowWith(table_joint_.data(), 0);
		}
	}

	void DlgAddTaskStage::onModeIndexChanged(int Index, QWidget* Group)
	{
		removeLayout(Group, 2);
		display_->clearInteractiveMarkers();
		radio_joint_space_->setEnabled(true);
		radio_cartesian_space_->setEnabled(true);

		if (Index == 0)
		{
			((QVBoxLayout*)Group->layout())->insertWidget(2, initGroupMoveTo());
		}
		else if (Index == 1)
		{
			((QVBoxLayout*)Group->layout())->insertWidget(2, initGroupMoveRelative());
		}
		else if (Index == 2)
		{
			((QVBoxLayout*)Group->layout())->insertWidget(2, initGroupWaypoints());
			radio_joint_space_->setEnabled(false);
			radio_cartesian_space_->setChecked(true);
		}
	}

	void DlgAddTaskStage::onMoveToTargetChecked(bool Checked, const QString& Name, QWidget* Group)
	{
		removeLayout(Group, 1);
		display_->clearInteractiveMarkers();
		
		if (Name == "Pose Group")
		{
			target_type_ = TARGET_TYPE_POSE_GROUP;
			((QVBoxLayout*)Group->layout())->addLayout(initMoveToPoseGroup());
		}
		else if (Name == "TCP")
		{
			target_type_ = TARGET_TYPE_TCP;
			((QVBoxLayout*)Group->layout())->addLayout(initMoveToTcp());
		}
	}

	void DlgAddTaskStage::onMoveToTcpCurrentClicked()
	{
		geometry_msgs::Pose currentPose = current_goal_ ? *current_goal_ : queryCurrent();
		tf::Quaternion quat(currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z,
			currentPose.orientation.w);
  		double roll = 0.0, pitch = 0.0, yaw = 0.0;
  		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

		std::shared_ptr<QStringList> data = std::make_shared<QStringList>(QStringList({
			QString::number(currentPose.position.x), QString::number(currentPose.position.y),
			QString::number(currentPose.position.z), QString::number(angles::to_degrees(roll)),
			QString::number(angles::to_degrees(pitch)), QString::number(angles::to_degrees(yaw)) }));
		fillRowWith(table_tcp_.data(), 0, data);

		visualizeWaypoints(table_tcp_.data(), 0);
	}

	void DlgAddTaskStage::onMoveRelativeTargetChecked(bool Checked, const QString& Name, QWidget* Group)
	{
		removeLayout(Group, 1);
		display_->clearInteractiveMarkers();

		if (Name == "TCP")
		{
			((QVBoxLayout*)Group->layout())->addLayout(initMoveRelativeTcp());

			radio_joint_space_->setEnabled(false);
			radio_cartesian_space_->setEnabled(true);
			radio_cartesian_space_->setChecked(true);
		}
		else if (Name == "Joint")
		{
			((QVBoxLayout*)Group->layout())->addLayout(initMoveRelativeJoint());

			radio_joint_space_->setEnabled(true);
			radio_joint_space_->setChecked(true);
			radio_cartesian_space_->setEnabled(false);
		}
	}

	void DlgAddTaskStage::onWaypointsAddClicked(QCheckBox* CheckCurrent)
	{
		table_waypoints_.data()->blockSignals(true);
		table_waypoints_.data()->insertRow(table_waypoints_.data()->rowCount());

		std::shared_ptr<QStringList> data = nullptr;
		if (CheckCurrent->isChecked())
		{
			geometry_msgs::Pose currentPose = queryCurrent();
			tf::Quaternion quat(currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z,
				currentPose.orientation.w);
  			double roll = 0.0, pitch = 0.0, yaw = 0.0;
  			tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

			data.reset(new QStringList({ QString::number(currentPose.position.x), QString::number(currentPose.position.y),
				QString::number(currentPose.position.z), QString::number(angles::to_degrees(roll)),
				QString::number(angles::to_degrees(pitch)), QString::number(angles::to_degrees(yaw)) }));
		}
		
		fillRowWith(table_waypoints_.data(), table_waypoints_.data()->rowCount() - 1, data);
		table_waypoints_.data()->blockSignals(false);

		table_waypoints_.data()->setCurrentCell(table_waypoints_.data()->rowCount() - 1, 0);
	}

	void DlgAddTaskStage::onWaypointsInsertClicked(QCheckBox* CheckCurrent)
	{
		table_waypoints_.data()->blockSignals(true);
		table_waypoints_.data()->insertRow(table_waypoints_.data()->currentRow());

		std::shared_ptr<QStringList> data = nullptr;
		if (CheckCurrent->isChecked())
		{
			geometry_msgs::Pose currentPose = queryCurrent();
			tf::Quaternion quat(currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z,
				currentPose.orientation.w);
  			double roll = 0.0, pitch = 0.0, yaw = 0.0;
  			tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

			data.reset(new QStringList({ QString::number(currentPose.position.x), QString::number(currentPose.position.y),
				QString::number(currentPose.position.z), QString::number(angles::to_degrees(roll)),
				QString::number(angles::to_degrees(pitch)), QString::number(angles::to_degrees(yaw)) }));
		}
		
		fillRowWith(table_waypoints_.data(), table_waypoints_.data()->currentRow() - 1, data);
		table_waypoints_.data()->blockSignals(false);

		table_waypoints_.data()->setCurrentCell(table_waypoints_.data()->currentRow() - 1, 0);
	}

	void DlgAddTaskStage::onWaypointsRemoveClicked(QCheckBox* CheckCurrent)
	{
		int highlightRow = table_waypoints_.data()->currentRow() == table_waypoints_.data()->rowCount() - 1 ?
			table_waypoints_.data()->rowCount() - 2 : table_waypoints_.data()->currentRow();
		table_waypoints_.data()->blockSignals(true);
		table_waypoints_.data()->removeRow(table_waypoints_.data()->currentRow());
		table_waypoints_.data()->blockSignals(false);

		table_waypoints_.data()->setCurrentCell(highlightRow, 0);
		visualizeWaypoints(table_waypoints_.data(), highlightRow);
	}

	void DlgAddTaskStage::fillRowWith(QTableWidget* Table, int Row, const std::shared_ptr<QStringList> Data/* = nullptr*/)
	{
		if (Data)
		{
			for (int i = 0; i < std::min(Table->columnCount(), Data->size()); ++i)
			{
				Table->setItem(Row, i, new QTableWidgetItem(Data->at(i)));
			}
		}
		else
		{
			for (int i = 0; i < Table->columnCount(); ++i)
			{
				Table->setItem(Row, i, new QTableWidgetItem("0.0"));
			}
		}
	}

	geometry_msgs::Pose DlgAddTaskStage::queryCurrent()
	{
		geometry_msgs::Pose currentPose;

		moveit::planning_interface::MoveGroupInterface moveGroupInterface(joint_model_group_name_);
		const std::string& link_name = moveGroupInterface.getEndEffectorLink();
  		const moveit::core::LinkModel* link = moveGroupInterface.getRobotModel()->getLinkModel(link_name);
		if (link)
		{
#ifdef SCENE_MONITOR
			moveit::core::RobotState state =
				planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentState();
			currentPose = tf2::toMsg(state.getGlobalLinkTransform(link));
#else
			const robot_model::RobotModelConstPtr& robotModel = planning_scene_monitor_->getRobotModel();
			auto scene = std::make_shared<planning_scene::PlanningScene>(robotModel);

			ros::NodeHandle h;
			ros::ServiceClient client = h.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
			if (client.waitForExistence(ros::Duration(5.0)))
			{
				moveit_msgs::GetPlanningScene::Request req;
				moveit_msgs::GetPlanningScene::Response res;

				req.components.components =
					moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS |
					moveit_msgs::PlanningSceneComponents::ROBOT_STATE |
					moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
					moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES |
					moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
					moveit_msgs::PlanningSceneComponents::OCTOMAP |
					moveit_msgs::PlanningSceneComponents::TRANSFORMS |
					moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
					moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING |
					moveit_msgs::PlanningSceneComponents::OBJECT_COLORS;

				if (client.call(req, res))
				{
					scene->setPlanningSceneMsg(res.scene);
					const robot_state::RobotState& state = scene->getCurrentState();
					/*std::vector<double> jointGroupPositions;
					state.copyJointGroupPositions(joint_model_group_, jointGroupPositions);
					for (const auto& it : jointGroupPositions)
					{
						std::cout << it << std::endl;
					}*/
					currentPose = tf2::toMsg(state.getGlobalLinkTransform(link));
				}
			}
#endif
		}

#ifdef DEBUG
		std::cout << currentPose.position.x << " " << currentPose.position.y << " " << currentPose.position.z << std::endl;
#endif
		return currentPose;
	}

	void DlgAddTaskStage::retrieveWaypoints(QTableWidget* Table, std::vector<geometry_msgs::Pose>& Waypoints)
	{
  		for (int i = 0; i < Table->rowCount(); ++i)
		{
			geometry_msgs::Pose pose;
			pose.position.x = Table->item(i, 0)->text().toDouble();
			pose.position.y = Table->item(i, 1)->text().toDouble();
			pose.position.z = Table->item(i, 2)->text().toDouble();

			tf2::Quaternion orientation;
			orientation.setRPY(angles::from_degrees(Table->item(i, 3)->text().toDouble()),
				angles::from_degrees(Table->item(i, 4)->text().toDouble()),
				angles::from_degrees(Table->item(i, 5)->text().toDouble()));

			pose.orientation = tf2::toMsg(orientation);
#ifdef DEBUG
			std::cout << "quaternion from euler " << pose.orientation.x << " " << pose.orientation.y << " "
				<< pose.orientation.z << " " << pose.orientation.w << std::endl;
#endif

			Waypoints.push_back(pose);
		}
	}

	void DlgAddTaskStage::retrieveWaypoints(QTableWidget* Table, std::vector<std::vector<double>>& Waypoints)
	{
  		for (int i = 0; i < Table->rowCount(); ++i)
		{
			std::vector<double> point;
			for (int j = 0; j < Table->columnCount(); ++j)
			{
				point.push_back(Table->item(i, j)->text().toDouble());
			}

			Waypoints.push_back(point);
		}
	}

	void DlgAddTaskStage::visualizeWaypoints(QTableWidget* Table, int Row)
	{
		std::vector<geometry_msgs::Pose> waypoints;
		retrieveWaypoints(Table, waypoints);

		std::vector<geometry_msgs::PoseStamped> stamped;
		for (const auto& it : waypoints)
		{
			geometry_msgs::PoseStamped poseStamped;
			poseStamped.pose = it;
			stamped.push_back(poseStamped);
		}
		display_->visualizeInteractiveMarkers(Row, stamped,
			std::bind(&DlgAddTaskStage::interactiveMarkerProcessFeedback, this, std::placeholders::_1, std::placeholders::_2));
	}

	void DlgAddTaskStage::interactiveMarkerProcessFeedback(int Index, visualization_msgs::InteractiveMarkerFeedback& Feedback)
	{
		QTableWidget* table = nullptr;
		if (!table_tcp_.isNull())
		{
			table = table_tcp_.data();
		}
		else if (!table_joint_.isNull())
		{
			table = table_joint_.data();
		}
		else if (!table_waypoints_.isNull())
		{
			table = table_waypoints_.data();
		}

		if (table)
		{
			tf::Quaternion quatFeedback(Feedback.pose.orientation.x, Feedback.pose.orientation.y,
				Feedback.pose.orientation.z, Feedback.pose.orientation.w);
			double rollFeedback = 0.0, pitchFeedback = 0.0, yawFeedback = 0.0;
			tf::Matrix3x3(quatFeedback).getRPY(rollFeedback, pitchFeedback, yawFeedback);

			tf::Quaternion quatReference(frame_reference_.orientation.x, frame_reference_.orientation.y,
				frame_reference_.orientation.z, frame_reference_.orientation.w);
			double rollReference = 0.0, pitchReference = 0.0, yawReference = 0.0;
			tf::Matrix3x3(quatReference).getRPY(rollReference, pitchReference, yawReference);

			table->blockSignals(true);
			table->setItem(Index, 0, new QTableWidgetItem(QString::number(Feedback.pose.position.x - frame_reference_.position.x)));
			table->setItem(Index, 1, new QTableWidgetItem(QString::number(Feedback.pose.position.y - frame_reference_.position.y)));
			table->setItem(Index, 2, new QTableWidgetItem(QString::number(Feedback.pose.position.z - frame_reference_.position.z)));
			table->setItem(Index, 3, new QTableWidgetItem(QString::number(angles::to_degrees(rollFeedback - rollReference))));
			table->setItem(Index, 4, new QTableWidgetItem(QString::number(angles::to_degrees(pitchFeedback - pitchReference))));
			table->setItem(Index, 5, new QTableWidgetItem(QString::number(angles::to_degrees(yawFeedback - yawReference))));
			table->blockSignals(false);
		}
#ifdef DEBUG
		ROS_INFO_STREAM( Feedback.marker_name << " is now at "
			<< Feedback.pose.position.x << ", " << Feedback.pose.position.y << ", " << Feedback.pose.position.z );
#endif
	}

	std::string DlgAddTaskStage::getPlanningGroupOfTask()
	{
		const moveit::task_constructor::Stage* stage = (*task_->stages())[task_->stages()->numChildren() - 1];
		if (task_->stages()->numChildren() > 1 && stage)
		{
			// stage current has no group property
			return boost::any_cast<std::string>(stage->properties().get("group"));
		}
		else
		{
			return std::string();
		}
	}

	void DlgAddTaskStage::add()
	{
		// create Cartesian interpolation "planner" to be used in various stages
		auto cartesian_interpolation = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
		// create a joint-space interpolation "planner" to be used in various stages
		auto joint_interpolation = std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();

		// start from a fixed robot state
		if (type_ == TYPE_TASK)
		{
			task_->stages()->setName(line_task_name_->text().toStdString());
			task_->add(std::make_unique<moveit::task_constructor::stages::CurrentState>("current"));
		}

		if (combx_mode_->currentIndex() == 0)
		{
			if (target_type_ == TARGET_TYPE_POSE_GROUP)
			{
				auto stage = radio_joint_space_->isChecked() ? 
					std::make_unique<moveit::task_constructor::stages::MoveTo>("moveTo " + combx_pose_group_->currentText().toStdString(), joint_interpolation) :
					std::make_unique<moveit::task_constructor::stages::MoveTo>("moveTo " + combx_pose_group_->currentText().toStdString(), cartesian_interpolation);
				stage->setGroup(joint_model_group_name_);
				stage->setGoal(combx_pose_group_->currentText().toStdString());
				stage->properties().set("duration_from_previous", spin_box_duration_->value());
				task_->add(std::move(stage));
			}
			else
			{
				std::vector<geometry_msgs::Pose> waypoints;
				retrieveWaypoints(table_tcp_.data(), waypoints);
				geometry_msgs::PoseStamped poseTcp;
				poseTcp.pose = waypoints.front();

				auto stage = radio_joint_space_->isChecked() ? 
					std::make_unique<moveit::task_constructor::stages::MoveTo>("moveTo TCP", joint_interpolation) :
					std::make_unique<moveit::task_constructor::stages::MoveTo>("moveTo TCP", cartesian_interpolation);
				stage->setGroup(joint_model_group_name_);
				stage->setGoal(poseTcp);
				stage->properties().set("duration_from_previous", spin_box_duration_->value());
				task_->add(std::move(stage));
			}
		}
		else if (combx_mode_->currentIndex() == 1)
		{
			std::vector<std::vector<double>> rawPoints;
			if (!table_tcp_.isNull())
			{
				retrieveWaypoints(table_tcp_.data(), rawPoints);

				std::string descriptionVector;
				descriptionVector += fabs(rawPoints.front()[0]) > 1e-5 ? "x" + std::to_string(rawPoints.front()[0]) : std::string();
				descriptionVector += fabs(rawPoints.front()[1]) > 1e-5 ? "; y" + std::to_string(rawPoints.front()[1]) : std::string();
				descriptionVector += fabs(rawPoints.front()[2]) > 1e-5 ? "; z" + std::to_string(rawPoints.front()[2]) : std::string();
				if (!descriptionVector.empty())
				{
					if (descriptionVector[0] == ';')
					{
						descriptionVector = descriptionVector.substr(2);
					}
					auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>(descriptionVector, cartesian_interpolation);
					stage->setGroup(joint_model_group_name_);
					geometry_msgs::Vector3Stamped direction;
					direction.header.frame_id = "world";
					direction.vector.x = rawPoints.front()[0];
					direction.vector.y = rawPoints.front()[1];
					direction.vector.z = rawPoints.front()[2];
					stage->setDirection(direction);
					stage->properties().set("duration_from_previous", spin_box_duration_->value());
					task_->add(std::move(stage));
				}

				std::string descriptionTwist;
				descriptionTwist += fabs(rawPoints.front()[3]) > 1e-5 ? "rx" + std::to_string(rawPoints.front()[3]) : std::string();
				descriptionTwist += fabs(rawPoints.front()[4]) > 1e-5 ? "; ry" + std::to_string(rawPoints.front()[4]) : std::string();
				descriptionTwist += fabs(rawPoints.front()[5]) > 1e-5 ? "; rz" + std::to_string(rawPoints.front()[5]) : std::string();
				if (!descriptionTwist.empty())
				{
					if (descriptionTwist[0] == ';')
					{
						descriptionTwist = descriptionTwist.substr(2);
					}
					auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>(descriptionTwist, cartesian_interpolation);
					stage->setGroup(joint_model_group_name_);
					geometry_msgs::TwistStamped twist;
					twist.header.frame_id = "world";
					twist.twist.angular.x = angles::from_degrees(rawPoints.front()[3]);
					twist.twist.angular.y = angles::from_degrees(rawPoints.front()[4]);
					twist.twist.angular.z = angles::from_degrees(rawPoints.front()[5]);
					stage->setDirection(twist);
					stage->properties().set("duration_from_previous", spin_box_duration_->value());
					task_->add(std::move(stage));
				}
			}
			else if (!table_joint_.isNull())
			{
				retrieveWaypoints(table_joint_.data(), rawPoints);
				std::vector<std::string> jointsName = joint_model_group_->getVariableNames();

				std::map<std::string, double> offsets;
				for (std::size_t i = 0; i < std::min(jointsName.size(), rawPoints.front().size()); ++i)
				{
					offsets.emplace(jointsName[i], angles::from_degrees(rawPoints.front()[i]));
				}

				auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("joint offset", cartesian_interpolation);
				stage->setGroup(joint_model_group_name_);
				stage->setDirection(offsets);
				stage->properties().set("duration_from_previous", spin_box_duration_->value());
				task_->add(std::move(stage));
			}
		}

		if (task_->plan())
		{
			task_->introspection().publishSolution(*task_->solutions().back());
		}
	}
}  // namespace moveit_rviz_plugin

