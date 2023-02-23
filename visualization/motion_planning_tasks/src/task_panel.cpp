/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Robert Haschke
   Desc:   Monitor manipulation tasks and visualize their solutions
*/

#include <stdio.h>

#include "task_panel_p.h"
#include "meta_task_list_model.h"
#include "local_task_model.h"
#include "factory_model.h"
#include "pluginlib_factory.h"
#include "task_display.h"
#include <moveit/visualization_tools/task_solution_visualization.h>
#include <moveit/visualization_tools/display_solution.h>
#include <moveit/task_constructor/stage.h>

#include <rviz/properties/property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/display_group.h>
#include <rviz/visualization_manager.h>
#include <rviz/window_manager_interface.h>
#include <rviz/visualization_frame.h>
#include <rviz/panel_dock_widget.h>
#include <ros/console.h>
#include <QPointer>
#include <QButtonGroup>
#include <QFileDialog>

#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit_task_constructor_msgs/Solution.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <angles/angles.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <fstream>
#include <thread>

namespace moveit_rviz_plugin {

rviz::PanelDockWidget* getStageDockWidget(rviz::WindowManagerInterface* mgr) {
	static QPointer<rviz::PanelDockWidget> widget = nullptr;
	if (!widget && mgr) {  // create widget
		StageFactoryPtr factory = getStageFactory();
		if (!factory)
			return nullptr;
		QTreeView* view = new QTreeView();
		view->setModel(new FactoryModel(*factory, factory->mimeType(), view));
		view->expandAll();
		view->setHeaderHidden(true);
		view->setDragDropMode(QAbstractItemView::DragOnly);
		widget = mgr->addPane("Motion Planning Stages", view);
	}
	widget->show();
	return widget;
}

// TaskPanel singleton
static QPointer<TaskPanel> SINGLETON;
// count active TaskDisplays
static uint DISPLAY_COUNT = 0;

TaskPanel::TaskPanel(QWidget* parent) : rviz::Panel(parent), d_ptr(new TaskPanelPrivate(this)) {
	Q_D(TaskPanel);

	// sync checked tool button with displayed widget
	connect(d->tool_buttons_group, static_cast<void (QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked),
	        d->stackedWidget, [d](int index) { d->stackedWidget->setCurrentIndex(index); });
	connect(d->stackedWidget, &QStackedWidget::currentChanged, d->tool_buttons_group,
	        [d](int index) { d->tool_buttons_group->button(index)->setChecked(true); });

	auto* task_view = new TaskView(this, d->property_root);
	connect(d->button_exec_solution, SIGNAL(clicked()), task_view, SLOT(onExecCurrentSolution()));

	// create sub widgets with corresponding tool buttons
	addSubPanel(task_view, "Tasks View", QIcon(":/icons/tasks.png"));
	d->stackedWidget->setCurrentIndex(0);  // Tasks View is show by default

	// settings widget should come last
	addSubPanel(new GlobalSettingsWidget(this, d->property_root), "Global Settings", QIcon(":/icons/settings.png"));

	connect(d->button_show_stage_dock_widget, SIGNAL(clicked()), this, SLOT(showStageDockWidget()));

	// if still undefined, this becomes the global instance
	if (SINGLETON.isNull())
		SINGLETON = this;
}

TaskPanel::~TaskPanel() {
	delete d_ptr;
}

void TaskPanel::addSubPanel(SubPanel* w, const QString& title, const QIcon& icon) {
	Q_D(TaskPanel);

	auto button = new QToolButton(w);
	button->setToolTip(title);
	button->setIcon(icon);
	button->setCheckable(true);

	int index = d->stackedWidget->count();
	d->tool_buttons_layout->insertWidget(index, button);
	d->tool_buttons_group->addButton(button, index);
	d->stackedWidget->addWidget(w);

	w->setWindowTitle(title);
	connect(w, SIGNAL(configChanged()), this, SIGNAL(configChanged()));
}

/* Realizing a singleton Panel is a nightmare with rviz...
 * Formally, Panels (as a plugin class) cannot be singleton, because new instances are created on demand.
 * Hence, we decided to use a true singleton for the underlying model only and a fake singleton for the panel.
 * Thus, all panels (in case multiple were created) show the same content.
 * The fake singleton shall ensure that only a single panel is created, even if several displays are created.
 * To this end, the displays request() the need for a panel during their initialization and they release()
 * this need during their destruction. This, in principle, allows to create a panel together with the first
 * display and destroy it when the last display is gone.
 * Obviously, the user can still decide to explicitly delete the panel (or create new ones).

 * The nightmare arises from the order of loading of displays and panels: Displays are loaded first.
 * However, directly creating a panel with the first loaded display doesn't work, because panel loading
 * will create another panel instance later (because there is no singleton support).
 * Hence, we need to postpone the actual panel creation from displays until panel loading is finished as well.
 * This was initially done, by postponing panel creation to TaskDisplay::update(). However, update()
 * will never be called if the display is disabled...
 */

void TaskPanel::request(rviz::WindowManagerInterface* window_manager) {
	++DISPLAY_COUNT;

	rviz::VisualizationFrame* vis_frame = dynamic_cast<rviz::VisualizationFrame*>(window_manager);
	if (SINGLETON || !vis_frame)
		return;  // already defined, nothing to do

	QDockWidget* dock = vis_frame->addPanelByName(
	    "Motion Planning Tasks", "moveit_task_constructor/Motion Planning Tasks", Qt::LeftDockWidgetArea);
	Q_UNUSED(dock);
	assert(dock->widget() == SINGLETON);
}

void TaskPanel::release() {
	Q_ASSERT(DISPLAY_COUNT > 0);
	if (--DISPLAY_COUNT == 0 && SINGLETON)
		SINGLETON->deleteLater();
}

TaskPanelPrivate::TaskPanelPrivate(TaskPanel* panel) : q_ptr(panel) {
	setupUi(panel);
	tool_buttons_group = new QButtonGroup(panel);
	tool_buttons_group->setExclusive(true);
	button_show_stage_dock_widget->setEnabled(bool(getStageFactory()));
	button_show_stage_dock_widget->setToolTip("Show available stages");
	property_root = new rviz::Property("Global Settings");
}

void TaskPanel::onInitialize() {
	d_ptr->window_manager_ = vis_manager_->getWindowManager();
}

void TaskPanel::save(rviz::Config config) const {
	rviz::Panel::save(config);
	for (int i = 0; i < d_ptr->stackedWidget->count(); ++i) {
		SubPanel* w = static_cast<SubPanel*>(d_ptr->stackedWidget->widget(i));
		w->save(config.mapMakeChild(w->windowTitle()));
	}
}

void TaskPanel::load(const rviz::Config& config) {
	rviz::Panel::load(config);
	for (int i = 0; i < d_ptr->stackedWidget->count(); ++i) {
		SubPanel* w = static_cast<SubPanel*>(d_ptr->stackedWidget->widget(i));
		w->load(config.mapGetChild(w->windowTitle()));
	}
}

void TaskPanel::showStageDockWidget() {
	rviz::PanelDockWidget* dock = getStageDockWidget(d_ptr->window_manager_);
	if (dock)
		dock->show();
}

// expand all children up to given depth
void setExpanded(QTreeView* view, const QModelIndex& index, bool expand, int depth = -1) {
	if (!index.isValid())
		return;

	// recursively expand all children
	if (depth != 0) {
		for (int row = 0, rows = index.model()->rowCount(index); row < rows; ++row)
			setExpanded(view, index.model()->index(row, 0, index), expand, depth - 1);
	}

	view->setExpanded(index, expand);
}

TaskViewPrivate::TaskViewPrivate(TaskView* view) : q_ptr(view), exec_action_client_("execute_task_solution") {
	setupUi(view);

	MetaTaskListModel* meta_model = &MetaTaskListModel::instance();
	StageFactoryPtr factory = getStageFactory();
	if (factory)
		meta_model->setMimeTypes({ factory->mimeType() });
	tasks_view->setModel(meta_model);
	QObject::connect(meta_model, SIGNAL(rowsInserted(QModelIndex, int, int)), q_ptr,
	                 SLOT(configureInsertedModels(QModelIndex, int, int)));

	tasks_view->setSelectionMode(QAbstractItemView::ExtendedSelection);
	tasks_view->setAcceptDrops(true);
	tasks_view->setDefaultDropAction(Qt::CopyAction);
	tasks_view->setDropIndicatorShown(true);
	tasks_view->setDragEnabled(true);

	actionShowTimeColumn->setChecked(true);

	// init actions
	// TODO(v4hn): add actionAddLocalTask once there is something meaningful to add
	tasks_view->addActions({ actionAddLocalTask, actionRemoveTaskTreeRows, actionShowTimeColumn });
}

std::pair<TaskListModel*, TaskDisplay*> TaskViewPrivate::getTaskListModel(const QModelIndex& index) const {
	auto* meta_model = static_cast<MetaTaskListModel*>(tasks_view->model());
	return meta_model->getTaskListModel(index);
}

std::pair<BaseTaskModel*, QModelIndex> TaskViewPrivate::getTaskModel(const QModelIndex& index) const {
	auto* meta_model = static_cast<MetaTaskListModel*>(tasks_view->model());
	return meta_model->getTaskModel(index);
}

void TaskViewPrivate::configureTaskListModel(TaskListModel* model) {
	QObject::connect(q_ptr, &TaskView::oldTaskHandlingChanged, model, &TaskListModel::setOldTaskHandling);
	model->setOldTaskHandling(q_ptr->old_task_handling->getOptionInt());
}

void TaskViewPrivate::configureExistingModels() {
	auto* meta_model = static_cast<MetaTaskListModel*>(tasks_view->model());
	for (int row = meta_model->rowCount() - 1; row >= 0; --row)
		configureTaskListModel(meta_model->getTaskListModel(meta_model->index(row, 0)).first);
}

void TaskViewPrivate::configureInsertedModels(const QModelIndex& parent, int first, int last) {
	if (parent.isValid() && !parent.parent().isValid()) {  // top-level task items inserted
		int expand = q_ptr->initial_task_expand->getOptionInt();
		for (int row = first; row <= last; ++row) {
			// set expanded state of items
			QModelIndex child = parent.model()->index(row, 0, parent);
			if (expand != TaskView::EXPAND_NONE) {
				// recursively expand all inserted items
				setExpanded(tasks_view, child, true);
			}
			if (expand == TaskView::EXPAND_TOP) {
				// collapse up to first level
				setExpanded(tasks_view, child, false, 1);
				// expand inserted item
				setExpanded(tasks_view, child, true, 0);
			}

			configureTaskListModel(getTaskListModel(child).first);
		}
		tasks_view->setExpanded(parent, true);  // expand parent group item
	}
}

void TaskViewPrivate::lock(TaskDisplay* display) {
	if (locked_display_ && locked_display_ != display) {
		locked_display_->clearMarkers();
		locked_display_->visualization()->unlock();
	}
	locked_display_ = display;
}

TaskView::TaskView(moveit_rviz_plugin::TaskPanel* parent, rviz::Property* root)
  : SubPanel(parent), d_ptr(new TaskViewPrivate(this)) {
	Q_D(TaskView);

	d_ptr->tasks_property_splitter->setStretchFactor(0, 1);
	d_ptr->tasks_property_splitter->setStretchFactor(1, 3);
	d_ptr->tasks_property_splitter->setStretchFactor(2, 1);
	d_ptr->tasks_property_splitter->setStretchFactor(3, 1);

	// connect signals
	connect(d->actionRemoveTaskTreeRows, SIGNAL(triggered()), this, SLOT(removeStages()));
	connect(d->actionAddLocalTask, SIGNAL(triggered()), this, SLOT(addTask()));
	connect(d->actionShowTimeColumn, &QAction::triggered, [this](bool checked) { show_time_column->setValue(checked); });

	connect(d->tasks_view->selectionModel(), SIGNAL(currentChanged(QModelIndex, QModelIndex)), this,
	        SLOT(onCurrentStageChanged(QModelIndex, QModelIndex)));

	onCurrentStageChanged(d->tasks_view->currentIndex(), QModelIndex());

	// propagate infos about config changes
	connect(d_ptr->tasks_property_splitter, SIGNAL(splitterMoved(int, int)), this, SIGNAL(configChanged()));
	connect(d_ptr->tasks_solutions_splitter, SIGNAL(splitterMoved(int, int)), this, SIGNAL(configChanged()));
	connect(d_ptr->tasks_view->header(), SIGNAL(sectionResized(int, int, int)), this, SIGNAL(configChanged()));
	connect(d_ptr->solutions_view->header(), SIGNAL(sectionResized(int, int, int)), this, SIGNAL(configChanged()));
	connect(d_ptr->solutions_view->header(), SIGNAL(sortIndicatorChanged(int, Qt::SortOrder)), this,
	        SIGNAL(configChanged()));

	// configuration settings
	auto configs = new rviz::Property("Task View Settings", QVariant(), QString(), root);
	initial_task_expand =
	    new rviz::EnumProperty("Task Expansion", "All Expanded", "Configure how to initially expand new tasks", configs);
	initial_task_expand->addOption("Top-level Expanded", EXPAND_TOP);
	initial_task_expand->addOption("All Expanded", EXPAND_ALL);
	initial_task_expand->addOption("All Closed", EXPAND_NONE);

	old_task_handling =
	    new rviz::EnumProperty("Old task handling", "Keep",
	                           "Configure what to do with old tasks whose solutions cannot be queried anymore", configs);
	old_task_handling->addOption("Keep", OLD_TASK_KEEP);
	old_task_handling->addOption("Replace", OLD_TASK_REPLACE);
	old_task_handling->addOption("Remove", OLD_TASK_REMOVE);
	connect(old_task_handling, &rviz::Property::changed, this, &TaskView::onOldTaskHandlingChanged);

	show_time_column = new rviz::BoolProperty("Show Computation Times", true, "Show the 'time' column", configs);
	connect(show_time_column, &rviz::Property::changed, this, &TaskView::onShowTimeChanged);

	d_ptr->configureExistingModels();

	// WHI version
	std::cout << "\nWHI MoveIt Task Constructor GUI demo VERSION 00.08" << std::endl;
	std::cout << "Copyright © 2022-2024 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;
	// WHI logo
	boost::filesystem::path path(ros::package::getPath("moveit_task_constructor_visualization"));
	QImage logo;
	if (logo.load(QString(path.string().c_str()) + "/icons/classes/whi_logo.png"))
	{
		QImage scaled = logo.scaledToHeight(48);
		d_ptr->label_logo->setPixmap(QPixmap::fromImage(scaled));
	}
	// widget properties
	d_ptr->doubleSpinBox_span->setRange(0.0, 3600.0); // 1 hour
	d_ptr->doubleSpinBox_span->setValue(0.2);
	d_ptr->doubleSpinBox_span->setSingleStep(0.1);
	// signals
	connect(d_ptr->pushButton_load, &QPushButton::clicked, this, [=]()
	{
		QString fileName = QFileDialog::getOpenFileName(this, tr("Open Task"), "/home/whi", tr("Task Files (*.yaml)"));
		loadTasks(fileName.toStdString());
	});
	connect(d_ptr->pushButton_save, &QPushButton::clicked, this, [=]()
	{
		QString fileName = QFileDialog::getSaveFileName(this, tr("Save Task"), "/home/whi/untitled.yaml", tr("Task Files (*.yaml)"));
		if (!fileName.contains(".yaml"))
		{
			fileName += ".yaml";
		}
		saveTasks(fileName.toStdString());
	});
	connect(d_ptr->pushButton_execute, &QPushButton::clicked, this, [=]()
	{
		d_ptr->pushButton_execute->setEnabled(false);
		execute();
	});
}

TaskView::~TaskView() {
	delete d_ptr;
}

void TaskView::save(rviz::Config config) {
	auto write_splitter_sizes = [&config](QSplitter* splitter, const QString& key) {
		rviz::Config group = config.mapMakeChild(key);
		for (int s : splitter->sizes()) {
			rviz::Config item = group.listAppendNew();
			item.setValue(s);
		}
	};
	write_splitter_sizes(d_ptr->tasks_property_splitter, "property_splitter");
	write_splitter_sizes(d_ptr->tasks_solutions_splitter, "solutions_splitter");

	auto write_column_sizes = [&config](QHeaderView* view, const QString& key) {
		rviz::Config group = config.mapMakeChild(key);
		for (int c = 0, end = view->count(); c != end; ++c) {
			rviz::Config item = group.listAppendNew();
			item.setValue(view->sectionSize(c));
		}
	};
	write_column_sizes(d_ptr->tasks_view->header(), "tasks_view_columns");
	write_column_sizes(d_ptr->solutions_view->header(), "solutions_view_columns");

	const QHeaderView* view = d_ptr->solutions_view->header();
	rviz::Config group = config.mapMakeChild("solution_sorting");
	group.mapSetValue("column", view->sortIndicatorSection());
	group.mapSetValue("order", view->sortIndicatorOrder());
}

void TaskView::load(const rviz::Config& config) {
	if (!config.isValid())
		return;

	auto read_sizes = [&config](const QString& key) {
		rviz::Config group = config.mapGetChild(key);
		QList<int> sizes, empty;
		for (int i = 0; i < group.listLength(); ++i) {
			rviz::Config item = group.listChildAt(i);
			if (item.getType() != rviz::Config::Value)
				return empty;
			QVariant value = item.getValue();
			bool ok = false;
			int int_value = value.toInt(&ok);
			if (!ok)
				return empty;
			sizes << int_value;
		}
		return sizes;
	};
	d_ptr->tasks_property_splitter->setSizes(read_sizes("property_splitter"));
	d_ptr->tasks_solutions_splitter->setSizes(read_sizes("solutions_splitter"));

	int column = 0;
	for (int w : read_sizes("tasks_view_columns"))
		d_ptr->tasks_view->setColumnWidth(++column, w);
	column = 0;
	for (int w : read_sizes("solutions_view_columns"))
		d_ptr->tasks_view->setColumnWidth(++column, w);

	QTreeView* view = d_ptr->solutions_view;
	rviz::Config group = config.mapGetChild("solution_sorting");
	int order = 0;
	if (group.mapGetInt("column", &column) && group.mapGetInt("order", &order))
		view->sortByColumn(column, static_cast<Qt::SortOrder>(order));
}

void TaskView::addTask()
{
	QModelIndex current = d_ptr->tasks_view->currentIndex();
	if (!current.isValid())
		return;
	bool is_top_level = !current.parent().isValid();

	TaskListModel* task_list_model = d_ptr->getTaskListModel(current).first;
	TaskDisplay* display = d_ptr->getTaskListModel(current).second;

	// attribute 'WA_DeleteOnClose' restore the resouce to which the dialog points,
	// but leaves its pointer valid, therefore use QPoint to check if its reference is deleted
	static QPointer<DlgAddTaskStage> smart;
	if (smart.isNull())
	{
		if (is_top_level)
		{
			tasks_.push_back(std::make_shared<moveit::task_constructor::Task>("whi_gui"));
			smart = new DlgAddTaskStage(tasks_.back(), display, DlgAddTaskStage::TYPE_TASK, this);
		}
		else
		{
			smart = new DlgAddTaskStage(tasks_[current.row()], display, DlgAddTaskStage::TYPE_STAGE, this);
		}
	}
    smart.data()->setAttribute(Qt::WA_DeleteOnClose);
    smart.data()->show();
    smart.data()->raise();
    smart.data()->activateWindow();

	connect(smart.data(), &QDialog::finished, [=](int Result)
	{
		if (Result && is_top_level && !tasks_.empty())
		{
			tasks_.erase(tasks_.begin() + current.row());
		}
	});

	return;

	task_list_model->insertModel(task_list_model->createLocalTaskModel(), is_top_level ? -1 : current.row());

	// select and edit newly inserted model
	if (is_top_level)
		current = current.model()->index(task_list_model->rowCount() - 1, 0, current);
	d_ptr->tasks_view->scrollTo(current);
	d_ptr->tasks_view->setCurrentIndex(current);
	d_ptr->tasks_view->edit(current);
}

void TaskView::removeSelectedStages() {
	auto* m = d_ptr->tasks_view->model();
	for (const auto& range : d_ptr->tasks_view->selectionModel()->selection())
		m->removeRows(range.top(), range.bottom() - range.top() + 1, range.parent());
}

void TaskView::removeStages()
{
	QModelIndex current = d_ptr->tasks_view->currentIndex();
	bool isTaskLevel = (current.parent().isValid() & !current.parent().parent().isValid());
	if (isTaskLevel)
	{
		if (!tasks_.empty())
		{
			tasks_.erase(tasks_.begin() + current.row());
		}

		removeSelectedStages();
	}
	else
	{
		int taskIndex = current.parent().row();
		tasks_[taskIndex]->stages()->remove(current.row());
		tasks_[taskIndex]->reset();
		if (tasks_[taskIndex]->plan())
		{
			d_ptr->tasks_view->scrollTo(current.parent());
			d_ptr->tasks_view->setCurrentIndex(current.parent());

			removeSelectedStages();

			tasks_[taskIndex]->introspection().publishSolution(*tasks_[taskIndex]->solutions().back());
		}
	}
}

void TaskView::onCurrentStageChanged(const QModelIndex& current, const QModelIndex& /*previous*/) {
	// adding task is allowed on top-level items and sub-top-level items
	d_ptr->actionAddLocalTask->setEnabled(current.isValid() &&
	                                      (!current.parent().isValid() || !current.parent().parent().isValid()));
	// removing stuff is allowed any valid selection except top-level items
	// and stages other than the 1st one
	d_ptr->actionRemoveTaskTreeRows->setEnabled(current.isValid() && ((current.parent().isValid() && !current.parent().parent().isValid())
		|| (current.parent().isValid() && current.row() > 0)));

	BaseTaskModel* task;
	QModelIndex task_index;
	std::tie(task, task_index) = d_ptr->getTaskModel(current);

	d_ptr->lock(nullptr);  // unlocks any locked_display_

	// update the SolutionModel
	QTreeView* view = d_ptr->solutions_view;
	int sort_column = view->header()->sortIndicatorSection();
	Qt::SortOrder sort_order = view->header()->sortIndicatorOrder();

	QItemSelectionModel* sm = view->selectionModel();
	QAbstractItemModel* m = task ? task->getSolutionModel(task_index) : nullptr;
	if (view->model() != m) {
		view->setModel(m);
		view->sortByColumn(sort_column, sort_order);
		delete sm;  // we don't store the selection model

		sm = view->selectionModel();
		connect(sm, SIGNAL(currentChanged(QModelIndex, QModelIndex)), this,
		        SLOT(onCurrentSolutionChanged(QModelIndex, QModelIndex)));
		connect(sm, SIGNAL(selectionChanged(QItemSelection, QItemSelection)), this,
		        SLOT(onSolutionSelectionChanged(QItemSelection, QItemSelection)));
	}

	// update the PropertyModel
	view = d_ptr->property_view;
	sm = view->selectionModel();
	m = task ? task->getPropertyModel(task_index) : nullptr;
	if (view->model() != m) {
		view->setModel(m);
		delete sm;  // we don't store the selection model
	}

	// update introspection and solution, the active model
	d_ptr->pushButton_execute->setEnabled(false);
	bool isTaskLevel = (current.parent().isValid() & !current.parent().parent().isValid());
	if (isTaskLevel)
	{
		int index = current.row() >= tasks_.size() ? tasks_.size() - 1 : current.row();
		if (!tasks_.empty() && !tasks_[index]->solutions().empty())
		{
			tasks_[index]->introspection().publishSolution(*tasks_[index]->solutions().back());
			d_ptr->pushButton_execute->setEnabled(true);
		}

		TaskListModel* taskListModel = d_ptr->getTaskListModel(current).first;
		taskListModel->setActiveTaskModel(d_ptr->getTaskModel(current).first);
	}	
	// behaviour of pushbutton save
	d_ptr->pushButton_save->setEnabled(!tasks_.empty());
}

void TaskView::onCurrentSolutionChanged(const QModelIndex& current, const QModelIndex& /*previous*/) {
	TaskDisplay* display = d_ptr->getTaskListModel(d_ptr->tasks_view->currentIndex()).second;
	d_ptr->lock(display);

	if (!display || !current.isValid())
		return;

	BaseTaskModel* task = d_ptr->getTaskModel(d_ptr->tasks_view->currentIndex()).first;
	Q_ASSERT(task);

	TaskSolutionVisualization* vis = display->visualization();
	DisplaySolutionPtr solution;
	try {
		solution = task->getSolution(current);
		display->setSolutionStatus(bool(solution));
	} catch (const std::invalid_argument& e) {
		ROS_ERROR_STREAM(e.what());
		display->setSolutionStatus(false, e.what());
	}
	vis->interruptCurrentDisplay();
	vis->showTrajectory(solution, true);
}

void TaskView::onSolutionSelectionChanged(const QItemSelection& /*selected*/, const QItemSelection& /*deselected*/) {
	QItemSelectionModel* sm = d_ptr->solutions_view->selectionModel();
	const QModelIndexList& selected_rows = sm->selectedRows();

	TaskDisplay* display = d_ptr->getTaskListModel(d_ptr->tasks_view->currentIndex()).second;
	Q_ASSERT(display);
	BaseTaskModel* task = d_ptr->getTaskModel(d_ptr->tasks_view->currentIndex()).first;
	Q_ASSERT(task);

	display->clearMarkers();
	for (const auto& index : selected_rows) {
		DisplaySolutionPtr solution;
		try {
			solution = task->getSolution(index);
			display->setSolutionStatus(bool(solution));
		} catch (const std::invalid_argument& e) {
			ROS_ERROR_STREAM(e.what());
			display->setSolutionStatus(false, e.what());
		}
		display->addMarkers(solution);
	}
}

void TaskView::onExecCurrentSolution() const {
	const QModelIndex& current = d_ptr->solutions_view->currentIndex();
	if (!current.isValid())
		return;

	BaseTaskModel* task = d_ptr->getTaskModel(d_ptr->tasks_view->currentIndex()).first;
	Q_ASSERT(task);

	const DisplaySolutionPtr& solution = task->getSolution(current);

	if (!d_ptr->exec_action_client_.waitForServer(ros::Duration(0.1))) {
		ROS_ERROR("Failed to connect to task execution action");
		return;
	}

	moveit_task_constructor_msgs::ExecuteTaskSolutionGoal goal;
	solution->fillMessage(goal.solution);
	d_ptr->exec_action_client_.sendGoal(goal);
}

void TaskView::onShowTimeChanged() {
	auto* header = d_ptr->tasks_view->header();
	bool show = show_time_column->getBool();
	if (header->count() > 3)
		d_ptr->tasks_view->header()->setSectionHidden(3, !show);
	d_ptr->actionShowTimeColumn->setChecked(show);
}

void TaskView::onOldTaskHandlingChanged() {
	Q_EMIT oldTaskHandlingChanged(old_task_handling->getOptionInt());
}

bool TaskView::loadTasks(std::string File)
{
	try
	{
		YAML::Node tasks = YAML::LoadFile(File);

		for (const auto& task : tasks)
		{
			std::string taskName = task["task"].as<std::string>();
			auto found = std::find_if(tasks_.begin(), tasks_.end(), [taskName](std::shared_ptr<moveit::task_constructor::Task> Task)
			{
				return taskName == Task->name();
			});
			if (found == tasks_.end())
			{
				auto newTask = std::make_shared<moveit::task_constructor::Task>("whi_gui");

				// create Cartesian interpolation "planner" to be used in various stages
				auto cartesian_interpolation = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
				// create a joint-space interpolation "planner" to be used in various stages
				auto joint_interpolation = std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();

				// name and current state
				newTask->setName(taskName);
				newTask->add(std::make_unique<moveit::task_constructor::stages::CurrentState>("current"));

				// stages in file
				const auto& stages = task["stage"];
				for (const auto& stage : stages)
				{
					std::string stageName = stage["name"].as<std::string>();
					std::string planningGroup = stage["group"].as<std::string>();
					std::string type = stage["type"].as<std::string>();
					std::string space = stage["space"].as<std::string>();
					std::shared_ptr<std::string> targetGroup(nullptr);
					std::shared_ptr<std::vector<double>> targetPose(nullptr);
					if (stage["target"].size() == 0)
					{
						targetGroup = std::make_shared<std::string>(stage["target"].as<std::string>());	
					}
					else
					{
						targetPose = std::make_shared<std::vector<double>>();
						for (const auto& it : stage["target"])
						{
							targetPose->push_back(it.as<double>());
						}
					}

					double duration = stage["duration"].as<double>();
					if (type == "move_to")
					{
						if (targetGroup)
						{
							auto stage = space == "joint" ? 
								std::make_unique<moveit::task_constructor::stages::MoveTo>(stageName, joint_interpolation) :
								std::make_unique<moveit::task_constructor::stages::MoveTo>(stageName, cartesian_interpolation);
							stage->setGroup(planningGroup);
							stage->setGoal(*targetGroup);
							stage->properties().set("duration_from_previous", duration);
							newTask->add(std::move(stage));
						}
						else if (targetPose)
						{
							geometry_msgs::Pose pose;
							pose.position.x = targetPose->at(0);
							pose.position.y = targetPose->at(1);
							pose.position.z = targetPose->at(2);
							tf2::Quaternion orientation;
							orientation.setRPY(angles::from_degrees(targetPose->at(3)),
								angles::from_degrees(targetPose->at(4)),
								angles::from_degrees(targetPose->at(5)));
							pose.orientation = tf2::toMsg(orientation);

							geometry_msgs::PoseStamped poseTcp;
							poseTcp.pose = pose;

							auto stage = space == "joint" ? 
								std::make_unique<moveit::task_constructor::stages::MoveTo>(stageName, joint_interpolation) :
								std::make_unique<moveit::task_constructor::stages::MoveTo>(stageName, cartesian_interpolation);
							stage->setGroup(planningGroup);
							stage->setGoal(poseTcp);
							stage->properties().set("duration_from_previous", duration);
							newTask->add(std::move(stage));
						}
					}
					else if (type == "move_relative")
					{
						if (space == "cartesian")
						{
							auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>(stageName, cartesian_interpolation);
							stage->setGroup(planningGroup);
							if (fabs(targetPose->at(0) + targetPose->at(1) + targetPose->at(2)) > 1e-5)
							{
								geometry_msgs::Vector3Stamped direction;
								direction.header.frame_id = "world";
								direction.vector.x = targetPose->at(0);
								direction.vector.y = targetPose->at(1);
								direction.vector.z = targetPose->at(2);
								stage->setDirection(direction);
							}
							else
							{
								geometry_msgs::TwistStamped twist;
								twist.header.frame_id = "world";
								twist.twist.angular.x = angles::from_degrees(targetPose->at(3));
								twist.twist.angular.y = angles::from_degrees(targetPose->at(4));
								twist.twist.angular.z = angles::from_degrees(targetPose->at(5));
								stage->setDirection(twist);
							}
							stage->properties().set("duration_from_previous", duration);
							newTask->add(std::move(stage));
						}
						else if (space == "joint")
						{
							newTask->loadRobotModel();
							std::vector<std::string> jointsName = newTask->getRobotModel()->getVariableNames();

							std::map<std::string, double> offsets;
							for (std::size_t i = 0; i < std::min(jointsName.size(), targetPose->size()); ++i)
							{
								offsets.emplace(jointsName[i], angles::from_degrees(targetPose->at(i)));
							}

							auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>(stageName, cartesian_interpolation);
							stage->setGroup(planningGroup);
							stage->setDirection(offsets);
							stage->properties().set("duration_from_previous", duration);
							newTask->add(std::move(stage));
						}
					}
				}

				// add to task list
				tasks_.push_back(newTask);

				if (tasks_.back()->plan())
				{
					tasks_.back()->introspection().publishSolution(*tasks_.back()->solutions().back());
				}
			}
		}
	}
	catch(const std::exception& e)
	{
		std::cout << "failed to load task file " << File << std::endl;
		return false;
	}
}

void TaskView::saveTasks(std::string File)
{
	std::string serialized;

	if (d_ptr->tasks_view->selectionModel()->currentIndex().parent().isValid())
	{
		int index = d_ptr->tasks_view->selectionModel()->currentIndex().parent().parent().isValid() ? 
			d_ptr->tasks_view->selectionModel()->currentIndex().parent().row() : d_ptr->tasks_view->selectionModel()->currentIndex().row();
		
		serialized = serializeTask(tasks_[index]);
	}
	else
	{
		for (const auto& it : tasks_)
		{
			serialized += serializeTask(it);
		}
	}

	if (!serialized.empty())
	{
		std::ofstream ofs(File, std::ios::out | std::ios::trunc);
		if (ofs.good())
		{
			ofs.write(serialized.c_str(), serialized.length());

			ofs.close();
		}
	}
}

bool TaskView::getGoal(const moveit::task_constructor::Stage* Stage, std::string& Goal)
{
	try
	{
		// try named joint pose
		Goal = boost::any_cast<std::string>(Stage->properties().get("goal"));
		return true;
	}
	catch (const boost::bad_any_cast&)
	{
		return false;
	}
}

bool TaskView::getGoal(const moveit::task_constructor::Stage* Stage, geometry_msgs::PoseStamped& Goal)
{
	try
	{
		// try named joint pose
		Goal = boost::any_cast<geometry_msgs::PoseStamped>(Stage->properties().get("goal"));
		return true;
	}
	catch (const boost::bad_any_cast&)
	{
		return false;
	}
}

bool TaskView::getOffset(const moveit::task_constructor::Stage* Stage, geometry_msgs::Vector3Stamped& Offset)
{
	try
	{
		// try named joint pose
		Offset = boost::any_cast<geometry_msgs::Vector3Stamped>(Stage->properties().get("direction"));
		return true;
	}
	catch (const boost::bad_any_cast&)
	{
		return false;
	}
}

bool TaskView::getOffset(const moveit::task_constructor::Stage* Stage, geometry_msgs::TwistStamped& Offset)
{
	try
	{
		// try named joint pose
		Offset = boost::any_cast<geometry_msgs::TwistStamped>(Stage->properties().get("direction"));
		return true;
	}
	catch (const boost::bad_any_cast&)
	{
		return false;
	}
}

bool TaskView::getOffset(const moveit::task_constructor::Stage* Stage, std::map<std::string, double>& Offset)
{
	try
	{
		// try named joint pose
		Offset = boost::any_cast<std::map<std::string, double>>(Stage->properties().get("direction"));
		return true;
	}
	catch (const boost::bad_any_cast&)
	{
		return false;
	}
}

std::string TaskView::serializeTask(std::shared_ptr<moveit::task_constructor::Task> Task)
{
	std::string serialized;

	std::string line("- task: " + Task->name() + "\n");
	serialized += line;
	line.assign("  stage:\n");
	serialized += line;
	for (size_t i = 1; i < Task->stages()->numChildren(); ++i)
	{
		// name
		const moveit::task_constructor::Stage* stage = (*Task->stages())[i];
		line.assign("    - name: " + stage->name() + "\n");
		serialized += line;
		// group
		line.assign("      group: " + boost::any_cast<std::string>(stage->properties().get("group")) + "\n");
		serialized += line;
		// type, space, target, duration
		std::string space;
		std::string target;
		std::string duration;
		line.assign("      type: ");
		if (const moveit::task_constructor::stages::MoveTo* dStage =
			dynamic_cast<const moveit::task_constructor::stages::MoveTo*>(stage); dStage != nullptr)
		{
			// type
			line += "move_to\n";

			// space
			if (const moveit::task_constructor::solvers::JointInterpolationPlanner* dPlanner = 
				dynamic_cast<const moveit::task_constructor::solvers::JointInterpolationPlanner*>(dStage->planner().get()); dPlanner != nullptr)
			{
				space = "joint\n";
			}
			else if (const moveit::task_constructor::solvers::CartesianPath* dPlanner = 
				dynamic_cast<const moveit::task_constructor::solvers::CartesianPath*>(dStage->planner().get()); dPlanner != nullptr)
			{
				space = "cartesian\n";
			}
			else
			{
				space = "joint\n";
			}

			// target
			std::string goalGroup;
			geometry_msgs::PoseStamped goalPose;
			if (getGoal(dStage, goalGroup))
			{
				target = goalGroup + "\n";
			}
			else if (getGoal(dStage, goalPose))
			{
				tf::Quaternion quat(goalPose.pose.orientation.x, goalPose.pose.orientation.y,
					goalPose.pose.orientation.z, goalPose.pose.orientation.w);
				double roll = 0.0, pitch = 0.0, yaw = 0.0;
				tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

				target.assign("[" + std::to_string(goalPose.pose.position.x) + ", " +
					std::to_string(goalPose.pose.position.y) + ", " +
					std::to_string(goalPose.pose.position.z) + ", " +
					std::to_string(angles::to_degrees(roll)) + ", " +
					std::to_string(angles::to_degrees(pitch)) + ", " +
					std::to_string(angles::to_degrees(yaw)) + "]\n");
			}

			// duration from previous
			double preDuration = dStage->properties().get<double>("duration_from_previous");
			duration.assign(std::to_string(preDuration) + "\n");
		}
		else if (const moveit::task_constructor::stages::MoveRelative* dStage =
			dynamic_cast<const moveit::task_constructor::stages::MoveRelative*>(stage); dStage != nullptr)
		{
			// type
			line += "move_relative\n";

			// space
			space = "cartesian\n";

			// target
			geometry_msgs::Vector3Stamped offsetPose;
			geometry_msgs::TwistStamped offsetTwist;
			std::map<std::string, double> offsetJoints;
			if (getOffset(dStage, offsetPose))
			{
				target.assign("[" + std::to_string(offsetPose.vector.x) + ", " +
					std::to_string(offsetPose.vector.y) + ", " +
					std::to_string(offsetPose.vector.z) + ", 0.0, 0.0, 0.0]\n");
			}
			else if (getOffset(dStage, offsetTwist))
			{
				target.assign("[0.0, 0.0, 0.0, " + std::to_string(offsetTwist.twist.angular.x) + ", " +
					std::to_string(offsetTwist.twist.angular.y) + ", " +
					std::to_string(offsetTwist.twist.angular.z) + "]\n");
			}
			else if (getOffset(dStage, offsetJoints))
			{
				target.assign("[");
				for (const auto it : offsetJoints)
				{
					target += std::to_string(it.second) + ", ";
				}
				target.pop_back();
				target.pop_back();
				target += "]\n";
			}

			// duration from previous
			double preDuration = dStage->properties().get<double>("duration_from_previous");
			duration.assign(std::to_string(preDuration) + "\n");
		}
		else
		{
			line += "move_to\n";
		}
		serialized += line;
		// space
		line.assign("      space: " + space);
		serialized += line;
		// target
		line.assign("      target: " + target);
		serialized += line;
		// duration
		if (!duration.empty())
		{
			line.assign("      duration: " + duration);
		}
		serialized += line;
	}

	return serialized;
}

void TaskView::execute()
{
	std::thread{ std::bind(&TaskView::threadExecute, this) }.detach();
}

void TaskView::threadExecute()
{
	QModelIndex current = d_ptr->tasks_view->currentIndex();
	if (!tasks_[current.row()]->solutions().empty())
	{
		do
		{
			tasks_[current.row()]->execute(*tasks_[current.row()]->solutions().back());
#ifdef VIEW_TRAJECTORY
			moveit_task_constructor_msgs::ExecuteTaskSolutionGoal traj;
			tasks_[current.row()]->solutions().back()->fillMessage(traj.solution);
			std::cout << "count of sub trajectory " << traj.solution.sub_trajectory.size() << std::endl;
			for (int i = 0; i < traj.solution.sub_trajectory.size(); ++i)
			{
				std::cout << "traj " << i << " with points " << traj.solution.sub_trajectory[i].trajectory.joint_trajectory.points.size() << std::endl;
			}
#endif

			std::this_thread::sleep_for(std::chrono::milliseconds(int(d_ptr->doubleSpinBox_span->value() * 1000.0)));
		}
		while (d_ptr->checkBox_loop->isChecked());
	}

	d_ptr->pushButton_execute->setEnabled(true);
}

GlobalSettingsWidgetPrivate::GlobalSettingsWidgetPrivate(GlobalSettingsWidget* widget, rviz::Property* root)
  : q_ptr(widget) {
	setupUi(widget);
	properties = new rviz::PropertyTreeModel(root, widget);
	view->setModel(properties);
}

GlobalSettingsWidget::GlobalSettingsWidget(moveit_rviz_plugin::TaskPanel* parent, rviz::Property* root)
  : SubPanel(parent), d_ptr(new GlobalSettingsWidgetPrivate(this, root)) {
	Q_D(GlobalSettingsWidget);

	d->view->expandAll();
	connect(d->properties, &rviz::PropertyTreeModel::configChanged, this, &GlobalSettingsWidget::configChanged);
}

GlobalSettingsWidget::~GlobalSettingsWidget() {
	delete d_ptr;
}

void GlobalSettingsWidget::save(rviz::Config config) {
	d_ptr->properties->getRoot()->save(config);
}

void GlobalSettingsWidget::load(const rviz::Config& config) {
	d_ptr->properties->getRoot()->load(config);
}
}  // namespace moveit_rviz_plugin

#include "moc_task_panel.cpp"
