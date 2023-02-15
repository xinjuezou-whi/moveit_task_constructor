# Simple GUI for MoveIt Task Constructor
A simple GUI for MoveIt Task Constructor to ease the task construction. Though MoveIt Task Constructor provides a flexible and transparent way to define and plan actions that consist of multiple interdependent subtasks, it requires the user to code the logic of tasks which could be a technical threshold for some end users. This thread attempts to introduce a graphic user interface to define and plan the actions of the arm free of coding.

Currently, it only supports the serial container that contains the MoveTo and MoveRelative stages. Meanwhile, it introduces the interactive marker for users to define the TCP position and orientation by interacting with a 6DOF marker.
![intro](https://user-images.githubusercontent.com/72239958/194801084-118962c7-c118-460e-b1a0-23cbdb880357.gif)


## Refactor and Implementation
| File                                       | Package                                 | Class    | Comment                                                           |
|--------------------------------------------|-----------------------------------------|----------|-------------------------------------------------------------------|
| task_view.ui                               | moveit_task_constructor_visualization   | refactor | added Execute push button; renamed action menu; load/save buttons |
| task_panel_p.h task_panel.h task_panel.cpp | moveit_task_constructor_visualization   | refactor | refactored the addTask and removeStages                           |
| task_display.h task_display.cpp            | moveit_task_constructor_visualization   | refactor | added interactive marker                                          |
| container.h container.cpp                  | moveit_task_constructor_core            | refactor | added access element function by index                            |
| cartesian_path.h joint_interpolation.h     | moveit_task_constructor_core            | refactor | added planner access                                              |
| introspection.cpp                          | moveit_task_constructor_core            | refactor | refactored the namespace from private to public                   |
| move_to.cpp move_relative.cpp              | moveit_task_constructor_core            | refactor | modify the duration from previous by property                     |
| move_to.h move_relative.h                  | moveit_task_constructor_core            | refactor | add get planner interface                                         |
| CMakeLists.txt                             | moveit_task_constructor_visualization   | refactor | new file's manifest                                               |
| add_task_stage.h add_task_stage.cpp        | moveit_task_constructor_visualization   | new      | properties and behaviors of widgets of GUI logics                 |
| whi_logo.png                               | moveit_task_constructor_visualization   | new      | logo image                                                        |

## Setup
Go into your catkin workspace and initialize wstool if necessary (assuming ~/catkin_workspace as workspace path):
```
cd ~/catkin_workspace/src
git clone https://github.com/xinjuezou-whi/moveit_task_constructor.git
```

Install missing packages with rosdep:
```
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO
```

Build the workspace:
```
catkin build
```

## Usage
### By the demo of MoveIt Task Constructor
The MoveIt Task Constructor package contains several basic examples. Here we just launch the basic environment:
```
roslaunch moveit_task_constructor_demo demo.launch
```

Right-click the root node("Motion Planning Tasks") of "Task Tree", and then select the "Add task of stage" on popup menu. Once the wanted stage of the task is constructed, click "Add" button. Then specify the topic of "Task Solution Topic" to:
```
/whi_gui/solution
```
![added](https://user-images.githubusercontent.com/72239958/194798304-6e08bf91-43ad-4808-9c28-447c47898a7b.gif)


### By the demo of MoveIt
If you are running other launch of MoveIt like "demo.launch", first introduce the capability of "ExecuteTaskSolutionCapability" otherwise the execute will not be respond. Add following content following the include of "move_group.launch":
```
<param name="move_group/capabilities" value="move_group/ExecuteTaskSolutionCapability" />
```
![image](https://user-images.githubusercontent.com/72239958/193467049-597e3795-b11d-4762-895c-787b3dec2b71.png)

or set the value "move_group/ExecuteTaskSolutionCapability" to param "capabilities" in file "move_group.launch":
![image](https://user-images.githubusercontent.com/72239958/218240301-1441bb69-38f8-47fe-884f-5c60587a7eb7.png)

Please note that MoveIt Task Construct requires the tip of end effector at motion planning for frame transform, otherwise the planning would be failed. This can be infered from the function "getRobotTipForFrame" of file "utils.cpp":
![image](https://user-images.githubusercontent.com/72239958/193467355-90e21e41-bef8-4ffe-a0c7-7199e1d4990a.png)


Therefore, please make sure your arm is [configured with end effector](https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html).

Last, add "Motion Planning Tasks" to RViz and organize the layout with your preference:
![add](https://user-images.githubusercontent.com/72239958/193468115-1e5deedb-89cd-4b15-bfc0-3ec117eb6b25.gif)


Once the above pre-requisites are met, you can leverage MoveIt Task Constructor to define the motion tasks from both coding or this simple GUI. Here are two examples of 6DOF arm, the first is a demo arm of WHI, and the second is AR2/3 step motor arm:

#### WHI's demo arm
![whi](https://user-images.githubusercontent.com/72239958/193468240-b7973374-b099-49ec-abc4-993e9b20559d.gif)


#### AR2/3 arm
![ar3](https://user-images.githubusercontent.com/72239958/193468707-fbd6298b-2032-4d4c-909f-381b7bb339e9.gif)


### Looping execution
Once the Loop is checked, the constructed trajectory will be executed ciclely:
![tasks_execute](https://user-images.githubusercontent.com/72239958/199387810-c902eaed-46ba-4a1a-b993-4243a5ae0406.gif)


### Save and Load
Constructed task and its stages can be saved as the yaml file, and reused by loading from the saved yaml files:
![saved](https://user-images.githubusercontent.com/72239958/194716016-2e3a1246-6bb1-47dc-b875-3ef4891dea9b.gif)

<br>

> Please firstly specify the topic of "Task Solution Topic" to "/whi_gui/solution" before loading yaml that contains multiple tasks, otherwise only the last task will be displayed in task tree

![loaded](https://user-images.githubusercontent.com/72239958/194798919-d8ffc03b-a05e-49f3-b530-39f9aa82d12a.gif)

<br>

> ***Bellowing contents are the original of MoveIt Task Constructor Framework:***
# MoveIt Task Constructor Framework

The Task Constructor framework provides a flexible and transparent way to define and plan actions that consist of multiple interdependent subtasks.
It draws on the planning capabilities of [MoveIt](https://moveit.ros.org/) to solve individual subproblems in black-box *planning stages*.
A common interface, based on MoveIt's PlanningScene is used to pass solution hypotheses between stages.
The framework enables the hierarchical organization of basic stages using *containers*, allowing for sequential as well as parallel compositions.

## Videos

- Demo video associated with [ICRA 2019 paper](https://pub.uni-bielefeld.de/download/2918864/2933599/paper.pdf)

  [![](https://img.youtube.com/vi/fCORKVYsdDI/0.jpg)](https://www.youtube.com/watch?v=fCORKVYsdDI)

- [Presentation @ ROSCon 2018 (Madrid)](https://vimeo.com/293432325)
- [Presentation @ MoveIt workshop 2019 (Macau)](https://www.youtube.com/watch?v=a8r7O2bs1Mc)

## Tutorial

We provide a tutorial for a pick-and-place pipeline without bells & whistles [as part of the MoveIt tutorials](https://ros-planning.github.io/moveit_tutorials/doc/moveit_task_constructor/moveit_task_constructor_tutorial.html).

## Roadmap

**Feedback, reports and contributions are very welcome.**

The current roadmap is to replace MoveIt's old pick&place pipeline and provide a *transparent mechanism* to enable and debug complex motion sequences.

Further planned features include

- Entwined planning and execution for early execution, monitoring and code hooks
- Subsolution blending
- Parallel planning
- Iterative solution improvement

Ideas and requests for other interesting/useful features are welcome.

## Citation

If you use this framework in your project, please cite the associated paper:

Michael Görner*, Robert Haschke*, Helge Ritter, and Jianwei Zhang,
"MoveIt! Task Constructor for Task-Level Motion Planning",
_International Conference on Robotics and Automation (ICRA)_, 2019, Montreal, Canada.
[[DOI]](https://doi.org/10.1109/ICRA.2019.8793898) [[PDF]](https://pub.uni-bielefeld.de/download/2918864/2933599/paper.pdf).


```plain
@inproceedings{goerner2019mtc,
  title={{MoveIt! Task Constructor for Task-Level Motion Planning}},
  author={Görner, Michael* and Haschke, Robert* and Ritter, Helge and Zhang, Jianwei},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  year={2019}
}
```
