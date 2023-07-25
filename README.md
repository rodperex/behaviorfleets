# behaviorfleets
This repo proposes a way of distributing some parts of [behavior trees (BTs)](https://arxiv.org/pdf/1709.00084v6.pdf) among a fleet of (potentially) heterogeneous robots. This without having to redo BT action nodes that were already functional. It makes use of the [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) package.

It can be of special interest to be used *underneath* higher level strategies to assing tasks among a team of robots minimizing the cost of executing an overall plan. 

The main idea behind this package is having a single *source BT where part of it wants to be delegated to another robot. This *source BT* can be either run by a coordinator robot or by an independent process. In this version, those parts of the tree to be delegated must be defined in an independent *.xml* file, that will be referenced from the *source .xml*.

To achive this, 2 ROS 2 nodes are proposed:

* **DelegateActionNode** &rarr; this node, that inherits from `BT::ActionNodeBase`, is the one that needs to be defined within the *source BT* to explicit that the *.xml* it references will be delegated to some other robot.
* **RemoteDelegateActionNode** &rarr; this node will be running in all other robots that want to participate from the delegation (ie. they might be delegated certain task).

Furthermore, the **behaviorfleets** package also implements a **shared blackboard** for all robots participating in the execution of the BT. To achieve this shared resource, these ROS 2 nodes are proposed:
* **BlackboardManager** &rarr; node that orchestrates the shared resource. Althoug other configurations are allowed, it can be run by the agent executing the *source BT*, and will make sure all updates in its blackboard are transmitted to the remote agents and vice versa.
* **BlackBoardHandler** &rarr; node that will be run by the remote robot to interface with the manager.

Some new types of ROS 2 messages has been defined. These are defined in package **bf_messages**.

Delegation phylosophy consists in publishing those parts of the *source BT* to be delegated through a ROS 2 topic. Potential candidates listen to these missions being offered, and apply in case they are capable of carrying them out. The remote BT node will publish SUCCESS, FAILURE or RUNNING status depending on the execution of the *delegated BT*.

## initial setup

Import thirdparty repos (execute the command below within the root of your workspace):

```bash
vcs-import src/ < src/behaviorfleets/thirdparty.repos
```

Run the commands below to install all dependencies within the root of your workspace.

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src/behaviorfleets/thirdparty --ignore-src -r -y
```

Source your ROS 2 distro:

```bash
source /opt/ros/<ros2-distro>/setup.bash
```

Compile it:

```bash
colcon build --symlink-install
```

Source the workspace:
```bash
source install/setup.bash
```

## delegating a behavior tree

This is done as easy as including a **DelegateActionNode** in your *source BT*. Below a very simple example.

FIGURE

Configuration parameters of the new `BT::ActionNodeBase`:

* **mission_id** &rarr; mission identifier. Useful to discriminate the type of missions robots can do. For instance, a remote robot may decide not to propose to carry out the mission if the identifier corresponds to something it is not capable to do.
* **remote_tree** &rarr; file (*.xml*) containing the subtree to delegate.
* **remote_id** &rarr; in case the *source BT* requieres a specific robot to carry the mission out, its identifier will be specified by this parameter. If it is not set, any node could request executing this task.
* **exclude** &rarr; parameter used in case the *source BT* wants to exclude some robots (separated by ',') from executing the mission.
* **plugins** &rarr; list of pluigins (separated by ',') that the remote robots needs to have to execute the mission.
* **timeout** &rarr; time (in seconds) to consider before interpreting the remote robot is lost.
* **max_tries** &rarr; maximum number of tries attepmting to know the status of the remote robot before considering it lost.

To run a *source BT*, follow the same phylosophy as running a standard BT (see [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)). They new things you will need to do are:

* Registering the new plugin:

```cpp
factory.registerFromPlugin(loader.getOSName("delegate_action_node"));
```

* Loading the parameters for the **DelegateActionNode**:
```cpp
std::string params_file = "config.yaml"; // or any other

std::string pkgpath = ament_index_cpp::get_package_share_directory("behaviorfleets");

std::ifstream fin(pkgpath + "/params/" + params_file);
YAML::Node params = YAML::Load(fin);

std::string xml_file = pkgpath + params["source_tree"].as<std::string>();
```

* Add the route to the folder with the *.xml* to delegate to the blackboard:

```cpp
auto blackboard = BT::Blackboard::create();
blackboard->set("pkgpath", pkgpath + "/bt_xml/");
```
A full example can be found in *src/behaviorfleets/behaviorfleets/src/exec/source_main.cpp*.

## remote robots

Remote robots need to run a **RemoteDelegateActionNode**, that will handle the communication with the **DelegateActionNode** coordinating the *source BT*. 

These robots needs to be configured prior their execution, specifying their name, which are the plugins they have available as well as the missions they are allowed to carry out. This configuration can be carried out either by hard coding or by means of a *.yaml* file.

## examples

Some **very basic** examples of *.xml* files are left in folder *behaviorfleets/bt_xml*. For a full example, please visit [bf_patrol](https://github.com/rodperex/bf_patrol).

## shared blackboard stress tests

Stress test parameters are defined in a *.yaml* file. See *behaviorfleets/src/params/test_\*.yaml* to see different examples.

To execute the test, first run the blackboard manager:

```bash
ros2 run behaviorfleets bb_manager
```

And then launch the stresser (edit the launcher to set the configuration file):
```bash
ros2 launch behaviorfleets bb.stress.launch.py
```
Once the execution is over (**ctrl+c**), all performance parameters are dumped in several *.txt* files which will be located in a folder called *results* in the root of the workspace. To analyze them, the script *check_results.py* can be used.


TO DO:
    - Number of updates from the global bb correspond to the summatory of all successful updates from other nodes
    - Make the stressers work until all requests have been attended?
    - When there are many nodes working, only a couple of requests are attended during the operation time
    - Instead of providing operation time, we can provide number os successful updates as a condition to stop


Measuring waiting time in the server does not isolate the measurement from the client Hz since the bb is locked till the client releases it.