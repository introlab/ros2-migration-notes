# ros2-migration-notes
ROS2 migration notes

This assumes that you are migrating from ROS1 Noetic on Ubuntu 20.04 to ROS2 Humble (either on Ubuntu 22.04 or compiled from source on Ubuntu 20.04).
Using Foxy is not recommended: it is distributed in binary form on Ubuntu 20.04, but it is already past its end-of-support date.
Also, Foxy and Humble have a few incompatibilities (for instance, names of created interfaces [msgs, srvs, actions] CMake targets and packages).
Foxy was not as mature as Humble is, and some features were missing (for instance, declaring parameters without default values, but with a type).
Use Humble.

Also, this does not cover migration from using Gazebo Classic to the newer Ignition Gazebo.
Humble still supports Gazebo Classic, but Jazzy (Ubuntu 24.04) does not.
Migration will be needed when moving to Jazzy.

## Building Humble on Ubuntu 20.04
Install a few dependencies:

```bash
sudo apt install -y ros-dev-tools python3-rosinstall-generator
```

Run these commands to prepare the workspace with everything you need:

```bash
mkdir -p ros2_humble_ws/src
cd ros2_humble_ws

rosinstall_generator --deps --rosdistro humble desktop_full \
    launch_xml \
    launch_yaml \
    launch_testing \
    launch_testing_ament_cmake \
    demo_nodes_cpp \
    demo_nodes_py \
    example_interfaces \
    camera_calibration_parsers \
    camera_info_manager \
    cv_bridge \
    v4l2_camera \
    vision_opencv \
    vision_msgs \
    image_geometry \
    image_pipeline \
    image_transport \
    compressed_image_transport \
    compressed_depth_image_transport \
    rosbag2_storage_mcap \
    rtabmap \
    rtabmap_ros \
    diagnostics \
    turtlebot3_gazebo \
    turtlebot3_description \
    turtlebot3_navigation2 \
    gazebo_ros_pkgs \
    joint_state_publisher_gui \
    rqt_tf_tree \
> ros2.humble.opentera_webrtc_ros.rosinstall

sed -i '$d' ros2.humble.opentera_webrtc_ros.rosinstall

cat <<EOF >> ros2.humble.opentera_webrtc_ros.rosinstall
- git:
    local-name: cv_camera
    uri: https://github.com/Kapernikov/cv_camera.git
    version: master
- git:
    local-name: xtl
    uri: https://github.com/xtensor-stack/xtl.git
    version: 0.7.2
- git:
    local-name: xtensor
    uri: https://github.com/xtensor-stack/xtensor.git
    version: 0.23.10
- git:
    local-name: xsimd
    uri: https://github.com/xtensor-stack/xsimd.git
    version: 7.6.0
EOF
```
This combines a `rosinstall_generator` call to get all repos from the `desktop_full` bundle, and additionnal dependencies. It also adds the `cv_camera` package, which was ported to ROS2 in a fork that is not available via `rosinstall_generator`, and the `xtl`, `xtensor` and `xsimd` C++ libraries using the versions they have on Ubuntu 22.04. for maximum compatibility.

Then run these commands to clone all the repos and install their dependencies:
```bash
vcs import src < ros2.humble.opentera_webrtc_ros.rosinstall
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers xsimd xtensor test_pluginlib" --rosdistro humble
```
The `--skip-keys` option is there to skip some dependencies that are not available in the Ubuntu 20.04 repositories. Some are installed separatly (`xsimd` and `xtensor`, as shown above), and the others come from the [documentation for building Humble from source](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#install-dependencies-using-rosdep).

You will need to apply a few patches as shown [here](https://github.com/introlab/t-top/blob/ros2-migration/tools/setup_scripts/ros2_humble_install.sh#L241-L242). The patch files are [here (raw libg2o)](https://raw.githubusercontent.com/introlab/t-top/ros2-migration/tools/setup_scripts/patch/libg2o.patch) and [here (raw octomap_msgs)](https://raw.githubusercontent.com/introlab/t-top/ros2-migration/tools/setup_scripts/patch/octomap_msgs.patch).
The libg2o patch essentially renames the library, as well as adding some ament stuff required to correctly generate the setup files for the package. Without the patch, every time the workspace will be sourced, there will be a warning about a missing file.
The octomap_msgs patch fixes a wrong installation path for a header file which, prevents compilation of dependent packages such as rtabmap.


Create a file named `colcon_defaults.yaml` in the root of the workspace with the following content:
```yaml
build:
  cmake-clean-cache: true
  cmake-args:
    - -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    - --no-warn-unused-cli
    - -DCMAKE_BUILD_TYPE=Release
    - -DPYTHON_EXECUTABLE=/usr/bin/python3
    - -DCMAKE_POLICY_DEFAULT_CMP0135=NEW # DOWNLOAD_EXTRACT_TIMESTAMP
    - -DBUILD_TESTING=OFF
```

If you want to enable CUDA for librealsense, add `-DBUILD_WITH_CUDA=ON` to the `cmake-args` list.
If you want to enable math optimizations for you platform, you can also add `-DCMAKE_CXX_FLAGS='-march=native -ffast-math'` and `-DCMAKE_C_FLAGS='-march=native -ffast-math'`.
See [this T-Top installation script](https://github.com/introlab/t-top/blob/ros2-migration/tools/setup_scripts/jetson_configuration.sh#L433) for an example.
With all of these options, the file content would look like this:
```yaml
build:
  cmake-clean-cache: true
  cmake-args:
    - -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    - --no-warn-unused-cli
    - -DCMAKE_BUILD_TYPE=Release
    - -DPYTHON_EXECUTABLE=/usr/bin/python3
    - -DCMAKE_POLICY_DEFAULT_CMP0135=NEW # DOWNLOAD_EXTRACT_TIMESTAMP
    - -DBUILD_TESTING=OFF
    - -DBUILD_WITH_CUDA=ON
    - -DCMAKE_CXX_FLAGS='-march=native -ffast-math'
    - -DCMAKE_C_FLAGS='-march=native -ffast-math'
```

You can now build the workspace:
```bash
colcon build
```

This will take a while.
On the Jetson AGX Orin, it will probably take more than four hours.
On a personnal computer, it will also probably be between two and three hours.
On a laptop with an Intel i7-9750H and 32 GB of RAM, it took about two and a half hours.

You will be able to use this ROS2 installation by sourcing the `ros2_humble_ws/install/setup.bash` file.
Il you won't use any other ROS1 or ROS2 distribution, you can chose to add this line to your `~/.bashrc` (if using bash) or `~/.zshrc` (if using zsh):
```bash
source ~/ros2_humble_ws/install/setup.bash
```
It you might want to use multiple ROS distributions but want to reduce typying, you could create an alias by adding this to your `~/.bashrc` (if using bash) or `~/.zshrc` (if using zsh):
```bash
alias source_humble='. ~/ros2_humble_ws/install/setup.bash'
```
You can name the alias however you want.
You will then be able to just type `source_humble` in a terminal to source the file.

## Workspace migration

### Isolated build
If you were using `catkin_make`, start by first migrating to `catkin-tools`, which uses `catkin build` to build the workspace.
In ROS2, the packages are built in an isolated fashion. By first building the ROS1 workspace using `catkin build`, you will be able to find out and fix your dependency problems before adding the ROS2 stuff to the mix.

### Devel space
There is no devel space in ROS2: every file that is needed will need to be installed using a CMake install rule.
The `--symlink-install` flag can be used to create symlinks instead of copies when installing, giving similar advantages to the devel space (no need to rebuild between each modification of a file if it is not part of a compiled executable, like Python files and launch files and config files).

### Using `colcon`, the ROS2 build tool
In ROS2, `colcon` is used to build the workspace. Like `catkin_make`, but not as `catkin build`, it will need to be invoked from the source of the workspace.
If you invoke it from a nested directory, it will happily proceed to create `build`, `install` and `log` directories at this nested place and will report a success, but it will not have done what you wanted it to do.
Also, much like `catkin_make` and not as `catkin build`, it can't be preconfigured with a set of default arguments.
There is a way to create a file to define default arguments to pass to it every invocation, which is similar.
It is barely documented [here](https://colcon.readthedocs.io/en/released/user/configuration.html#defaults-yaml). Note that the example is JSON, but the actual format is YAML: since JSON is valid YAML, the example will work, but you could use YAML instead.
Also, it is not documented there, but you can create such a file per-workspace. It needs to be named `colcon_defaults.yaml` and be placed in the root of the workspace. (The source for this information is the [source code of colcon-defaults](https://github.com/colcon/colcon-defaults/blob/master/colcon_defaults/argument_parser/defaults.py#L34).)

#### `colcon` usage
You will mostly use `colcon build` to build the workspace. If you want to build a subset of the packages, pass `--packages-select <package1> [<package2...]` to the command. (This is useful during the migration: migrate one package at a time, and build only the migrated packages.)

There is a `colcon clean` verb that can be used in two ways:
1. `colcon clean workspace` will completely remove the `build`, `install` and `log` folders in the workspace, and all their content.
2. `colcon clean packages` will remove only the selected packages from these folders. You can use standard `colcon` package selection arguments, like `--packages-select`, for the selection.

#### `colcon` idiosyncrasies
The equivalent of `--verbose` in `colcon` is `--event-handlers console_cohesion+`.
By default, colcon keeps the standars output and standard error in a buffer fo a given package, and it displays it as a whole at the end of this package build. If you want output to be displayed as soon as possible, without this buffering, use `--event-handlers console_direct+`. You can have multiple `--event-handlers` options in the same colcon invocation.


## Migration strategy
This is a suggestion of a way to migrate your packages.
To migrate, for each packages until they are all migrated:
1. Think about the dependency graph of your packages, and choose one that has no other dependencies in your packages.
2. Rename it by adding an `_OLD` suffix to the folder
3. Run the `ros2 pkg create --build-type ament_cmake <package_name>` command
    Note: In ROS2, you can create pure Python packages using `--build-type ament_python`, but the migration is more work. Only the CMake approach will be described.
4. Use a diff tool to compare the `package.xml` of the old and new package. Migrate the dependencies. Use the [ROS Index](https://index.ros.org/) to check for package availability in ROS2. You might need to replace or remove some packages that have a ROS2 alternative, but no ROS2 version.
5. Use a diff tool to compare the `CMakeLists.txt` of the old and new package. Most complex CMake logic can be directly copy-pasted. You will need to change the way that the dependencies are included, and also the way they are exported. Also, if you were building interfaces (messages, services or actions) as part of another package, you will need to move them to a dedicated interface package. Make sure that everything is installed, as there is no devel space anymore. Any file without an install rule will not be available by ROS. Also, you will need to replace `CATKIN_*` CMake varibles with new forms. Most are simpler. For instance, `CATKIN_PACKAGE_SHARE_DIRECTORY` becomes `share/${PROJECT_NAME}`.
You can check the packages in our [`opentera-webrtc-ros`](https://github.com/introlab/opentera-webrtc-ros) or [`audio_utils`](https://github.com/introlab/audio_utils) repositories for examples or inspiration.
6. Move all the remaining files in the new package. You might want to move your header files if you used C++ and did not already respect the layout created by `ros2 pkg create`, especially if you are making a library: this will require changes to the `CMakeLists.txt`.
7. Delete the old package.
8. Build using `--packages-select` to include only the migrated packages.
9. Fix any errors until step 8. works and everything builds
10. Move on to the next package

Starting with the second package, you could have dependency errors, especially if you are exporting a library that depends on another library. Check [`audio_utils`](https://github.com/introlab/audio_utils) for an example of how to export such library if needed.

## Migrating

### Migrating interfaces
- `Header` is now `std_msgs/Header`
- Services no longer have a boolean return value to indicate failure. If you need one, add a `success` boolean in the response part of the interface.
-

### Python nodes boilerplate
Python nodes are relatively easy to migrate.
1. Replace `rospy` with `rclpy`. You will also need to import `rclpy.node`.
2. If you had a node class, inherit from `rclpy.node.Node`. If you did not, refactor so that you do, or just create a `rclpy.node.Node` and use it as you would have used a `NodeHandle` in ROS1 in C++ (pass it around).
3. Replace `rospy.init_node(<name>)` with `rclpy.init()`. Move the name to the creation of the `rclpy.node.Node` (or in `super().__init__` if inheriting from it).
4. Logging now requires the node instance. Use `{self/node}.get_logger().{info/warn/error}`, and pass it a single string. Use f-strings.
5. Creating subscribers and publishers now require the node instance. Use `{self/node}.create_{publisher/subscription}`. Invert the arguments of the type of the message and the name: the type is now first, the name is now second. If you had a `queue_size` argument, keep it, but remove the keyword if you were using it as a kwarg. Same goes for services.
6. Timers are also created from the node (`node.create_timer`), and time needs to be obtained from it as well (`node.get_clock().now()`).
7. Getting parameters now require the node instance, and a declaration. If you want to declare it and get it in one line, use the form `{self/node}.declare_parameter(<name>, <defaulf_value>).get_parameter_value().<type>_value`, where type is `string`, `bool`, `double`, etc. Use autocompletion.
8. `spin` now takes the node as a parameter.
9. Use `KeyboardInterrupt` directly, not a weird ROS version of it like in ROS1.
10. Make sure to call `node.destroy_node()` and `rclpy.shutdown()` at the end, to prevent zombie nodes.

### C++ nodes boilerplate
1. Replace `ros/ros.h` with `rclcpp/rclcpp.hpp` in includes.
2. For every message, service and action, replace `<package>/MessageType.h` with `<package>/<interface_type>/message_type.hpp` in includes. For instance, `#include <std_msgs/String.h>` becomes `#include <std_msgs/msg/string.hpp>`.
3. For every message, service and action, add the `::<interface_type>` subnamespace. For instance, `std_msgs::String` becomes `std_msgs::msg::String`.
4. Every `ros::Publisher`, `ros::Subscriber` and stuff like that is now templated on the message type. You will to hunt down which thing is connected to which callback of which message type, and bring back this information where you declare the thing, probably in the header file. Also, `Subscriber` is now `Subscription`. Also, store shared ptr. For instance, if you had a `ros::Subscriber` that received `std_msgs/String` messages, you will now have a `rclcpp::Subscription<std_msgs::msg::String>::SharedPtr`.
5. For every `advertiseService`, `advertise` and `subscribe`, you will need to use the node instance as `node->create_{service/publisher/subscription}`. Also, you can't use the `(<name>, &Class::callback, this)` form anymore: use a lambda to wrap the call in a self-contained callback. You can use [a helper like this](https://github.com/introlab/opentera-webrtc-ros/blob/ros2/opentera_webrtc_ros/include/opentera_webrtc_ros/utils.h#L11-L57) if you want to help you wrap everything. Same goes for services. If you are using `image_transport`, the API has not changed and you can still use the `(<name>, &Class::callback, this)` form with `image_transport`.
6. Timers are also created from the node (`node->create_timer`), and time needs to be obtained from it as well (`node->now()/node->get_clock()->now()`). Please note that `Time` now lacks useful methods to convert to and from an integer number of nanoseconds, so you will need to do this manually, or use [these helper functions](https://github.com/introlab/opentera-webrtc-ros/blob/ros2/opentera_webrtc_ros/include/opentera_webrtc_ros/utils.h#L59-L81). If you do your own, be careful with integer overflows.
7. Callbacks used to be able to receive their arguments as references or const& or const& to shared ptrs or basically anything. Now, it needs to be a shared ptr to non-const. When wrapping the callback inside of a lambda, you can change the constness: the lambda can receive a shared ptr to non-const, but the callback it will call with it can require a shared ptr to const, or even a const& shared ptr to const, and the implicit conversion will work, allowing you to have const-correctness in your callback if you wish. Also, don't spell out the shared ptr name, use `MessageType::SharedPtr` or `MessageType::ConstSharedPtr`. For services, there is `MessageType::{Request/Response}::[Const]SharedPtr`.
8. Like in Python, parameters now require the node instance, and a declaration. Use `node->declare_parameter(<name>, <default_value>)`, this will directly return the value of the parameter (unlike in Python, where there is much more boilerplate). If you want a parameter with no default value, check [this example](https://github.com/introlab/opentera-webrtc-ros/blob/ros2/face_cropping/src/FaceCroppingNodeConfiguration.cpp#L7-L40).
9. `spin` now takes the node as a parameter.


### More complex migrations in nodes
1. If you were using `tf`, you will need to use `tf2` and ̀`tf2_ros`. You had a TransformListener. You will now also need a Buffer. Construct the buffer with the node's clock (`get_clock()`), and construct the listener with the buffer. The buffer will be used to get transforms instead of the listener.
2. If you were using `tf.transformations` in Python, there is no equivalent in ROS2 as this module is deprecated. Instead, use the `transforms3d` Python package. The API is different, so be careful. For instance, in quaternions, `tf.transormations` placed `w` last, while `transforms3d` places it first. Here is [an example](https://github.com/introlab/opentera-webrtc-ros/blob/ros2/opentera_webrtc_ros/opentera_webrtc_ros/libmapimageconverter.py#L14-L24) of using it to get the same API as in ROS1.
You can also use the `tf_transformations` ROS package (note the underscore), which wraps `transforms3d` with the same API as `tf.transformations` had in ROS1. There is an example [here](https://github.com/introlab/t-top/blob/ros2-migration/ros/t_top/t_top/movement_commands.py#L12).
3. `Rate`s are harder to use. If you can, use a `Timer` instead. If you need a rate, there is an example [here](https://github.com/introlab/opentera-webrtc-ros/blob/ros2/map_image_generator/src/main.cpp#L47).
4. In ROS2, service calls are asynchronous. You can register a callback that will be called with the response when it is received. If you used to block on a service call in a callback (topic or other service), this can deadlock in ROS2. Either use asynchronous and callbacks, or dive deep into ROS2's [callback groups](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html). There is an example using asynchronous [in C++ here](https://github.com/introlab/opentera-webrtc-ros/blob/ros2/map_image_generator/src/MapLabelsConverter.cpp#L26-L53), and one using callback groups [in Python here](https://github.com/introlab/t-top/blob/ros2-migration/ros/t_top/t_top/movement_commands.py#L77-L78) and [in C++ here](https://github.com/introlab/t-top/blob/ros2-migration/ros/demos/smart_speaker/src/states/task/WeatherForecastState.cpp#L21).
5. In Python, typing is much more strict than it was. For instance, you can't directly publish a `str` now: you need to wrap it in a `String` message (using `String(data="...")` is an easy fix). Same thing for numbers: integers will not convert to floating point types in ROS messages. If you are trying to set the `x` field of a Pose as `pose.x = 0`, you will get a runtime error. Use `pose.x = 0.0` or use `pose.x = float(integer_value)`.
6. If you were using a ROS parameter before creating the main node object (maybe it was to pre-configure how this main node would be created, or to instantiate a different node classe based on a parameter), you can't do that anymore, as getting parameters requires a node instance. You have a few choices:
    1. Move the selection/choice inside the constructor of the one single node class. This breaks the single-responsibility principle and will make the code harder to reason about, probably.
    2. Create a dummy temporary node, get the parameter, and destroy the dummy node. Then, use the parameter, and create the real node based on it. There is [a C++ example of this here](https://github.com/introlab/opentera-webrtc-ros/blob/ros2/face_cropping/src/face_cropping_node.cpp#L94-L103), and [a Python example here](https://github.com/introlab/audio_utils/blob/ros2/audio_utils/scripts/resampling_node.py#L304-L319).
    3. Use composition and not inheritance. Create a node instance, get the parameters you need, then pass the node instance to the constructor of the main class, which will store it and use it as its node.
7. `rclcpp::Node` inherits from `std::enable_shared_from_this`, which means that instances of `Node` are always meant to be stored inside a `shared_ptr`, and you can get a new `shared_ptr` to it even if you don't have a `shared_ptr`, but a direct reference to the node object. Most of the ROS2 API takes the node by `shared_ptr`, too. If you use composition, you will probably want to pass a reference to the `node` object to parts of your main node class: use a reference, you are guaranteed that it will live long enough because of composition, references can't be null, and they can call `node->shared_from_this()` if they ever need a shared ptr to pass to a ROS2 API function. But some things, like `image_transport`, need a `shared_ptr` to the node in their constructor. If you have composition of this inside your main node class which inherits from `Node`, you won't be able to initialize it in the constructor as `this->shared_from_this()` will not work in the constructor (you will get some sort of segmentation fault, probably). In this case, it might be easier to forgo inheritance altogether, and to use composition for the node: store a `shared_ptr<Node>` in your main node class, and use it instead of yourself when you need a node. It will act similar to a `NodeHandle` in ROS1. Place it before the `image_transport` thing in your class, and it will be fully initialized when you need it to initialize `image_transport`.
8. In C++, if you call a service asynchronously and the service server is not available, the service request will get stucked. You will never know that the service call failed, and you will leak memory. To prevent this, you need to cleanup the in-flight requests that have been there for too long. Check [this class that does this automatically](https://github.com/introlab/opentera-webrtc-ros/blob/ros2/opentera_webrtc_ros/include/opentera_webrtc_ros/utils.h#L83-L125). For some reason, this does not seem to be a concern in Python (We have not seen anywhere that this should also be done in Python).
9. If the node is both Qt and ROS, you will need to have a ROS spinner in another thread. There is a simple class to do it [in C++ here](https://github.com/introlab/opentera-webrtc-ros/blob/ros2/opentera_webrtc_robot_gui/src/main.cpp#L37-L54). You could also use Executors as seen [here](https://github.com/introlab/t-top/blob/ros2-migration/ros/demos/control_panel/src/control_panel_node.cpp#L63-L73).
10. In ROS1, generic subscribers and publishers are easy to create with the [ShapeShifter class](http://docs.ros.org/en/indigo/api/topic_tools/html/classtopic__tools_1_1ShapeShifter.html). In ROS2, the node class has methods to create generic subscribers and publishers (`create_generic_subscription` and `create_generic_publisher`), but they require the topic type as a string. To get them, you can use the `get_topic_names_and_types` method to get all topic names and types. However, the retrieved topic names are already remapped, thus you cannot use the remapping features of ROS2 for generic subscribers and publishers. You can see an example [here](https://github.com/introlab/hbba_lite/blob/ros2/hbba_lite/src/arbitration_node.cpp).

### Launch file migration
This is relatively straightforward if you decide to stick to XML launch files, even though the documentation is not really good.
This [ros2-launch migration guide](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1/Migrating-Launch-Files.html) is useful.
- You will need to use the `.launch.xml` suffix for your launch files.
- `if` and `unless` are way more restricted and can only be placed on a handful of things now, check the migration guide.
- If you used to pass a launch parameter to change the `output=` of nodes, you can't anymore: it needs an hardcoded string that is either "log", "screen" or "both". A tip: use "log" (or nothing as "log" is the default), and pass the `-a` flag to the `ros2-launch` command when you need to debug: this will redirect everything to the console. You can also use the `OVERRIDE_LAUNCH_PROCESS_OUTPUT` environment variable (this is what `-a` does).
- In ROS1, `eval` tags were way easier to use than in ROS2. Now, you need a pair of quotes englobing the whole thing that you want to evaluate, which means that you'll need a bunch of escaping of strings. Also, you used to have access to substitutions using `arg('name')` inside the evaluated expression: you can't do that anymore, you need to use launch file substitutions, which are textual. This is painful, and it also means that using `eval` for doing an `OR` on two conditions, for instance, will have weird results, because it will operate on strings and not booleans. You can compare to the "true" or "false" strings explicitly, or you can use the new operators substitutions like shown [here](https://github.com/introlab/odas_ros/blob/ros2/odas_ros/launch/odas.launch.xml#L32) with `$(or ...)`.
- If you used `rosparam` tags to pass YAML structured parameters, this does not work anymore. Use `param`. You can use nested `param` tags to reproduce a nested/mapping structure.
- If you need to pass an empty array to a parameter, you will have to be careful. Passing `"[]"` will be rejected as the type of the array cannot be deduced. For a string array, you can use `"['']"` and filter for empty strings in your code, if you don't need empty strings usually. For numeric arrays, use a special value that you will filter that is out of the range you use, or combine the array with a boolean that chooses wether the array should be ignored/considered empty or wether it should be used.
- `include` works differently now: it does not create a different scope for arguments and stuff. Combine with `group` to isolate arguments.
- `group` can't also add a namespace. Use `push-ros-namespace`.
- Namespacing seems to work differently. You will most likely have a bunch of things that don't connect (topics publishers-subscribers, service clients-servers) correctly because of bad namespaces. Same goes for remapping, that will fail because the `from=` will now be wrong.s
- `rtabmap` takes most of its parameters as string, even when they are booleans of numbers. The node will crash if passed a number of the form `"1"`, use the form `"'1'"`. If you need substitutions, place the additionnal quotes when the raw value is defined: they will be ignored around a substitution (see [this file](https://github.com/introlab/t-top/blob/ros2-migration/ros/t_top/launch/perceptions/rtabmap.launch.xml) for examples).
- Float arguments that have integer values need a `.0` or they will be rejected as being the wrong type.
- Replace `tf` with `tf2_ros`, and `rviz` with `rviz2`.

The most painful thing here is that `ros2-launch` is really really bad to help you spot errors: you will get random Python tracebacks coming from the `ros2-launch` code, without much information on what the error was, and no information at all about where in the launch file it originated from. Even when using the `--debug` flag, you will only get more Python tracebacks.
A few tips:
- Make sure that your `arg` and `let` tags in the outer scope have `default=`, not `value=`, and that the opposite is true for `arg` tags inside `include` tags.
- Make sure that your substitutions use `var` and not `arg` as in ROS1. Same thing for `find-package-prefix` or `find-package-share` instead of `find`.
- Make sure that you use `exec=` and not `name=` in `node` tags.
- Make sure that the launch files you `include` have the right suffix (probably `.launch.xml` for your's, probably `.launch.py` for externals, but not `.launch`: this is probably a ROS1 artifact).


### Gazebo, simulation, URDF and navigation migration
1. Need to pass `use_sim_time:=true` to every node in launch. Use special operations to set a parameter in every node (`set_parameter` tag at global scope in XML).
2. Use [this page](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki) to check how to migrate a given gazebo-ros plugin.
3. For your models to appear in Gazebo, you might need a line [like this one in your `package.xml`](https://github.com/introlab/opentera-webrtc-ros/blob/ros2/turtlebot3_beam_description/package.xml#L26).
4. The differential drive plugin has a `odometry_source` option. It used to be a string, now it's an integer (see [here](https://github.com/introlab/opentera-webrtc-ros/blob/ros2/turtlebot3_beam_description/urdf/turtlebot3_waffle.gazebo.xacro#L71)).
5. Starting gazebo from a launch file is different. Same goes for the `robot_description`, which was a global parameter in ROS1, and is now a normal parameter to the `robot_state_publisher` node in ROS2 and received via the `/robot_description` topic by any other node (published by `robot_state_publisher`). Compare [before](https://github.com/introlab/opentera-webrtc-ros/blob/ros1/opentera_webrtc_demos/launch/opentera_turtlebot_sim.launch#L27-L42) and [after](https://github.com/introlab/opentera-webrtc-ros/blob/ros2/opentera_webrtc_demos/launch/opentera_turtlebot_sim.launch.xml#L26-L41).
6. Every reference to `move_base` need to be replaced with the ROS2 `nav2` equivalent. Compare [before](https://github.com/introlab/opentera-webrtc-ros/blob/ros1/opentera_webrtc_demos/launch/opentera_turtlebot_sim.launch#L98-L101) and [after](https://github.com/introlab/opentera-webrtc-ros/blob/ros2/opentera_webrtc_demos/launch/opentera_turtlebot_sim.launch.xml#L99-L101) for a simple example in launch file.

### RVIZ config files migration
1. The `rviz` package is now `rviz2`. Replace `rviz` with `rviz2` in your launch files.
2. All the components have moved. They no longer have the `rviz` prefix, but usually `rviz_common` or `rviz_default_plugins`. Use existing RVIZ config files for ROS2 to check the new names.
3. A bunch of stuff changes in configuration of components
  1. `robot_description` received via topic
  2. Different component altogether to send goals to `nav2` than used with ROS1 `move_base`
  3. A bunch of others.

The easiest way is probably to re-create the config file from scratch by re-adding and re-configuring the components you need.
Checking a diff between your old config file and a new one using similar components can also work, migrating the changes parts that seem important, but not touching window sizes and stuff like that.

## Other tips

### Debugging nodes
Using a visual debugger with ROS is hard to configure. If you can use GDB, you can use a command of this form to debug specific nodes:
```bash
ros2 launch -a my_package my_launch_file.launch.xml --launch-prefix-filter '.*executable_name.*' --launch-prefix 'gnome-terminal --wait -- gdb -ex run --args'
```
This will launch a new terminal window with GDB for every node that matches the filter regex. You will need to press "Enter" to start the node in every terminal, and you will need to kill the terminals manually at the end of your debug session.

### Multiple robots on the same network
The DDS backend that ROS2 uses does not require a ROS Master (`roscore`) to be running anymore: every node can detect and communicate with other nodes by itself, but this detection can cross the system boundaries and nodes on different machines will find each other by default.
In ROS1, this was different: you had to export the `ROS_MASTER_URI` on the other machines to make sure that the nodes on this machine would connect to the same `roscore` as the nodes on the other machine, which allowed communication between those nodes.
If you want to work across machines (for instance, to debug a ROS system running on a robot using your personnal computer), this makes it much easier.
But it you have multiple computers or multiple robots connected to the same netweork, which are not meant to connect with each other, you might see some weird behavior if you are not aware of the fact that they might connect and communicate with each other.
To prevent this, you can configure each device with a different DDS domain ID for ROS, by setting the `ROS_DOMAIN_ID` environment variable. See the [documentation about this](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html) for more details on how to set this up.
