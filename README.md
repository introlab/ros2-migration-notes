# ros2-migration-notes
ROS2 migration notes

This assumes that you are migrating from ROS1 Noetic on Ubuntu 20.04 to ROS2 Humble (either on Ubuntu 22.04 or compiled from source on Ubuntu 20.04).

## Workspace

### Isolated build
If you were using `catkin_make`, start by first migrating to `catkin-tools`, which uses `catkin build` to build the workspace.
In ROS2, the packages are built in an isolated fashion. By first building the ROS1 workspace using `catkin build`, you will be able to find out and fix your dependency problems before adding the ROS2 stuff to the mix.

### Devel space
There is no devel space in ROS2: every file that is needed will need to be installed using a CMake install rule.
The `--symlink-install` flag can be used to create symlinks instead of copies when installing, giving similar advantages to the devel space (no need to rebuild between each modification of a file if it is not part of a compiled executable, like Python files and launch files and config files).

### Using `colcon`
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

## Migration strategy
This is a suggestion of a way to migrate your packages.
To migrate, for each packages until they are all migrated:
1. Think about the dependency graph of you packages, and choose one that has no other dependencies in your packages.
2. Rename it by adding an `_OLD` suffix to the folder
3. Run the `ros2 pkg create --build-type ament_cmake <package_name>` command
    Note: In ROS2, you can create pure Python packages using `--build-type ament_python`, but the migration is more work. Only the CMake approach will be described.
4. Use a diff tool to compare the `package.xml` of the old and new package. Migrate the dependencies. Use the [ROS Index](https://index.ros.org/) to check for package availability in ROS2. You might need to replace or remove some packages that have a ROS2 alternative, but no ROS2 version.
5. Use a diff tool to compare the `CMakeLists.txt` of the old and new package. Most complex CMake logic can be directly copy-pasted. You will need to change the way that the dependencies are included, and also the way they are exported. Also, if you were building interfaces (messages, services or actions) as part of another package, you will need to move them to a dedicated interface package. Make sure that everything is installed, as there is no devel space anymore. Any file without an install rule will not be available by ROS.
You can check check the packages in our [`opentera-webrtc-ros`](https://github.com/introlab/opentera-webrtc-ros) or [`audio_utils`](https://github.com/introlab/audio_utils) repositories for examples or inspiration.
6. Move all the remaining files in the new package. You might want to move your header files if you used C++ and did not already respect the layout created by `ros2 pkg create`, especially if you are making a library: this will require changes to the `CMakeLists.txt`.
7. Delete the old package.
8. Build using `--packages-select` to include only the migrated packages.
9. Fix any errors until step 8. works and everything builds
10. Move on to the next package

Starting with the second package, you could have dependency errors, especially if you are exporting a library that depends on another library. Check [`audio_utils`](https://github.com/introlab/audio_utils) for an example of how to export such library if needed.

## Migrating nodes

### Python nodes boilerplate
Python nodes are relatively easy to migrate.
1. Replace `rospy` with `rclpy`. You will also need to import `rclpy.node`.
2. If you had a node class, inherit from `rclpy.node.Node`. If you did not, refactor so that you do, or just create a `rclpy.node.Node` and use it as you would have used a `NodeHandle` in ROS1 in C++ (pass it around).
3. Replace `rospy.init_node(<name>)` with `rclpy.init()`. Move the name to the creation of the `rclpy.node.Node` (or in `super().__init__` if inheriting from it).
4. Logging now requires the node instance. Use `{self/node}.get_logger().{info/warn/error}`, and pass it a single string. Use f-strings.
5. Creating subscribers and publishers now require the node instance. Use `{self/node}.create_{publisher/subbscription}`. Invert the arguments of the type of the message and the name: the type is now first, the name is now second. If you had a `queue_size` argument, keep it, but remove the keyword if you were using it as a kwarg.
6. Getting parameters now require the node instance, and a declaration. If you want to declare it and get it in one line, use the form `{self/node}.declare_parameter(<name>, <defaulf_value>).get_parameter_value().<type>_value`, where type if `string`, `bool`, `float`, etc. Use autocompletion.

### C++ nodes boilerplate
1. Replace `ros/ros.h` with `rclcpp/rclcpp.hpp` in includes.
2. For every message, service and action, replace `<package>/MessageType.h` with `<package>/<interface_type>/message_type.hpp` in includes. For instance, `#include <std_msgs/String.h>` becomes `#include <std_msgs/msg/string.hpp>`.
3. For every message, service and action, add the `::<interface_type>` subnamespace. For instance, `std_msgs::String` becomes `std_msgs::msg::String`.

### More complex migrations in nodes
1. If you were using `tf`, you will need to use `tf2` and ̀`tf2_ros`. You had a TransformListener. You will now also need a Buffer. Construct the buffer with the node's clock (`get_clock()`), and construct the listener with the buffer. The buffer will be used to get transforms instead of the listener.
2. If you were using `tf.transformations` in Python, there is no equivalent in ROS2 as this module is deprecated. Instead, use the `transforms3d` Python package. The API is different, so be careful. For instance, in quaternions, `tf.transormations` placed `w` last, while `transforms3d` places it first. Here is [an example](https://github.com/introlab/opentera-webrtc-ros/blob/ros2/opentera_webrtc_ros/opentera_webrtc_ros/libmapimageconverter.py#L14-L24) of using it to get the same API as in ROS1.
You can also use the `tf_transformations` ROS package (note the underscore), which wraps `transforms3d` with the same API as `tf.transformations` had in ROS1. There is an example [here](https://github.com/introlab/t-top/blob/ros2-migration/ros/t_top/t_top/movement_commands.py#L12).
3. `Rate`s are harder to use. If you can, use a `Timer` instead. If you need a rate, there is an example [here](https://github.com/introlab/opentera-webrtc-ros/blob/ros2/map_image_generator/src/main.cpp#L47).
4. In ROS2, service calls are asynchronous. You can register a callback that will be called with the response when it is received. If you used to block on a service call in a callback (topic or other service), this can deadlock in ROS2. Either use asynchronous and callbacks, or dive deep into ROS2's [callback groups](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html). There is an example using asynchronous [in C++ here](https://github.com/introlab/opentera-webrtc-ros/blob/ros2/map_image_generator/src/MapLabelsConverter.cpp#L26-L53), and one using callback groups [in Python here](https://github.com/introlab/t-top/blob/ros2-migration/ros/t_top/t_top/movement_commands.py#L77-L78) and [in C++ here](https://github.com/introlab/t-top/blob/ros2-migration/ros/demos/smart_speaker/src/states/task/WeatherForecastState.cpp#L21).
5. In Python, typing is much more strict than it was. For instance, you can't directly publish a `str` now: you need to wrap it in a `String` message (using `String(data="...")` is an easy fix). Same thing for numbers: integers will not convert to floating point types in ROS messages. If you are trying to set the `x` field of a Pose as `pose.x = 0`, you will get a runtime error. Use `pose.x = 0.0` or use `pose.x = float(integer_value)`.

### Launch file migration
This is relatively straightforward if you decide to stick to XML launch files, even though the documentation is not really good.
This [ros2-launch migration guide](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1/Migrating-Launch-Files.html) is useful.
You will need to use the `.launch.xml` suffix for your launch files.
Also, `if` and `unless` are way more restricted and can only be placed on a handful of things now, check the migration guide.
If you used to pass a launch parameter to change the `output=` of nodes, you can't anymore: it needs an hardcoded string that is either "log", "screen" or "both". A tip: use "log" (or nothing as "log" is the default), and pass the `-a` flag to the `ros2-launch` command when you need to debug: this will redirect everything to the console. You can also use the `OVERRIDE_LAUNCH_PROCESS_OUTPUT` environment variable (this is what `-a` does).
In ROS1, `eval` tags were way easier to use than in ROS2. Now, you need a pair of quotes englobing the whole thing that you want to evaluate, which means that you'll need a bunch of escaping of strings. Also, you used to have access to substitutions using `arg('name')` inside the evaluated expression: you can't do that anymore, you need to use launch file substitutions, which are textual. This is painful, and it also means that using `eval` for doing an `OR` on two conditions, for instance, will have weird results, because it will operate on strings and not booleans. You can compare to the "true" or "false" strings explicitly, or you can use the new operators substitutions like shown [here](https://github.com/introlab/odas_ros/blob/ros2/odas_ros/launch/odas.launch.xml#L32) with `$(or ...)`.
If you used `rosparam` tags to pass YAML structured parameters, this does not work anymore. Use `param`. You can use nested `param` tags to reproduce a nested/mapping structure.
Also, if you need to pass an empty array to a parameter, you will suffer. Passing `"[]"` will be rejected as the type of the array cannot be deduced. For a string array, you can use `"['']"` and filter for empty strings in your code, if you don't need empty strings usually. For numeric arrays, use a special value that you will filter that is out of the range you use, or combine the array with a boolean that chooses wether the array should be ignored/considered empty or wether should be used.

The most painful thing here is that `ros2-launch` is really really bad to help you spot errors: you will get random Python tracebacks coming from the `ros2-launch` code, without much information on what the error was, and no information at all about where in the launch file it originated from. Even when using the `--debug` flag, you will only get more Python tracebacks.
A few tips:
- Make sure that your `arg` and `let` tags in the outer scope have `default=`, not `value=`, and that the opposite is true for `arg` tags inside `include` tags.
- Make sure that your substitutions use `var` and not `arg` as in ROS1. Same thing for `find-package-prefix` or `find-package-share` instead of `find`.
- Make sure that you use `exec=` and not `name=` in `node` tags.
- Make sure that the launch files you `include` have the right suffix (probably `.launch.xml` for your's, probably `.launch.py` for externals, but not `.launch`: this is probably a ROS1 artifact).

## Other tips

### Debugging nodes
Using a visual debugger with ROS is hard to configure. If you can use GDB, you can use a command of this form to debug specific nodes:
```bash
ros2 launch -a my_package my_launch_file.launch.xml --launch-prefix-filter '.*executable_name.*' --launch-prefix 'gnome-terminal --wait -- gdb -ex run --args'
```
This will launch a new terminal window with GDB for every node that matches the filter regex. You will need to press "Enter" to start the node in every terminal, and you will need to kill the terminals manually at the end of your debug session.


