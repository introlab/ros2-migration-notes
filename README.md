# ros2-migration-notes
ROS2 migration notes

## Workspace

### Isolated build
If you were using `catkin_make`, start by first migrating to `catkin-tools`, which uses `catkin build` to build the workspace.
In ROS2, the packages are built in an isolated fashion. By first building the ROS1 workspace using `catkin build`, you will be able to find out and fix your dependency problems before adding the ROS2 stuff to the mix.

### Using `colcon`
In ROS2, `colcon` is used to build the workspace. Like `catkin_make`, but not as `catkin build`, it will need to be invoked from the source of the workspace.
If you invoke it from a nested directory, it will happily proceed to create `build`, `install` and `log` directories at this nested place and will report a success, but it will not have done what you wanted it to do.
Also, much like `catkin_make` and not as `catkin build`, it can't be preconfigured with a set of default arguments.
There is a way to create a file to define default arguments to pass to it every invocation, which is similar.
It is barely documented [here](https://colcon.readthedocs.io/en/released/user/configuration.html#defaults-yaml). Note that the example is JSON, but the actual format is YAML: since JSON is valid YAML, the example will work, but you could use YAML instead.
Also, it is not documented there, but you can create such a file per-workspace. It needs to be named `colcon_defaults.yaml` and be placed in the root of the workspace. (The source for this information is the [source code of colcon-defaults](https://github.com/colcon/colcon-defaults/blob/master/colcon_defaults/argument_parser/defaults.py#L34).)

#### `colcon` usage
You will mostly use `colcon build` to build the workspace. If you want to build a subset of the packages, pass `--packages-select <package1> [<package2...]` to the command. (This is useful during the migration: migrate one package at a time, and build only the migrated packages.)

