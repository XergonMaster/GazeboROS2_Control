Creating a Gazebo Plugin
=========================

In this tutorial, you will learn how to create a custom plugin for Gazebo.

.. literalinclude:: ../../src/Gazebo/gazebo_plugin_tutorial/pluguins/hello_world.cpp
   :language: cpp
   :caption: hello_world.cpp

Step-by-Step Explanation
------------------------

1. **Include Headers**: Include necessary Gazebo and ROS2 headers.
2. **Define Plugin Class**: Define your plugin class inheriting from `gazebo::ModelPlugin`.
3. **Load Method**: Implement the `Load` method to initialize the plugin.
4. **OnUpdate Method**: Implement the `OnUpdate` method to perform actions during the simulation.
5. **Compile and Run**: Use CMake to compile the plugin and run it in Gazebo.

For a detailed walkthrough, refer to the code comments in `example_plugin.cpp`.
