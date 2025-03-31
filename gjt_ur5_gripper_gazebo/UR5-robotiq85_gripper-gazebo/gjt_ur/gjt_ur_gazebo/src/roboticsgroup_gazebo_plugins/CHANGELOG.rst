^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roboticsgroup_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2020-05-18)
------------------
* changed maintainers
* Merge pull request `#14 <https://github.com/LCAS/roboticsgroup_gazebo_plugins/issues/14>`_ from roboticsgroup/pid_ns
  Add option to change the namespace of the pid
* Add option to change the namespace of the pid
* Merge pull request `#13 <https://github.com/LCAS/roboticsgroup_gazebo_plugins/issues/13>`_ from roboticsgroup/parameters
  Update parameters
* Update parameters
  * Default max effort to limit from sdf model
  * Default namespace to empty string
  * Fix sensitiveness calculation
* Merge pull request `#12 <https://github.com/LCAS/roboticsgroup_gazebo_plugins/issues/12>`_ from mintar/gazebo8
  Adjust to Gazebo 8 API and fix gazebo_ros_pkgs`#612 <https://github.com/LCAS/roboticsgroup_gazebo_plugins/issues/612>`_
* Improve namespace handling
* Fix README
* Add warning when triggering gazebo_ros_pkgs`#612 <https://github.com/LCAS/roboticsgroup_gazebo_plugins/issues/612>`_
* Add fix for gazebo_ros_pkgs`#612 <https://github.com/LCAS/roboticsgroup_gazebo_plugins/issues/612>`_
  This issue also affects the mimic joint plugin:
  https://github.com/ros-simulation/gazebo_ros_pkgs/issues/612
  The commit here fixes that issue for Gazebo 9. We should change the
  GAZEBO_MAJOR_VERSION check to >= 7 if the following PR gets backported
  to Gazebo 7 and 8:
  https://bitbucket.org/osrf/gazebo/pull-requests/2814/fix-issue-2111-by-providing-options-to/diff
* Remove unnecessary kill_sim member
* Adjust to Gazebo 8 API
  Note about the DisconnectWorldUpdateBegin: This function was deprecated
  in favor of resetting the ConnectionPtr, see here:
  https://bitbucket.org/osrf/gazebo/pull-requests/2329/deprecate-event-disconnect-connectionptr/diff
* Merge pull request `#10 <https://github.com/LCAS/roboticsgroup_gazebo_plugins/issues/10>`_ from mikaelarguedas/install_rules_for_plugins
  add install rules for the gazebo plugins
* add install rules for the gazebo plugins
* Fix README
* Merge pull request `#9 <https://github.com/LCAS/roboticsgroup_gazebo_plugins/issues/9>`_ from mintar/pid_gains
  Support all PID gain parameters, dynamic_reconfigure
* Support all PID gain parameters, dynamic_reconfigure
  This change does the following:
  * the PID controllers will read all PID gain parameters (p, i, d, i_clamp, antiwindup, publish_state, ...)
  * a warning will be printed if none of those parameters could be found
  * it's possible to adjust the parameters using dynamic_reconfigure
* Minor formatting
* Merge pull request `#8 <https://github.com/LCAS/roboticsgroup_gazebo_plugins/issues/8>`_ from roboticsgroup/formatting
  Formatting
* New error messages
* New formatting
* Merge pull request `#7 <https://github.com/LCAS/roboticsgroup_gazebo_plugins/issues/7>`_ from roboticsgroup/gazebo7
  Gazebo7 Support
* Better CMake
* Merge pull request `#6 <https://github.com/LCAS/roboticsgroup_gazebo_plugins/issues/6>`_ from projectmrk/master
  Support of Gazebo 7 in process
* Support of Gazebo 7 was added.
* Merge pull request `#3 <https://github.com/LCAS/roboticsgroup_gazebo_plugins/issues/3>`_ from tu-darmstadt-ros-pkg/add_set_force_for_pid_option_pr_upstream
  Add missing setForce() call (otherwise PID option doesn't do anything)
* Add missing setForce() call (otherwise PID option doesn't do anything)
* Merge pull request `#2 <https://github.com/LCAS/roboticsgroup_gazebo_plugins/issues/2>`_ from tu-darmstadt-ros-pkg/master
  Move catkin_package macro so it is called before targets are defined.
* Move catkin_package macro so it is called before targets are defined.
  Fixes plugins not getting found when doing isolated builds
* Code cleaning...
* README typo
* Added PID control capability to mimic joint plugin
* Added maxEffort parameter to MimicJoint plugin
* Added sensitiveness parameter to MimicJointPlugin
* Changed name of package
* Fixed typo in README and added checks in mimic plugin
* Added DisableLink Model Plugin
* Updated README
* Fixed small error in MimicJointPlugin
* Fixed type in README
* Initial commit..MimicJointPlugin..
* Contributors: Dmitry Suvorov, Konstantinos Chatzilygeroudis, Marc Hanheide, Martin Günther, Mikael Arguedas, Stefan Kohlbrecher, costashatz, nlamprian
