^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jackal_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.8 (2023-06-05)
------------------

0.8.7 (2023-04-19)
------------------

0.8.6 (2022-11-16)
------------------

0.8.5 (2022-05-17)
------------------

0.8.4 (2022-05-09)
------------------
* Disable absolute yaw in default imu configuration
* Set subst_value=true when loading the control_extras file to allow envar-defined configuration inside the file
* Contributors: Chris I-B, Chris Iverach-Brereton

0.8.3 (2022-03-08)
------------------

0.8.2 (2022-02-15)
------------------
* Updated control.launch to new microstrain envvars and moved definition of ekf-localization paramaters into it
* Added Microstrain GX5 to jackal_control
* Contributors: Luis Camero

0.8.1 (2022-01-18)
------------------
* predict odom->base_link tf to current time
* Contributors: Ebrahim Shahrivar

0.8.0 (2021-04-23)
------------------
* Merge branch 'noetic-devel-bkup' into noetic-devel
* Fix the link_name parameter for the interactive marker server; the default for the package includes a leading '/', which prevents the markers from working on Noetic.  We can revert this if/when the default for interactive_marker_twist_server is modified.
* Contributors: Chris Iverach-Brereton

0.7.5 (2021-03-24)
------------------

0.7.4 (2021-03-16)
------------------
* Bumped CMake version to avoid author warning.
* Add the JACKAL_JOY_DEVICE envar to optionally override the joy device more easily.
* Contributors: Chris Iverach-Brereton, Tony Baltovski

0.7.3 (2021-03-08)
------------------

0.7.2 (2020-09-29)
------------------
* Load the control extras last (`#75 <https://github.com/jackal/jackal/issues/75>`_)
* Remove the PS4 device from the yaml file, always apply the parameter from the joy_dev argument instead (`#73 <https://github.com/jackal/jackal/issues/73>`_)
* Contributors: Chris I-B

0.7.1 (2020-08-24)
------------------
* Disable ekf option (`#71 <https://github.com/jackal/jackal/issues/71>`_)
  * added env var and if-statement to disable robot ekf
  * changed if to unless
  * clearer wording
* Contributors: jmastrangelo-cpr

0.7.0 (2020-04-20)
------------------
* Mark the PS3 controller launch file as deprecated, update the default joystick device to point to /dev/input/ps4, as per the new udev rules added to the bringup package
* Contributors: Chris I-B

0.6.4 (2020-03-04)
------------------
* Update control.launch
  Somehow a ">" has gone missing. This change adds it back in.
* [jackal_control] Added control extras.
* Contributors: Jeff Schmidt, Tony Baltovski

0.6.3 (2019-07-18)
------------------

0.6.2 (2019-04-18)
------------------

0.6.1 (2018-08-02)
------------------
* Added missing twist_mux.yaml.  Fixed malformed meshes
* Added twist_mux and minor kinetic syntax changes
* Contributors: Dave Niewinski

0.6.0 (2018-04-12)
------------------
* [jackal_control] Made the PS4 controller default.
* Made minor changes to syntax for kinetic warnings
* Contributors: Dave Niewinski, Tony Baltovski

0.5.4 (2018-04-12)
------------------

0.5.3 (2016-06-01)
------------------
* Added support for PS4 controller.
* Contributors: Tony Baltovski

0.5.2 (2016-02-10)
------------------
* Removed reference to FootprintLayer.
* Increased inflation radius to account for uneven wall in Jackal_world.
* Added pointgrey camera and accessories.
* Improve robot_localiztion params
* Added Sick LMS1XX URDF.
* Fixed example calibration output.
* Added tutorials.
* Contributors: Mike Purvis, Martin Cote, Tony Baltovski, James Servos


0.5.1 (2015-02-02)
------------------

0.5.0 (2015-01-20)
------------------
* Support disabling the joystick for simulation when only the interactive markers are desired for teleop.
* Contributors: Mike Purvis

0.4.2 (2015-01-14)
------------------
* Shorten timeout for the controller spawner's shutdown.
* Contributors: Mike Purvis

0.4.1 (2015-01-07)
------------------
* Remove fork of diff_drive_controller.
* Contributors: Mike Purvis

0.4.0 (2014-12-12)
------------------
* added joystick argumant.
* Adding imu0_differential setting (=true) to control.yaml
* Add dep for joint state controller.
* Contributors: Mike Purvis, Shokoofeh Pourmehr, Tom Moore

0.3.0 (2014-09-10)
------------------

0.2.1 (2014-09-10)
------------------
* Depend on diff_drive_controller.
* Contributors: Mike Purvis

0.2.0 (2014-09-09)
------------------
* Add fork of diff_drive_controller.
* Fix run_depend elements.
* Fix remap for the interactive markers.
* New jackal_control package.
  This is launchers and configuration common to simulated and real
  Jackal, including controller, localization, and teleop.
* Contributors: Mike Purvis
