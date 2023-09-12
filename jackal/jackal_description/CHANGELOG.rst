^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jackal_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.8 (2023-06-05)
------------------
* change location of onav config
* added outdoornav enable variable and hardware kit selector for urdf
* Contributors: José Mastrangelo

0.8.7 (2023-04-19)
------------------
* Added Environment Variables
* Contributors: luis-camero

0.8.6 (2022-11-16)
------------------
* Set the GPS plugin's reference heading to 90 so it's ENU
* Use xacro properties defined from environment variables for Microstrain URDF (`#123 <https://github.com/jackal/jackal/issues/123>`_)
* Add GAZEBO_WORLD\_{LAT|LON} envars to change the reference coordinate of the robot's integral GPS
* Contributors: Chris Iverach-Brereton, Joey Yang

0.8.5 (2022-05-17)
------------------
* Added Blackfly  entry to URDF
* Added Blackfly description to package.xml
* Contributors: Luis Camero

0.8.4 (2022-05-09)
------------------

0.8.3 (2022-03-08)
------------------
* Added the option to remove tower from VLP16 mount
* Added SICK TIM551 to URDF and package.xml
* Added UTM30 (`#106 <https://github.com/jackal/jackal/issues/106>`_)
* Updated Navsat and LMS1xx mounts (`#103 <https://github.com/jackal/jackal/issues/103>`_)
  * Updated hokuyo_ust10_mount to include min and max angle
  * Removed extra spaces
  * Updated SICK LMS1XX mount and NAVSAT mount
  * Maintained backward compatibility with LMS1xx standard upright poisition by adding mount types
* Updated hokuyo_ust10_mount to include min and max angle (`#102 <https://github.com/jackal/jackal/issues/102>`_)
  * Updated hokuyo_ust10_mount to include min and max angle
  * Removed extra spaces
* Contributors: Luis Camero, luis-camero

0.8.2 (2022-02-15)
------------------
* Moved microstrain link to accessories.urdf and updated envvars
* Added velodyne tower mesh
* Added Microstrain GX5 to description
* Removed unnecessary URDF
* Added Wibotic mesh and STL
* Contributors: Luis Camero

0.8.1 (2022-01-18)
------------------
* Updated to match melodic-devel
* Contributors: Luis Camero

0.8.0 (2021-04-23)
------------------

0.7.5 (2021-03-24)
------------------
* Add the origin block to the fender UST-10 macros; otherwise enabling them crashes
* Contributors: Chris I-B

0.7.4 (2021-03-16)
------------------
* Bumped CMake version to avoid author warning.
* Contributors: Tony Baltovski

0.7.3 (2021-03-08)
------------------
*  Add VLP16 support, refactor main/secondary laser envar support (#79)
* Contributors: Chris I-B

0.7.2 (2020-09-29)
------------------

0.7.1 (2020-08-24)
------------------

0.7.0 (2020-04-20)
------------------
* [jackal_description] Re-added pointgrey_camera_description as run depend.
* Contributors: Tony Baltovski

0.6.4 (2020-03-04)
------------------
* Modify the hokuyo accessory so that it works properly in gazebo/rviz.  Add an additional environment var JACKAL_LASER_HOKUYO which overrides the default lms1xx sensor with the ust10.
* use env_run.bat on Windows (`#3 <https://github.com/jackal/jackal/issues/3>`_)
* add setlocal
* Fix jackal_description install location & fold xacro includes (`#2 <https://github.com/jackal/jackal/issues/2>`_)
  * Fix install location.
  * Fold xacro includes
* add env-hook batch scripts (`#1 <https://github.com/jackal/jackal/issues/1>`_)
* Contributors: Chris I-B, James Xu, Sean Yen, Tony Baltovski

0.6.3 (2019-07-18)
------------------
* Added all extra fender changes
* Contributors: Dave Niewinski

* Made minor changes to syntax for kinetic warnings
* Contributors: Dave Niewinski

* Added stereo camera accessory.
* Removed unused variable jackal_description_dir
* Make urdf refer explicitly to jackal_description, rather than relying on current working directory being correct, for easier external includes
* Contributors: Arnold Kalmbach, Tony Baltovski, akalmbach

0.5.1 (2015-02-02)
------------------
* Modified the accessories.urdf.xacro to include both the GPS and mount plate, including standoffs.
* Eliminate rosrun from the xacro wrapper.
* Contributors: BryceVoort, Mike Purvis

0.5.0 (2015-01-20)
------------------
* Add hook for custom URDF insertion to jackal.urdf.xacro.
* Add xacro wrapper script to provide some pre-cooked "configs", especially for simulated Jackal.
* Switch to parameterizing URDF with optenv.
* Add laser bracket STL.
* Contributors: Mike Purvis

0.4.2 (2015-01-14)
------------------

0.4.1 (2015-01-07)
------------------

0.4.0 (2014-12-12)
------------------
* add pointgrey camera
* Removed inertial and geometry of the base_link.
* hector gazebo plugin for gps is added.
* hector gazebo plugin for imu sensor is added
* Contributors: Mike Purvis, spourmehr

0.3.0 (2014-09-10)
------------------
* Add comment about accessory args.
* Add front laser accessory to description.
* Contributors: Mike Purvis

0.2.1 (2014-09-10)
------------------

0.2.0 (2014-09-09)
------------------
* Changed physical and collision properties.
* Fixed inertia parameters. Added imu plugin--not working
* Install launch directory.
* Contributors: Mike Purvis, Shokoofeh

0.1.1 (2014-09-06)
------------------
* Remove unnecessary find packages.
* Contributors: Mike Purvis

0.1.0 (2014-09-05)
------------------
* Updated description with v0.9 hardware changes.
* Contributors: Mike Purvis
