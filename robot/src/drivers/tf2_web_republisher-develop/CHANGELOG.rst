^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_web_republisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.2 (2017-11-20)
------------------
* Merge pull request `#25 <https://github.com/RobotWebTools/tf2_web_republisher/issues/25>`_ from VictorLamoine/develop
  Fix compilation with c++11
* add rostest
* Update travis configuration to test against kinetic (`#24 <https://github.com/RobotWebTools/tf2_web_republisher/issues/24>`_)
* Merge pull request `#22 <https://github.com/RobotWebTools/tf2_web_republisher/issues/22>`_ from daniel86/develop
  Added dependency to message_generation and message_runtime
* Contributors: Daniel Bessler, Jihoon Lee, Nils Berg, Victor Lamoine

0.3.1 (2017-03-01)
------------------
* Merge pull request `#17 <https://github.com/RobotWebTools/tf2_web_republisher/issues/17>`_ from T045T/develop
  Only log errors for the first exception on each source/target pair
* set is_okay in TFPair default c'tor
* don't log every exception, but only changes in state for each pair of source and target frame
* Contributors: Nils Berg, Russell Toris

0.3.0 (2014-12-11)
------------------
* Merge pull request #16 from T045T/tf2_republisher_service
  Tf2 republisher service
* add back action server functionality
* modularize code that can be used by both the action and service implementation
* remove test folder
* change terminology to reflect the fact that this is now a service, rather than an Action server
  also remove some high-frequency debug output
* change TFArray member name from data to transforms
* change tf2_web_republisher to use a service and dynamically advertised topics instead of an action call
  TODO: testing
* consistent 2 space indentation
* Contributors: Nils Berg, Russell Toris

0.2.2 (2014-08-15)
------------------
* release prepare
* Merge pull request #14 from T045T/send_initial_transformation
  make sure at least one transformation is sent
* make sure at least one transformation is sent, even if the tf is (and stays) the identity
* Merge pull request #13 from T045T/develop
  stop sending TF updates when removing a cancelled goal
* stop sending TF updates when removing a cancelled goal
* URLs added to package XML
* Contributors: Nils Berg, Russell Toris

0.2.1 (2014-04-08)
------------------
* cleanup
* ROS version checked for tf2 API namespace
* Contributors: Russell Toris

0.2.0 (2013-08-29)
------------------
* General clean up
* Use new tf2_ros namespace in hydro
* Added CI with travis-ci

0.1.0 (2013-02-13)
------------------
* version 0.1.0
* Merge branch 'groovy-devel' of git://github.com/KaijenHsiao/tf2_web_republisher into groovy-devel
  Conflicts:
  CMakeLists.txt
* more debug output
* merge
* Merge branch 'groovy-devel' of github.com:RobotWebTools/tf2_web_republisher into groovy-devel
  Conflicts:
  CMakeLists.txt
  src/tf_web_republisher.cpp
* catkin fixes
* added unit test files and fixed CMakeLists.txt
* added test from fuerte-devel
* updated CMakeLists to fix test_subscription2
* Merge pull request `#3 <https://github.com/RobotWebTools/tf2_web_republisher/issues/3>`_ from jkammerl/groovy-devel
  adding rate control parameter to action goal
* adding rate control per action client
* Merge branch 'fuerte-devel' of github.com:RobotWebTools/tf2_web_republisher into groovy-devel
* catkin fixes
* added function to remove leading slash in tf frame ids to avoid tf2 error
* deleting Makefile
* deleting manifest.xml
* catkinization of tf2_web_republisher.. not yet working due to missing geometry_experimental dependency
* revised version that handles multiple goals/actions
* initial commit
