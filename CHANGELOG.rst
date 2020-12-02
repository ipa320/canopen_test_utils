^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_test_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#14 <https://github.com/ipa320/canopen_test_utils/issues/14>`_ from fmessmer/test_noetic
  test noetic
* Merge pull request `#1 <https://github.com/ipa320/canopen_test_utils/issues/1>`_ from lindemeier/test_noetic
  fix pthread linker error
* fix pthread linker error
* set CMAKE_CXX_FLAGS -pthread
* add_compile_options
* Bump CMake version to avoid CMP0048 warning
* add noetic jobs
* Contributors: Felix Messmer, fmessmer, tsl

0.7.3 (2020-11-12)
------------------
* Merge pull request `#15 <https://github.com/ipa320/canopen_test_utils/issues/15>`_ from fmessmer/fix_catkin_lint
  remove dashboard launch file
* remove dashboard launch file
* Merge pull request `#13 <https://github.com/ipa320/canopen_test_utils/issues/13>`_ from fmessmer/add_nanotec_mapping
  add mapping for nanotec
* add mapping for nanotec
* Contributors: Felix Messmer, fmessmer, test-terminal

0.7.2 (2020-03-18)
------------------
* Merge pull request `#12 <https://github.com/ipa320/canopen_test_utils/issues/12>`_ from fmessmer/fix_travis_config
  [ci_updates] pylint + Python3 compatibility
* fix .travis.yml
* Revert "[ci_updates] pylint + Python3 compatibility"
* Merge pull request `#11 <https://github.com/ipa320/canopen_test_utils/issues/11>`_ from fmessmer/feature/python3_compatibility
  [ci_updates] pylint + Python3 compatibility
* fix pylint errors
* python3 compatibility via 2to3
* activate pylint checks from feature branch
* Merge pull request `#10 <https://github.com/ipa320/canopen_test_utils/issues/10>`_ from fmessmer/ci_updates
  [travis] ci updates
* fix xacro deprecation and namespace
* add CATKIN_LINT=pedantic
* update travis.yml
* catkin_lint fixes
* Contributors: Felix Messmer, fmessmer

0.7.1 (2019-11-07)
------------------
* Merge pull request `#9 <https://github.com/ipa320/canopen_test_utils/issues/9>`_ from ipa320/floweisshardt-patch-1
  migrate to travis-com
* migrate to travis-com
* Merge pull request `#8 <https://github.com/ipa320/canopen_test_utils/issues/8>`_ from benmaidel/feature/publish_pdos_script
  added script to publish pdo's as ros messages
* added script to publish pdo's
* Contributors: Benjamin Maidel, Felix Messmer, Florian Weisshardt

0.7.0 (2019-08-13)
------------------
* add missing tag in changelog
* Merge pull request `#7 <https://github.com/ipa320/canopen_test_utils/issues/7>`_ from benmaidel/feature/melodify
  [Melodic] melodify
* removed indigo checks
* add melodic checks
* using ros_canopen shared_pointer types
* added hardware_interface prefix
* Contributors: Benjamin Maidel, Felix Messmer, fmessmer

0.6.5 (2015-03-14)
------------------

0.6.4 (2015-07-03)
------------------

0.6.3 (2015-06-30)
------------------
* added missing dependency on cob_description
* added remarks for all configuration options
* depend on xacro was missing
* remove boost::posix_time::milliseconds from SyncProperties
* removed support for silence_us since bus timing cannot be guaranteed
* implemented plugin-based Master allocators, defaults to LocalMaster
* removed SM-based 402 implementation
* added timestamp to Elmo mapping
* added generic object publishers
* Removed overloaded functions and makes the handle functions protected
* Removes some logs
* Homing integrated
* example config to prevent homing
* c&p bug
* switch to new format, added heartbeat configuration
* use ring-buffer for IP mode
* removed target_interpolated_velocity from PDO mapping
* minor fixes for schunk dcf
* drive trajectory in IP mode
* bug fix in readable.py
* Schunk does not set operation mode via synchronized RPDO
* Merge remote-tracking branch 'mdl/indigo_dev' into refactor_sm
  Conflicts:
  canopen_402/include/canopen_402/canopen_402.h
  canopen_402/src/canopen_402/canopen_402.cpp
  canopen_motor_node/src/control_node.cpp
* Separates the hw with the SM test
  Advance on the Mode Switching Machine
* added sync silence feature
* simplified elmo_console
* Merge branch 'new_mapping' into indigo_dev
  Conflicts:
  canopen_test_utils/config/Elmo.dcf
* improved socketcan restart in prepare.sh
* removed 0x6081 (profile_velocity) from PDO mapping and added 0x6060 (op_mode)
* remove unused PDO map entries
* readable script with mapping loader
* new mapping scripts with PDO dictionaries
* corrected IP period, added 0x1014
* improved prepare script
* implemented threading in CANLayer
* moved ThreadedInterface to dedicated header
* removed bitrate, added loopback to DriverInterface::init
* * Eliminates Internal State conflict
  * Treats exceptions inside the state machine
* added canopen_elmo_console
* added dcf_overlay parameter
* updated joint configurations for new script server
* Merge branch 'indigo_dev' into merge
  Conflicts:
  canopen_chain_node/include/canopen_chain_node/chain_ros.h
  canopen_master/include/canopen_master/canopen.h
  canopen_master/include/canopen_master/layer.h
  canopen_master/src/node.cpp
  canopen_motor_node/CMakeLists.txt
  canopen_motor_node/src/control_node.cpp
* example config for unit factors
* add install tags
* Contributors: Florian Weisshardt, Mathias Lüdtke, thiagodefreitas

0.6.2 (2014-12-18)
------------------
* add dep
* Contributors: Florian Weisshardt

0.6.1 (2014-12-15)
------------------
* rename node
* remove ipa_* and IPA_* prefixes
* added descriptions and authors
* renamed ipa_canopen_test to canopen_test_utils
* Contributors: Florian Weisshardt, Mathias Lüdtke
