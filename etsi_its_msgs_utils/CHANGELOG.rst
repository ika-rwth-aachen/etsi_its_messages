^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package etsi_its_msgs_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.4.0 (2025-10-16)
------------------
* Merge pull request `#106 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/106>`_ from Tezozomoc47/make-converter-service-based
  Add the service-based approach for message conversion
* Merge pull request `#105 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/105>`_ from ika-rwth-aachen/fix-bit-string-access
  Fix getter/setter convenience access functions for BIT_STRING
* Contributors: Lennart Reiher, Tezozomoc47

3.3.0 (2025-08-07)
------------------
* Merge pull request `#89 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/89>`_ from ika-rwth-aachen/improvement/cpm-plugin
  Add object ids to RViz CPM plugin; rename plugins; add icons
* Merge pull request `#85 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/85>`_ from ika-rwth-aachen/mcm-access
  Add access functions for MCM
* Merge pull request `#81 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/81>`_ from FabianThomsen/feature/cam-yaw-rate
  Add setters and getters for yaw rate in CAM and CAM TS
* Merge pull request `#88 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/88>`_ from ika-rwth-aachen/kilted
* Merge pull request `#83 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/83>`_ from ika-rwth-aachen/deprecate-ros1
  Deprecate ROS 1 Noetic
* Merge pull request `#86 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/86>`_ from ika-rwth-aachen/fix/spatem_time_mark_interpretation
  Fix/spatem time mark interpretation
* Merge pull request `#82 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/82>`_ from ika-rwth-aachen/fix/missing_inline_on_functions
  Fix and adjust Spatem utils
* Contributors: AlexanderWilczynski, Fabian Thomsen, Jean-Pierre Busch, Lennart Reiher

3.2.1 (2025-05-16)
------------------

3.2.0 (2025-04-22)
------------------
* Merge pull request `#76 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/76>`_ from ika-rwth-aachen/feature/sensorinformation-utils
  Add setter functions for SensorInformationContainer
* Merge branch 'main' into uulm-mcm
* Merge pull request `#74 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/74>`_ from FabianThomsen/feature/var-utils
  Add setters and getters for (co)variances
* Contributors: Jean-Pierre Busch

3.1.0 (2025-02-17)
------------------
* Merge pull request `#67 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/67>`_ from ika-rwth-aachen/update-copyright
  Update copyright
* Merge pull request `#64 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/64>`_ from ika-rwth-aachen/denm-v2
  Support DENM v2 (TS)
* Merge branch 'main' into denm-v2
* Merge pull request `#65 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/65>`_ from ika-rwth-aachen/improvement/display_spatem_timing_information
  Improvement/display spatem timing information
* Merge pull request `#66 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/66>`_ from ika-rwth-aachen/fix/default-leap-seconds
  Use `rbegin()` instead of `end()` to access last element of `std::map` storing the leap-second insertions
* Merge remote-tracking branch 'origin/main' into improvement/display_spatem_timing_information
* Merge pull request `#63 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/63>`_ from cubesys-GmbH/fix_mapem_message_id
* Merge pull request `#55 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/55>`_ from ika-rwth-aachen/feature/spat-map-plugin
  RViz Plugin for SPATEM/MAPEM
* Merge pull request `#58 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/58>`_ from ika-rwth-aachen/fix/cam-setter-example
  Fix example for access-functions in README
* Merge branch 'main' into feature/spat-map-plugin
* Contributors: Alexander Wilczynski, AlexanderWilczynski, Guido Küppers, Jean-Pierre Busch, Lennart Reiher

3.0.0 (2024-12-10)
------------------
* Merge pull request `#48 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/48>`_ from ika-rwth-aachen/improvement/cpm-codegen-py
  Enable CLASS and SET support in codegen-py
* Merge pull request `#28 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/28>`_ from ika-rwth-aachen/feature/spatem-mapem
  SPATEM/MAPEM Support
* Contributors: Jean-Pierre Busch, Lennart Reiher

2.4.0 (2024-11-19)
------------------
* Merge pull request `#42 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/42>`_ from ika-rwth-aachen/feature/auto-cmake-generation
  Add automatic generation of CMakeLists.txt for msg packages
* Merge branch 'main' into feature/auto-cmake-generation
* Merge pull request `#44 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/44>`_ from ika-rwth-aachen/improvement/sample-scripts
  Relocate sample scripts
* Merge branch 'main' into feature/denm-launchfile-transformation
* Contributors: Jean-Pierre Busch, Lennart Reiher

2.3.0 (2024-10-15)
------------------
* Merge pull request `#38 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/38>`_ from ika-rwth-aachen/feature/cpm_utils
  Adding CPM Access-Functions and RViz Plugin; and improve utils package for all message types
* Contributors: Jean-Pierre Busch

2.2.0 (2024-08-01)
------------------
* Merge pull request `#29 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/29>`_ from ika-rwth-aachen/cam-release2
  Add ETSI ITS CAM TS (release2)
* Merge pull request `#2 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/2>`_ from lreiher/fix-converter-memleak
  Resolve conflicts with ika-rwth-aachen/etsi_its_messages and free ret.buffer
* Merge remote-tracking branch 'ika/main' into fix-converter-memleak
* Contributors: Jean-Pierre Busch, Lennart Reiher, v0-e

2.1.0 (2024-07-09)
------------------
* Merge pull request #20 from ika-rwth-aachen/further-msgs
  Switch to rust-based generation and add additional message types
* Contributors: Jean-Pierre Busch

2.0.2 (2024-05-31)
------------------

2.0.1 (2024-02-27)
------------------

2.0.0 (2024-01-05)
------------------
* Merge pull request #5 from ika-rwth-aachen/improvement/satisfy-package-conventions
  Refactoring to satisfy ROS package conventions
* Contributors: Lennart Reiher
