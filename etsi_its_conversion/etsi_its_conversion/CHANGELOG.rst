^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package etsi_its_conversion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.4.0 (2025-10-16)
------------------
* Merge pull request `#106 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/106>`_ from Tezozomoc47/make-converter-service-based
  Add the service-based approach for message conversion
* Merge pull request `#108 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/108>`_ from ika-rwth-aachen/fix-udp-src-port
  Fix setting of BTP destination port in `udp_msg.src_port`
* Merge pull request `#99 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/99>`_ from ika-rwth-aachen/codex/enable-parallel-execution-for-etsi_its_conversion
* Merge pull request `#100 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/100>`_ from Tezozomoc47/use-udppacket-srcport-for-btp
  Added filling of the UdpPacket.src_port to transport BtpDestinationPort information without touching the byte payload.
* Contributors: Lennart Reiher, Tezozomoc47

3.3.0 (2025-08-07)
------------------
* Merge pull request `#83 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/83>`_ from ika-rwth-aachen/deprecate-ros1
  Deprecate ROS 1 Noetic
* Contributors: Lennart Reiher

3.2.1 (2025-05-16)
------------------
* Merge pull request `#77 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/77>`_ from cubesys-GmbH/vam_constraint_check
* Contributors: Lennart Reiher

3.2.0 (2025-04-22)
------------------
* Merge pull request `#75 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/75>`_ from ika-rwth-aachen/uulm-mcm
  Add MCM version of Ulm University
* Merge branch 'main' into uulm-mcm
* Merge pull request `#72 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/72>`_ from ika-rwth-aachen/fix-runtime-error
  Fix critical runtime bug
* Merge pull request `#69 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/69>`_ from ika-rwth-aachen/fix-udp-pub-sub
  Only create UDP publisher/subscriber if required
* Contributors: Jean-Pierre Busch, Lennart Reiher

3.1.0 (2025-02-17)
------------------
* Merge pull request `#67 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/67>`_ from ika-rwth-aachen/update-copyright
  Update copyright
* Merge pull request `#64 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/64>`_ from ika-rwth-aachen/denm-v2
  Support DENM v2 (TS)
* Merge branch 'main' into feature/spat-map-plugin
* Contributors: Guido Küppers, Jean-Pierre Busch, Lennart Reiher

3.0.0 (2024-12-10)
------------------
* Merge pull request `#51 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/51>`_ from ika-rwth-aachen/improvement/converter-launch-file
  Improve converter launch file for better configuration
* Merge pull request `#28 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/28>`_ from ika-rwth-aachen/feature/spatem-mapem
  SPATEM/MAPEM Support
* Contributors: Jean-Pierre Busch, Lennart Reiher

2.4.0 (2024-11-19)
------------------
* Merge branch 'main' into feature/denm-launchfile-transformation
* Contributors: Lennart Reiher

2.3.0 (2024-10-15)
------------------
* Merge pull request `#36 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/36>`_ from ika-rwth-aachen/vam-ts
  Integrate VAM TS
* Contributors: Guido Küppers

2.2.0 (2024-08-01)
------------------
* Merge pull request `#29 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/29>`_ from ika-rwth-aachen/cam-release2
  Add ETSI ITS CAM TS (release2)
* Merge pull request `#24 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/24>`_ from ika-rwth-aachen/refactor/conversion
* Merge pull request `#26 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/26>`_ from ika-rwth-aachen/fix/cpm-ts
  Fix missing cpm_ts typos after pr `#21 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/21>`_
* Merge branch 'main' into refactor/conversion
* Merge pull request `#21 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/21>`_ from v0-e/fix-converter-memleak
  fix: Converter memleak
* Merge pull request `#2 <https://github.com/ika-rwth-aachen/etsi_its_messages/issues/2>`_ from lreiher/fix-converter-memleak
  Resolve conflicts with ika-rwth-aachen/etsi_its_messages and free ret.buffer
* Merge remote-tracking branch 'ika/main' into fix-converter-memleak
* Merge remote-tracking branch 'gitlab/conversion-queue-size' into refactor/conversion
* Contributors: Jean-Pierre Busch, Lennart Reiher, v0-e

2.1.0 (2024-07-09)
------------------
* Merge pull request #22 from ika-rwth-aachen/improvement/package-naming
  Enable parallel handling for TS and EN versions of the same message type
* Merge pull request #20 from ika-rwth-aachen/further-msgs
  Switch to rust-based generation and add additional message types
* Contributors: Jean-Pierre Busch

2.0.2 (2024-05-31)
------------------

2.0.1 (2024-02-27)
------------------

2.0.0 (2024-01-05)
------------------
* Merge pull request #7 from ika-rwth-aachen/improvement/conversion-node-params
  Improve configuration of conversion node
* Contributors: Lennart Reiher
