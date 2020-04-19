^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bluefox2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2020-04-19)
------------------
* Add toggle to enable using ros::Time::now() directly in image header timestamps
  Replace the ROS_WARN large time offset printout with a published ros
  message
* Avoid printing use constant exposure time when tune_exposure_time is its default negative value
* Add bash script to update headers and library files to be compatible with the system-wide bluefox driver installation
* Add headers and library files from bluefox driver v2.26.0
* Delete headers and library files from bluefox driver v2.18.3
* Fix bug in previous commit
  If tune_exposure_time was set to a negative value, the driver attempted
  to wait a negative amount of time before clamping the exposure time to
  a fixed value. This resulted in auto-exposure being enabled permanently.
  This change only starts the auto exposure time interval when
  tune_exposure_time is set to a positive value.
* Set constant exposure time automatically after initialization
  Add an option to enable auto-exposure for user-defined time interval
  after camera initialization. At the end of this time interval, the
  current exposure value is fixed. This capability is useful when want
  constant exposure time images in environments with a priori unknown
  lighting conditions (i.e. outdoor field tests around dusk when lighting
  changes very rapidly between flight experiments).
* Print warning if shifted hardware timestamp differs from ros::Time::now() by more than 0.05 s
* Only set base time if hardware time is positive
  This avoids the case where the initial hardware time is a very small but
  negative value, which causes the offset to be equal to the initial
  ros::Time::now(), which results in a zero shift in the compensated
  hardware timestamps. This check forces SyncBase to wait until hardware
  time is positive before setting the offset, which results in a nonzero
  shift in the compensated hardware timestamps.
* Use embedded hardware timestamps instead of ros::Time::now() in image message header
* Populate sequence field of image message header with frame number from driver
* Update FindmvIMPACT to work with arm64 and x86_64
* turn on automatica exposure control upon startup
* Changes to support AArch64
* Adding launch file for Vector-NAV
* Fixing override error
* Cleaning up code
* Adding launch file for IMU triggered stereo set-up
* Working bluefox
* remove CameraInfo from GrabImage
* Merge branch 'master' into feature/hardware-sync
* add ${PROJECT_NAME} as target prefix
* Restore the travis status
  This reverts commit 3556dc104d39da90ae1e56fc731787f085505fe8.
* Merge branch 'add/gitignore' of github.com:KumarRobotics/bluefox2
* Really fix travis.yml
* Fix travis.yml
* Update travis.yml
* Update mvIMPACT SDK to v2.13.7
* fix nh to pnh
* export mvIMPACT lib and include
* export include
* update README
* add hardware trigger functionality
* Merge pull request `#14 <https://github.com/johnyao/bluefox2/issues/14>`_ from ke-sun/refactor/compilation
  Add AcquireOnce function to single/stereo_node
* add AcquireOnce function to single/stereo_node
* try adding aoi settings
* trivial fix
* fix boost with FillCaptureQueue
* add pixel clock setting
* fix dark current filter
* fix hdr
* fix some issue with acs
* fix auto control parameter
* fix exposure and gain
* fix binning mode
* fix idpf setting
* fix not unlock request
* trivial changes
* tested multi_node
* add a multi camera node
* forward declare Bluefox2Ros
* Update README.md
* add -k option to stereo calib
* Merge branch 'master' of https://github.com/KumarRobotics/bluefox2
* rename list_mvdevices to list_cameras
* fix exposure compenstaion
* removed libs from gitignore
* Adding a gitignore
* add option to set desired average grey scale value when using auto control parameters
* trivial
* improved dcfm calibration
* small changes
* try moving settings into CameraSetting
* Merge branch 'master' of https://github.com/KumarRobotics/bluefox2
* use predefined aec control
* add jpeg_quality to launch file
* Merge pull request `#11 <https://github.com/johnyao/bluefox2/issues/11>`_ from KumarRobotics/refactor/general
  Refactor/general
* small changes
* trivial stuff and start moving settings into mvimpact_helper
* update to package format 2
* better exposure compensation
* use pnh for private nodehandle
* trivial stuff
* oops
* add something from bluefox3, including list_mvdevice
* fix logic in SetCtm
* Update README.md
* bug fix in ctmsupported
* Some hack to compensate for some mechanical design, which will do:
  1. set mirror mode only at startup, and cannot change it later
  2. simple support for hardware synchronization on opto-isolated variant
  version of bluefox2-MLC
* fix ctm range
* Some hack to quickly enable hardware sync
  Will be improved later
* Update README.md
  Disable travis build until they upgrade to 14.04
* change naming convention of FindPkg.cmake
* remove unnecessary / in calib_url
* use new default calib_url, default to ~/.ros/camera_info
* Update README.md
  Add detailed instruction on dark current filter calibration
* Merge pull request `#9 <https://github.com/johnyao/bluefox2/issues/9>`_ from KumarRobotics/flippy
  Flippy
* fixed typo
* updated launch files
* Update README.md
  Add explanation for mirror mode
* add support for global mirror mode
* Merge pull request `#8 <https://github.com/johnyao/bluefox2/issues/8>`_ from KumarRobotics/flippy
  added option to flip camera
* added option to flip camera
* update readme, change default blue gain to 1
* Update README.md
  trivail
* Add gain control for bayer conversion
  the default is set to r:1, g:1, b:2.65, because that's what I
  usually get from the bluefox white balance calibration
* update install.bash
* Merge branch 'master' of https://github.com/KumarRobotics/bluefox2
* add install script for mvimpact acquire
* Update README.md to reflect change in calib
* resolve merge conflict
* Move calib file out of package
  Change default calib url to empty
* update stereo claib
* add namespace camera_base
* Update README.md
* udpate README.md
* update launch files
* update stereo calib
* Merge remote-tracking branch 'upstream/master'
* version 0.2.0
* Update README.md
* Update README.md
* Merge branch 'master' of github.com:versatran01/bluefox2
* fix name aec
* change ace to aec
* change parameter names to match bluefox param name
* add white balance calibration
* add franks fancy autoexposecontrol
* Update README.md
* Merge remote-tracking branch 'upstream/master'
* update stereo and color calib
* fix typo in readme
* merge
* update launch files
* fix calib for 337
* update all galt cameras
* better binning support
* fix some bad camera calibration
* update single node and single nodelet launch file
* Merge remote-tracking branch 'upstream/master'
* update stereo calib again
* update
* update
* Merge branch 'master' of github.com:versatran01/bluefox2
* udpate stereo calib
* Update README.md
* Update README.md
* Update README.md
* hacky stuff for stereo camera
* add dark current filter
* Merge pull request `#2 <https://github.com/johnyao/bluefox2/issues/2>`_ from versatran01/feature/more-options
  add white balance support
* add white balance support
* revert
* update package.xml
* update calib
* update stereo calib
* update stereo calib
* Merge branch 'master' of github.com:versatran01/bluefox2
* update calib pattern
* remove status messgae
* add Findmvimpact.cmake
* api change due to camera base
* Merge branch 'master' of github.com:versatran01/bluefox2
* change fps to double and update readme
* Update README.md
* change fps to double and update readme
* Update README.md
* Update README.md
* update readme
* add new feature for auto_fix_expose
* safer trigger mode
* Update README.md
* use boost mode to increase fps to 85 for 200wg and 24 for 202bg
* change brace init back to paren init
* Changing deprecated PLUGINLIB_DECLARE_CLASS to PLUGINLIB_EXPORT_CLASS
* default constructor in nodelet
* fix pluginlib export bug
* use camera_base catkin_package
* use pluginlib_export_class
* minor update
* Merge pull request `#1 <https://github.com/johnyao/bluefox2/issues/1>`_ from versatran01/refactor
  Refactor
* calibrate stereo camera
* udpate stereo_node.launch and some more calib files
* Merge branch 'refactor' of github.com:versatran01/bluefox2 into refactor
* update some more cameras
* add proc and view to stereo_node.launch
* add support for software sync stereo camera
* add support for hdr
* add support for pixel clock
* add option for expose_us and gain_db
* add option for binning
* add a space in printing available device
* add option for color
* add stereo node and nodelet
* refactored version working with single camera
* Merge remote-tracking branch 'upstream/master'
* Update README.md
* Update README.md
  Add solution to acquisitio failure on ubuntu 14.04
* Merge remote-tracking branch 'upstream/master'
* move script folder into mvIMPACT
* Adding the udev rule file which needs to be installed on new systems
* add calibration pattern
* Adding the required Matrix-Vision libraries
  - Their libusb is required otherwise the driver hangs when enumerating
  the cameras
  - libmvBlueFOX is required to detect Bluefox devices
* add white balance for color camera
* Merge remote-tracking branch 'upstream/master'
* Adding libmvBlueFOX.so* for armv7l, cannot detect camera without it
* Merge remote-tracking branch 'upstream/master'
* update calib file for 25001185
* update calib file for 25001185
* CMakeLists.txt: Don't overwrite CMAKE_CXX_FLAGS, append instead
  Also setting default CMAKE_BUILD_TYPE=RelWithDebInfo so that
  optimizations are enabled even if no CMAKE_BUILD_TYPE is passed to
  catkin_make
* cfg/CameraDyn.cfg: Explicitly use python2
* some hacks to speed up capture frequency
* Last few changes to travis.yml
* put using statements in namespace
* add udev rule
* travis.yml: Limit parallel number of jobs + cleanup
* Merge branch 'master' of https://github.com/versatran01/bluefox2
* add arm version sdk
* Merge remote-tracking branch 'upstream/master'
* small style fix
* Merge pull request `#5 <https://github.com/johnyao/bluefox2/issues/5>`_ from versatran01/master
  update travis ci status
* Merge branch 'master' of https://github.com/versatran01/bluefox2
* Update travis ci status image
* Update working environment
* Merge pull request `#4 <https://github.com/johnyao/bluefox2/issues/4>`_ from versatran01/master
  Add travis.yml and support 12.04
* update travis.yml to use default gcc and boost
* remove std c++11 features
* Adding boost1.55 to travis
* adding g++ 4.8 to travis.yml
* adding the mvIMPACT_acquire SDK
* remove clang from travis.yml
* Merge remote-tracking branch 'upstream/master'
* Update README.md
* Corrections to travis.yml
* Add travis.yml
* Commented out HDR option temporarily
* Recalibrated a camera
* merge nodelet to one library
* fix a bug of advertising duplicate service
  use the same frame_id for stereo node
* add break to loop for finding devcie
* fix dependency
* move timestamp before grabbing image
* Merge pull request `#3 <https://github.com/johnyao/bluefox2/issues/3>`_ from versatran01/feature/add-hdr
  Feature/add hdr
* remove debug print
* add hdr
* update calib file
* Merge pull request `#2 <https://github.com/johnyao/bluefox2/issues/2>`_ from versatran01/feature/add-service
  Feature/add service
* add service for setting camera expose, need to find a way to reflect change in dyanmic reconfigure gui
* intermediate work of adding set expose servcie
* add calibration file for 25000494
* add auto expose as an config option
* Merge branch 'master' of https://github.com/versatran01/bluefox2 into feature/add-stereo-sync
* update cmakelist and calib file for 25000495
* Update README.md
* Update README.md
* Create README.md
* New calibration params
* change image topic to image
* change dynamic reconfigure
* update new camera calibration file
* add respawn
* add error handling for both nodelet and improve launch file
* fix reversed master slave
* add stereo sync
* Merge pull request `#1 <https://github.com/johnyao/bluefox2/issues/1>`_ from versatran01/feature/add-nodelet
  Feature/add nodelet
* fix duplicate names in launch file
* finish stereo nodelet
* move time outside publish function
* working single nodelet
* some cleaning up and format
* working stereo pair
* rename all single node files
* small typo
* need to refactor
* rename files
* comment out some debugging print
* finish single camera
* finish camera node structure
* add dynamic reconfigure
* first commit
* Contributors: Alex Spitzer, Chao Qu, Gareth Cross, John Yao, Kartik Mohta, Mike Watterson, Shobhit Srivastava, ke, mwatterson
