# Mapper_MRPT

0. Introduction
Mapper RT-Component for OpenRTM-aist 1.1.

This RTC requires MRPT-1.0.2, OpenRTM-aist 1.1 C++ Release.

Currently, This component is tested in Windows 7 x64, Visual C++ 2010

1. How to build

1.1 Install
 - OpenRTM-aist 1.1 C++ Release : http://openrtm.org/openrtm/node/5019
 - CMake 2.8
 - Doxygen
 - Python2.6.6
 - PyYAML-3.10

 Please follow the instruction in openrtm.org to setup your developmental environment of OpenRTM-aist.

 - Install MRPT-1.0.2
  First, visit following URL:
   http://sourceforge.net/projects/mrpt/files/MRPT-all/MRPT-1.0.2/
  Then, you must download mrpt-1.0.2-msvc10-x32.exe.
  Do not select mrpt-1.0.2-msvc10-x32-kinect.exe. I've not tested.
  Currently, This version of MRPT is the only available package for VC2010.

1.2 How to use
 This component has simple interfaces

 Inport : 
   currentPose : TimedPose2D - Current Pose of Robot which is estimated with odometry.
   range : RangeData - Acquired by URG RTC. Please check following RTC
     https://github.com/sugarsweetrobotics/UrgRTC
 OutPort : 
   estimatedPose : TimedPose2D - Estimated Pose by SLAM algorithm (icpClassic)

 ServicePort : 
   gridMapper : ssr::OGMapper - This interface is used to controll Mapper_MRPT RT-Component. The IDL is following :

  /*!
   * @interface OGMapper
   * @brief Occupancy Grid Map Builder Service Interface
   */
  interface OGMapper {
    
    /// Initialize Current Build Map Data
    RETURN_VALUE initializeMap(in OGMapConfig config, in Pose2D initialPose);

    RETURN_VALUE startMapping();

    RETURN_VALUE stopMapping();

    RETURN_VALUE suspendMapping();

    RETURN_VALUE resumeMapping();

    RETURN_VALUE getState(out MAPPER_STATE state);

    /// Request Current Build Map Data
    RETURN_VALUE requestCurrentBuiltMap(out OGMap map);
  };

  To start Mapping, you should call "startMapping" function. Map Data is provided as OGMap data type. OGMap Data type is defined in IDL like following:
  
  /*!
   * @struct OCMap
   * @brief OccupancyGridMap Data
   */
  struct OGMap
  {
    /// Time stamp.
    Time tm;
    /// OccupancyGridMap Configuration
    OGMapConfig config;
    /// OccupancyGridMap Data
    OGMapTile map;
  };

  Types of OGMapConfig, and OGMapTile is defined in InterfaceDataTypes.idl which is included in OpenRTM-aist package.


    //------------------------------------------------------------
    // Map
    //------------------------------------------------------------

    /*!
     * @struct OGMapConfig
     * @brief Configuration of a occupancy-grip map.
     */
    struct OGMapConfig
    {
        /// Scale on the x axis (metres per cell).
        double xScale;
        /// Scale on the y axis (metres per cell).
        double yScale;
        /// Number of cells along the x axis.
        unsigned long width;
        /// Number of cells along the y axis.
        unsigned long height;
        /// Pose of the cell at (0, 0) in the real world.
        Pose2D origin;
    };

    /*!
     * @typedef OGMapCells
     */
    typedef sequence<octet> OGMapCells;

    /*!
     * @struct OGMapTile
     * @brief A tile from an occupancy-grid map.
     */
    struct OGMapTile
    {
        /// X coordinate of the (0, 0) cell of this tile in the whole map.
        unsigned long column;
        /// Y coordinate of the (0, 0) cell of this tile in the whole map.
        unsigned long row;
        /// Number of cells along the x axis in this tile;
        unsigned long width;
        /// Number of cells along the y axis in this tile;
        unsigned long height;
        /// Tile cells in (row, column) order.
        OGMapCells cells;
    };

 If you just want to use Mapper, you can use MapperViewer RTC to controll this Mapper. Visit, https://github.com/sugarsweetrobotics/MapperViewer.


Author: Yuki Suga (Sugar Sweet Robotics, co ltd.) ysuga@sugarsweetrobotics.com
Copyright : 2014, Sugar Sweet Robotics, Co. LTD.
License : GPLv3 (Read attached file : COPYING)
