// -*- C++ -*-
/*!
 * @file  Mapper_MRPT.h
 * @brief Mapper RTC using MRPT
 * @date  $Date$
 *
 * $Id$
 */

#ifndef MAPPER_MRPT_H
#define MAPPER_MRPT_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "MobileRobotSVC_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>



#include "MapBuilder.h"
using namespace RTC;

/*!
 * @class Mapper_MRPT
 * @brief Mapper RTC using MRPT
 *
 */
class Mapper_MRPT
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  Mapper_MRPT(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~Mapper_MRPT();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>


	 // Configuration variable declaration
	 // <rtc-template block="config_declare">
	 /*!
	 * For Debug, set 1 to get verbosity output on console.
	 * - Name:  debug
	 * - DefaultValue: 0
	 */
	 int m_debug;
	 /*!
	 * If true,
	 * - Name:  start_map_update_in_activated
	 * - DefaultValue: false
	 */
	 std::string m_start_map_update_in_activated;
	 /*!
	 * Select algorithm of ICP mapping. icpClassic,
	 * icpLevenbergMarquardt, icpIKF
	 * - Name:  ICP_algorithm
	 * - DefaultValue: icpClassic
	 */
	 std::string m_ICP_algorithm;
	 /*!
	 * The usual approach: to consider only the closest
	 * correspondence for each local point (Default to true)
	 * - Name:  ICP_onlyClosestCorrespondences
	 * - DefaultValue: true
	 */
	 std::string m_ICP_onlyClosestCorrespondences;
	 /*!
	 * Apart of "onlyClosestCorrespondences=true", if this option
	 * is enabled only the closest correspondence for each
	 * reference point will be kept (default=false).
	 * - Name:  ICP_onlyUniqueRobust
	 * - DefaultValue: false
	 */
	 std::string m_ICP_onlyUniqueRobust;
	 /*!
	 * Maximum number of iterations to run.
	 * - Name:  ICP_maxIterations
	 * - DefaultValue: 80
	 */
	 unsigned int m_ICP_maxIterations;
	 /*!
	 * If the correction in all translation coordinates (X,Y,Z) is
	 * below this threshold (in meters), iterations are terminated
	 * (Default:1e-6)
	 * - Name:  ICP_minAbsStep_trans
	 * - DefaultValue: 0.000001
	 */
	 float m_ICP_minAbsStep_trans;
	 /*!
	 * If the correction in all rotation coordinates
	 * (yaw,pitch,roll) is below this threshold (in radians),
	 * iterations are terminated (Default:1e-6)
	 * - Name:  ICP_minAbsStep_rot
	 * - DefaultValue: 0.000001
	 */
	 float m_ICP_minAbsStep_rot;
	 /*!
	 * Initial threshold distance for two points to become a
	 * correspondence.
	 * - Name:  ICP_thresholdDist
	 * - DefaultValue: 0.2
	 */
	 float m_ICP_thresholdDist;
	 /*!
	 * Initial threshold distance for two points to become a
	 * correspondence.
	 * - Name:  ICP_thresholdAng
	 * - DefaultValue: 0.1745
	 */
	 float m_ICP_thresholdAng;
	 /*!
	 * The scale factor for threshold everytime convergence is
	 * achieved.
	 * - Name:  ICP_ALFA
	 * - DefaultValue: 0.8
	 */
	 float m_ICP_ALFA;
	 /*!
	 * The size for threshold such that iterations will stop, since
	 * it is considered precise enough.
	 * - Name:  ICP_smallestThresholdDist
	 * - DefaultValue: 0.05
	 */
	 float m_ICP_smallestThresholdDist;
	 /*!
	 * This is the normalization constant \sigma^2_p that is used
	 * to scale the whole 3x3 covariance.  This has a default value
	 * of (0.02)^2 , that is, a 2cm sigma.
	 * - Name:  ICP_covariance_varPoints
	 * - DefaultValue: 0.0004
	 */
	 float m_ICP_covariance_varPoints;
	 /*!
	 * Perform a RANSAC step after the ICP convergence, to obtain a
	 * better estimation of the pose PDF.
	 * - Name:  ICP_doRANSAC
	 * - DefaultValue: false
	 */
	 std::string m_ICP_doRANSAC;
	 /*!
	 * RANSAC-step options
	 * - Name:  ICP_ransac_nSimulations
	 * - DefaultValue: 100
	 */
	 unsigned int m_ICP_ransac_nSimulations;
	 /*!
	 * RANSAC-step options
	 * - Name:  ICP_ransac_minSetSize
	 * - DefaultValue: 5
	 */
	 unsigned int m_ICP_ransac_minSetSize;
	 /*!
	 * RANSAC-step options
	 * - Name:  ICP_ransac_maxSetSize
	 * - DefaultValue: 20
	 */
	 unsigned int  m_ICP_ransac_maxSetSize;
	 /*!
	 * RANSAC-step options
	 * - Name:  ICP_ransac_mahalanobisDistanceThreshold
	 * - DefaultValue: 3.0
	 */
	 float m_ICP_ransac_mahalanobisDistanceThreshold;
	 /*!
	 * RANSAC-step option: The standard deviation in X,Y of
	 * landmarks/points which are being matched (used to compute
	 * covariances in the SoG)
	 * - Name:  ICP_ransac_normalizationStd
	 * - DefaultValue: 0.2
	 */
	 float m_ICP_ransac_normalizationStd;
	 /*!
	 * RANSAC-step option: The standard deviation in X,Y of
	 * landmarks/points which are being matched (used to compute
	 * covariances in the SoG)
	 * - Name:  ICP_ransac_fuseByCorrsMatch
	 * - DefaultValue: false
	 */
	 std::string m_ICP_ransac_fuseByCorrsMatch;
	 /*!
	 * RANSAC-step option: The standard deviation in X,Y of
	 * landmarks/points which are being matched (used to compute
	 * covariances in the SoG)
	 * - Name:  ICP_ransac_fuseMaxDiffXY
	 * - DefaultValue: 0.01
	 */
	 float m_ICP_ransac_fuseMaxDiffXY;
	 /*!
	 * RANSAC-step option: The standard deviation in X,Y of
	 * landmarks/points which are being matched (used to compute
	 * covariances in the SoG)
	 * - Name:  ICP_ransac_fuseMaxDiffPhi
	 * - DefaultValue: 0.001745
	 */
	 float m_ICP_ransac_fuseMaxDiffPhi;
	 /*!
	 * Cauchy kernel rho, for estimating the optimal transformation
	 * covariance, in meters (default = 0.07m).
	 * - Name:  ICP_kernel_rho
	 * - DefaultValue: 0.07
	 */
	 float m_ICP_kernel_rho;
	 /*!
	 * Whether to use kernel_rho to smooth distances, or use
	 * distances directly (default=true)
	 * - Name:  ICP_use_kernel
	 * - DefaultValue: true
	 */
	 std::string m_ICP_use_kernel;
	 /*!
	 * The size of the perturbance in x & y used to estimate the
	 * Jacobians of the square error (in LM & IKF methods,
	 * default=0.05).
	 * - Name:  ICP_Axy_aprox_derivatives
	 * - DefaultValue: 0.05
	 */
	 float m_ICP_Axy_aprox_derivatives;
	 /*!
	 * The initial value of the lambda parameter in the LM method
	 * (default=1e-4).
	 * - Name:  ICP_LM_initial_lambda
	 * - DefaultValue: 0.0001
	 */
	 float m_ICP_LM_initial_lambda;
	 /*!
	 * Skip the computation of the covariance (saves some time)
	 * (default=false)
	 * - Name:  ICP_skip_cov_calculation
	 * - DefaultValue: false
	 */
	 std::string m_ICP_skip_cov_calculation;
	 /*!
	 * Skip the (sometimes) expensive evaluation of the term
	 * 'quality' at ICP output (Default=true)
	 * - Name:  ICP_skip_quality_calculation
	 * - DefaultValue: true
	 */
	 std::string m_ICP_skip_quality_calculation;
	 /*!
	 * Decimation of the point cloud being registered against the
	 * reference one (default=5) - set to 1 to have the older (MRPT
	 * <0.9.5) behavior   of not approximating ICP by ignoring the
	 * correspondence of some points. The speed-up comes from a
	 * decimation of the number of KD-tree queries,   the most
	 * expensive step in ICP.
	 * - Name:  ICP_corresponding_points_decimation
	 * - DefaultValue: 5
	 */
	 unsigned int m_ICP_corresponding_points_decimation;
	 /*!
	 * (default:false) Match against the occupancy grid or the
	 * points map? The former is quicker but less precise.
	 * - Name:  ICP_matchAgainstTheGrid
	 * - DefaultValue: 0
	 */
	 std::string m_ICP_matchAgainstTheGrid;
	 /*!
	 * Minimum robot linear (m) displacement for a new observation
	 * to be inserted in the map.
	 * - Name:  ICP_insertionLinDistance
	 * - DefaultValue: 0.5
	 * - Unit: meter
	 */
	 double m_ICP_insertionLinDistance;
	 /*!
	 * Minimum robot angular (rad, deg when loaded from the .ini)
	 * displacement for a new observation to be inserted in the
	 * map.
	 * - Name:  ICP_insertionAngDistance
	 * - DefaultValue: 0.8
	 * - Unit: rad
	 */
	 double m_ICP_insertionAngDistance;
	 /*!
	 * Minimum robot linear (m) displacement for a new observation
	 * to be used to do ICP-based localization (otherwise,
	 * dead-reckon with odometry).
	 * - Name:  ICP_localizationLinDistance
	 * - DefaultValue: 0.5
	 * - Unit: meter
	 */
	 double m_ICP_localizationLinDistance;
	 /*!
	 * Minimum robot angular (rad, deg when loaded from the .ini)
	 * displacement for a new observation to be used to do
	 * ICP-based localization (otherwise, dead-reckon with
	 * odometry).
	 * - Name:  ICP_localizationAngDistance
	 * - DefaultValue: 0.8
	 */
	 double m_ICP_localizationAngDistance;
	 /*!
	 * Minimum ICP goodness (0,1) to accept the resulting corrected
	 * position (default: 0.40)
	 * - Name:  ICP_minICPgoodnessToAccept
	 * - DefaultValue: 0.40
	 */
	 float m_ICP_minICPgoodnessToAccept;
	 /*!
	 * Initial size of Occupancy Grid Map
	 * - Name:  MAP_min_x
	 * - DefaultValue: -10.0
	 * - Unit: meter
	 */
	 float m_MAP_min_x;
	 /*!
	 * Initial size of Occupancy Grid Map
	 * - Name:  MAP_max_x
	 * - DefaultValue: 10.0
	 * - Unit: meter
	 */
	 float m_MAP_max_x;
	 /*!
	 * Initial size of Occupancy Grid Map
	 * - Name:  MAP_min_y
	 * - DefaultValue: -10.0
	 * - Unit: meter
	 */
	 float m_MAP_min_y;
	 /*!
	 * Initial size of Occupancy Grid Map
	 * - Name:  MAP_max_y
	 * - DefaultValue: 10.0
	 * - Unit: meter
	 */
	 float m_MAP_max_y;
	 /*!
	 * Resolution of Grid Map. Length of the grid
	 * - Name:  MAP_resolution
	 * - DefaultValue: 0.05
	 * - Unit: meter
	 */
	 float m_MAP_resolution;
	 /*!
	 * The altitude (z-axis) of 2D scans (within a 0.01m tolerance)
	 * for they to be inserted in this map!
	 * - Name:  MAP_insertion_mapAltitude
	 * - DefaultValue: 0.0
	 * - Unit: meter
	 */
	 float m_MAP_insertion_mapAltitude;
	 /*!
	 * The parameter "mapAltitude" has effect while inserting
	 * observations in the grid only if this is true.
	 * - Name:  MAP_insertion_useMapAltitude
	 * - DefaultValue: false
	 */
	 std::string m_MAP_insertion_useMapAltitude;
	 /*!
	 * The largest distance at which cells will be updated (Default
	 * 15 meters)
	 * - Name:  MAP_insertion_maxDistanceInsertion
	 * - DefaultValue: 15
	 */
	 float m_MAP_insertion_maxDistanceInsertion;
	 /*!
	 * A value in the range [0.5,1] used for updating cell with a
	 * bayesian approach (default 0.8)
	 * - Name:  MAP_insertion_maxOccupancyUpdateCertainty
	 * - DefaultValue: 0.55
	 */
	 float m_MAP_insertion_maxOccupancyUpdateCertainty;
	 /*!
	 * If set to true (default), invalid range values (no echo
	 * rays) as consider as free space until
	 * "maxOccupancyUpdateCertainty", but ONLY when the previous
	 * and next rays are also an invalid ray.
	 * - Name:  MAP_insertion_considerInvalidRangesAsFreeSpace
	 * - DefaultValue: true
	 */
	 std::string m_MAP_insertion_considerInvalidRangesAsFreeSpace;
	 /*!
	 * Specify the decimation of the range scan (default=1 : take
	 * all the range values!)
	 * - Name:  MAP_insertion_decimation
	 * - DefaultValue: 1
	 */
	 unsigned short int m_MAP_insertion_decimation;
	 /*!
	 * The tolerance in rads in pitch & roll for a laser scan to be
	 * considered horizontal, then processed by calls to this class
	 * (default=0).
	 * - Name:  MAP_insertion_horizontalTolerance
	 * - DefaultValue: 0
	 */
	 float m_MAP_insertion_horizontalTolerance;
	 /*!
	 * Gaussian sigma of the filter used in getAsImageFiltered (for
	 * features detection) (Default=1) (0:Disabled)
	 * - Name:  MAP_insertion_CFD_features_gaussian_size
	 * - DefaultValue: 1
	 */
	 float m_MAP_insertion_CFD_features_gaussian_size;
	 /*!
	 * Size of the Median filter used in getAsImageFiltered (for
	 * features detection) (Default=3) (0:Disabled)
	 * - Name:  MAP_insertion_CFD_features_median_size
	 * - DefaultValue: 3
	 */
	 float m_MAP_insertion_CFD_features_median_size;
	 /*!
	 * Enabled: Rays widen with distance to approximate the real
	 * behavior of lasers, disabled: insert rays as simple lines
	 * (Default=false)
	 * - Name:  MAP_insertion_wideningBeamsWithDistance
	 * - DefaultValue: false
	 */
	 std::string m_MAP_insertion_wideningBeamsWithDistance;
	 /*!
	 * Initial Pose of Robot
	 * - Name:  initial_pose_x
	 * - DefaultValue: 0.0
	 */
	 double m_initial_pose_x;
	 /*!
	 * Initial Pose of Robot
	 * - Name:  initial_pose_y
	 * - DefaultValue: 0.0
	 * - Unit: meter
	 */
	 double m_initial_pose_y;
	 /*!
	 * Initial Pose of Robot
	 * - Name:  initial_pose_phi
	 * - DefaultValue: 0.0
	 * - Unit: rad
	 */
	 double m_initial_pose_phi;

	 // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::RangeData m_range;
  /*!
   */
  InPort<RTC::RangeData> m_rangeIn;
  RTC::TimedPose2D m_odometry;
  /*!
   */
  InPort<RTC::TimedPose2D> m_odometryIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedPose2D m_estimatedPose;
  /*!
   */
  OutPort<RTC::TimedPose2D> m_estimatedPoseOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  /*!
   */
  RTC::CorbaPort m_gridMapperPort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  /*!
   */
  OGMapperSVC_impl m_mapper;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>


  ssr::MapBuilder* m_pMapBuilder;
  bool m_odomUpdated;
  bool m_rangeUpdated;
  ssr::Pose2D m_OldPose;
  ssr::Map m_Map;

  public:

	void getCurrentMap(RTC::OGMap_out map);
	int32_t startMapping();
	int32_t stopMapping();

  RTC::MAPPER_STATE getState();
  coil::Mutex m_mapperMutex;

};


extern "C"
{
  DLL_EXPORT void Mapper_MRPTInit(RTC::Manager* manager);
};

#endif // MAPPER_MRPT_H
