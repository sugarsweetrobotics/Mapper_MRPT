// -*- C++ -*-
/*!
 * @file  Mapper_MRPT.cpp
 * @brief Mapper RTC using MRPT
 * @date $Date$
 *
 * $Id$
 */

#include "Mapper_MRPT.h"
#include "MapBuilder_MRPT.h"

// Module specification
// <rtc-template block="module_spec">
static const char* mapper_mrpt_spec[] =
  {
    "implementation_id", "Mapper_MRPT",
    "type_name",         "Mapper_MRPT",
    "description",       "Mapper RTC using MRPT",
    "version",           "1.0.2",
    "vendor",            "ssr",
    "category",          "Navigation",
    "activity_type",     "EVENTDRIVEN",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
	// Configuration variables
	"conf.default.debug", "0",
	"conf.default.start_map_update_in_activated", "false",
	"conf.default.ICP_algorithm", "icpClassic",
	"conf.default.ICP_onlyClosestCorrespondences", "true",
	"conf.default.ICP_onlyUniqueRobust", "false",
	"conf.default.ICP_maxIterations", "80",
	"conf.default.ICP_minAbsStep_trans", "0.000001",
	"conf.default.ICP_minAbsStep_rot", "0.000001",
	"conf.default.ICP_thresholdDist", "0.2",
	"conf.default.ICP_thresholdAng", "0.1745",
	"conf.default.ICP_ALFA", "0.8",
	"conf.default.ICP_smallestThresholdDist", "0.05",
	"conf.default.ICP_covariance_varPoints", "0.0004",
	"conf.default.ICP_doRANSAC", "false",
	"conf.default.ICP_ransac_nSimulations", "100",
	"conf.default.ICP_ransac_minSetSize", "5",
	"conf.default.ICP_ransac_maxSetSize", "20",
	"conf.default.ICP_ransac_mahalanobisDistanceThreshold", "3.0",
	"conf.default.ICP_ransac_normalizationStd", "0.2",
	"conf.default.ICP_ransac_fuseByCorrsMatch", "false",
	"conf.default.ICP_ransac_fuseMaxDiffXY", "0.01",
	"conf.default.ICP_ransac_fuseMaxDiffPhi", "0.001745",
	"conf.default.ICP_kernel_rho", "0.07",
	"conf.default.ICP_use_kernel", "true",
	"conf.default.ICP_Axy_aprox_derivatives", "0.05",
	"conf.default.ICP_LM_initial_lambda", "0.0001",
	"conf.default.ICP_skip_cov_calculation", "false",
	"conf.default.ICP_skip_quality_calculation", "true",
	"conf.default.ICP_corresponding_points_decimation", "5",
	"conf.default.ICP_matchAgainstTheGrid", "0",
	"conf.default.ICP_insertionLinDistance", "0.5",
	"conf.default.ICP_insertionAngDistance", "0.8",
	"conf.default.ICP_localizationLinDistance", "0.5",
	"conf.default.ICP_localizationAngDistance", "0.8",
	"conf.default.ICP_minICPgoodnessToAccept", "0.40",
	"conf.default.MAP_min_x", "-10.0",
	"conf.default.MAP_max_x", "10.0",
	"conf.default.MAP_min_y", "-10.0",
	"conf.default.MAP_max_y", "10.0",
	"conf.default.MAP_resolution", "0.05",
	"conf.default.MAP_insertion_mapAltitude", "0.0",
	"conf.default.MAP_insertion_useMapAltitude", "false",
	"conf.default.MAP_insertion_maxDistanceInsertion", "15",
	"conf.default.MAP_insertion_maxOccupancyUpdateCertainty", "0.55",
	"conf.default.MAP_insertion_considerInvalidRangesAsFreeSpace", "true",
	"conf.default.MAP_insertion_decimation", "1",
	"conf.default.MAP_insertion_horizontalTolerance", "0",
	"conf.default.MAP_insertion_CFD_features_gaussian_size", "1",
	"conf.default.MAP_insertion_CFD_features_median_size", "3",
	"conf.default.MAP_insertion_wideningBeamsWithDistance", "false",
	"conf.default.initial_pose_x", "0.0",
	"conf.default.initial_pose_y", "0.0",
	"conf.default.initial_pose_phi", "0.0",
	// Widget
	"conf.__widget__.debug", "text",
	"conf.__widget__.start_map_update_in_activated", "radio",
	"conf.__widget__.ICP_algorithm", "radio",
	"conf.__widget__.ICP_onlyClosestCorrespondences", "radio",
	"conf.__widget__.ICP_onlyUniqueRobust", "radio",
	"conf.__widget__.ICP_maxIterations", "text",
	"conf.__widget__.ICP_minAbsStep_trans", "text",
	"conf.__widget__.ICP_minAbsStep_rot", "text",
	"conf.__widget__.ICP_thresholdDist", "text",
	"conf.__widget__.ICP_thresholdAng", "text",
	"conf.__widget__.ICP_ALFA", "text",
	"conf.__widget__.ICP_smallestThresholdDist", "text",
	"conf.__widget__.ICP_covariance_varPoints", "text",
	"conf.__widget__.ICP_doRANSAC", "radio",
	"conf.__widget__.ICP_ransac_nSimulations", "text",
	"conf.__widget__.ICP_ransac_minSetSize", "text",
	"conf.__widget__.ICP_ransac_maxSetSize", "text",
	"conf.__widget__.ICP_ransac_mahalanobisDistanceThreshold", "text",
	"conf.__widget__.ICP_ransac_normalizationStd", "text",
	"conf.__widget__.ICP_ransac_fuseByCorrsMatch", "radio",
	"conf.__widget__.ICP_ransac_fuseMaxDiffXY", "text",
	"conf.__widget__.ICP_ransac_fuseMaxDiffPhi", "text",
	"conf.__widget__.ICP_kernel_rho", "text",
	"conf.__widget__.ICP_use_kernel", "radio",
	"conf.__widget__.ICP_Axy_aprox_derivatives", "text",
	"conf.__widget__.ICP_LM_initial_lambda", "text",
	"conf.__widget__.ICP_skip_cov_calculation", "radio",
	"conf.__widget__.ICP_skip_quality_calculation", "radio",
	"conf.__widget__.ICP_corresponding_points_decimation", "text",
	"conf.__widget__.ICP_matchAgainstTheGrid", "radio",
	"conf.__widget__.ICP_insertionLinDistance", "text",
	"conf.__widget__.ICP_insertionAngDistance", "text",
	"conf.__widget__.ICP_localizationLinDistance", "text",
	"conf.__widget__.ICP_localizationAngDistance", "text",
	"conf.__widget__.ICP_minICPgoodnessToAccept", "text",
	"conf.__widget__.MAP_min_x", "text",
	"conf.__widget__.MAP_max_x", "text",
	"conf.__widget__.MAP_min_y", "text",
	"conf.__widget__.MAP_max_y", "text",
	"conf.__widget__.MAP_resolution", "text",
	"conf.__widget__.MAP_insertion_mapAltitude", "text",
	"conf.__widget__.MAP_insertion_useMapAltitude", "radio",
	"conf.__widget__.MAP_insertion_maxDistanceInsertion", "text",
	"conf.__widget__.MAP_insertion_maxOccupancyUpdateCertainty", "text",
	"conf.__widget__.MAP_insertion_considerInvalidRangesAsFreeSpace", "radio",
	"conf.__widget__.MAP_insertion_decimation", "text",
	"conf.__widget__.MAP_insertion_horizontalTolerance", "text",
	"conf.__widget__.MAP_insertion_CFD_features_gaussian_size", "text",
	"conf.__widget__.MAP_insertion_CFD_features_median_size", "text",
	"conf.__widget__.MAP_insertion_wideningBeamsWithDistance", "radio",
	"conf.__widget__.initial_pose_x", "text",
	"conf.__widget__.initial_pose_y", "text",
	"conf.__widget__.initial_pose_phi", "text",
	// Constraints
	"conf.__constraints__.start_map_update_in_activated", "(true,false)",
	"conf.__constraints__.ICP_algorithm", "(icpClassic,icpLevenbergMarquardt,icpIKF)",
	"conf.__constraints__.ICP_onlyClosestCorrespondences", "(true,false)",
	"conf.__constraints__.ICP_onlyUniqueRobust", "(true,false)",
	"conf.__constraints__.ICP_doRANSAC", "(true,false)",
	"conf.__constraints__.ICP_ransac_fuseByCorrsMatch", "(true,false)",
	"conf.__constraints__.ICP_use_kernel", "(true,false)",
	"conf.__constraints__.ICP_skip_cov_calculation", "(true,false)",
	"conf.__constraints__.ICP_skip_quality_calculation", "(true,false)",
	"conf.__constraints__.ICP_matchAgainstTheGrid", "(true,false)",
	"conf.__constraints__.MAP_insertion_useMapAltitude", "(true,false)",
	"conf.__constraints__.MAP_insertion_considerInvalidRangesAsFreeSpace", "(true,false)",
	"conf.__constraints__.MAP_insertion_wideningBeamsWithDistance", "(true,false)",
	""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
Mapper_MRPT::Mapper_MRPT(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_rangeIn("range", m_range),
    m_odometryIn("odometry", m_odometry),
    m_estimatedPoseOut("estimatedPose", m_estimatedPose),
    m_gridMapperPort("gridMapper")
    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
Mapper_MRPT::~Mapper_MRPT()
{
}



RTC::ReturnCode_t Mapper_MRPT::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("range", m_rangeIn);
  addInPort("odometry", m_odometryIn);
  
  // Set OutPort buffer
  addOutPort("estimatedPose", m_estimatedPoseOut);
  
  // Set service provider to Ports
  m_gridMapperPort.registerProvider("OGMapper", "RTC::OGMapper", m_mapper);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_gridMapperPort);
  
  // </rtc-template>

  m_mapper.setMapperRTC(this);
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "0");
  bindParameter("start_map_update_in_activated", m_start_map_update_in_activated, "false");
  bindParameter("ICP_algorithm", m_ICP_algorithm, "icpClassic");
  bindParameter("ICP_onlyClosestCorrespondences", m_ICP_onlyClosestCorrespondences, "true");
  bindParameter("ICP_onlyUniqueRobust", m_ICP_onlyUniqueRobust, "false");
  bindParameter("ICP_maxIterations", m_ICP_maxIterations, "80");
  bindParameter("ICP_minAbsStep_trans", m_ICP_minAbsStep_trans, "0.000001");
  bindParameter("ICP_minAbsStep_rot", m_ICP_minAbsStep_rot, "0.000001");
  bindParameter("ICP_thresholdDist", m_ICP_thresholdDist, "0.2");
  bindParameter("ICP_thresholdAng", m_ICP_thresholdAng, "0.1745");
  bindParameter("ICP_ALFA", m_ICP_ALFA, "0.8");
  bindParameter("ICP_smallestThresholdDist", m_ICP_smallestThresholdDist, "0.05");
  bindParameter("ICP_covariance_varPoints", m_ICP_covariance_varPoints, "0.0004");
  bindParameter("ICP_doRANSAC", m_ICP_doRANSAC, "false");
  bindParameter("ICP_ransac_nSimulations", m_ICP_ransac_nSimulations, "100");
  bindParameter("ICP_ransac_minSetSize", m_ICP_ransac_minSetSize, "5");
  bindParameter("ICP_ransac_maxSetSize", m_ICP_ransac_maxSetSize, "20");
  bindParameter("ICP_ransac_mahalanobisDistanceThreshold", m_ICP_ransac_mahalanobisDistanceThreshold, "3.0");
  bindParameter("ICP_ransac_normalizationStd", m_ICP_ransac_normalizationStd, "0.2");
  bindParameter("ICP_ransac_fuseByCorrsMatch", m_ICP_ransac_fuseByCorrsMatch, "false");
  bindParameter("ICP_ransac_fuseMaxDiffXY", m_ICP_ransac_fuseMaxDiffXY, "0.01");
  bindParameter("ICP_ransac_fuseMaxDiffPhi", m_ICP_ransac_fuseMaxDiffPhi, "0.001745");
  bindParameter("ICP_kernel_rho", m_ICP_kernel_rho, "0.07");
  bindParameter("ICP_use_kernel", m_ICP_use_kernel, "true");
  bindParameter("ICP_Axy_aprox_derivatives", m_ICP_Axy_aprox_derivatives, "0.05");
  bindParameter("ICP_LM_initial_lambda", m_ICP_LM_initial_lambda, "0.0001");
  bindParameter("ICP_skip_cov_calculation", m_ICP_skip_cov_calculation, "false");
  bindParameter("ICP_skip_quality_calculation", m_ICP_skip_quality_calculation, "true");
  bindParameter("ICP_corresponding_points_decimation", m_ICP_corresponding_points_decimation, "5");
  bindParameter("ICP_matchAgainstTheGrid", m_ICP_matchAgainstTheGrid, "0");
  bindParameter("ICP_insertionLinDistance", m_ICP_insertionLinDistance, "0.5");
  bindParameter("ICP_insertionAngDistance", m_ICP_insertionAngDistance, "0.8");
  bindParameter("ICP_localizationLinDistance", m_ICP_localizationLinDistance, "0.5");
  bindParameter("ICP_localizationAngDistance", m_ICP_localizationAngDistance, "0.8");
  bindParameter("ICP_minICPgoodnessToAccept", m_ICP_minICPgoodnessToAccept, "0.40");
  bindParameter("MAP_min_x", m_MAP_min_x, "-10.0");
  bindParameter("MAP_max_x", m_MAP_max_x, "10.0");
  bindParameter("MAP_min_y", m_MAP_min_y, "-10.0");
  bindParameter("MAP_max_y", m_MAP_max_y, "10.0");
  bindParameter("MAP_resolution", m_MAP_resolution, "0.05");
  bindParameter("MAP_insertion_mapAltitude", m_MAP_insertion_mapAltitude, "0.0");
  bindParameter("MAP_insertion_useMapAltitude", m_MAP_insertion_useMapAltitude, "false");
  bindParameter("MAP_insertion_maxDistanceInsertion", m_MAP_insertion_maxDistanceInsertion, "15");
  bindParameter("MAP_insertion_maxOccupancyUpdateCertainty", m_MAP_insertion_maxOccupancyUpdateCertainty, "0.55");
  bindParameter("MAP_insertion_considerInvalidRangesAsFreeSpace", m_MAP_insertion_considerInvalidRangesAsFreeSpace, "true");
  bindParameter("MAP_insertion_decimation", m_MAP_insertion_decimation, "1");
  bindParameter("MAP_insertion_horizontalTolerance", m_MAP_insertion_horizontalTolerance, "0");
  bindParameter("MAP_insertion_CFD_features_gaussian_size", m_MAP_insertion_CFD_features_gaussian_size, "1");
  bindParameter("MAP_insertion_CFD_features_median_size", m_MAP_insertion_CFD_features_median_size, "3");
  bindParameter("MAP_insertion_wideningBeamsWithDistance", m_MAP_insertion_wideningBeamsWithDistance, "false");
  bindParameter("initial_pose_x", m_initial_pose_x, "0.0");
  bindParameter("initial_pose_y", m_initial_pose_y, "0.0");
  bindParameter("initial_pose_phi", m_initial_pose_phi, "0.0");

  /*
  bindParameter("x_min", m_x_min, "-10.0");
  bindParameter("x_max", m_x_max, "10.0");
  bindParameter("y_min", m_y_min, "-10.0");
  bindParameter("y_max", m_y_max, "10.0");
  bindParameter("resolution", m_resolution, "0.05");
  bindParameter("log_dir", m_log_dir, "log_out");
  bindParameter("log_enable", m_log_enable, "log_enable");
  bindParameter("init_pose_x", m_init_pose_x, "0.0");
  bindParameter("init_pose_y", m_init_pose_y, "0.0");
  bindParameter("init_pose_th", m_init_pose_th, "0.0");
  
  bindParameter("insertion_lin_distance", m_insertion_lin_distance, "1.0");
  bindParameter("insertion_ang_distance", m_insertion_ang_distance, "1.0");
  bindParameter("localization_lin_distance", m_localization_lin_distance, "0.5");
  bindParameter("localization_ang_distance", m_localization_ang_distance, "0.8");
  */

  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Mapper_MRPT::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Mapper_MRPT::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Mapper_MRPT::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t Mapper_MRPT::onActivated(RTC::UniqueId ec_id)
{
	std::cout << "[RTC::Mapper_MRPT] onActivated." << std::endl;
	m_pMapBuilder = ssr::createMapBuilder();
 
	std::cout << "[RTC::Mapper_MRPT] Loading Configurations." << std::endl;
	ssr::MapBuilderParam_MRPT param;

	param.ICP_algorithm = m_ICP_algorithm;
	param.ICP_onlyClosestCorrespondences = coil::toBool(m_ICP_onlyClosestCorrespondences, "true", "false", true);
	param.ICP_onlyUniqueRobust = coil::toBool(m_ICP_onlyUniqueRobust, "true", "false", true);
	param.ICP_maxIterations = m_ICP_maxIterations;
	param.ICP_minAbsStep_trans = m_ICP_minAbsStep_trans;
	param.ICP_minAbsStep_rot = m_ICP_minAbsStep_rot;
	param.ICP_thresholdDist = m_ICP_thresholdDist;
	param.ICP_thresholdAng = m_ICP_thresholdAng;
	param.ICP_ALFA = m_ICP_ALFA;
	param.ICP_smallestThresholdDist = m_ICP_smallestThresholdDist;
	param.ICP_covariance_varPoints = m_ICP_covariance_varPoints;
	param.ICP_doRANSAC = coil::toBool(m_ICP_doRANSAC, "true", "false", true);
	param.ICP_ransac_nSimulations = m_ICP_ransac_nSimulations;
	param.ICP_ransac_minSetSize = m_ICP_ransac_minSetSize;
	param.ICP_ransac_maxSetSize = m_ICP_ransac_maxSetSize;
	param.ICP_ransac_mahalanobisDistanceThreshold = m_ICP_ransac_mahalanobisDistanceThreshold;
	param.ICP_normalizationStd = m_ICP_ransac_normalizationStd;
	param.ICP_ransac_fuseByCorrsMatch = coil::toBool(m_ICP_ransac_fuseByCorrsMatch, "true", "false", true);
	param.ICP_ransac_fuseMaxDiffXY = m_ICP_ransac_fuseMaxDiffXY;
	param.ICP_ransac_fuseMaxDiffPhi = m_ICP_ransac_fuseMaxDiffPhi;
	param.ICP_kernel_rho = m_ICP_kernel_rho;
	param.ICP_use_kernel = coil::toBool(m_ICP_use_kernel, "true", "false", true);
	param.ICP_Axy_aprox_derivatives = m_ICP_Axy_aprox_derivatives;
	param.ICP_LM_initial_lambda = m_ICP_LM_initial_lambda;
	param.ICP_skip_cov_calculation = coil::toBool(m_ICP_skip_cov_calculation, "true", "false", true);
	param.ICP_skip_quality_calculation = coil::toBool(m_ICP_skip_quality_calculation, "true", "false", true);
	param.ICP_corresponding_points_decimation = m_ICP_corresponding_points_decimation;
	param.ICP_matchAgainstTheGrid = coil::toBool(m_ICP_matchAgainstTheGrid, "true", "false", true);
	param.ICP_insertionLinDistance = m_ICP_insertionLinDistance;
	param.ICP_insertionAngDistance = m_ICP_insertionAngDistance;
	param.ICP_localizationLinDistance = m_ICP_localizationLinDistance;
	param.ICP_localizationAngDistance = m_ICP_localizationAngDistance;
	param.ICP_minICPgoodnessToAccept = m_ICP_minICPgoodnessToAccept;
	param.MAP_min_x = m_MAP_min_x;
	param.MAP_max_x = m_MAP_max_x;
	param.MAP_min_y = m_MAP_min_y;
	param.MAP_max_y = m_MAP_max_y;
	param.MAP_resolution = m_MAP_resolution;
	param.MAP_insertion_mapAltitude = m_MAP_insertion_mapAltitude;
	param.MAP_insertion_useMapAltitude = coil::toBool(m_MAP_insertion_useMapAltitude, "true", "false", true);
	param.MAP_insertion_maxDistanceInsertion = m_MAP_insertion_maxDistanceInsertion;
	param.MAP_insertion_maxOccupancyUpdateCertainty = m_MAP_insertion_maxOccupancyUpdateCertainty;
	param.MAP_insertion_considerInvalidRangesAsFreeSpace = coil::toBool(m_MAP_insertion_considerInvalidRangesAsFreeSpace, "true", "false", true);
	param.MAP_insertion_decimation = m_MAP_insertion_decimation;
	param.MAP_insertion_horizontalTolerance = m_MAP_insertion_horizontalTolerance;
	param.MAP_insertion_CFD_features_gaussian_size = m_MAP_insertion_CFD_features_gaussian_size;
	param.MAP_insertion_CFD_features_median_size = m_MAP_insertion_CFD_features_median_size;
	param.MAP_insertion_wideningBeamsWithDistance = coil::toBool(m_MAP_insertion_wideningBeamsWithDistance, "true", "false", true);
	param.initial_pose_phi = m_initial_pose_phi;
	param.initial_pose_x = m_initial_pose_x;
	param.initial_pose_y = m_initial_pose_y;

	m_pMapBuilder->initialize(param);
	std::cout << "[RTC::Mapper_MRPT] Initialization OK." << std::endl;

	std::cout << "[RTC::Mapper_MRPT] Waiting for odometry information." << std::endl;
	while (!m_odometryIn.isNew()) {
	}
	m_odometryIn.read();
	m_OldPose = ssr::Pose2D(m_odometry.data.position.x, m_odometry.data.position.y, m_odometry.data.heading);
	std::cout << "[RTC::Mapper_MRPT] Odometry Information received." << std::endl;

	std::cout << "[RTC::Mapper_MRPT] Waiting for RangeData information." << std::endl;
	while (!m_rangeIn.isNew()) {
	}
	m_rangeIn.read();
	std::cout << "[RTC::Mapper_MRPT] RangeData received." << std::endl;

	m_odomUpdated = m_rangeUpdated = false;
	return RTC::RTC_OK;
}


RTC::ReturnCode_t Mapper_MRPT::onDeactivated(RTC::UniqueId ec_id)
{
	delete m_pMapBuilder;
	return RTC::RTC_OK;
}

void Mapper_MRPT::getCurrentMap(RTC::OGMap_out map_out) {

	m_mapperMutex.lock();
	ssr::Map map;
	m_pMapBuilder->getCurrentMap(map);
	m_mapperMutex.unlock();
	RTC::OGMap_var map_var = new RTC::OGMap();
	//map_out = new RTC::OGMap();
	map_var->config.width = map.getWidth();
	map_var->config.height = map.getHeight();
	map_var->config.xScale = map.getResolution();
	map_var->config.yScale = map.getResolution();
	map_var->config.origin.position.x = -map.getOriginX() * map.getResolution();
	map_var->config.origin.position.y = -map.getOriginY() * map.getResolution();
	map_var->config.origin.heading = 0.0;
	map_var->map.width = map.getWidth();
	map_var->map.height = map.getHeight();
	map_var->map.row = 0;
	map_var->map.column = 0;
	map_var->map.cells.length(map.getWidth() * map.getHeight());
	for(uint32_t i = 0;i < map.getHeight();i++) {
		for(uint32_t j = 0;j < map.getWidth();j++) {
			map_var->map.cells[(i)*map.getWidth() + j] = map.getCell(j, i);
		}
	}
	map_out = map_var._retn();
}

int32_t Mapper_MRPT::startMapping() {
	return m_pMapBuilder->startMapping();
}

int32_t Mapper_MRPT::stopMapping() {
	return m_pMapBuilder->stopMapping();
}

RTC::ReturnCode_t Mapper_MRPT::onExecute(RTC::UniqueId ec_id)
{

	if (m_odometryIn.isNew()) {
		m_odometryIn.read();
		ssr::Pose2D CurrentPose(m_odometry.data.position.x, m_odometry.data.position.y, m_odometry.data.heading);
		ssr::Pose2D deltaPose = CurrentPose - m_OldPose;
		m_OldPose = CurrentPose;
		m_pMapBuilder->addPose(deltaPose);
		m_odomUpdated = true;
	}

	if (m_rangeIn.isNew()) {
		m_rangeIn.read();
		ssr::Range range(&(m_range.ranges[0]), m_range.ranges.length(), m_range.config.maxAngle - m_range.config.minAngle);
		range.offset.x     = m_range.geometry.geometry.pose.position.x;
		range.offset.y     = m_range.geometry.geometry.pose.position.y;
		range.offset.z     = m_range.geometry.geometry.pose.position.z;
		range.offset.roll  = m_range.geometry.geometry.pose.orientation.r;
		range.offset.pitch = m_range.geometry.geometry.pose.orientation.p;
		range.offset.yaw   = m_range.geometry.geometry.pose.orientation.y;
		m_pMapBuilder->addRange(range);
		m_rangeUpdated = true;
	}

	if (m_odomUpdated && m_rangeUpdated) {

		m_mapperMutex.lock();
		m_pMapBuilder->processMap();
		m_mapperMutex.unlock();

		ssr::Pose2D pose = m_pMapBuilder->getEstimatedPose();
		m_estimatedPose.data.position.x = pose.x;
		m_estimatedPose.data.position.y = pose.y;
		m_estimatedPose.data.heading = pose.th;
		setTimestamp<RTC::TimedPose2D>(m_estimatedPose);
		m_estimatedPoseOut.write();


		m_odomUpdated = m_rangeUpdated = false;
	}


	/**
	static int counter;
	if (counter % 10 == 0) {
	int width = m_Map.getWidth();
	int height= m_Map.getHeight();
	if (this->m_map.cells.length() != width*height) {
	m_map.cells.length(width*height);
	}
	for(int i = 0;i < height;i++) {
	for(int j = 0;j < width;j++) {
	m_map.cells[i * width + j] = m_Map.getCell(i, j);
	}
	}

	m_map.column = width / 2;
	m_map.row = height / 2;
	m_map.width = width;
	m_map.height = height;
	m_mapOut.write();
	}
	**/

	return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Mapper_MRPT::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Mapper_MRPT::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t Mapper_MRPT::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Mapper_MRPT::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Mapper_MRPT::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::MAPPER_STATE Mapper_MRPT::getState() {
  if(m_pMapBuilder->isMapping()) {
    return MAPPER_MAPPING;
  } else {
    return MAPPER_STOPPED;
  }
}


extern "C"
{
 
  void Mapper_MRPTInit(RTC::Manager* manager)
  {
    coil::Properties profile(mapper_mrpt_spec);
    manager->registerFactory(profile,
                             RTC::Create<Mapper_MRPT>,
                             RTC::Delete<Mapper_MRPT>);
  }
  
};


