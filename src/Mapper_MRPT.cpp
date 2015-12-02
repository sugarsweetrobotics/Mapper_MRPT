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
    "conf.default.x_min", "-10.0",
    "conf.default.x_max", "10.0",
    "conf.default.y_min", "-10.0",
    "conf.default.y_max", "10.0",
    "conf.default.resolution", "0.05",
	"conf.default.init_pose_x", "0.0",
	"conf.default.init_pose_y", "0.0",
	"conf.default.init_pose_th", "0.0",
    "conf.default.log_dir", "log_out",
    "conf.default.log_enable", "log_enable",
    "conf.default.localization_lin_distance", "0.1",
    "conf.default.localization_ang_distance", "0.3",
    "conf.default.insertion_lin_distance", "1.0",
    "conf.default.insertion_ang_distance", "1.0",
    // Widget
    "conf.__widget__.debug", "text",
    "conf.__widget__.map_update", "spin",
    "conf.__widget__.x_min", "spin",
    "conf.__widget__.x_max", "spin",
    "conf.__widget__.y_min", "spin",
    "conf.__widget__.y_max", "spin",
    "conf.__widget__.resolution", "spin",
    "conf.__widget__.log_dir", "spin",
    "conf.__widget__.log_enable", "spin",
    // Constraints
    "conf.__constraints__.map_update", "true,false",
    "conf.__constraints__.log_enable", "true,false",
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

  bindParameter("ICP_algorithm", ICP_algorithm, "icpClassic");
  bindParameter("ICP_onlyClosestCorrespondences", ICP_onlyClosestCorrespondences, "true");
  bindParameter("ICP_onlyUniqueRobust", ICP_onlyUniqueRobust, "false");

  bindParameter("ICP_maxIterations", ICP_maxIterations, "80");
  bindParameter("ICP_minAbsStep_trans", ICP_minAbsStep_trans, "0.000001");
  bindParameter("ICP_minAbsStep_rot", ICP_minAbsStep_rot, "0.000001");
  bindParameter("ICP_thresholdDist", ICP_thresholdDist, "0.2");
  bindParameter("ICP_thresholdAng", ICP_thresholdAng, "0.1745");
  bindParameter("ICP_ALFA", ICP_ALFA, "0.8");
  bindParameter("ICP_smallestThresholdDist", ICP_smallestThresholdDist, "0.05");
  bindParameter("ICP_covariance_varPoints", ICP_covariance_varPoints, "0.0004");

  bindParameter("ICP_doRANSAC", ICP_doRANSAC, "false");
  bindParameter("ICP_ransac_minSetSize", ICP_ransac_minSetSize, "5"); // 3?
  bindParameter("ICP_ransac_maxSetSize", ICP_ransac_maxSetSize, "20"); 
  bindParameter("ICP_ransac_nSimulations", ICP_ransac_nSimulations, "100");
  bindParameter("ICP_ransac_mahalanobisDistanceThreshold", ICP_ransac_mahalanobisDistanceThreshold, "3.0");
  bindParameter("ICP_ransac_normalizationStd", ICP_ransac_normalizationStd, "0.2");
  bindParameter("ICP_ransac_fuseByCorrsMatch", ICP_ransac_fuseByCorrsMatch, "false");
  bindParameter("ICP_ransac_fuseMaxDiffXY", ICP_ransac_fuseMaxDiffXY, "0.01");
  bindParameter("ICP_ransac_fuseMaxDiffPhi", ICP_ransac_fuseMaxDiffPhi, "0.001745");

  bindParameter("ICP_kernel_rho", ICP_kernel_rho, "0.07");
  bindParameter("ICP_use_kernel", ICP_use_kernel, "true");
  bindParameter("ICP_Axy_aprox_derivatives", ICP_Axy_aprox_derivatives, "0.05");
  bindParameter("ICP_LM_initial_lambda", ICP_LM_initial_lambda, "0.0001");
  bindParameter("ICP_skip_cov_calculation", ICP_skip_cov_calculation, "false");
  bindParameter("ICP_skip_quality_calculation", ICP_skip_quality_calculation, "true");

  bindParameter("ICP_corresponding_points_decimation", ICP_corresponding_points_decimation, "5");
  bindParameter("ICP_matchAgainstTheGrid", ICP_matchAgainstTheGrid, "0");
  bindParameter("ICP_insertionLinDistance", ICP_insertionLinDistance, "0.5");
  bindParameter("ICP_insertionAngDistance", ICP_insertionLinDistance, "0.8");
  bindParameter("ICP_localizationLinDistance", ICP_localizationLinDistance, "0.5");
  bindParameter("ICP_localizationAngDistance", ICP_localizationAngDistance, "0.8");
  bindParameter("ICP_minICPgoodnessToAccept", ICP_minICPgoodnessToAccept, "0.40");

  bindParameter("MAP_max_x", MAP_max_x, "10.0");
  bindParameter("MAP_min_x", MAP_min_x, "-10.0");
  bindParameter("MAP_max_y", MAP_max_y, "10.0");
  bindParameter("MAP_min_y", MAP_min_y, "-10.0");
  bindParameter("MAP_resolution", MAP_resolution, "0.05");

  bindParameter("MAP_insertion_mapAltitude", MAP_insertion_mapAltitude, "0.0");
  bindParameter("MAP_insertion_useMapAltitude", MAP_insertion_useMapAltitude, "false");
  bindParameter("MAP_insertion_maxDistanceInsertion", MAP_insertion_maxDistanceInsertion, "25");
  bindParameter("MAP_insertion_maxOccupancyUpdateCertainty", MAP_insertion_maxOccupancyUpdateCertainty, "0.55");
  bindParameter("MAP_insertion_considerInvalidRangesAsFreeSpace", MAP_insertion_considerInvalidRangesAsFreeSpace, "true");
  bindParameter("MAP_insertion_wideningBeamsWithDistance", MAP_insertion_wideningBeamsWithDistance, "false");

  bindParameter("initial_pose_x", initial_pose_x, "0.0");
  bindParameter("initial_pose_y", initial_pose_y, "0.0");
  bindParameter("initial_pose_phi", initial_pose_phi, "0.0");

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


