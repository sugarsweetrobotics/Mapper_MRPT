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
    "version",           "1.0.0",
    "vendor",            "ssr",
    "category",          "Navigation",
    "activity_type",     "EVENTDRIVEN",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "0",
    "conf.default.map_update", "true",
    "conf.default.x_min", "-10.0",
    "conf.default.x_max", "10.0",
    "conf.default.y_min", "-10.0",
    "conf.default.y_max", "10.0",
    "conf.default.resolution", "0.05",
    "conf.default.log_dir", "log_out",
    "conf.default.log_enable", "log_enable",
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
  bindParameter("map_update", m_map_update, "true");
  bindParameter("x_min", m_x_min, "-10.0");
  bindParameter("x_max", m_x_max, "10.0");
  bindParameter("y_min", m_y_min, "-10.0");
  bindParameter("y_max", m_y_max, "10.0");
  bindParameter("resolution", m_resolution, "0.05");
  bindParameter("log_dir", m_log_dir, "log_out");
  bindParameter("log_enable", m_log_enable, "log_enable");
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
  m_pMapBuilder = new ssr::MapBuilder_MRPT();
  ssr::NamedString ns;
  
  m_pMapBuilder->initialize(ns);
  m_pMapBuilder->setRangeSensorPosition(ssr::Position3D(0.2, 0, 0.3));
  this->m_FirstExecution = true;
  //m_pMapBuilder->initialize(m_mrptSettingFileLocation + "/" +  m_mappingIniFileName, m_inputMap);
  //m_pMapBuilder->setRangeSensorPosition(Position3D(0.2, 0, 0.3));
  //m_pMapBuilder->setRangeSensorRange(m_range_min, m_range_max);
  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Mapper_MRPT::onDeactivated(RTC::UniqueId ec_id)
{
  m_pMapBuilder->save();
  delete m_pMapBuilder;
  return RTC::RTC_OK;
}

void Mapper_MRPT::getCurrentMap(RTC::OGMap_out map_out) {

	ssr::Map map;
	m_pMapBuilder->getCurrentMap(map);
	map_out = new RTC::OGMap();
	map_out->config.width = map.getWidth();
	map_out->config.height = map.getHeight();
	map_out->config.xScale = map.getResolution();
	map_out->config.yScale = map.getResolution();
	map_out->config.origin.position.x = map.getOriginX() * map.getResolution();
	map_out->config.origin.position.y = map.getOriginY() * map.getResolution();
	map_out->config.origin.heading = 0.0;
	map_out->map.width = map.getWidth();
	map_out->map.height = map.getHeight();
	map_out->map.row = map.getOriginX();
	map_out->map.column = map.getOriginY();
	map_out->map.cells.length(map.getWidth() * map.getHeight());
	for(uint32_t i = 0;i < map.getHeight();i++) {
		for(uint32_t j = 0;j < map.getWidth();j++) {
			map_out->map.cells[i*map.getWidth() + j] = map.getCell(i, j);
		}
	}
}

int32_t Mapper_MRPT::startMapping() {
	return m_pMapBuilder->startMapping();
}

int32_t Mapper_MRPT::stopMapping() {
	return m_pMapBuilder->stopMapping();
}

RTC::ReturnCode_t Mapper_MRPT::onExecute(RTC::UniqueId ec_id)
{

  if(m_odometryIn.isNew()) {
    m_odometryIn.read();
    if(m_FirstExecution) {
      m_FirstExecution = false;
      m_OldPose = ssr::Pose2D(m_odometry.data.position.x, m_odometry.data.position.y, m_odometry.data.heading);
      return RTC::RTC_OK;
    }
    ssr::Pose2D CurrentPose(m_odometry.data.position.x, m_odometry.data.position.y, m_odometry.data.heading);
    ssr::Pose2D deltaPose = CurrentPose - m_OldPose;
    m_OldPose = CurrentPose;
    m_pMapBuilder->addPose(deltaPose);
  }
  
  if(m_rangeIn.isNew()) {
    m_rangeIn.read();
    ssr::Range range(&(m_range.ranges[0]), m_range.ranges.length(), m_range.config.maxAngle - m_range.config.minAngle);
    for(size_t i = 0;i < m_range.ranges.length();i++) {
      range.range[i] = m_range.ranges[i];
    }
    m_pMapBuilder->addRange(range);
  }
  
  m_pMapBuilder->processMap();
  
  m_pMapBuilder->log();
  
  ssr::Pose2D pose = m_pMapBuilder->getEstimatedPose();
  m_estimatedPose.data.position.x = pose.x;
  m_estimatedPose.data.position.y = pose.y;
  m_estimatedPose.data.heading =    pose.th;
  m_estimatedPoseOut.write();

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


