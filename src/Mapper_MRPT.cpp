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
    "conf.default.map_update", "1",
    // Widget
    "conf.__widget__.debug", "text",
    "conf.__widget__.map_update", "text",
    // Constraints
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
    m_mapOut("map", m_map)

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
  addOutPort("map", m_mapOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "0");
  bindParameter("map_update", m_map_update, "1");
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
  
  //m_pMapBuilder->
  m_pMapBuilder->log();
  
  ssr::Pose2D pose = m_pMapBuilder->getEstimatedPose();
  m_estimatedPose.data.position.x = pose.x;
  m_estimatedPose.data.position.y = pose.y;
  m_estimatedPose.data.heading =    pose.th;
  m_estimatedPoseOut.write();

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


