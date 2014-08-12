#pragma once
#include "MapBuilder.h"



#define TAG_APP "MappingApplication"
#define TAG_ICP "ICP"
#define TAG_RAWLOGOFFSET "rawlog_offset"
#define TAG_LOGOUTPUT_DIR "logOutput_dir"
#define TAG_LOGOUT "log_out"
#define TAG_LOG_FREQUENCY "LOG_FREQUENCY"
#define TAG_SAVE_POSE_LOG "SAVE_POSE_LOG"
#define TAG_SAVE_3D_SCENE "SAVE_3D_SCENE"
#define TAG_CAMERA_3DSCENE_FOLLOWS_ROBOT "CAMERA_3DSCENE_FOLLOWS_ROBOT"
#define TAG_ALWAYS_INSERT_BY_CLASS "alwaysInsertByClass"
#define TAG_SHOW_PROGRESS_3D  "SHOW_PROGRESS_3D_REAL_TIME"
#define TAG_ENABLE_LOGGING "enable_logging"
#define TAG_VERBOSE  "verbose"

#define TAG_MAP_MAX_X "map_max_x"
#define TAG_MAP_MAX_Y "map_max_y"
#define TAG_MAP_MIN_X "map_min_x"
#define TAG_MAP_MIN_Y "map_min_y"
#define TAG_MAP_RESOLUTION "map_resolution"
#define TAG_INIT_X "init_x"
#define TAG_INIT_Y "init_y"
#define TAG_INIT_TH "init_th"

#define DEFAULT_RAWLOGOFFSET 0
#define DEFAULT_LOGOUTDIR "log_out"
#define DEFAULT_LOGFREQ 5 
#define DEFAULT_SAVE_POSE_LOG false
#define DEFAULT_SAVE_3D_SCENE false
#define DEFAULT_CAMERA_3DSCENE_FOLLOWS_ROBOT true
#define DEFAULT_SHOW_PROGRESS_3D_REAL_TIME false
#define DEFAULT_SHOW_PROGRESS_REAL_TIME_DELAY_MS 0

#define DEFAULT_MAP_MAX_X 10.0f
#define DEFAULT_MAP_MAX_Y 10.0f
#define DEFAULT_MAP_MIN_X -10.0f
#define DEFAULT_MAP_MIN_Y -10.0f
#define DEFAULT_MAP_RESOLUTION 0.05
#define DEFAULT_INIT_X 0.0f
#define DEFAULT_INIT_Y 0.0f
#define DEFAULT_INIT_TH 0.0f

#define DEFAULT_VERBOSE false
#define DEFAULT_ENABLE_LOGGING false 


#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/gui.h>
#include <mrpt/utils.h>


namespace ssr {

class Map_MRPT : public ssr::Map {
private:
	mrpt::slam::CMultiMetricMap* m_pMap;
public:
	Map_MRPT();
	Map_MRPT(mrpt::slam::CMultiMetricMap* pMap);
	~Map_MRPT();
public:
	bool load(const std::string& inputFileName);
	
	bool save(const std::string& outputFileName);

};


class MapBuilder_MRPT : public ssr::MapBuilder {
private:
	int m_LogCount;

	int m_RawLogOffset;
	std::string m_LogOutDir;
	int m_LogFrequency;
	bool m_SavePoseLog;
	bool m_Save3DScene;
	bool m_Camera3DSceneFollowsRobot;
	bool m_ShowProgress3DRealTime;
	bool m_Logging;
	bool m_Verbose;
	//int	m_ShowProgress3DRealTimeDelayMs;

	float m_range_min;
	float m_range_max;
	
	mrpt::slam::CMetricMapBuilderICP m_MapBuilder;

	mrpt::utils::CFileOutputStream m_TimeStampLogFile;
	mrpt::utils::CFileOutputStream m_EstimatedPathLogFile;
	mrpt::utils::CFileOutputStream m_OdometryPathLogFile;

	
	mrpt::gui::CDisplayWindow3DPtr	m_3DWindow;

	mrpt::slam::CActionCollection m_ActionCollection;
	mrpt::slam::CSensoryFrame m_SensoryFrame;
	mrpt::poses::CPose3D m_RangeSensorPose;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	/**
	 * Constructor
	 */
	MapBuilder_MRPT();

	/**
	 * Destructor
	 */
	~MapBuilder_MRPT();


public:
	void setRangeSensorRange(float min, float max) {
		m_range_min = min; m_range_max = max;
	}

public:


	
	bool initialize(ssr::NamedString& parameter, ssr::Map* pMap=NULL);

	bool setRangeSensorPosition(const ssr::Position3D& position) {
		m_RangeSensorPose = mrpt::poses::CPose3D(position.x, position.y, position.z, position.roll, position.pitch, position.yaw);
		return true;
	}

	bool addPose(const ssr::Pose2D& pose);

	bool addRange(const ssr::Range& range);

	bool processMap();

	void log();

	ssr::Pose2D getEstimatedPose();

	void update3DWindow();

	void save();

	void getCurrentMap(ssr::Map& map);

	
	
	int32_t startMapping();

	int32_t stopMapping();

	void setCurrentMap(const ssr::Map& map);
};

};