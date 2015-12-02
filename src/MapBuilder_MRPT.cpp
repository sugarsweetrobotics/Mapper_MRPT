#include "MapBuilder_MRPT.h"

#ifdef HAVE_INTTYPES_H
#undef HAVE_INTTYPES_H
#endif

#include <mrpt/base.h>
#include <mrpt/obs.h>

#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/maps.h>
#include <mrpt/gui.h>
#include <mrpt/utils.h>

using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::math;

using namespace ssr;

class MapBuilder_MRPT : public MapBuilder {

	float m_range_min;
	float m_range_max;

	mrpt::slam::CMetricMapBuilderICP m_MapBuilder;

	mrpt::utils::CFileOutputStream m_TimeStampLogFile;
	mrpt::utils::CFileOutputStream m_EstimatedPathLogFile;
	mrpt::utils::CFileOutputStream m_OdometryPathLogFile;


	mrpt::gui::CDisplayWindow3DPtr	m_3DWindow;

	mrpt::obs::CActionCollection m_ActionCollection;
	mrpt::obs::CSensoryFrame m_SensoryFrame;
	mrpt::poses::CPose3D m_RangeSensorPose;


	bool m_isMapping;

	MapBuilderParam param;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		/**
		* Constructor
		*/
		MapBuilder_MRPT();

	/**
	* Destructor
	*/
	virtual ~MapBuilder_MRPT();


public:
	void setRangeSensorRange(float min, float max) {
		m_range_min = min; m_range_max = max;
	}

public:

	bool initialize(MapBuilderParam& param_);

	bool addPose(const ssr::Pose2D& pose);

	bool addRange(const ssr::Range& range);

	bool processMap();

	//void log();

	ssr::Pose2D getEstimatedPose();

	void getCurrentMap(ssr::Map& map);


	int32_t startMapping();

	int32_t stopMapping();

	void setCurrentMap(const ssr::Map& map);

	bool isMapping() { return m_isMapping; }
};


ssr::MapBuilder* ssr::createMapBuilder() {
	return new MapBuilder_MRPT();
}


/*
Map_MRPT::Map_MRPT()
{
	m_pMap = NULL;
}

Map_MRPT::Map_MRPT(mrpt::maps::CMultiMetricMap* pMap)
{
  m_pMap = new mrpt::maps::CMultiMetricMap(*pMap);
}

Map_MRPT::~Map_MRPT()
{
	if(m_pMap != NULL) {
		delete m_pMap; m_pMap = NULL;
	}
}
*/

/*
bool Map_MRPT::load(const std::string& fileName)
{
  m_pMap = new mrpt::maps::CMultiMetricMap();

	std::string simpleMapExt = ".simplemap";
	if(fileName.compare(fileName.length() - simpleMapExt.length(), simpleMapExt.length(), simpleMapExt) == 0) {
		CFileInputStream simpleMapFile(fileName);
		mrpt::maps::CSimpleMap map;
		simpleMapFile >> map;
		m_pMap->loadFromSimpleMap(map);
	}

	std::string gridMapExt = ".gridmap";
	if(fileName.compare(fileName.length() - gridMapExt.length(), gridMapExt.length(), gridMapExt) == 0) {
		CFileInputStream gridMapFile(fileName);
		mrpt::maps::COccupancyGridMap2D map;
		gridMapFile >> map;
		
	}
	return false;
}

bool Map_MRPT::save(const std::string& fileName)
{
	if(m_pMap->m_pointsMaps[0]) {
		CFileOutputStream simpleMapFile(fileName + ".simplemap");
		simpleMapFile << (m_pMap->m_pointsMaps)[0];
		simpleMapFile.close();
	}

	if(m_pMap->m_gridMaps[0]) {
		CFileOutputStream gridMapFile(fileName + ".gridmap");
		gridMapFile << (m_pMap->m_gridMaps)[0];
		gridMapFile.close();
	}
	return false;
}
*/

MapBuilder_MRPT::MapBuilder_MRPT()
{
	this->m_range_max = 20.0;
	this->m_range_min = 0.10;
	this->m_isMapping = false;

}

MapBuilder_MRPT::~MapBuilder_MRPT()
{

}

bool MapBuilder_MRPT::initialize(MapBuilderParam& param_) 
{
	m_ActionCollection.clear();
	m_SensoryFrame.clear();

	MapBuilderParam_MRPT& param = (MapBuilderParam_MRPT&)param_;

	/* ICP_param */
	if (param.ICP_algorithm == "icpClassic") {
		m_MapBuilder.ICP_params.ICP_algorithm = icpClassic;
	}
	else if (param.ICP_algorithm == "icpLevenbergMarquardt") {
		m_MapBuilder.ICP_params.ICP_algorithm = icpLevenbergMarquardt;
	}
	else if (param.ICP_algorithm == "icpIKF") {
		m_MapBuilder.ICP_params.ICP_algorithm = icpIKF;
	}

	m_MapBuilder.ICP_params.onlyClosestCorrespondences = param.ICP_onlyClosestCorrespondences;
	//m_MapBuilder.ICP_params.onlyUniqueRobust;

	m_MapBuilder.ICP_params.maxIterations = param.ICP_maxIterations;
	m_MapBuilder.ICP_params.minAbsStep_trans = param.ICP_minAbsStep_trans;
	m_MapBuilder.ICP_params.minAbsStep_rot = param.ICP_minAbsStep_rot;

	m_MapBuilder.ICP_params.thresholdDist = param.ICP_thresholdDist;
	m_MapBuilder.ICP_params.thresholdAng = param.ICP_thresholdAng;
	m_MapBuilder.ICP_params.ALFA = param.ICP_ALFA;
	m_MapBuilder.ICP_params.smallestThresholdDist = param.ICP_smallestThresholdDist;

	//m_MapBuilder.ICP_params.covariance_varPoints;

	//m_MapBuilder.ICP_params.doRANSAC;
	//m_MapBuilder.ICP_params.ransac_minSetSize;
	//m_MapBuilder.ICP_params.ransac_maxSetSize;
	//m_MapBuilder.ICP_params.ransac_nSimulations;
	//m_MapBuilder.ICP_params.ransac_mahalanobisDistanceThreshold;
	//m_MapBuilder.ICP_params.ransac_fuseMaxDiffXY;
	//m_MapBuilder.ICP_params.ransac_fuseMaxDiffPhi;
	//m_MapBuilder.ICP_params.ransac_fuseByCorrsMatch;
	//m_MapBuilder.ICP_params.normalizationStd;

	m_MapBuilder.ICP_params.corresponding_points_decimation = param.ICP_corresponding_points_decimation;

	m_MapBuilder.ICP_options.insertionLinDistance = param.ICP_insertionLinDistance;
	m_MapBuilder.ICP_options.insertionAngDistance = param.ICP_insertionAngDistance;
	m_MapBuilder.ICP_options.localizationLinDistance = param.ICP_localizationLinDistance;
	m_MapBuilder.ICP_options.localizationAngDistance = param.ICP_localizationAngDistance;

	m_MapBuilder.ICP_options.minICPgoodnessToAccept = param.ICP_minICPgoodnessToAccept;	// Minimum ICP quality to accept correction [0,1].

	m_MapBuilder.ICP_options.matchAgainstTheGrid = param.ICP_matchAgainstTheGrid;
	
	mrpt::maps::COccupancyGridMap2D::TMapDefinition mapDefinition;
	mapDefinition.max_x = param.MAP_max_x;
	mapDefinition.max_y = param.MAP_max_y;
	mapDefinition.min_x = param.MAP_min_x;
	mapDefinition.min_y = param.MAP_min_y;
	mapDefinition.resolution = param.MAP_resolution;

	mapDefinition.insertionOpts.mapAltitude = param.MAP_insertion_mapAltitude;
	mapDefinition.insertionOpts.useMapAltitude = param.MAP_insertion_useMapAltitude;
	mapDefinition.insertionOpts.maxDistanceInsertion = param.MAP_insertion_maxDistanceInsertion;
	mapDefinition.insertionOpts.maxOccupancyUpdateCertainty = param.MAP_insertion_maxOccupancyUpdateCertainty;
	mapDefinition.insertionOpts.considerInvalidRangesAsFreeSpace = param.MAP_insertion_considerInvalidRangesAsFreeSpace;
	mapDefinition.insertionOpts.wideningBeamsWithDistance = param.MAP_insertion_wideningBeamsWithDistance;
	//mapDefinition.insertionOpts.CFD_features_gaussian_size;
	//mapDefinition.insertionOpts.CFD_features_median_size;
	//mapDefinition.insertionOpts.decimation;
	//mapDefinition.insertionOpts.horizontalTolerance;

	mapDefinition.likelihoodOpts.likelihoodMethod = mrpt::maps::COccupancyGridMap2D::lmLikelihoodField_Thrun;
	mapDefinition.likelihoodOpts.LF_decimation = 5;
	mapDefinition.likelihoodOpts.LF_stdHit = 0.20;

	mapDefinition.likelihoodOpts.LF_maxCorrsDistance = 0.30;
	mapDefinition.likelihoodOpts.LF_zHit = 0.999;
	mapDefinition.likelihoodOpts.LF_zRandom = 0.001;
	mapDefinition.likelihoodOpts.LF_maxRange = 30;
	mapDefinition.likelihoodOpts.LF_alternateAverageMethod = 0;

	m_MapBuilder.ICP_options.mapInitializers.push_back(mapDefinition);

	/*
	mrpt::maps::CSimplePointsMap::TMapDefinition i2;
	//i2.metricMapClassType = CLASS_ID( mrpt::maps::CSimplePointsMap );
	i2.insertionOpts.minDistBetweenLaserPoints = 0.05;
	i2.insertionOpts.fuseWithExisting = false;
	i2.insertionOpts.isPlanarMap = 1;
	m_MapBuilder.ICP_options.mapInitializers.push_back(i2);
	*/

	m_MapBuilder.options.enableMapUpdating = false;

	mrpt::poses::CPosePDFGaussian initialPose(mrpt::poses::CPose2D(param.initial_pose_x, param.initial_pose_y, param.initial_pose_phi));
//	mrpt::maps::COccupancyGridMap2D gmap;

	m_MapBuilder.initialize(mrpt::maps::CSimpleMap(), &initialPose);

	/* For Logging and Verbosity */
	//m_MapBuilder.options.alwaysInsertByClass.fromString("");

	/*
	if(m_Verbose) {
		m_MapBuilder.options.verbose = true;
		m_MapBuilder.ICP_params.dumpToConsole();
		m_MapBuilder.ICP_options.dumpToConsole();
	}
	*/

	/*
	if(m_Logging) {
		deleteFilesInDirectory(m_LogOutDir.c_str());
		createDirectory(m_LogOutDir.c_str());
		m_LogCount = 0;
		deleteFilesInDirectory(m_LogOutDir.c_str());
		createDirectory(m_LogOutDir.c_str());
	}
	*/
//	m_TimeStampLogFile.open(format("%s/log_times.txt", m_LogOutDir.c_str()));
//	m_EstimatedPathLogFile.open(format("%s/log_estimated_path.txt", m_LogOutDir.c_str()));
//	m_OdometryPathLogFile.open(format("%s/log_odometry_path.txt", m_LogOutDir.c_str()));

	/*
#if MRPT_HAS_WXWIDGETS
	if (m_ShowProgress3DRealTime)
	{
		m_3DWindow = CDisplayWindow3D::Create("ICP-SLAM @ MRPT C++ Library", 600, 500);
		m_3DWindow->setCameraZoom(20);
		m_3DWindow->setCameraAzimuthDeg(-45);
	}
#endif
	*/

	return true;
}

bool MapBuilder_MRPT::addPose(const ssr::Pose2D& deltaPose)
{
  mrpt::obs::CActionRobotMovement2D action;
  mrpt::obs::CActionRobotMovement2D::TMotionModelOptions options;
	action.computeFromOdometry(CPose2D(deltaPose.x, deltaPose.y, deltaPose.th), options);
	action.timestamp = mrpt::system::getCurrentTime();
	static TTimeStamp oldTimestamp;
	if(action.timestamp == oldTimestamp) {
		action.timestamp = oldTimestamp +1;
	}
	oldTimestamp = action.timestamp;
	m_ActionCollection.insert(action);
	return true;
}

bool MapBuilder_MRPT::addRange(const ssr::Range& range)
{
	mrpt::obs::CObservation2DRangeScanPtr observation = mrpt::obs::CObservation2DRangeScan::Create();
	observation->rightToLeft = true;
	observation->validRange.resize(range.size);
	observation->scan.resize(range.size);
	observation->aperture = range.aperture;
	observation->timestamp = mrpt::system::getCurrentTime();
	for(int i = 0;i < range.size; i++) {
		observation->scan[i] = range.range[i];
		//if(observation->scan[i] > m_range_min && observation->scan[i] < m_range_max) {
			observation->validRange[i] = 1;
		//} else {
		//	observation->validRange[i] = 0;
		//}
	}
	m_RangeSensorPose.x(range.offset.x);
	m_RangeSensorPose.y(range.offset.y);
	m_RangeSensorPose.z(range.offset.z);
	m_RangeSensorPose.setYawPitchRoll(range.offset.yaw, range.offset.pitch, range.offset.roll);
	observation->setSensorPose(m_RangeSensorPose);
	m_SensoryFrame.insert(observation);

	return true;
}

bool MapBuilder_MRPT::processMap()
{
	m_MapBuilder.processActionObservation(m_ActionCollection, m_SensoryFrame);
	m_SensoryFrame.clear();
	m_ActionCollection.clear();
	return true;
}

Pose2D MapBuilder_MRPT::getEstimatedPose()
{
	CPose3D robotPose;
	m_MapBuilder.getCurrentPoseEstimation()->getMean(robotPose);
	return Pose2D(robotPose.x(), robotPose.y(), robotPose.yaw());
}

void MapBuilder_MRPT::getCurrentMap(ssr::Map& map)
{
  mrpt::maps::CMultiMetricMap *pMap = m_MapBuilder.getCurrentlyBuiltMetricMap();
	if (pMap->m_gridMaps.size() == 0) {
		std::cerr << "[MRPT] No Grid Map Error" << std::endl;
		return;
	}

	int width = pMap->m_gridMaps[0]->getSizeX();
	int height = pMap->m_gridMaps[0]->getSizeY();
	//int height = pMap->m_gridMaps[0]->getSizeX();
	//int width = pMap->m_gridMaps[0]->getSizeY();

	float resolution = pMap->m_gridMaps[0]->getResolution();
	float xmax = pMap->m_gridMaps[0]->getXMax();
	float xmin = pMap->m_gridMaps[0]->getXMin();
	float ymax = pMap->m_gridMaps[0]->getYMax();
	float ymin = pMap->m_gridMaps[0]->getYMin();
	//float ymax = pMap->m_gridMaps[0]->getXMax();
	//float ymin = pMap->m_gridMaps[0]->getXMin();
	//float xmax = pMap->m_gridMaps[0]->getYMax();
	//float xmin = pMap->m_gridMaps[0]->getYMin();

	map.setSize(width, height, xmin/resolution, ymax/resolution);
	map.setResolution(pMap->m_gridMaps[0]->getResolution());
	for(int i = 0;i < height;i++) {
		for(int j = 0;j < width;j++) {
			map.setCell(j, (height-i-1), static_cast<uint8_t>(255 * pMap->m_gridMaps[0]->getCell(j, i)));
//			map.setCell((width-j-1), i, static_cast<uint8_t>(255 * pMap->m_gridMaps[0]->getCell(i, j)));
//			map.setCell(j, (height-1-i), static_cast<uint8_t>(255 * pMap->m_gridMaps[0]->getCell(i, j)));
		}
	}
}

int32_t MapBuilder_MRPT::startMapping()
{
	this->m_MapBuilder.enableMapUpdating(true);
	this->m_isMapping = true;
	return 0;
}

int32_t MapBuilder_MRPT::stopMapping()
{
	this->m_MapBuilder.enableMapUpdating(false);
	this->m_isMapping = false;
	return 0;
}


void MapBuilder_MRPT::setCurrentMap(const ssr::Map& map)
{

}
