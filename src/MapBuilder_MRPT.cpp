#include "MapBuilder_MRPT.h"

#ifdef HAVE_INTTYPES_H
#undef HAVE_INTTYPES_H
#endif

#include <mrpt/base.h>
#include <mrpt/obs.h>


using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::math;

using namespace ssr;


#define RADIANS(x) (x)/180.0*M_PI

Map_MRPT::Map_MRPT()
{
	m_pMap = NULL;
}

Map_MRPT::Map_MRPT(CMultiMetricMap* pMap)
{
	m_pMap = new CMultiMetricMap(*pMap);
}

Map_MRPT::~Map_MRPT()
{
	if(m_pMap != NULL) {
		delete m_pMap; m_pMap = NULL;
	}
}

bool Map_MRPT::load(const std::string& fileName)
{
	m_pMap = new CMultiMetricMap();

	std::string simpleMapExt = ".simplemap";
	if(fileName.compare(fileName.length() - simpleMapExt.length(), simpleMapExt.length(), simpleMapExt) == 0) {
		CFileInputStream simpleMapFile(fileName);
		CSimpleMap map;
		simpleMapFile >> map;
		m_pMap->loadFromSimpleMap(map);
	}

	std::string gridMapExt = ".gridmap";
	if(fileName.compare(fileName.length() - gridMapExt.length(), gridMapExt.length(), gridMapExt) == 0) {
		CFileInputStream gridMapFile(fileName);
		COccupancyGridMap2D map;
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


MapBuilder_MRPT::MapBuilder_MRPT()
{
	this->m_range_max = 20.0;
	this->m_range_min = 0.10;
	this->m_isMapping = false;
}

MapBuilder_MRPT::~MapBuilder_MRPT()
{

}


bool MapBuilder_MRPT::initialize(ssr::NamedString& parameter) 
{
	m_ActionCollection.clear();
	m_SensoryFrame.clear();

	m_RawLogOffset = parameter.getInt(TAG_RAWLOGOFFSET, DEFAULT_RAWLOGOFFSET);
	m_LogOutDir = parameter.getString(TAG_LOGOUTPUT_DIR, DEFAULT_LOGOUTDIR);
	m_LogFrequency = parameter.getInt(TAG_LOG_FREQUENCY, DEFAULT_LOGFREQ);
	m_SavePoseLog = parameter.getBool(TAG_SAVE_POSE_LOG, DEFAULT_SAVE_POSE_LOG);
	m_Save3DScene = parameter.getBool(TAG_SAVE_3D_SCENE, DEFAULT_SAVE_3D_SCENE);
	m_Camera3DSceneFollowsRobot = parameter.getBool(TAG_CAMERA_3DSCENE_FOLLOWS_ROBOT, DEFAULT_CAMERA_3DSCENE_FOLLOWS_ROBOT);
	m_ShowProgress3DRealTime = parameter.getBool(TAG_SHOW_PROGRESS_3D, DEFAULT_SHOW_PROGRESS_3D_REAL_TIME);
	m_Save3DScene = true;

	m_Logging = parameter.getBool(TAG_ENABLE_LOGGING, DEFAULT_ENABLE_LOGGING);
	m_Verbose = parameter.getBool(TAG_VERBOSE, DEFAULT_VERBOSE);

	/* ICP_param */
	m_MapBuilder.ICP_params.maxIterations    = 80;    // The maximum number of iterations to execute if convergence is not achieved before
	m_MapBuilder.ICP_params.minAbsStep_trans = 1e-6;  // If the correction in all translation coordinates (X,Y,Z) is below this threshold (in meters), iterations are terminated:
	m_MapBuilder.ICP_params.minAbsStep_rot   = 1e-6;  // If the correction in all rotation coordinates (yaw,pitch,roll) is below this threshold (in radians), iterations are terminated:

	m_MapBuilder.ICP_params.thresholdDist    = 0.3;   // Initial maximum distance for matching a pair of points
	m_MapBuilder.ICP_params.thresholdAng = RADIANS(5.0);     // An angular factor (in degrees) to increase the matching distance for distant points.

	m_MapBuilder.ICP_params.ALFA             = 0.8;   // After convergence, the thresholds are multiplied by this constant and ICP keep running (provides finer matching)

	m_MapBuilder.ICP_params.smallestThresholdDist=0.05; // This is the smallest the distance threshold can become after stopping ICP and accepting the result.
	m_MapBuilder.ICP_params.onlyClosestCorrespondences=true; // 1: Use the closest points only, 0: Use all the correspondences within the threshold (more robust sometimes, but slower)

	// 0: icpClassic
	// 1: icpLevenbergMarquardt
	m_MapBuilder.ICP_params.ICP_algorithm = icpClassic;

	// decimation to apply to the point cloud being registered against the map
	// Reduce to "1" to obtain the best accuracy
	m_MapBuilder.ICP_params.corresponding_points_decimation = 5;;

	m_MapBuilder.ICP_options.localizationLinDistance = 0.2;// The distance threshold for correcting odometry with ICP (meters)  
	m_MapBuilder.ICP_options.localizationAngDistance = 5;// The distance threshold for correcting odometry with ICP (degrees)


	m_MapBuilder.ICP_options.insertionLinDistance	= 1.2;	// The distance threshold for inserting observations in the map (meters)
	m_MapBuilder.ICP_options.insertionAngDistance	= 45.0;	// The distance threshold for inserting observations in the map (degrees)

	m_MapBuilder.ICP_options.minICPgoodnessToAccept	= 0.40;	// Minimum ICP quality to accept correction [0,1].

// Neeeded for LM method, which only supports point-map to point-map matching.
	m_MapBuilder.ICP_options.matchAgainstTheGrid = 0;

	mrpt::slam::TMetricMapInitializer i;
	i.metricMapClassType = CLASS_ID( COccupancyGridMap2D );
	i.occupancyGridMap2D_options.max_x = parameter.getFloat(TAG_MAP_MAX_X, DEFAULT_MAP_MAX_X);
	i.occupancyGridMap2D_options.max_y = parameter.getFloat(TAG_MAP_MAX_Y, DEFAULT_MAP_MAX_Y);
	i.occupancyGridMap2D_options.min_x = parameter.getFloat(TAG_MAP_MIN_X, DEFAULT_MAP_MIN_X);
	i.occupancyGridMap2D_options.min_y = parameter.getFloat(TAG_MAP_MIN_Y, DEFAULT_MAP_MIN_Y);
	i.occupancyGridMap2D_options.resolution = parameter.getFloat(TAG_MAP_RESOLUTION, DEFAULT_MAP_RESOLUTION);

	i.occupancyGridMap2D_options.insertionOpts.mapAltitude = 0;
	i.occupancyGridMap2D_options.insertionOpts.useMapAltitude = 0;
	i.occupancyGridMap2D_options.insertionOpts.maxDistanceInsertion = 25;
	i.occupancyGridMap2D_options.insertionOpts.maxOccupancyUpdateCertainty = 0.55;
	i.occupancyGridMap2D_options.insertionOpts.considerInvalidRangesAsFreeSpace = 1;
	i.occupancyGridMap2D_options.insertionOpts.wideningBeamsWithDistance = 0;

	i.occupancyGridMap2D_options.likelihoodOpts.likelihoodMethod = mrpt::slam::COccupancyGridMap2D::lmLikelihoodField_Thrun;
	i.occupancyGridMap2D_options.likelihoodOpts.LF_decimation = 5;
	i.occupancyGridMap2D_options.likelihoodOpts.LF_stdHit = 0.20;
	i.occupancyGridMap2D_options.likelihoodOpts.LF_maxCorrsDistance = 0.30;
	i.occupancyGridMap2D_options.likelihoodOpts.LF_zHit = 0.999;
	i.occupancyGridMap2D_options.likelihoodOpts.LF_zRandom = 0.001;
	i.occupancyGridMap2D_options.likelihoodOpts.LF_maxRange = 30;
	i.occupancyGridMap2D_options.likelihoodOpts.LF_alternateAverageMethod = 0;
	m_MapBuilder.ICP_options.mapInitializers.push_back(i);

	mrpt::slam::TMetricMapInitializer i2;
	i2.metricMapClassType = CLASS_ID( CSimplePointsMap );
	i2.pointsMapOptions_options.insertionOpts.minDistBetweenLaserPoints = 0.05;
	i2.pointsMapOptions_options.insertionOpts.fuseWithExisting = false;
	i2.pointsMapOptions_options.insertionOpts.isPlanarMap = 1;
	m_MapBuilder.ICP_options.mapInitializers.push_back(i2);

	m_MapBuilder.options.enableMapUpdating = false;

	double init_x = parameter.getFloat(TAG_INIT_X, DEFAULT_INIT_X);
	double init_y = parameter.getFloat(TAG_INIT_Y, DEFAULT_INIT_Y);
	double init_th = parameter.getFloat(TAG_INIT_TH, DEFAULT_INIT_TH);

	//m_MapBuilder.initialize();
	mrpt::poses::CPosePDFGaussian pose(mrpt::poses::CPose2D(init_x, init_y, init_th));
	mrpt::slam::CSimpleMap map;
	mrpt::slam::COccupancyGridMap2D gmap;
	//gmap.

	m_MapBuilder.initialize(mrpt::slam::CSimpleMap(), &pose);
	

	/* For Logging and Verbosity */
	m_MapBuilder.options.alwaysInsertByClass.fromString("");

	if(m_Verbose) {
		m_MapBuilder.options.verbose = true;
		m_MapBuilder.ICP_params.dumpToConsole();
		m_MapBuilder.ICP_options.dumpToConsole();
	}

	m_ActionCollection.clear();
	m_SensoryFrame.clear();

	if(m_Logging) {
		deleteFilesInDirectory(m_LogOutDir.c_str());
		createDirectory(m_LogOutDir.c_str());
		m_LogCount = 0;
		deleteFilesInDirectory(m_LogOutDir.c_str());
		createDirectory(m_LogOutDir.c_str());
	}

//	m_TimeStampLogFile.open(format("%s/log_times.txt", m_LogOutDir.c_str()));
//	m_EstimatedPathLogFile.open(format("%s/log_estimated_path.txt", m_LogOutDir.c_str()));
//	m_OdometryPathLogFile.open(format("%s/log_odometry_path.txt", m_LogOutDir.c_str()));

#if MRPT_HAS_WXWIDGETS
	if (m_ShowProgress3DRealTime)
	{
		m_3DWindow = CDisplayWindow3D::Create("ICP-SLAM @ MRPT C++ Library", 600, 500);
		m_3DWindow->setCameraZoom(20);
		m_3DWindow->setCameraAzimuthDeg(-45);
	}
#endif


	return true;
}

bool MapBuilder_MRPT::addPose(const ssr::Pose2D& deltaPose)
{
	CActionRobotMovement2D action;
	CActionRobotMovement2D::TMotionModelOptions options;
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
	CObservation2DRangeScanPtr observation = CObservation2DRangeScan::Create();
	observation->rightToLeft = true;
	observation->validRange.resize(range.size);
	observation->scan.resize(range.size);
	observation->aperture = range.aperture;
	observation->timestamp = mrpt::system::getCurrentTime();
	for(int i = 0;i < range.size; i++) {
		observation->scan[i] = range.range[i];
		if(observation->scan[i] > m_range_min && observation->scan[i] < m_range_max) {
			observation->validRange[i] = 1;
		} else {
			observation->validRange[i] = 0;
		}
	}
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

void MapBuilder_MRPT::log()
{
	if (m_Logging) {
	if(m_LogCount % this->m_LogFrequency == 0) {
		CPose3DPDFPtr poseEstimation = m_MapBuilder.getCurrentPoseEstimation();
		if(m_SavePoseLog) {
			poseEstimation->saveToTextFile( format("%s/mapbuilder_posepdf_%03d.txt", m_LogOutDir.c_str(), m_LogCount));
		}
	}
	}
	//m_EstimatedPathLogFile.printf("%i %f %f %f\n", m_LogCount, poseEstimation->getMeanVal().x(), poseEstimation->getMeanVal().y(), poseEstimation->getMeanVal().yaw());
	m_LogCount++;
}
void MapBuilder_MRPT::update3DWindow()
{
	if(m_3DWindow.present()) {
		CPose3D robotPose;
		m_MapBuilder.getCurrentPoseEstimation()->getMean(robotPose);

		COpenGLScenePtr scene = COpenGLScene::Create();
		COpenGLViewportPtr view = scene->getViewport("main");
		ASSERT_(view);

		COpenGLViewportPtr view_map = scene->createViewport("mini-map");
		view_map->setBorderSize(2);
		view_map->setViewportPosition(0.01, 0.01, 0.35, 0.35);
		view_map->setTransparent(false);

		CCamera &cam = view_map->getCamera();
		if(m_Camera3DSceneFollowsRobot) {
			scene->enableFollowCamera(true);
			cam.setAzimuthDegrees(-45);
			cam.setElevationDegrees(45);
		} else {
			cam.setAzimuthDegrees(-90);
			cam.setElevationDegrees(90);
		}
		cam.setPointingAt(robotPose);
		cam.setZoomDistance(20);
		cam.setOrthogonal();


		CGridPlaneXYPtr gridPlane =CGridPlaneXY::Create(-200, 200, -200, 200, 0, 5);
		gridPlane->setColor(0.4, 0.4, 0.4);
		view->insert(gridPlane);
		view_map->insert(CRenderizablePtr(gridPlane));

		CSetOfObjectsPtr objects = CSetOfObjects::Create();
		CMultiMetricMap* currentMap = m_MapBuilder.getCurrentlyBuiltMetricMap();
		currentMap->getAs3DObject(objects);
		view->insert(objects);

		CSetOfObjectsPtr pointMap = CSetOfObjects::Create();
		if(currentMap->m_pointsMaps.size() > 0) {
			currentMap->m_pointsMaps[0]->getAs3DObject(pointMap);
			view_map->insert(pointMap);
		}

		CSetOfObjectsPtr robo = stock_objects::RobotPioneer();
		robo->setPose(robotPose);
		view->insert(robo);

		CSetOfObjectsPtr robo_mini = stock_objects::RobotPioneer();
		robo_mini->setPose(robotPose);
		view_map->insert(robo_mini);

		if(m_3DWindow) {
			COpenGLScenePtr &scenePtr = m_3DWindow->get3DSceneAndLock();
			scenePtr = scene;
			m_3DWindow->unlockAccess3DScene();
			m_3DWindow->setCameraPointingToPoint(robotPose.x(), robotPose.y(), robotPose.z());
			m_3DWindow->forceRepaint();
		}
		
		// Save as file:
		if (m_Save3DScene)
		{
			static int step;
			if(step++ % 100) {
				CFileGZOutputStream	f( format( "%s/buildingmap_%05u.3Dscene", m_LogOutDir.c_str(), step ));
				f << *scene;
			}
		}


	}
}


void MapBuilder_MRPT::save(void)
{
	if (m_Logging) {
		CSimpleMap map;
		m_MapBuilder.getCurrentlyBuiltMap(map);


		std::string str = format("%s/_finalmap_.simplemap", m_LogOutDir.c_str());
		printf("Dumping final map in binary format to: %s\n", str.c_str() );
		m_MapBuilder.saveCurrentMapToFile(str);

		CMultiMetricMap  *finalPointsMap = m_MapBuilder.getCurrentlyBuiltMetricMap();
		str = format("%s/_finalmaps_.txt", m_LogOutDir.c_str());
		printf("Dumping final metric maps to %s_XXX\n", str.c_str() );
		finalPointsMap->saveMetricMapRepresentationToFile( str );

		if(finalPointsMap->m_gridMaps.size() != 0) {
			///finalPointsMap->
			str = format("%s/_finalmaps_.gridmap", m_LogOutDir.c_str());
			CFileOutputStream out_s(str);
			out_s << (finalPointsMap->m_gridMaps)[0];
			out_s.close();

			//(finalPointsMap->m_gridMaps)[0]->saveMetricMapRepresentationToFile ("map_rep_gridmap");
		}
	}
}

void MapBuilder_MRPT::getCurrentMap(ssr::Map& map)
{
	CMultiMetricMap *pMap = m_MapBuilder.getCurrentlyBuiltMetricMap();
	if (pMap->m_gridMaps.size() == 0) {
		std::cerr << "[MRPT] No Grid Map Error" << std::endl;
		return;
	}

	int width = pMap->m_gridMaps[0]->getSizeX();
	int height = pMap->m_gridMaps[0]->getSizeY();

	float resolution = pMap->m_gridMaps[0]->getResolution();
	float xmax = pMap->m_gridMaps[0]->getXMax();
	float xmin = pMap->m_gridMaps[0]->getXMin();
	float ymax = pMap->m_gridMaps[0]->getYMax();
	float ymin = pMap->m_gridMaps[0]->getYMin();

	map.setSize(width, height, -xmin/resolution, -ymin/resolution);
	map.setResolution(pMap->m_gridMaps[0]->getResolution());
	for(int i = 0;i < height;i++) {
		for(int j = 0;j < width;j++) {
			map.setCell(j, (height-1-i), static_cast<uint8_t>(255 * pMap->m_gridMaps[0]->getCell(i, j)));
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
