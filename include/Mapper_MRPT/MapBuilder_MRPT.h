#pragma once
#include "MapBuilder.h"

#include <string>

/*
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

#define TAG_LOCALIZATION_LIN_DIST "localizationLinDistance"
#define TAG_LOCALIZATION_ANG_DIST "localizationAngDistance"

#define TAG_INSERTION_LIN_DIST "insertionLinDistance"
#define TAG_INSERTION_ANG_DIST "insertionAngDistance"

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

#define DEFAULT_LOCALIZATION_LIN_DIST 0.5
#define DEFAULT_LOCALIZATION_ANG_DIST 0.8
#define DEFAULT_INSERTION_LIN_DIST 1.0
#define DEFAULT_INSERTION_ANG_DIST 1.0

#define DEFAULT_VERBOSE false
#define DEFAULT_ENABLE_LOGGING false 
*/



#define RADIANS(x) (x)/180.0*M_PI

namespace ssr {

	/*
	class Map_MRPT : public ssr::Map {
	private:
		mrpt::maps::CMultiMetricMap* m_pMap;
	public:
		Map_MRPT();
		Map_MRPT(mrpt::maps::CMultiMetricMap* pMap);
		~Map_MRPT();
	public:

	};
	*/

	class MapBuilderParam_MRPT : public MapBuilderParam {
	public:
		std::string ICP_algorithm; //!< icpClassic, icpLevenbergMarquardt, icpIKF

		bool	ICP_onlyClosestCorrespondences;  //!< The usual approach: to consider only the closest correspondence for each local point (Default to true)
		bool 	ICP_onlyUniqueRobust; //! Apart of "onlyClosestCorrespondences=true", if this option is enabled only the closest correspondence for each reference point will be kept (default=false).

		unsigned int	ICP_maxIterations;  //!< Maximum number of iterations to run.
		float           ICP_minAbsStep_trans; //!< If the correction in all translation coordinates (X,Y,Z) is below this threshold (in meters), iterations are terminated (Default:1e-6)
		float           ICP_minAbsStep_rot;   //!< If the correction in all rotation coordinates (yaw,pitch,roll) is below this threshold (in radians), iterations are terminated (Default:1e-6)

		float	ICP_thresholdDist, ICP_thresholdAng; //!< Initial threshold distance for two points to become a correspondence.
		float	ICP_ALFA;  //!< The scale factor for threshold everytime convergence is achieved.
		float	ICP_smallestThresholdDist;  //!< The size for threshold such that iterations will stop, since it is considered precise enough.

		/** This is the normalization constant \f$ \sigma^2_p \f$ that is used to scale the whole 3x3 covariance.
		*  This has a default value of \f$ (0.02)^2 \f$, that is, a 2cm sigma.
		*  See the paper: ....
		*/
		float	ICP_covariance_varPoints;

		bool	ICP_doRANSAC;  //!< Perform a RANSAC step after the ICP convergence, to obtain a better estimation of the pose PDF.

		/** RANSAC-step options: */
		unsigned int	ICP_ransac_minSetSize, ICP_ransac_maxSetSize, ICP_ransac_nSimulations;
		float			ICP_ransac_mahalanobisDistanceThreshold;

		/** RANSAC-step option: The standard deviation in X,Y of landmarks/points which are being matched (used to compute covariances in the SoG) */
		float			ICP_normalizationStd;
		bool			ICP_ransac_fuseByCorrsMatch;
		float			ICP_ransac_fuseMaxDiffXY, ICP_ransac_fuseMaxDiffPhi;

		/** Cauchy kernel rho, for estimating the optimal transformation covariance, in meters (default = 0.07m). */
		float			ICP_kernel_rho;

		/** Whether to use kernel_rho to smooth distances, or use distances directly (default=true) */
		bool			ICP_use_kernel;

		/** The size of the perturbance in x & y used to estimate the Jacobians of the square error (in LM & IKF methods, default=0.05).*/
		float			ICP_Axy_aprox_derivatives;

		/** The initial value of the lambda parameter in the LM method (default=1e-4). */
		float			ICP_LM_initial_lambda;

		/** Skip the computation of the covariance (saves some time) (default=false) */
		bool			ICP_skip_cov_calculation;

		/** Skip the (sometimes) expensive evaluation of the term 'quality' at ICP output (Default=true) */
		bool            ICP_skip_quality_calculation;

		/** Decimation of the point cloud being registered against the reference one (default=5) - set to 1 to have the older (MRPT <0.9.5) behavior
		*  of not approximating ICP by ignoring the correspondence of some points. The speed-up comes from a decimation of the number of KD-tree queries,
		*  the most expensive step in ICP.
		*/
		uint32_t        ICP_corresponding_points_decimation;


		/** (default:false) Match against the occupancy grid or the points map? The former is quicker but less precise. */
		bool	ICP_matchAgainstTheGrid;

		double ICP_insertionLinDistance;	//!< Minimum robot linear (m) displacement for a new observation to be inserted in the map.
		double ICP_insertionAngDistance;	//!< Minimum robot angular (rad, deg when loaded from the .ini) displacement for a new observation to be inserted in the map.
		double ICP_localizationLinDistance;	//!< Minimum robot linear (m) displacement for a new observation to be used to do ICP-based localization (otherwise, dead-reckon with odometry).
		double ICP_localizationAngDistance;//!< Minimum robot angular (rad, deg when loaded from the .ini) displacement for a new observation to be used to do ICP-based localization (otherwise, dead-reckon with odometry).

		double ICP_minICPgoodnessToAccept;  //!< Minimum ICP goodness (0,1) to accept the resulting corrected position (default: 0.40)


		float	MAP_min_x, MAP_max_x, MAP_min_y, MAP_max_y, MAP_resolution;	//!< See COccupancyGridMap2D::COccupancyGridMap2D



		/** The altitude (z-axis) of 2D scans (within a 0.01m tolerance) for they to be inserted in this map!
		*/
		float	MAP_insertion_mapAltitude;

		/** The parameter "mapAltitude" has effect while inserting observations in the grid only if this is true.
		*/
		bool	MAP_insertion_useMapAltitude;

		/** The largest distance at which cells will be updated (Default 15 meters)
		*/
		float	MAP_insertion_maxDistanceInsertion;

		/** A value in the range [0.5,1] used for updating cell with a bayesian approach (default 0.8)
		*/
		float	MAP_insertion_maxOccupancyUpdateCertainty;

		/** If set to true (default), invalid range values (no echo rays) as consider as free space until "maxOccupancyUpdateCertainty", but ONLY when the previous and next rays are also an invalid ray.
		*/
		bool	MAP_insertion_considerInvalidRangesAsFreeSpace;

		/** Specify the decimation of the range scan (default=1 : take all the range values!)
		*/
		uint16_t	MAP_insertion_decimation;

		/** The tolerance in rads in pitch & roll for a laser scan to be considered horizontal, then processed by calls to this class (default=0). */
		float MAP_insertion_horizontalTolerance;

		/** Gaussian sigma of the filter used in getAsImageFiltered (for features detection) (Default=1) (0:Disabled) */
		float	MAP_insertion_CFD_features_gaussian_size;

		/** Size of the Median filter used in getAsImageFiltered (for features detection) (Default=3) (0:Disabled) */
		float	MAP_insertion_CFD_features_median_size;

		bool	MAP_insertion_wideningBeamsWithDistance;	//!< Enabled: Rays widen with distance to approximate the real behavior of lasers, disabled: insert rays as simple lines (Default=false)

		double initial_pose_x;
		double initial_pose_y;
		double initial_pose_phi;

		void defaultSetup(void) {
			ICP_algorithm = "icpClassic";

			/* ICP_param */
			ICP_onlyClosestCorrespondences = true; // 1: Use the closest points only, 0: Use all the correspondences within the threshold (more robust sometimes, but slower)
			ICP_onlyUniqueRobust = false;

			ICP_maxIterations = 80;    // The maximum number of iterations to execute if convergence is not achieved before
			ICP_minAbsStep_trans = 1e-6f;  // If the correction in all translation coordinates (X,Y,Z) is below this threshold (in meters), iterations are terminated:
			ICP_minAbsStep_rot = 1e-6f;  // If the correction in all rotation coordinates (yaw,pitch,roll) is below this threshold (in radians), iterations are terminated:

			ICP_thresholdDist = 0.2f;   // Initial maximum distance for matching a pair of points
			ICP_thresholdAng = 0.1745f;//  RADIANS(10.0);     // An angular factor (in degrees) to increase the matching distance for distant points.
			ICP_ALFA = 0.8f;   // After convergence, the thresholds are multiplied by this constant and ICP keep running (provides finer matching)
			ICP_smallestThresholdDist = 0.05f; // This is the smallest the distance threshold can become after stopping ICP and accepting the result.


			ICP_covariance_varPoints = 0.0004f;

			ICP_doRANSAC = false;
			/** RANSAC-step options: */
			ICP_ransac_minSetSize = 5;
			ICP_ransac_maxSetSize = 20;
			ICP_ransac_nSimulations = 100;
			ICP_ransac_mahalanobisDistanceThreshold = 3.0;

			/** RANSAC-step option: The standard deviation in X,Y of landmarks/points which are being matched (used to compute covariances in the SoG) */
			ICP_normalizationStd = 0.2f;
			ICP_ransac_fuseByCorrsMatch = false;
			ICP_ransac_fuseMaxDiffXY = 0.01f;
			ICP_ransac_fuseMaxDiffPhi = 0.001745f;// RADIANS(0.1);

			ICP_kernel_rho = 0.07f;
			ICP_use_kernel = true;
			ICP_Axy_aprox_derivatives = 0.05f;
			ICP_LM_initial_lambda = 1e-4f;
			ICP_skip_cov_calculation = false;
			ICP_skip_quality_calculation = true;

			// decimation to apply to the point cloud being registered against the map
			// Reduce to "1" to obtain the best accuracy
			ICP_corresponding_points_decimation = 5;

			// Neeeded for LM method, which only supports point-map to point-map matching.
			ICP_matchAgainstTheGrid = 0;

			ICP_insertionLinDistance = 0.5; /// The distance threshold for inserting observations in the map (meters)
			ICP_insertionAngDistance = 0.8; /// The distance threshold for inserting observations in the map (rad)
			ICP_localizationLinDistance = 0.5; /// The distance threshold for correcting odometry with ICP (meters)  
			ICP_localizationAngDistance = 0.8; /// The distance threshold for correcting odometry with ICP (rad)

			ICP_minICPgoodnessToAccept = 0.40;	// Minimum ICP quality to accept correction [0,1].

			MAP_max_x = 10.0f;
			MAP_min_x = -10.0f;
			MAP_max_y = 10.0f;
			MAP_min_y = -10.0f;
			MAP_resolution = 0.05f;

			MAP_insertion_mapAltitude = 0.0f;
			MAP_insertion_useMapAltitude = false;
			MAP_insertion_maxDistanceInsertion = 25;
			MAP_insertion_maxOccupancyUpdateCertainty = 0.55f;
			MAP_insertion_considerInvalidRangesAsFreeSpace = true;
			MAP_insertion_wideningBeamsWithDistance = false;

			initial_pose_x = initial_pose_y = initial_pose_phi = 0;
		}


		public:
			MapBuilderParam_MRPT() {
				defaultSetup();
			}

			virtual ~MapBuilderParam_MRPT() {
			}
	};

	
	ssr::MapBuilder* createMapBuilder();

};
