#pragma once

#include <string>

#define _USE_MATH_DEFINES
#include <math.h>

#include <map>
#include <string>
#include <algorithm>


namespace ssr {

	/**
	 *
	 */
	class Position3D {
	public:
		Position3D(double X, double Y, double Z) {
			x = X; y = Y; z = Z;
		}

	public:
		double x;
		double y;
		double z;
	};

	/**
	 *
	 */
	class Pose2D {

	public:
		double x;
		double y;
		double th;

	public:

		Pose2D() {
			x = 0; y = 0; th = 0;
		}

		Pose2D(const double X, const double Y, const double Th) {
			this->x = X; this->y = Y; this->th = Th;
		}

		Pose2D(const Pose2D& pose) {
			this->x = pose.x; this->y = pose.y; this->th = pose.th;
		}

	};

	inline static ssr::Pose2D operator-(const ssr::Pose2D& pose1, const ssr::Pose2D& pose2) {
		double dx = pose1.x - pose2.x;
		double dy = pose1.y - pose2.y;
		double dth = pose1.th - pose2.th;
		if(dth > M_PI) {
			dth -= 2*M_PI;
		} else if(dth < -M_PI) {
			dth += 2*M_PI;
		}
		return ssr::Pose2D( dx*cos(pose2.th) + dy*sin(pose2.th), 
			-dx*sin(pose2.th) + dy*cos(pose2.th),
			dth);
	}


	/**
	 *
	 */
	class Range {
	public:
		double* range;
		int size;
		float aperture;
	public:
		Range(const double *Range, int Size, const float Aperture) {
			this->aperture = Aperture;
			this->size = Size;
			this->range = new double[size];
			memcpy(this->range, Range, Size);
		}
	};

	/**
	 *
	 */
	class Map {
	public:

		virtual bool load(const std::string& inputFileName) = 0;

		virtual bool save(const std::string& outputFileName) = 0;
	};

	/**
	 *
	 */
	class NamedString : public std::map<std::string, std::string>{
	public:
		NamedString() {}
		~NamedString() {}

	public:
		std::string getString(const char* key, const char* defaultVal) {
			if (find(key) == this->end()) { return defaultVal; }
			return this->operator[](key);
		}

		int getInt(const char* key, const int defaultVal)  {
			if (this->find(key) == this->end()) { return defaultVal; }
			return atoi(this->operator[](key).c_str());
		}

		bool getBool(const char* key, const bool defaultVal) {
			if (this->find(key) == this->end()) { return defaultVal; }
			
			std::string val = this->operator[](key);
			std::transform(val.begin(), val.end(), val.begin(), ::tolower);
			if (val == "true") return true;
			else return false;
		}
	};


	/**
	 *
	 */
	class MapBuilder {
	public:
		MapBuilder() {}
		virtual ~MapBuilder() {}

	public:
		virtual bool initialize( NamedString& namedString) = 0;

		virtual bool setRangeSensorPosition(const ssr::Position3D& position) = 0;

		virtual bool addPose(const ssr::Pose2D& pose) = 0;

		virtual bool addRange(const ssr::Range& range) = 0;

		virtual bool processMap() = 0;

		virtual void log() = 0;

		virtual void setRangeSensorRange(float min, float max) = 0;

		virtual ssr::Pose2D getEstimatedPose() = 0;

		virtual void save() = 0;
	};

};