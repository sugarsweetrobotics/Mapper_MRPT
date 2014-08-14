#pragma once

#include <stdint.h>
#include <string>

#define _USE_MATH_DEFINES
#include <math.h>

#include <map>
#include <string>
#include <algorithm>
#include <exception>

#include <sstream>

namespace ssr {

	/**
	*
	*/
	class Position3D {
	public:
		Position3D(double X, double Y, double Z, double Roll, double Pitch, double Yaw) {
			x = X; y = Y; z = Z;
			roll = Roll; pitch = Pitch; yaw = Yaw;
		}

	public:
		double x;
		double y;
		double z;

		double roll;
		double pitch;
		double yaw;
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
			memcpy(this->range, Range, Size*sizeof(double));
		}
	};

	class IndexOutOfRangeException : public ::std::exception {
	public:
	};

	/**
	*
	*/
	class Map {
	private:
		double m_Resolution;
		int32_t m_XMax, m_XMin;
		int32_t m_YMax, m_YMin;

		uint8_t *m_pGrid;

	public:
		Map() : m_Resolution(0), m_XMax(0), m_XMin(0), m_YMax(0), m_YMin(0), m_pGrid(NULL) {

		}

		/*
		Map(double resolution, uint32_t width, uint32_t height) : m_Resolution(resolution), m_Width(width), m_Height(height) {
		if(width == 0 || height == 0) {
		m_pGrid = NULL;
		}
		m_pGrid = new uint8_t[width*height];
		}*/

		~Map() {delete m_pGrid;}

	public:
		double getResolution() const { return m_Resolution; }
		uint32_t getWidth() const { return m_XMax-m_XMin; }
		uint32_t getHeight() const { return m_YMax - m_YMin; }
		int32_t getOriginX() const { return getWidth() - m_XMax; }
		int32_t getOriginY() const { return getHeight() - m_YMax; }

		uint8_t getCell(const uint32_t x, const uint32_t y) {
			if (x >= getWidth() || y >= getHeight()) {
				throw IndexOutOfRangeException();
			}
			return m_pGrid[y*getWidth()+x];
		}

		void setResolution(const double resolution) {m_Resolution = resolution;}

		void setSize(const uint32_t w, const uint32_t h, const uint32_t origin_x, const uint32_t origin_y) { 
			if (getWidth() != w || getHeight() != h) {
				delete m_pGrid;
				m_pGrid = new uint8_t[w*h];
				m_XMax = w - origin_x;
				m_XMin = m_XMax - w;
				m_YMax = h - origin_y;
				m_YMin = m_YMax - h;
			}
		}

		void setCell(const uint32_t x, const uint32_t y, const uint8_t value) {
			if (x >= getWidth() || y >= getHeight()) {
				throw IndexOutOfRangeException();
			}
			m_pGrid[y*getWidth()+x] = value;
		}



		//virtual bool load(const std::string& inputFileName) = 0;

		//virtual bool save(const std::string& outputFileName) = 0;
	};

	/**
	*
	*/
	class NamedString : public std::map<std::string, std::string>{
	public:
		NamedString() {}
		~NamedString() {}

		/*
		NamedString(const NamedString& ns) {
			
		}*/

	public:
		void setFloat(const char* key, const float value) {
			std::ostringstream oss;
			oss << value;
			this->operator[](key) = oss.str(); 
		}

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

		float getFloat(const char* key, const float defaultVal) {
			if (this->find(key) == this->end()) { return defaultVal; }

			std::string val = this->operator[](key);
			return (float)atof(val.c_str());
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
		virtual bool initialize(ssr::NamedString& namedString /*, ssr::Map* pMap=NULL*/) = 0;

		virtual bool setRangeSensorPosition(const ssr::Position3D& position) = 0;

		virtual bool addPose(const ssr::Pose2D& pose) = 0;

		virtual bool addRange(const ssr::Range& range) = 0;

		virtual bool processMap() = 0;

		virtual void log() = 0;

		virtual void setRangeSensorRange(float min, float max) = 0;

		virtual ssr::Pose2D getEstimatedPose() = 0;

		virtual void save() = 0;

		virtual void getCurrentMap(ssr::Map& map) = 0;

		virtual int32_t startMapping() = 0;

		virtual int32_t stopMapping() = 0;

		virtual void setCurrentMap(const ssr::Map& map) = 0;
	};

};