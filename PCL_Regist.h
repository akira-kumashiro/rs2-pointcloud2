#pragma once

#include <chrono>
#include <random>
#include <conio.h>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#define _USE_MATH_DEFINES
#include <math.h>

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


class PCL_Regist
{
private:
	class PCL_ResistParameters
	{
	public:
		double transformationEpsilon, maxCorrespondenceDistance, leafSize;
		int maximumIterations, loopNum;

		PCL_ResistParameters(double _transformationEpsilon, double _maxCorrespondenceDistance, int _maximumIterations, int _loopNum, double _leafSize)
		{
			transformationEpsilon = _transformationEpsilon;
			maxCorrespondenceDistance = _maxCorrespondenceDistance;
			maximumIterations = _maximumIterations;
			loopNum = _loopNum;
			leafSize = _leafSize;
		}

		void showParameters(void)
		{
			PCL_INFO("transformationEpsilon=%lf\n", transformationEpsilon);
			PCL_INFO("maxCorrespondenceDistance=%lf\n", maxCorrespondenceDistance);
			PCL_INFO("maximumIterations=%d\n", maximumIterations);
			PCL_INFO("loopNum=%d\n", loopNum);
			PCL_INFO("leafSize=%lf\n", leafSize);
		}
	};

	PCL_ResistParameters param;

	struct PCD
	{
		PointCloud::Ptr cloud;
		std::string f_name;

		PCD() : cloud(new PointCloud) {};
	};

	struct PCDComparator
	{
		bool operator () (const PCD& p1, const PCD& p2)
		{
			return (p1.f_name < p2.f_name);
		}
	};

	// Define a new point representation for < x, y, z, curvature >
	class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
	{
		using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
	public:
		MyPointRepresentation()
		{
			// Define the number of dimensions
			nr_dimensions_ = 4;
		}

		// Override the copyToFloatArray method to define our feature vector
		virtual void copyToFloatArray(const PointNormalT &p, float * out) const
		{
			// < x, y, z, curvature >
			out[0] = p.x;
			out[1] = p.y;
			out[2] = p.z;
			out[3] = p.curvature;
		}
	};

	Eigen::Matrix4f transformMat;

public:
	PCL_Regist(double _transformationEpsilon, double _maxCorrespondenceDistance, int _maximumIterations, int _loopNum, double _leafSize);
	~PCL_Regist();
	Eigen::Matrix4f getTransformMatrix(const PointCloud::Ptr cloud_source, const PointCloud::Ptr cloud_target, Eigen::Matrix4f prevTransformation);
	Eigen::Matrix4f getTransformMatrix(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target, Eigen::Matrix4f prevTransformation);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, Eigen::Matrix4f transMat);
	Eigen::Matrix4f calcTransformMatrix(float angle);
	void calcCenterOfGravity(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target, Eigen::Matrix4f transformMatrix);
	void calcCenterOfGravity(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target);
	void changeParam(double _transformationEpsilon, double _maxCorrespondenceDistance, int _maximumIterations, int _loopNum, double _leafSize);
private:
	//void loadData(int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models);
	void print4x4Matrix(const Eigen::Matrix4f & matrix);
	double random(double min, double max);
	void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, Eigen::Matrix4f prevTransformation);
	PointNormalT calcAverage(const PointCloud::Ptr cloud);
	inline float isNan(float num);
};