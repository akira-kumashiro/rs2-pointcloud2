#include <chrono>
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

#include <Windows.h>
#include <vector>
#include <string>
#include <stdexcept>

//https://qiita.com/tkymx/items/f9190c16be84d4a48f8a
std::vector<std::string> get_file_path_in_dir(const std::string& dir_name, const std::string& extension) noexcept(false)
{
	HANDLE hFind;
	WIN32_FIND_DATA win32fd;//defined at Windwos.h
	std::vector<std::string> file_names;

	//ägí£éqÇÃê›íË
	std::string search_name = dir_name + "\\*." + extension;

	hFind = FindFirstFile(search_name.c_str(), &win32fd);

	if (hFind == INVALID_HANDLE_VALUE) {
		throw std::runtime_error("file not found");
	}

	do {
		if (win32fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
		}
		else {
			file_names.push_back(win32fd.cFileName);
			printf("%s\n", file_names.back().c_str());

		}
	} while (FindNextFile(hFind, &win32fd));

	FindClose(hFind);

	return file_names;
}

int main(int argc, char** argv)
{
	//// Load data
	//std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
	//loadData(argc, argv, data);
	std::map<std::string, std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> data;

	// Check user input
	if (data.empty())
	{
		PCL_ERROR("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
		PCL_ERROR("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
		return (-1);
	}

	param.showParameters();

	PCL_INFO("Loaded %d datasets.", (int)data.size());

	// Create a PCLVisualizer object
	p = new pcl::visualization::PCLVisualizer(argc, argv, "Pairwise Incremental Registration example");
	p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
	p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);

	PointCloud::Ptr result(new PointCloud), source, target;
	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;

	for (size_t i = 1; i < data.size(); ++i)
	{
		source = data[i - 1].cloud;
		target = data[i].cloud;

		// Add visualization data
		showCloudsLeft(source, target);

		PointCloud::Ptr temp(new PointCloud);
		PCL_INFO("Aligning %s (%d) with %s (%d).\n", data[i - 1].f_name.c_str(), source->points.size(), data[i].f_name.c_str(), target->points.size());
		pairAlign(source, target, temp, pairTransform, true);

		//transform current pair into the global transform
		pcl::transformPointCloud(*temp, *result, GlobalTransform);

		//update the global transform
		GlobalTransform = GlobalTransform * pairTransform;

		//save aligned pair, transformed into the first cloud's frame
		std::stringstream ss;
		ss << i << ".pcd";
		pcl::io::savePCDFile(ss.str(), *result, true);

	}
}