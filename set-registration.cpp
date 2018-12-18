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
#include <filesystem> // std::tr2::sys::path etc.

#include "PCL_Regist.h"

////https://qiita.com/tkymx/items/f9190c16be84d4a48f8a
//std::vector<std::string> get_file_path_in_dir(const std::string& dir_name, const std::string& extension) noexcept(false)
//{
//	HANDLE hFind;
//	WIN32_FIND_DATA win32fd;//defined at Windwos.h
//	std::vector<std::string> file_names;
//
//	//拡張子の設定
//	std::string search_name = dir_name + "\\*." + extension;
//
//	hFind = FindFirstFile(search_name.c_str(), &win32fd);
//
//	if (hFind == INVALID_HANDLE_VALUE) {
//		throw std::runtime_error("file not found");
//	}
//
//	do {
//		if (win32fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
//		}
//		else {
//			file_names.push_back(win32fd.cFileName);
//			printf("%s\n", file_names.back().c_str());
//
//		}
//	} while (FindNextFile(hFind, &win32fd));
//
//	FindClose(hFind);
//
//	return file_names;
//}

class PCD_Container
{
public:
	PCD_Container() :
		cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
		list()
	{

	}

	PCD_Container(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string data_type, const std::tr2::sys::path file_name) :
		cloud(cloud),
		list(file_name, data_type)
	{
		list.type = data_type;
	}

	void show_info(void)
	{
		std::cout << "file_name:" << list.file_name.string() << std::endl;
		std::cout << "cloud_num:" << std::to_string(cloud->size()) << std::endl;
		std::cout << "data_type:" << list.type << std::endl;
		std::cout << "data_serial_number:" << list.serial_number << std::endl;
		std::cout << "data_char:" << list.character << std::endl;
		std::cout << "data_num:" << std::to_string(list.num) << std::endl;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	class Data_List
	{
	public:
		std::string type;
		int num;
		char character;
		std::string serial_number;
		std::tr2::sys::path file_name;

		Data_List(const std::tr2::sys::path file_name, const std::string type) :
			file_name(file_name),
			type(type)
		{
			std::string temp = file_name.string();
			num = get_data_num(temp);
			character = get_data_char(temp);
			serial_number = get_data_serial_number(temp);
		}

		Data_List()
		{

		}
	private:
		char get_data_char(const std::string file_name)
		{
			return file_name.front();
		}

		int get_data_num(const std::string file_name)
		{
			return std::stoi(file_name.substr(2, 2));
		}

		std::string get_data_serial_number(const std::string file_name)
		{
			return file_name.substr(file_name.rfind('(') + 1, file_name.rfind(')') - file_name.rfind('(') - 1);
		}
	};

	Data_List list;
};

int main(int argc, char** argv)
{
	namespace sys = std::tr2::sys;
	std::map<std::string, std::map<std::string, std::vector< PCD_Container>>> data;
	std::string dataFolderPass;
	printf_s("data folder pass is:");
	std::cin >> dataFolderPass;
	std::map<std::string, std::vector<sys::path>> pass;

	std::vector<std::string> serial_numbers;

	std::map<std::string, Eigen::Matrix4f> transformationMat;
	std::map<std::string, PCL_Regist> regist;

	const std::vector<std::string> point_cloud_data_type = { "hand","tip" };

	//https://qiita.com/episteme/items/0e3c2ee8a8c03780f01e
	sys::path p(dataFolderPass); // 列挙の起点
	//std::for_each(sys::directory_iterator(p), sys::directory_iterator(),
		//  再帰的に走査するならコチラ↓
	std::for_each(sys::recursive_directory_iterator(p), sys::recursive_directory_iterator(),
		[&pass, point_cloud_data_type, &data, &serial_numbers](const sys::path& p)
	{
		const std::string point_cloud_data_extension = ".pcd";
		const std::string point_cloud_data_identifier = "PCL";

		if (sys::is_regular_file(p) && p.extension() == point_cloud_data_extension)
			if (p.string().find(point_cloud_data_identifier) != std::string::npos)
				for (const auto name : point_cloud_data_type)
					if (p.string().find(point_cloud_data_identifier + "_" + name) != std::string::npos)
					{
						pass[name].push_back(p);
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
						PCD_Container::Data_List list(p.filename(), name);
						std::cout << p.string() << std::endl;
						pcl::io::loadPCDFile(p.string(), *temp);
						data[name][list.serial_number].push_back(PCD_Container(temp, name, p.filename()));
						serial_numbers.push_back(list.serial_number);
					}
		//if (sys::is_regular_file(p))
		//{ // ファイルなら...
		//	std::cout << "file: " << p.filename() << std::endl;
		//}
		//else if (sys::is_directory(p))
		//{ // ディレクトリなら...
		//	std::cout << "dir.: " << p.string() << std::endl;
		//}
	});

	//for (const auto& name : point_cloud_data_type)
	//{
	//	std::cout << name << std::endl;
	//	for (const auto fullpass : pass.at(name))
	//	{
	//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	//		PCD_Container::Data_List list(fullpass.filename(), name);
	//		std::cout << fullpass.string() << std::endl;
	//		pcl::io::loadPCDFile(fullpass.string(), *temp);
	//		data[name][list.serial_number].push_back(PCD_Container(temp, name, fullpass.filename()));
	//		serial_numbers.push_back(list.serial_number);
	//	}
	//}

	std::sort(serial_numbers.begin(), serial_numbers.end());
	serial_numbers.erase(std::unique(serial_numbers.begin(), serial_numbers.end()), serial_numbers.end());

	for (const auto& name : point_cloud_data_type)
	{
		for (const auto& serial_number : serial_numbers)
		{
			for (auto& datum : data.at(name).at(serial_number))
			{
				datum.show_info();
			}
		}
	}

	//fileName = get_file_path_in_dir(dataFolderPass, "pcd");

	// Check user input
	//if (data.empty())
	//{
	//	PCL_ERROR("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
	//	PCL_ERROR("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
	//	return (-1);
	//}

	//param.showParameters();

	//PCL_INFO("Loaded %d datasets.", (int)data.size());

	//// Create a PCLVisualizer object
	//auto p = new pcl::visualization::PCLVisualizer(argc, argv, "Pairwise Incremental Registration example");
	//p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
	//p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);

	//PointCloud::Ptr result(new PointCloud), source, target;
	//Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;

	//for (size_t i = 1; i < data.size(); ++i)
	//{
	//	source = data[i - 1].cloud;
	//	target = data[i].cloud;

	//	// Add visualization data
	//	showCloudsLeft(source, target);

	//	PointCloud::Ptr temp(new PointCloud);
	//	PCL_INFO("Aligning %s (%d) with %s (%d).\n", data[i - 1].f_name.c_str(), source->points.size(), data[i].f_name.c_str(), target->points.size());
	//	pairAlign(source, target, temp, pairTransform, true);

	//	//transform current pair into the global transform
	//	pcl::transformPointCloud(*temp, *result, GlobalTransform);

	//	//update the global transform
	//	GlobalTransform = GlobalTransform * pairTransform;

	//	//save aligned pair, transformed into the first cloud's frame
	//	std::stringstream ss;
	//	ss << i << ".pcd";
	//	pcl::io::savePCDFile(ss.str(), *result, true);

	//}
}