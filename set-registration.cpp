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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <Windows.h>
#include <vector>
#include <string>
#include <stdexcept>
#include <filesystem> // std::tr2::sys::path etc.

#include "PCL_Regist.h"
//#define DISABLE_AUTO_COLORING

#ifndef DISABLE_AUTO_COLORING
//#define VP_1_ORIGINAL_COLOR
//#define VP_2_ORIGINAL_COLOR
//#define VP_3_ORIGINAL_COLOR
#define VP_4_ORIGINAL_COLOR
#endif


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
		std::cout << "file name:" << list.file_name.string() << std::endl;
		std::cout << "cloud_num:" << std::to_string(cloud->size()) << std::endl;
		std::cout << "type:" << list.type << std::endl;
		std::cout << "serial number:" << list.serial_number << std::endl;
		std::cout << "data number:" << list.character << "-"; printf_s("%02d\n", list.num);
		//std::cout << "data_num:" << std::to_string(list.num) << std::endl;
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

void pointcloudShifter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double x, double y, double z)
{
	for (auto& point : *cloud)
	{
		point.x += x;
		point.y += y;
		point.z += z;
	}
}

int main(int argc, char** argv)
{
	namespace sys = std::tr2::sys;

	//point cloudの種類(point_cloud_data_type)->シリアルナンバー(s_n)->文字->番号
	std::map<std::string, std::map<std::string, std::map<char, std::map<int, PCD_Container>>>> data;

	//std::mapのアクセス
	//std::map<key,data> tempのとき
	//temp[key]←あればアクセス，なければ作成
	//temp.at(key)←あればアクセス，なければ例外
	//std::vector<std::map<std::string, std::map<std::string, PCD_Container>>> data;
	std::string dataFolderPass;
	printf_s("data folder pass is:");
	std::cin >> dataFolderPass;
	//std::map<std::string, std::vector<sys::path>> pass;

	//変換行列(シリアルナンバー(s_n)->文字->番号->行列の名前(matNames))
	std::map<std::string, std::map<char, std::map<int, std::map<std::string, Eigen::Matrix4f>>>> tMat;

	Eigen::Matrix4f farMat;
	farMat << 1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0;

	//変換用のクラス(シリアルナンバー(s_n)->文字->番号->行列の名前(matNames))
	std::map<std::string, std::map<char, std::map<int, std::map<std::string, PCL_Regist>>>> regist_hand_only, regist_tip, regist_hand_comp;

	std::vector<std::string> matNames = { "handだけ","tip->hand(欠損なし)","tip->hand(欠損なし)-temp","tip->hand(欠損あり)" };
	auto matNamesTip(matNames);
	matNamesTip.erase(matNamesTip.begin());

	//pclのviewer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer")); //viewerのコンストラクタ

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(0.01);
	viewer->initCameraParameters();

	int vp_1, vp_2, vp_3, vp_4;
	viewer->createViewPort(0.0, 0.0, 0.5, 0.5, vp_1);
	viewer->createViewPort(0.0, 0.5, 0.5, 1.0, vp_2);
	viewer->createViewPort(0.5, 0.0, 1.0, 0.5, vp_4);
	viewer->createViewPort(0.5, 0.5, 1.0, 1.0, vp_3);

	//カメラのシリアルナンバー
	std::vector<std::string> s_n;

	//文字+番号(ex:ｱ-01)がregistration可能かどうか(point cloudの種類(point_cloud_data_type)->文字->番号)
	std::map<std::string, std::map<char, std::map<int, bool>>> unique_numbers;

	const std::vector<std::string> point_cloud_data_type = { "hand","tip" };

	//https://qiita.com/episteme/items/0e3c2ee8a8c03780f01e
	sys::path p(dataFolderPass); // 列挙の起点
	std::for_each(sys::recursive_directory_iterator(p), sys::recursive_directory_iterator(),
		[point_cloud_data_type, &data, &s_n, &unique_numbers, matNames, &tMat, &regist_hand_only, &regist_tip, &regist_hand_comp](const sys::path& p)
	{
		const std::string point_cloud_data_extension = ".pcd";
		const std::string point_cloud_data_identifier = "PCL";

		if (sys::is_regular_file(p) && p.extension() == point_cloud_data_extension)
			if (p.string().find(point_cloud_data_identifier) != std::string::npos)
				for (const auto name : point_cloud_data_type)
					if (p.string().find(point_cloud_data_identifier + "_" + name) != std::string::npos)
					{
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
						PCD_Container::Data_List list(p.filename(), name);
						std::cout << p.string() << std::endl;
						pcl::io::loadPCDFile(p.string(), *temp);
						for (const auto& matName : matNames)
						{
							tMat[list.serial_number][list.character][list.num][matName] = Eigen::Matrix4f::Identity();
							regist_hand_only[list.serial_number][list.character][list.num].emplace(matName, PCL_Regist(1e-2, 0.2, 50, 100, 2.0e-3));
							if (matName != "handだけ")
							{
								regist_tip[list.serial_number][list.character][list.num].emplace(matName, PCL_Regist(1e-20, 1.0, 500, 20, 0.0));
								regist_hand_comp[list.serial_number][list.character][list.num].emplace(matName, PCL_Regist(1e-2, 0.05, 50, 10, 2.0e-3));
							}
						}
						if (temp->size() < 5)
						{
							std::cout << "\"" << p.filename().string() << "\" is too small!" << std::endl;
							unique_numbers[name][list.character][list.num] = false;
							continue;
						}
						data[name][list.serial_number][list.character][list.num] = PCD_Container(temp, name, p.filename());
						s_n.push_back(list.serial_number);
						unique_numbers[name][list.character][list.num] = true;
					}
	});
	//↓シリアルナンバーの重複の削除
	std::sort(s_n.begin(), s_n.end());
	s_n.erase(std::unique(s_n.begin(), s_n.end()), s_n.end());

	//for (auto& name : unique_numbers)
	//{
	//	if (!name.second.size())
	//		continue;
	//	for (auto& character : name.second)
	//	{
	//		if (!character.second.size())
	//			continue;
	//		if (!(data.at(name.first).count(serial_number)))
	//		{
	//			num.second = false;
	//			continue;
	//		}
	//		for (auto& num : character.second)
	//		{
	//			for (const auto& serial_number : s_n)
	//			{

	//				if (!(data.at(name.first).at(serial_number).count(character.first)))
	//				{
	//					num.second = false;
	//					continue;
	//				}
	//				if (!(data.at(name.first).at(serial_number).at(character.first).count(num.first)))
	//					num.second = false;
	//			}
	//		}
	//	}
	//}


	{//tempを外に出したくないがためのかっこ
		auto temp = unique_numbers;
		for (auto&name : temp)//種類
		{
			for (auto& c : name.second)
			{
				for (auto& n : c.second)
				{
					if (!(n.second))//欠けてる番号があるとき
					{
						for (const auto& sn : s_n)
						{
							if (data.at(name.first).count(sn))if (data.at(name.first).at(sn).count(c.first))if (data.at(name.first).at(sn).at(c.first).count(n.first))
								data.at(name.first).at(sn).at(c.first).erase(n.first);

							for (const auto& matName : matNames)
							{
								if (matName == "handだけ")
									continue;
								else
								{
									if (tMat.at(sn).count(c.first))if (tMat.at(sn).at(c.first).count(n.first))if (tMat.at(sn).at(c.first).at(n.first).count(matName))
										tMat.at(sn).at(c.first).at(n.first).erase(matName);
									if (regist_tip.at(sn).count(c.first))if (regist_tip.at(sn).at(c.first).count(n.first))if (regist_tip.at(sn).at(c.first).at(n.first).count(matName))
										regist_tip.at(sn).at(c.first).at(n.first).erase(matName);
								}
							}
							if (name.first == "hand")
							{
								for (const auto& matName : matNames)
								{
									if (tMat.at(sn).at(c.first).at(n.first).count(matName))
										tMat.at(sn).at(c.first).at(n.first).erase(matName);
									if (regist_hand_only.at(sn).at(c.first).at(n.first).count(matName))
										regist_hand_only.at(sn).at(c.first).at(n.first).erase(matName);
									if (matName == "handだけ")
										continue;
									else if (regist_tip.at(sn).at(c.first).at(n.first).count(matName))
										regist_tip.at(sn).at(c.first).at(n.first).erase(matName);
								}
							}
						}
						unique_numbers.at(name.first).at(c.first).erase(n.first);
					}
					if (unique_numbers.at(name.first).at(c.first).size() == 0)
						unique_numbers.at(name.first).erase(c.first);
				}
				if (unique_numbers.at(name.first).size() == 0)
					unique_numbers.erase(name.first);
			}
		}
	}

	for (const auto& name : unique_numbers)
	{
		for (const auto& c : name.second)
		{
			for (const auto & n : c.second)
			{
				std::cout << name.first << c.first << n.first << std::to_string(n.second) << std::endl;
			}
			std::cout << name.first << c.first << std::to_string(c.second.size()) << std::endl;
		}
		std::cout << name.first << std::to_string(name.second.size()) << std::endl;
	}

	std::cout << "316" << std::endl;
	for (const auto& name : unique_numbers)
	{
		std::cout << "------------------" << name.first << "------------------" << std::endl;
		for (const auto& sn : s_n)
		{
			std::cout << sn << std::endl;
			for (const auto& c : name.second)
			{
				std::cout << c.first << std::endl;
				for (const auto& num : c.second)
				{

					std::cout << "前" << std::endl;
					data.at(name.first).at(sn).at(c.first).at(num.first).show_info();
					std::cout << "後" << std::endl;
					for (const auto& matName : matNames)
					{
						std::cout << matName << ":";
						if (tMat.at(sn).at(c.first).at(num.first).count(matName))
						{
							std::cout << "〇";
						}
						else
						{
							std::cout << "×";
						}
						std::cout << ",";
						if (regist_hand_only.at(sn).at(c.first).at(num.first).count(matName))
						{
							std::cout << "〇";
						}
						else
						{
							std::cout << "×";
						}
						std::cout << ",";
						if (regist_tip.at(sn).at(c.first).at(num.first).count(matName))
						{
							std::cout << "〇" << std::endl;
						}
						else
						{
							std::cout << "×" << std::endl;
						}
					}
				}
			}
			std::cout << std::endl;
		}
	}

	for (const auto& c : unique_numbers.at("hand"))
	{
		for (const auto& n : c.second)
		{
			if (unique_numbers.count("hand"))if (unique_numbers.at("hand").count(c.first))if (unique_numbers.at("hand").at(c.first).count(n.first))if (unique_numbers.at("hand").at(c.first).at(n.first))
			{
				for (int i = 1; i < s_n.size(); i++)
				{
					if (tMat.at(s_n[i]).at(c.first).at(n.first).count("handだけ"))
					{
						tMat.at(s_n[i]).at(c.first).at(n.first).at("handだけ") = farMat;

						tMat.at(s_n[i]).at(c.first).at(n.first).at("handだけ") = regist_hand_only.at(s_n[i]).at(c.first).at(n.first).at("handだけ").getTransformMatrix(data.at("hand").at(s_n[0]).at(c.first).at(n.first).cloud, data.at("hand").at(s_n[i]).at(c.first).at(n.first).cloud, tMat.at(s_n[i]).at(c.first).at(n.first).at("handだけ"));
					}
				}

				for (const auto& sn : s_n)
				{
					auto temp = sn;
					for (size_t c = temp.find_first_of("0"); c != std::string::npos; c = c = temp.find_first_of("0"))
					{
						temp.erase(c, 1);
					}
					std::cout << temp << std::endl;
					int serialNumber;
					if (temp.size() > 8)
						serialNumber = std::stoi(temp.substr(temp.size() - 8));
					else
						serialNumber = std::stoi(temp);
#ifndef DISABLE_AUTO_COLORING
					//int b = serialNumber % 256;
					//int g = ((serialNumber - b) / 256 % 256);
					//int r = ((serialNumber - g * 256 - b) / 256 / 256) % 256;
					int b = 0;
					int g = (sn == "611203000018" ? 255 : 0);
					int r = (sn != "611203000018" ? 255 : 0);
#endif

					//auto cloud = regist_hand_only.at(sn).at(c.first).at(n.first).at("handだけ").transformPointcloud(data.at("hand").at(sn).at(c.first).at(n.first).cloud);
					auto cloud = PCL_Regist::transformPointcloud(data.at("hand").at(sn).at(c.first).at(n.first).cloud, tMat.at(sn).at(c.first).at(n.first).at("handだけ"));

					viewer->removePointCloud("vp_1_" + sn);
#ifndef DISABLE_AUTO_COLORING
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> customColor(cloud, r, g, b);
#endif
#if !defined(VP_1_ORIGINAL_COLOR)&&!defined(DISABLE_AUTO_COLORING)
					viewer->addPointCloud(cloud, customColor, "vp_1_" + sn, vp_1);
#else
					viewer->addPointCloud(cloud, "vp_1_" + sn, vp_1);
#endif
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "vp_1_" + sn, vp_1);

					viewer->removePointCloud("vp_4_" + sn);
#ifndef DISABLE_AUTO_COLORING
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> customColorVP4(data.at("hand").at(sn).at(c.first).at(n.first).cloud, r, g, b);
#endif
#if !defined(VP_4_ORIGINAL_COLOR)&&!defined(DISABLE_AUTO_COLORING)
					viewer->addPointCloud(data.at("hand").at(sn).at(c.first).at(n.first).cloud, customColor, "vp_4_" + sn, vp_4);
#else
					viewer->addPointCloud(data.at("hand").at(sn).at(c.first).at(n.first).cloud, "vp_4_" + sn, vp_4);
#endif
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "vp_4_" + sn, vp_4);

					viewer->spinOnce();
				}

				std::cout << "handのみ" << std::endl;
			}

			if (unique_numbers.count("tip"))if (unique_numbers.at("tip").count(c.first))if (unique_numbers.at("tip").at(c.first).count(n.first))if (unique_numbers.at("tip").at(c.first).at(n.first))
			{
				for (int i = 1; i < s_n.size(); i++)
				{
					if (tMat.at(s_n[i]).at(c.first).at(n.first).count("tip->hand(欠損なし)"))
					{
						tMat.at(s_n[i]).at(c.first).at(n.first).at("tip->hand(欠損なし)") = farMat;
						tMat.at(s_n[i]).at(c.first).at(n.first).at("tip->hand(欠損なし)") = regist_tip.at(s_n[i]).at(c.first).at(n.first).at("tip->hand(欠損なし)").getTransformMatrix(data.at("tip").at(s_n[0]).at(c.first).at(n.first).cloud, data.at("tip").at(s_n[i]).at(c.first).at(n.first).cloud, tMat.at(s_n[i]).at(c.first).at(n.first).at("tip->hand(欠損なし)"));
						tMat.at(s_n[i]).at(c.first).at(n.first).at("tip->hand(欠損なし)-temp") = tMat.at(s_n[i]).at(c.first).at(n.first).at("tip->hand(欠損なし)");
						//regist_tip.at(s_n[i]).at(c.first).at(n.first).at("tip->hand(欠損なし)").changeParam(1e-5, 0.2, 300, 10, 2.0e-3);
						//tMat.at(s_n[i]).at(c.first).at(n.first).at("tip->hand(欠損なし)")*
						tMat.at(s_n[i]).at(c.first).at(n.first).at("tip->hand(欠損なし)") = regist_hand_comp.at(s_n[i]).at(c.first).at(n.first).at("tip->hand(欠損なし)").getTransformMatrix(data.at("hand").at(s_n[0]).at(c.first).at(n.first).cloud, data.at("hand").at(s_n[i]).at(c.first).at(n.first).cloud, tMat.at(s_n[i]).at(c.first).at(n.first).at("tip->hand(欠損なし)"));
					}
				}

				for (const auto& sn : s_n)
				{
					auto temp = sn;
					for (size_t c = temp.find_first_of("0"); c != std::string::npos; c = c = temp.find_first_of("0"))
					{
						temp.erase(c, 1);
					}
					std::cout << temp << std::endl;
					int serialNumber;
					if (temp.size() > 8)
						serialNumber = std::stoi(temp.substr(temp.size() - 8));
					else
						serialNumber = std::stoi(temp);
#ifndef DISABLE_AUTO_COLORING
					//int b = serialNumber % 256;
					//int g = ((serialNumber - b) / 256 % 256);
					//int r = ((serialNumber - g * 256 - b) / 256 / 256) % 256;
					int b = 0;
					int g = (sn == "611203000018" ? 255 : 0);
					int r = (sn != "611203000018" ? 255 : 0);
#endif
					auto cloudVP2 = regist_tip.at(sn).at(c.first).at(n.first).at("tip->hand(欠損なし)-temp").transformPointcloud(data.at("hand").at(sn).at(c.first).at(n.first).cloud, tMat.at(sn).at(c.first).at(n.first).at("tip->hand(欠損なし)-temp"));
					auto cloudVP2tip = regist_tip.at(sn).at(c.first).at(n.first).at("tip->hand(欠損なし)-temp").transformPointcloud(data.at("tip").at(sn).at(c.first).at(n.first).cloud, tMat.at(sn).at(c.first).at(n.first).at("tip->hand(欠損なし)-temp"));
#ifndef DISABLE_AUTO_COLORING
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> customColorVP2(cloudVP2, (int)r*0.5, (int)g*0.5, (int)b*0.5);
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> customColorTipVP2(cloudVP2tip, r, g, b);
#endif
					viewer->removePointCloud("vp_2_" + sn);
					viewer->removePointCloud("vp_2_tip_" + sn);
#if !defined(VP_2_ORIGINAL_COLOR)&&!defined(DISABLE_AUTO_COLORING)
					viewer->addPointCloud(cloudVP2, customColorVP2, "vp_2_" + sn, vp_2);
#else
					viewer->addPointCloud(cloudVP2, "vp_2_" + sn, vp_2);
#endif
#ifndef DISABLE_AUTO_COLORING
					viewer->addPointCloud(cloudVP2tip, customColorTipVP2, "vp_2_tip_" + sn, vp_2);
#endif
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.1, "vp_2_" + sn, vp_2);
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20.0, "vp_2_tip_" + sn, vp_2);

					auto cloudVP3 = regist_hand_comp.at(sn).at(c.first).at(n.first).at("tip->hand(欠損なし)").transformPointcloud(data.at("hand").at(sn).at(c.first).at(n.first).cloud, tMat.at(sn).at(c.first).at(n.first).at("tip->hand(欠損なし)"));
					auto cloudVP3tip = regist_hand_comp.at(sn).at(c.first).at(n.first).at("tip->hand(欠損なし)").transformPointcloud(data.at("tip").at(sn).at(c.first).at(n.first).cloud, tMat.at(sn).at(c.first).at(n.first).at("tip->hand(欠損なし)"));
#ifndef DISABLE_AUTO_COLORING
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> customColor(cloudVP3, r, g, b);
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> customColorTip(cloudVP3tip, r, g, b);
#endif
					viewer->removePointCloud("vp_3_" + sn);
					viewer->removePointCloud("vp_3_tip_" + sn);
#if !defined(VP_3_ORIGINAL_COLOR)&&!defined(DISABLE_AUTO_COLORING)
					viewer->addPointCloud(cloudVP3, customColor, "vp_3_" + sn, vp_3);
#else
					viewer->addPointCloud(cloudVP3, "vp_3_" + sn, vp_3);
#endif
#ifndef DISABLE_AUTO_COLORING
					viewer->addPointCloud(cloudVP3tip, customColorTip, "vp_3_tip_" + sn, vp_3);
#endif
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "vp_3_" + sn, vp_3);
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20.0, "vp_3_tip_" + sn, vp_3);

					viewer->removePointCloud("vp_1_tip_" + sn);
#ifndef DISABLE_AUTO_COLORING
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> customColorTipVP1(data.at("tip").at(sn).at(c.first).at(n.first).cloud, r, g, b);
					viewer->addPointCloud(regist_hand_only.at(sn).at(c.first).at(n.first).at("handだけ").transformPointcloud(data.at("tip").at(sn).at(c.first).at(n.first).cloud), customColorTipVP1, "vp_1_tip_" + sn, vp_1);
#else
					viewer->addPointCloud(regist_hand_only.at(sn).at(c.first).at(n.first).at("handだけ").transformPointcloud(data.at("tip").at(sn).at(c.first).at(n.first).cloud), "vp_1_tip_" + sn, vp_1);
#endif
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20.0, "vp_1_tip_" + sn, vp_1);

					viewer->removePointCloud("vp_4_tip_" + sn);
#ifndef DISABLE_AUTO_COLORING
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> customColorTipVP4(data.at("tip").at(sn).at(c.first).at(n.first).cloud, r, g, b);
					viewer->addPointCloud(data.at("tip").at(sn).at(c.first).at(n.first).cloud, customColorTipVP4, "vp_4_tip_" + sn, vp_4);
#else
					viewer->addPointCloud(data.at("tip").at(sn).at(c.first).at(n.first).cloud, "vp_4_tip_" + sn, vp_4);
#endif
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20.0, "vp_4_tip_" + sn, vp_4);
					viewer->spinOnce();
				}
			}
			std::cout << "press any key" << std::endl;
			while (!viewer->wasStopped())
			{
				viewer->spinOnce();
				if (_kbhit())
				{
					getch();
					break;
				}
			}
			return 0;
		}
	}
}