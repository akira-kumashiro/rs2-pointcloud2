#ifndef __MULTIREALSENSE__
#define __MULTIREALSENSE__

#include "realsense.h"
#include "PCL_Regist.h"

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include <vector>
#include <memory>

#include <windows.h>

class MultiRealSense
{
private:
	// RealSense
	std::map<std::string,std::unique_ptr<RealSense>> realsenses;

	//pclのviewer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	std::map<std::string,PCL_Regist> regist_tip, regist_near,regist_once;

	std::map<std::string,Eigen::Matrix4f> transformMat;
	std::map<std::string, Eigen::Matrix4f> transformMat_once;

	bool switchTransformation = false;
	
public:
	// Constructor
	MultiRealSense();

	// Destructor
	~MultiRealSense();

	// Processing
	void run();

private:
	// Initialize Sensor センサーの登録
	inline void initializeSensor(const rs2::device& device);

	// Initialize
	void initialize();

	//pclのviewerの初期設定
	inline void initializeViewer();

	// Finalize
	void finalize();

	bool keyboardCallBackSettings(int key);
	void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*);

	void setCameraAngle(void);

	enum
	{
		CV_WAITKEY_CURSORKEY_TOP = 2490368,
		CV_WAITKEY_CURSORKEY_BOTTOM = 2621440,
		CV_WAITKEY_CURSORKEY_RIGHT = 2555904,
		CV_WAITKEY_CURSORKEY_LEFT = 2424832,
	};

	//クラス内変数
	wchar_t directoryName[20];
	char nallowDirectoryName[20];
	std::string dataFileName;
	std::ofstream dataFile;
	inline std::string makeNameFolder(int hrgn);
	inline std::string makeNameFail(int hrgn, int num);
	inline void printText(int hrgn, int num);
	inline std::string getTime(void);
	void updateViewerText(void);

	const int numMax = 9; // 保存する一文字の数

	const std::string dataFolderName = "Data";

	int num = 0; // 番号格納用
	int hrgn = 0; // 文字格納用

	std::string _time;

	//どうでもいいWebカメラとかは全部この名前になる
	const std::string platformCameraName = "Platform Camera";
};

#endif // __MULTIREALSENSE__