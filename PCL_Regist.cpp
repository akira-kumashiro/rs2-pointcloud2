#include "PCL_Regist.h"


PCL_Regist::PCL_Regist(double _transformationEpsilon, double _maxCorrespondenceDistance, int _maximumIterations, int _loopNum, double _leafSize = 0.0) :
	param(_transformationEpsilon, _maxCorrespondenceDistance, _maximumIterations, _loopNum, _leafSize),
	transformMat(Eigen::Matrix4f::Identity())
{

}

PCL_Regist::~PCL_Regist()
{
}

Eigen::Matrix4f PCL_Regist::getTransformMatrix(const PointCloud::Ptr cloud_source, const PointCloud::Ptr cloud_target, Eigen::Matrix4f prevTransformation = Eigen::Matrix4f::Identity())
{
	PointCloud::Ptr result(new PointCloud), source, target;
	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;

	source = cloud_source;
	target = cloud_target;

	PointCloud::Ptr temp(new PointCloud);
	PCL_INFO("Aligning (%d) with (%d).\n", source->points.size(), target->points.size());
	pairAlign(source, target, temp, pairTransform, prevTransformation);

	//transform current pair into the global transform
	//pcl::transformPointCloud(*temp, *result, GlobalTransform);

	//update the global transform
	//GlobalTransform = GlobalTransform * pairTransform;
	transformMat = pairTransform;
	return pairTransform;
}
Eigen::Matrix4f PCL_Regist::getTransformMatrix(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target, Eigen::Matrix4f prevTransformation = Eigen::Matrix4f::Identity())
{
	PointCloud::Ptr trans_source(new PointCloud), trans_target(new PointCloud);
	for (int i = 0; i < cloud_source->size(); i++)
	{
		pcl::PointXYZ point;
		point.x = cloud_source->points[i].x;
		point.y = cloud_source->points[i].y;
		point.z = cloud_source->points[i].z;
		trans_source->push_back(point);
	}
	for (int i = 0; i < cloud_target->size(); i++)
	{
		pcl::PointXYZ point;
		point.x = cloud_target->points[i].x;
		point.y = cloud_target->points[i].y;
		point.z = cloud_target->points[i].z;
		trans_target->push_back(point);
	}
	return getTransformMatrix(trans_source, trans_target, prevTransformation);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCL_Regist::transformPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*input, *output, transformMat);
	return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCL_Regist::transformPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, Eigen::Matrix4f transMat)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*input, *output, transMat);
	return output;
}

Eigen::Matrix4f PCL_Regist::calcTransformMatrix(float angle)
{
	float r = 0.35;
	float rad = angle * M_PI / 180.0;
	wprintf_s(L"%lf\n", rad);
	Eigen::Matrix4f matrix;
	matrix << std::cos(rad), 0.0, std::sin(rad), -r * std::sin(rad),
		0.0, 1.0, 0.0, 0.0,
		-std::sin(rad), 0.0, std::cos(rad), -r * (1.0 - std::cos(rad)),
		0.0, 0.0, 0.0, 1.0;

	transformMat = matrix;

	print4x4Matrix(matrix);

	return matrix;
}

void PCL_Regist::calcCenterOfGravity(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target, Eigen::Matrix4f transformMatrix)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedTagret(new pcl::PointCloud<pcl::PointXYZRGB>);
	transformedTagret = transformPointcloud(cloud_target, transformMatrix);

	PointCloud::Ptr source(new PointCloud), target(new PointCloud);

	for (const auto& source_point : *cloud_source)
	{
		PointT point;
		point.x = source_point.x;
		point.y = source_point.y;
		point.z = source_point.z;
		source->push_back(point);
	}
	for (const auto& source_point : *transformedTagret)
	{
		PointT point;
		point.x = source_point.x;
		point.y = source_point.y;
		point.z = source_point.z;
		target->push_back(point);
	}

	PointNormalT source_normal_average = calcAverage(source), target_normal_average = calcAverage(target);
	
	printf_s("source=(%lf,%lf,%lf),normal=(%lf,%lf,%lf)\n", source_normal_average.x, source_normal_average.y, source_normal_average.z, source_normal_average.normal_x, source_normal_average.normal_y, source_normal_average.normal_z);
	printf_s("target=(%lf,%lf,%lf),normal=(%lf,%lf,%lf)\n", target_normal_average.x, target_normal_average.y, target_normal_average.z, target_normal_average.normal_x, target_normal_average.normal_y, target_normal_average.normal_z);
}

void PCL_Regist::calcCenterOfGravity(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target)
{
	calcCenterOfGravity(cloud_source, cloud_target, transformMat);
}

void PCL_Regist::changeParam(double _transformationEpsilon, double _maxCorrespondenceDistance, int _maximumIterations, int _loopNum, double _leafSize)
{
	param = PCL_ResistParameters(_transformationEpsilon, _maxCorrespondenceDistance, _maximumIterations, _loopNum, _leafSize);
}

//(PCL_ResistParameters::VoxelGlid(0.0005), 1e-2, 0.2, 1000, 100);

void PCL_Regist::print4x4Matrix(const Eigen::Matrix4f & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
	double scale;
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			double buff = 1;
			for (int k = 0; k < 3; k++)
			{
				int temp = (j + (int)pow(-1, i) * k) % 3;
				buff *= matrix(k, temp >= 0 ? temp : temp + 3);
			}
			scale = scale + pow(-1, i) * buff;
		}
	}
	printf("|R|=%f\n", scale);
}

double PCL_Regist::random(double min, double max)
{
	std::random_device rnd;
	std::mt19937 engine(rnd());
	std::uniform_real_distribution<double> distribution(min, max);
	return distribution(engine);
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
* \param cloud_src the source PointCloud
* \param cloud_tgt the target PointCloud
* \param output the resultant aligned source PointCloud
* \param final_transform the resultant transform between source and target
*/
void PCL_Regist::pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, Eigen::Matrix4f prevTransformation)
{
	PointCloud::Ptr src(new PointCloud);
	PointCloud::Ptr tgt(new PointCloud);
	pcl::VoxelGrid<PointT> grid;

	auto nowTime = std::chrono::system_clock::now();

	if (param.leafSize > 0.0)
	{
		grid.setLeafSize(param.leafSize, param.leafSize, param.leafSize);
		grid.setInputCloud(cloud_src);
		grid.filter(*src);

		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);

		PCL_INFO("src_cloud(%d->%d),tgt_cloud(%d->%d)\n", cloud_src->size(), src->size(), cloud_tgt->size(), tgt->size());
	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
	}

	// Compute surface normals and curvature
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);

	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*src, *points_with_normals_src);

	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

	//
	// Instantiate our custom point representation (defined above) ...
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);

	//
	// Align
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	reg.setTransformationEpsilon(param.transformationEpsilon);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance(param.maxCorrespondenceDistance);//ëŒâûÇèúäOÇ∑ÇÈãóó£ÇÃËáíl
																	  // Set the point representation
	reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));

	reg.setInputSource(points_with_normals_src);
	reg.setInputTarget(points_with_normals_tgt);

	//
	// Run the same optimization in a loop and visualize the results
	//Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	Eigen::Matrix4f Ti = prevTransformation, prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	reg.setMaximumIterations(param.maximumIterations);//ç≈ëÂîΩïúâÒêî
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	for (int i = 0; i < param.loopNum; ++i)
	{
		PCL_INFO("Iteration Nr. %d.\n", i);

		// save cloud for visualization purpose
		points_with_normals_src = reg_result;

		// Estimate
		reg.setInputSource(points_with_normals_src);
		reg.align(*reg_result);

		//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation() * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
		PCL_INFO("transformation distance:%e", fabs((reg.getLastIncrementalTransformation() - prev).sum()));

		if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - random(0.0, param.maxCorrespondenceDistance / param.loopNum * 2));
		else
			reg.setMaxCorrespondenceDistance(param.maxCorrespondenceDistance);
		PCL_INFO(" score:%e correspondence distance:%lf\n", reg.getFitnessScore(), reg.getMaxCorrespondenceDistance());
		//fitness score ì_åQä‘ÇÃãóó£ÇÃ2èÊÇÃçáåv(à¯êîÇ≈ëŒâûä÷åWÇåvéZÇ∑ÇÈç≈ëÂãóó£ãóó£Çê›íËÇ≈Ç´ÇÈ defaultÇÕdoubleÇÃç≈ëÂíl)

		prev = reg.getLastIncrementalTransformation();

		// visualize current state
		//showCloudsRight(points_with_normals_tgt, points_with_normals_src);

		print4x4Matrix(Ti);

		//if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) == 0.0e+0)
		//{
		//	PCL_INFO("no difference between these pointcloud\n");
		//	reg.setMaxCorrespondenceDistance(param.maxCorrespondenceDistance);
		//	//break;
		//}
	}

	//
	// Get the transformation from target to source
	targetToSource = Ti.inverse();

	//
	// Transform target back in source frame
	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

	auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - nowTime);

	std::cout << "Process time:" << diff.count() << "[ms]" << std::endl;

	//PCL_INFO("Press q to continue the registration.\n");

	//add the source to the transformed target
	*output += *cloud_src;

	final_transform = targetToSource;
}

PointNormalT PCL_Regist::calcAverage(const PointCloud::Ptr cloud)
{
	// Compute surface normals and curvature
	PointCloudWithNormals::Ptr points_with_normals(new PointCloudWithNormals);

	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);

	norm_est.setInputCloud(cloud);
	norm_est.compute(*points_with_normals);
	pcl::copyPointCloud(*cloud, *points_with_normals);

	//pcl::PointXYZ source_CoG, target_CoG;
	PointNormalT normal_average;

	for (const auto& source_point : *points_with_normals)
	{
		normal_average.x += source_point.x / points_with_normals->size();
		normal_average.y += source_point.y / points_with_normals->size();
		normal_average.z += source_point.z / points_with_normals->size();
		if (isNan(source_point.normal_x / points_with_normals->size()) == 0.0f)
			printf("n_x=%lf,%lf\n", source_point.normal_x, points_with_normals->size());
		else
			normal_average.normal_x += source_point.normal_x / points_with_normals->size();

		if (isNan(source_point.normal_y / points_with_normals->size()) == 0.0f)
			printf("n_y=%lf,%lf\n", source_point.normal_y, points_with_normals->size());
		else
			normal_average.normal_y += source_point.normal_y / points_with_normals->size();

		if (isNan(source_point.normal_z / points_with_normals->size()) == 0.0f)
			printf("n_z=%lf,%lf\n", source_point.normal_z, points_with_normals->size());
		else
			normal_average.normal_z += source_point.normal_z / points_with_normals->size();
		//normal_average.normal_y += isNan(source_point.normal_y / points_with_normals->size());
		//normal_average.normal_z += isNan(source_point.normal_z / points_with_normals->size());
	}
	return normal_average;
}

inline float PCL_Regist::isNan(float num)
{
	if (isnan(num))
		return 0.0f;
	else
		return num;
}
