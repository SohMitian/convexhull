#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/model_outlier_removal.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
cloud_voxel(new pcl::PointCloud<pcl::PointXYZRGB>),
cloud_false(new pcl::PointCloud<pcl::PointXYZRGB>)
;
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

//StatisticalOutlierRemoval関数
void stOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int mean, double threshold) {
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(mean);
	sor.setStddevMulThresh(threshold);
	sor.filter(*cloud);
}

//VoxelGrid filter関数
void voxGridFil(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out) {
	pcl::VoxelGrid<pcl::PointXYZRGB> vox;
	vox.setInputCloud(cloud);
	vox.setLeafSize(0.01f, 0.01f, 0.01f);
	vox.filter(*cloud_out);
}

//平面検出関数
void planeDetect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double threshold) {
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	//オプション
	seg.setOptimizeCoefficients(true);
	//必須
	seg.setInputCloud(cloud);
	seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);//モデル
	//seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);//モデル
	seg.setMethodType(pcl::SAC_RANSAC);//検出手法
	seg.setDistanceThreshold(threshold); //閾値 0.5とか
	seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
	seg.setEpsAngle(5 * (M_PI / 180));
	seg.segment(*inliers, *coefficients);
}

//ModelOutlierRemoval関数
void modelOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, pcl::ModelCoefficients::Ptr coefficients, double threshold, bool negative) {
	pcl::ModelOutlierRemoval<pcl::PointXYZRGB> mor;

	mor.setInputCloud(cloud);
	mor.setModelCoefficients(*coefficients);
	mor.setThreshold(threshold);
	mor.setModelType(pcl::SACMODEL_PLANE);
	
	mor.setNegative(negative);
	mor.filter(*cloud_out);
}

int
main(int argc, char** argv)
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;

	reader.read("./pcd/cloud_g1.pcd", *cloud);

	voxGridFil(cloud, cloud_voxel);
		
	planeDetect(cloud_voxel, 0.05);
	modelOutlierRemoval(cloud, cloud_false, coefficients, 0.05, false);
	writer.write("cloud_false.pcd", *cloud_false, false);

	 //Create a Concave Hull representation of the projected inliers
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ConcaveHull<pcl::PointXYZRGB> chull;
	chull.setInputCloud(cloud_false);
	chull.setAlpha(0.5);
	chull.reconstruct(*cloud_hull);

	
	modelOutlierRemoval(cloud, cloud, coefficients, 0.05, true);
	writer.write("cloud_true.pcd", *cloud, false);

	writer.write("g1_hull.pcd", *cloud_hull, false);

	return (0);
}