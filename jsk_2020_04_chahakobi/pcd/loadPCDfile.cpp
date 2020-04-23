#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
using namespace std;

// ビューワー起動時の一回だけ呼ばれる
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0.2, 0.2, 0.2);
	cout << "viewerOneOff" << std::endl;
}

// ビューワー起動中の毎フレーム実行される
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	cout << "viewerPsycho" << std::endl;
}


int main()
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	// 作成したPointCloudを読み込む
	// pcl::io::loadPCDFile("p_cloud_ascii.pcd", *p_cloud);
	pcl::io::loadPCDFile("sample_pcd_93662000.pcd", *p_cloud);

	// ビューワーの作成
	pcl::visualization::CloudViewer viewer("PointCloudViewer");
	viewer.showCloud(p_cloud);

	// ビューワー起動時の一回だけ呼ばれる関数をセット
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	// ビューワー起動中の毎フレーム実行される関数をセット
	viewer.runOnVisualizationThread(viewerPsycho);

	// ビューワー視聴用ループ
	while (!viewer.wasStopped())
	{

	}
	return 0;
}
