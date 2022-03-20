#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
using namespace std;

// �r���[���[�N�����̈�񂾂��Ă΂��
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0.2, 0.2, 0.2);
	cout << "viewerOneOff" << std::endl;
}

// �r���[���[�N�����̖��t���[�����s�����
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	cout << "viewerPsycho" << std::endl;
}


int main()
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	// �쐬����PointCloud��ǂݍ���
	// pcl::io::loadPCDFile("p_cloud_ascii.pcd", *p_cloud);
	//pcl::io::loadPCDFile("p_cloud_binary.pcd", *p_cloud);
	pcl::io::loadPCDFile("D:\\M2\\pcd_file\\1647487726.528030872.pcd", *p_cloud);

	// �r���[���[�̍쐬
	pcl::visualization::CloudViewer viewer("PointCloudViewer");
	viewer.showCloud(p_cloud);

	// �r���[���[�N�����̈�񂾂��Ă΂��֐����Z�b�g
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	// �r���[���[�N�����̖��t���[�����s�����֐����Z�b�g
	viewer.runOnVisualizationThread(viewerPsycho);

	// �r���[���[�����p���[�v
	while (!viewer.wasStopped())
	{
		
	}
	return 0;
}