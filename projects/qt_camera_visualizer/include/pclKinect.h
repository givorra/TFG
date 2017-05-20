#pragma once

#include <iostream>
#include <vector>
#include "MathMorph.h"
#include "OFFReader.h"
#include "common.h"
#include <boost/thread.hpp>

// Qt
//#include <QMainWindow>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QTimer>
#include <QFileDialog>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d_omp.h>


// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#define SIZEI 0.5
#define LEAFSIZEI 0.001

enum ObjectType {none, pointcloud, polygonmesh};

namespace Ui
{
  class KinectViewer;
}

class KinectViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit KinectViewer (QWidget *parent = 0);
  ~KinectViewer ();

	// Point cloud callback
	void cloud_cb_(const MY_POINT_CLOUD::ConstPtr &cloud);	
	// Run Kinect
	int run();
	// Stop Kinect
	int stop();


private:
	string last_path;
	// Grabber
	pcl::Grabber* interface;    
	// Running
	bool bRun; 
	// Copying
	bool bCopying;   
	// Error
	int er;
	bool processing;
	//
	bool firstCall;
	bool firstCapture;
	bool firstMesh;
  	bool runCamera;
  	// Esta variable dice que tipo de objeto está cargado en el navegador
  	ObjectType objectType;

  	QTimer *tmrTimer;
  	Ui::KinectViewer *ui; 

  	// MathMorph object 
  	MathMorph* mathMorph;
  	// Current cloud
  	MY_POINT_CLOUD::Ptr cloudViewer_1, cloudViewer_2;
  	pcl::PolygonMesh meshViewer_1, meshViewer_2;

	void processCurrentCloud(MY_POINT_CLOUD::Ptr currentPointCloud);
	void setCameraViewer1Parameters();
	void initVisualizers();
	void initViewer1();
	void initViewer2();
	void showPointCloudViewer1(MY_POINT_CLOUD::Ptr cloud);
	void showMeshViewer2();
	void greedyReconstruction();
	void poissonReconstruction();
	string getFilePathDialog();
	void boxCounting(pcl::PointCloud<MY_POINT_TYPE>::Ptr);


protected:
  	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;  
  	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_2;  

public slots:
  void processFrameAndUpdateGUI();

private slots:
	void on_btnBoxCounting_toggled(bool checked);
  	void on_btnRunCamera_toggled(bool checked);
  	void on_btnDilateCloud_toggled(bool checked);
  	void on_btnDeleteOutliners_toggled(bool checked);
  	void on_btnBilateralFilter_toggled(bool checked);
	void on_btnErodeCloud_toggled(bool checked);
  	void on_btnTriangulateCloud_toggled(bool checked);
  	void on_btnLoadPointCloud_clicked();
  	void on_btnSavePointCloud_clicked();
  	void on_btnCaptureCloud_clicked();
	void on_btnInitVisualizers_clicked();
	/*
	void on_btnResetCameraViewer1_clicked();
	void on_btnResetCameraViewer2_clicked();
	void on_btnGetParametersCameraViewer1_clicked();
	void on_btnGetParametersCameraViewer2_clicked();
	*/
	void on_btnViewer1to2_clicked();
	void on_btnViewer2to1_clicked();
};