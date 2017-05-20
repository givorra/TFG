#include "pclKinect.h"
#include "../build/ui_pclKinect.h"


KinectViewer::KinectViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::KinectViewer)
{
  last_path = ".";
  ui->setupUi (this);
  this->setWindowTitle ("Kinect viewer");

  // Timer for 3D/UI update
  tmrTimer = new QTimer(this);
  connect(tmrTimer,SIGNAL(timeout()),this,SLOT(processFrameAndUpdateGUI()));
  tmrTimer->start(10); // msec

  // Setup Kinect
  processing = runCamera = false;

  bCopying = firstCapture = firstCall = true;
  interface = NULL;  
  objectType = none;


  // Run Kinect grabber
  er = run(); 
  initVisualizers();
  mathMorph = new MathMorph();
  ui->btnRunCamera->setText("Start Camera");
  // Run camera, no cambiar, puede fallar boton en primer clic
  //ui->btnRunCamera->setChecked(true); 
}

void KinectViewer::initVisualizers()
{
  initViewer1();
  initViewer2();
}

void KinectViewer::initViewer1()
{
  if(!firstCall)
    cloudViewer_1->clear();
  // Stop camera grabber if is running
  ui->btnRunCamera->setChecked(false);
  // Set up the QVTK window (viewer)
  viewer.reset (new pcl::visualization::PCLVisualizer ("Viewer", false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();  
  firstCall = true;
}

void KinectViewer::initViewer2()
{
  //if(!firstCapture)
    //cloudViewer_2->clear();
  // Set up the QVTK window (viewer_2)
  viewer_2.reset (new pcl::visualization::PCLVisualizer ("Viewer 2", false));
  ui->qvtkWidget_2->SetRenderWindow (viewer_2->getRenderWindow ());
  viewer_2->setupInteractor (ui->qvtkWidget_2->GetInteractor (), ui->qvtkWidget_2->GetRenderWindow ());
  ui->qvtkWidget_2->update ();  
  firstCapture = true;
  firstMesh = true;
}

void KinectViewer::processFrameAndUpdateGUI() 
{
  if(runCamera && !processing && !bCopying)
  {
    processing = true;
    showPointCloudViewer1(cloudViewer_1);
    processing = false;
  }
  
}

void KinectViewer::showPointCloudViewer1(MY_POINT_CLOUD::Ptr cloud)
{
// Add point cloud on first call
  if(firstCall == true) 
  {
    // Init viewer 1
    viewer->addPointCloud(cloud,"cloudViewer_1");
    viewer->resetCamera();
    ui->qvtkWidget->update();
    firstCall = false;
  }
  // Update point cloud
  else 
  {
    viewer->updatePointCloud(cloud,"cloudViewer_1");
    ui->qvtkWidget->update();
  }
  cloudViewer_1 = cloud;
}

void KinectViewer::showMeshViewer2()
{
  if(firstMesh)
  {
    initViewer2();
    viewer_2->addPolygonMesh(meshViewer_2,"mesh");
    viewer_2->resetCamera();
  }
  else
  {
    viewer_2->updatePolygonMesh(meshViewer_2,"mesh");
    ui->qvtkWidget->update();
  }
}

/*
void KinectViewer::showPointCloudViewer1()
{
// Add point cloud on first call
  if(firstCall == true) 
  {
    initViewer1();
    // Init viewer 1
    viewer->addPointCloud(cloudViewer_1,"cloudViewer_1");
    viewer->resetCamera();
    ui->qvtkWidget->update();
    firstCall = false;
  }
  // Update point cloud
  else 
  {
    viewer->updatePointCloud(cloudViewer_1,"cloudViewer_1");
    ui->qvtkWidget->update();
  }
}*/


void KinectViewer::cloud_cb_(const MY_POINT_CLOUD::ConstPtr &cloud)
{
  if (runCamera && !processing)
  {
    //while(bCopying && !firstCall) 
      //sleep(0);
    bCopying = true;

    if(!firstCall)
      cloudViewer_1->clear();

    //MY_POINT_CLOUD::Ptr tmp(new MY_POINT_CLOUD(*cloud));
    MY_POINT_CLOUD::Ptr tmp(new MY_POINT_CLOUD(*cloud));
    //cloudViewer_1 = fastBilateralFilter(tmp);
    //std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(*cloud, *tmp, indices);
    cloudViewer_1 = tmp;

    bCopying = false;
  }
  
}

void KinectViewer::processCurrentCloud(MY_POINT_CLOUD::Ptr currentPointCloud)
{
  // Crear una variable booleana que se ponga a true con un boton para dilatar
  //if(dilate)
  MY_POINT_CLOUD::Ptr cloud = mathMorph->camera_dilate(currentPointCloud, ui->dilateSizeValue->value());
  //MY_POINT_CLOUD::Ptr cloud(new MY_POINT_CLOUD(*currentPointCloud));
  //viewer_2->updatePointCloud (currentPointCloud,"cloud");
  viewer_2->updatePointCloud (cloud,"cloudViewer_2");
  ui->qvtkWidget_2->update ();
}

KinectViewer::~KinectViewer ()
{
  er = stop();
  delete ui;
}

int KinectViewer::run() 
{
  if(runCamera == true)
    return -1;
  
  //interface = new pcl::OpenNIGrabber();
  
  //boost::function<void (const MY_POINT_CLOUD::ConstPtr&)> f = boost::bind (&KinectViewer::cloud_cb_, this, _1);   
  
  //interface->registerCallback(f);
  
  // Start interface
  //interface->start();
  //interface->stop(); 
  // Running
  //bRun = true;
  return 0;
}

int KinectViewer::stop() 
{
  if(runCamera == false)
    return 0;
  // Not running
  runCamera = false; 
  // Stop interface
  interface->stop(); 
  return 0;
} 

pcl::PointCloud<MY_POINT_TYPE>::Ptr
hardCopy (pcl::PointCloud<MY_POINT_TYPE> cloud_ptr) {
    pcl::PointCloud<MY_POINT_TYPE>::Ptr cloud_out (new pcl::PointCloud<MY_POINT_TYPE>);
    MY_POINT_TYPE point;

    for (int i=0; i<cloud_ptr.size(); i++) {
        MY_POINT_TYPE point(cloud_ptr.points[i]);
        cloud_out->points.push_back(point);
    }
    return cloud_out;
}

void KinectViewer::greedyReconstruction()
{

      // Load input file into a PointCloud<T> with an appropriate type
      MY_POINT_CLOUD::Ptr cloud(new MY_POINT_CLOUD(*cloudViewer_2));
      //pcl::PCLPointCloud2 cloud_blob;
      //pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
      //pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
      //* the data should be available in cloud

      // Normal estimation*
      pcl::NormalEstimation<MY_POINT_TYPE, pcl::Normal> n;
      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
      pcl::search::KdTree<MY_POINT_TYPE>::Ptr tree (new pcl::search::KdTree<MY_POINT_TYPE>);
      tree->setInputCloud (cloud);
      n.setInputCloud (cloud);
      n.setSearchMethod (tree);
      n.setKSearch (20);
      n.compute (*normals);
      //* normals should not contain the point normals + surface curvatures
      // Concatenate the XYZ and normal fields*
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
      //* cloud_with_normals = cloud + normals

      // Create search tree*
      pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
      tree2->setInputCloud (cloud_with_normals);


      // Initialize objects
      pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
      pcl::PolygonMesh triangles;

      // Set the maximum distance between connected points (maximum edge length)
      gp3.setSearchRadius (ui->triRadiusSearch->value());

      // Set typical values for the parameters
      gp3.setMu(2.5);
      gp3.setMaximumNearestNeighbors(ui->triMaxNeighbors->value());
      gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
      gp3.setMinimumAngle(M_PI/18); // 10 degrees
      gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
      gp3.setNormalConsistency(false);

      // Get result
      gp3.setInputCloud (cloud_with_normals);
      gp3.setSearchMethod (tree2);
      gp3.reconstruct (triangles);

      initViewer2();
      viewer_2->addPolygonMesh(triangles,"meshes",0);
      viewer_2->resetCamera();
      firstCapture = true;

      // Additional vertex information
      //std::vector<int> parts = gp3.getPartIDs();
      //std::vector<int> states = gp3.getPointStates();
}

void KinectViewer::poissonReconstruction()
{
  cout << "begin passthrough filter" << endl;
      MY_POINT_CLOUD::Ptr filtered(new MY_POINT_CLOUD());
      MY_POINT_CLOUD::Ptr cloud(new MY_POINT_CLOUD(*cloudViewer_2));
      pcl::PassThrough<MY_POINT_TYPE> filter;
      filter.setInputCloud(cloud);
      filter.filter(*filtered);
      cout << "passthrough filter complete" << endl;


      cout << "begin normal estimation" << endl;
      pcl::NormalEstimationOMP<MY_POINT_TYPE, pcl::Normal> ne;
      ne.setNumberOfThreads(8);
      ne.setInputCloud(filtered);
      ne.setRadiusSearch(0.01);
      Eigen::Vector4f centroid;
      compute3DCentroid(*filtered, centroid);
      ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
      ne.compute(*cloud_normals);
      cout << "normal estimation complete" << endl;
      cout << "reverse normals' direction" << endl;

      for(size_t i = 0; i < cloud_normals->size(); ++i){
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
      }

      cout << "combine points and normals" << endl;
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
      concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);

      cout << "begin poisson reconstruction" << endl;
      pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
      poisson.setDepth(9);
      poisson.setInputCloud(cloud_smoothed_normals);
      //pcl::PolygonMesh mesh;
      poisson.reconstruct(meshViewer_2);

      //initViewer2();
      //viewer_2->addPolygonMesh(meshViewer_2,"meshes",0);
      //viewer_2->resetCamera();
      showMeshViewer2();
}

string KinectViewer::getFilePathDialog()
{
  QString fileName = QFileDialog::getOpenFileName(this, tr("Open Point Cloud"), tr(last_path.c_str()), tr("Point Cloud (*.ply *.pcd)"));
  string path = qPrintable(fileName);
  // Guardo la ruta para proximo load
  string::size_type pos_end_dir = path.find_last_of("/");
  last_path = path.substr(0, pos_end_dir);

  cout << "last_path = " << last_path << "\n";
  return path;
}

MY_POINT_CLOUD::Ptr bilateralFilter(MY_POINT_CLOUD::Ptr cloud_ptr)
{

  MY_POINT_CLOUD::Ptr cloud_out;
  cloud_out = fastBilateralFilter(cloud_ptr);

  return cloud_out;
}

void KinectViewer::boxCounting(MY_POINT_CLOUD::Ptr cloud_ptr)
{

  string fileName = qPrintable(QFileDialog::getSaveFileName(this, tr("Save results"), "/home/box_counting_results", tr("All files (*.*)")));
  ofstream f;
  f.open(fileName.c_str());

  if(f.is_open())
  {
    const int iterations = 100;
    // Vectores con las soluciones y leaf size de cada solución
    std::vector<int> results;
    std::vector<float> leafSizes;

    MY_POINT_CLOUD::Ptr cloud_filtered(new MY_POINT_CLOUD());
    pcl::VoxelGrid<MY_POINT_TYPE> sor;
    sor.setInputCloud(cloud_ptr);

    //cout << "boxCounting()\n";
    MY_POINT_TYPE min_pt, max_pt;
    float xSize, ySize, zSize, maxSize, leafSize, increment;

    pcl::getMinMax3D(*cloud_ptr, min_pt, max_pt);

    //xSize = max_pt.x - min_pt.x;
    //ySize = max_pt.y - min_pt.y;
    //zSize = max_pt.z - min_pt.z;

    //leafSize.x = xSize/iterations;
    //leafSize.y = ySize/iterations;
    //leafSize.z = zSize/iterations;

    //maxSize = fmaxf(fmaxf(xSize, ySize), zSize);
    //maxSize = fmaxf(fmaxf(max_pt.x - min_pt.x, max_pt.y - min_pt.y), max_pt.z - min_pt.z);
    maxSize = fminf(fminf(max_pt.x - min_pt.x, max_pt.y - min_pt.y), max_pt.z - min_pt.z);  // Prueba cogiendo el minimo en vez del maximo
    increment = maxSize/iterations;
    leafSize = 0;

    #if DEBUG_MODE == 1
      cout << "BOX COUNTING DATA:\n";
      cout << "Max X = " << max_pt.x << ", Min X = " << min_pt.x << "\n";
      cout << "Max Y = " << max_pt.y << ", Min Y = " << min_pt.y << "\n";
      cout << "Max Z = " << max_pt.z << ", Min Z = " << min_pt.z << "\n";
      cout << "Max Size = " << maxSize << "\n";
      cout << "Increment = " << increment << "\n";
    #endif

    for(int i = 0; i < iterations; i++)
    {
      cloud_filtered->clear();

      leafSize += increment;
      sor.setLeafSize (leafSize, leafSize, leafSize);
      sor.filter (*cloud_filtered);

      results.push_back(cloud_filtered->size());
      leafSizes.push_back(leafSize);
    }
    cout << "Last leaf size = " << leafSize << "\n";

    Eigen::Vector3i nrDivisions = sor.getNrDivisions();

    cout << "Number of voxels in the last iteration = " << nrDivisions(0) * nrDivisions(1) * nrDivisions(2) << "\n";



    for(int i = 0; i < iterations; i++)
    {
      f << leafSizes[i] << ";" << results[i] << ";" << log(leafSizes[i]+1) << ";" << log(results[i])  << "\n";
    }
    f.close();
  }
  else
    cerr << "ERROR: No se ha podido crear el archivo " << fileName << "\n";

  
}

// ------------------------------------------------ SLOTS ------------------------------------------------------//

void KinectViewer::on_btnBoxCounting_toggled(bool checked)
{
  if(checked)
  {
    boxCounting(cloudViewer_2);
    ui->btnBoxCounting->setChecked(false);
  }
}

void KinectViewer::on_btnDeleteOutliners_toggled(bool checked)
{
  cout << "DeleteOutliers\n";
  if(checked)
  {
    if(cloudViewer_2 != NULL)
    {
      cloudViewer_2 = statisticalRemoveOutliers(cloudViewer_2);
      viewer_2->updatePointCloud(cloudViewer_2,"cloudViewer_2");
      ui->qvtkWidget_2->update();
    }
    ui->btnDeleteOutliners->setChecked(false);
  }
}
void KinectViewer::on_btnBilateralFilter_toggled(bool checked)
{
  cout << "BilateralFilter\n";
  if(checked)
  {
    if(cloudViewer_2 != NULL)
    {
      cloudViewer_2 = fastBilateralFilter(cloudViewer_2);
      viewer_2->updatePointCloud(cloudViewer_2,"cloudViewer_2");
      ui->qvtkWidget_2->update();
    }
    ui->btnDeleteOutliners->setChecked(false);
  }
}

void KinectViewer::on_btnInitVisualizers_clicked()
{
  initVisualizers();
}

void KinectViewer::on_btnSavePointCloud_clicked()
{
  //string fileName = getFilePathDialog();  
  string fileName = qPrintable(QFileDialog::getSaveFileName(this, tr("Save File"), "/home/untitled.ply", tr("PLY (*.ply)")));

  if(fileName != "")
  {
    pcl::PLYWriter ply_writer;
    string::size_type posExtension = fileName.find_last_of(".");

    if(posExtension != string::npos)
    {
      posExtension;
      string extension = fileName.substr(posExtension, fileName.size()-posExtension);
      cout << "Extension: " << extension << "\n";

      if(extension != ".ply")
      {
        fileName += ".ply";
      }
    }
    else
    {
        fileName += ".ply";
    }
    if(ply_writer.write(fileName, *cloudViewer_1, false, false) < 0)
      cerr << "Error writing .ply file" << std::endl; 
  }
}

void KinectViewer::on_btnLoadPointCloud_clicked()
{
  string fileName = getFilePathDialog();

  if(fileName != "")
  {
    string::size_type posExtension = fileName.find_last_of(".");

    if(posExtension != string::npos)
    {
      posExtension;
      string extension = fileName.substr(posExtension, fileName.size()-posExtension);
      cout << "Extension: " << extension << "\n";
      MY_POINT_CLOUD::Ptr tmp(new MY_POINT_CLOUD());

      if(extension == ".ply")
      {
        cout << "Extension: .ply\n";

        pcl::PLYReader ply_reader; 

        //if(ply_reader.read(fileName.c_str(), *tmp) < 0)
        if(pcl::io::loadPLYFile(fileName, *tmp) < 0) 
        {
           cerr << "Error while reading the .ply file" << std::endl; 
           return;
        }
        else
        {
          ui->btnRunCamera->setChecked(false);
          initViewer1();
          cloudViewer_1 = hardCopy(*tmp);
          showPointCloudViewer1(cloudViewer_1);
        }
      }
      else if(extension == ".pcd")
      {
        if(pcl::io::loadPCDFile(fileName, *tmp) < 0)
        {
           cerr << "ERROR: No se ha podido leer fichero PCD [" << fileName << "]\n";
        }
        else
        {
          ui->btnRunCamera->setChecked(false);
          initViewer1();
          cloudViewer_1 = hardCopy(*tmp);
          showPointCloudViewer1(cloudViewer_1);
        }
      }
      else
        cout << "Extensión de archivo inválida: " << extension << "\n";

    }

    

  }

  //qInfo() << filename.toLatin1() << "\n";
  //QString fileName = QFileDialog::getOpenFileName(this, tr("Open Text file"), "", tr("Text Files (*.txt)"));
}

void KinectViewer::on_btnTriangulateCloud_toggled(bool checked)
{
  if(checked)
  {
    if(cloudViewer_2 != NULL)
    {
      //greedyReconstruction();
      poissonReconstruction();
    }
    ui->btnTriangulateCloud->setChecked(false);
  }
  
}

void KinectViewer::on_btnRunCamera_toggled(bool checked)
{
  if(checked)
  {
    if(interface == NULL)
    {
      try
      {
        interface = new pcl::OpenNIGrabber();  
        boost::function<void (const MY_POINT_CLOUD::ConstPtr&)> f = boost::bind (&KinectViewer::cloud_cb_, this, _1);       
        interface->registerCallback(f);
      }
      catch(exception e)
      {
        cerr << "ERROR: No se ha encontrado dispositivo compatible\n";
        ui->btnRunCamera->setChecked(false);
        return;
      }
    }
    // Start interface
    if(!interface->isRunning())
      interface->start(); 
    runCamera = true;
    ui->btnRunCamera->setText("Stop Camera");
  }
  else 
  {
    if(interface == NULL)
      return;
    // Start interface
    if(interface->isRunning())
      interface->stop(); 
    // Running
    runCamera = false;
    ui->btnRunCamera->setText("Start Camera");
  }
}

void KinectViewer::on_btnDilateCloud_toggled(bool checked)
{
  if(checked)
  {
    if(cloudViewer_2 != NULL)
    {
      if(CAMERA_MODE == 1)
        cloudViewer_2 = mathMorph->camera_dilate(cloudViewer_2, ui->dilateSizeValue->value());

      viewer_2->updatePointCloud(cloudViewer_2,"cloudViewer_2");
      ui->qvtkWidget_2->update();
    }
    ui->btnDilateCloud->setChecked(false);
  }
}

void KinectViewer::on_btnErodeCloud_toggled(bool checked)
{
  if(checked)
  {
    if(cloudViewer_2 != NULL)
    {
      if(CAMERA_MODE == 1)
        cloudViewer_2 = mathMorph->camera_erode(cloudViewer_2, ui->erodeSizeValue->value());

      viewer_2->updatePointCloud(cloudViewer_2,"cloudViewer_2");
      ui->qvtkWidget_2->update();
    }
    ui->btnErodeCloud->setChecked(false);
  }
}

void KinectViewer::on_btnCaptureCloud_clicked()
{
  if(cloudViewer_1 != NULL)
  {
    ui->btnRunCamera->setChecked(false); 
    MY_POINT_CLOUD::Ptr tmp(new MY_POINT_CLOUD(*cloudViewer_1));
    initViewer2();
    // Set de same camera parameters
    on_btnViewer1to2_clicked();
    //cout << "Tamaño nube 2 = " << tmp->size() << "\n";

    // Load cloudViewer_2 in viewer_2
    if(firstCapture)
    {
      viewer_2->addPointCloud(tmp,"cloudViewer_2");
      //viewer_2->resetCamera();
      firstCapture = false;
    }
    else
    {
      cloudViewer_2->clear();
      viewer_2->updatePointCloud(tmp,"cloudViewer_2");
    }
    cloudViewer_2 = tmp;
    // Update visualizer
    ui->qvtkWidget_2->update();
  }
}
/*
void KinectViewer::on_btnResetCameraViewer1_clicked()
{
  viewer->setCameraPosition(ui->valuePosXViewer1->value(), 
                              ui->valuePosYViewer1->value(),
                              ui->valuePosZViewer1->value(), 
                              ui->valueViewXViewer1->value(), 
                              ui->valueViewYViewer1->value(),
                              ui->valueViewZViewer1->value());
  ui->qvtkWidget->update();
}


void KinectViewer::on_btnResetCameraViewer2_clicked()
{
  viewer_2->setCameraPosition(ui->valuePosXViewer2->value(), 
                              ui->valuePosYViewer2->value(),
                              ui->valuePosZViewer2->value(), 
                              ui->valueViewXViewer2->value(), 
                              ui->valueViewYViewer2->value(),
                              ui->valueViewZViewer2->value());
  ui->qvtkWidget_2->update();
}

void KinectViewer::on_btnGetParametersCameraViewer1_clicked()
{
  std::vector<pcl::visualization::Camera> camera;
  viewer->getCameras(camera);
  ui->valuePosXViewer1->setValue(camera[0].pos[0]);
  ui->valuePosYViewer1->setValue(camera[0].pos[1]);
  ui->valuePosZViewer1->setValue(camera[0].pos[2]);
  ui->valueViewXViewer1->setValue(camera[0].view[0]);
  ui->valueViewYViewer1->setValue(camera[0].view[1]);
  ui->valueViewZViewer1->setValue(camera[0].view[2]);
}

void KinectViewer::on_btnGetParametersCameraViewer2_clicked()
{
  std::vector<pcl::visualization::Camera> camera;
  viewer_2->getCameras(camera);
  ui->valuePosXViewer2->setValue(camera[0].pos[0]);
  ui->valuePosYViewer2->setValue(camera[0].pos[1]);
  ui->valuePosZViewer2->setValue(camera[0].pos[2]);
  ui->valueViewXViewer2->setValue(camera[0].view[0]);
  ui->valueViewYViewer2->setValue(camera[0].view[1]);
  ui->valueViewZViewer2->setValue(camera[0].view[2]);
}*/
void KinectViewer::on_btnViewer1to2_clicked()
{
  std::vector<pcl::visualization::Camera> camera;
  viewer->getCameras(camera);
  viewer_2->setCameraParameters(camera[0]);
  ui->qvtkWidget_2->update();
}

void KinectViewer::on_btnViewer2to1_clicked()
{
  std::vector<pcl::visualization::Camera> camera;
  viewer_2->getCameras(camera);
  viewer->setCameraParameters(camera[0]);
  ui->qvtkWidget->update();
}