cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (pcl-Kinect)
find_package (Qt4 REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL 1.7.0 REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

include_directories(include)
include_directories(lib)
include_directories(src)
include_directories(forms)

set  (project_SOURCES src/main.cpp lib/pclKinect.cpp lib/MathMorph.cpp lib/OFFReader.cpp lib/common.cpp)
set  (project_HEADERS include/pclKinect.h include/common.h include/MathMorph.h include/OFFReader.h)
set  (project_FORMS   forms/pclKinect.ui)
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (pcl_visualizer 	${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (pcl_visualizer ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})
