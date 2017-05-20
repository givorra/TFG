#include "pclKinect.h"
#include "common.h"
#include <QApplication>

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);
  KinectViewer w;
  w.show ();

  return a.exec ();
}
