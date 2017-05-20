#pragma once

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include "common.h"

using namespace std;

class OFFReader
{
public:
	int read(string &fileName, pcl::PolygonMesh &mesh);
};