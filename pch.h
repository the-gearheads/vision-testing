#pragma once
#include "ntcore_cpp.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <thread>
#include "json.hpp"

extern "C" {
#include "apriltag.h"
#include "tag16h5.h"
}

using json = nlohmann::json;
using namespace cv;

#define FONT FONT_HERSHEY_PLAIN
