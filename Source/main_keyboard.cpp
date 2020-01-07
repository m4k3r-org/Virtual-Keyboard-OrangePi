#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <ctype.h>
#include <vector>
#include <algorithm>
#include <functional>
#include <cstring>
#include <map>
#include <fstream>
#include "keyboard_detector.h"

using namespace cv;

int main()
{
  KeyboardDetector d;
  d.start();

  return 0;
}