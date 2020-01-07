#ifndef KEYBOARD_DETECTOR
#define KEYBOARD_DETECTOR

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <vector>
#include <algorithm>
#include <functional>

using namespace cv;
using namespace std;
class KeyboardDetector
{

public:

  void start();

private:

  Mat keyboardRegion, binKeyboard;
  const int horizontalPoints = 11;
  const int verticalPoints = 6;
  Point keyboardPoints[6][11];
  uchar medianKeyValues[5][10];
  char keySymbols[5][10] = { {'<','<','<',' ',' ',' ',' ','^','^','^'},
                             {'E','.',',','m','n','b','v','c','x','z'},
                             {'\n','l','k','j','h','g','f','d','s','a'},
                             {'p','o','i','u','y','t','r','e','w','q'},
                             {'0','9','8','7','6','5','4','3','2','1'} };

  int getKeyboardContourIdx(vector<vector<Point>> &contours);
  
  void getKeyboardVertices(vector<vector<Point>> &contours, int index, vector<Point> &vertices);

  void drawKeyboardLines(Mat &dst, vector<Point> &vertices);

  void keyboardVertexCorrection(vector<Point> &vertices);

  void setupBinKeyboard();

  int getMedianPixel(Mat &input);

  bool isKeyPressed(int row, int col, Mat &gray);

  void rotateAroundPoint(Point &p, Point pivot, double angle);
};



#endif
