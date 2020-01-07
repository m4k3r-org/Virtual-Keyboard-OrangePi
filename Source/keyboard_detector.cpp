#include "keyboard_detector.h"
#include <iostream>
#include <stdio.h> 
#include <unistd.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 
#include <arpa/inet.h>

#define PORT 8080 
#define CLIENT_ADDR "192.168.43.40"
#define EPS 0.000001

void KeyboardDetector::start()
{
	struct sockaddr_in address; 
	int sock = 0, valread; 
    	struct sockaddr_in serv_addr; 
    	char *message = (char*) "Car counting: "; 
    	char buffer[1024] = {0}; 
    	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
   	{ 
        	printf("\n Socket creation error \n"); 
        	return;
    	} 

    	memset(&serv_addr, '0', sizeof(serv_addr)); 
   
    	serv_addr.sin_family = AF_INET; 
    	serv_addr.sin_port = htons(PORT); 
       
    	// Convert IPv4 and IPv6 addresses from text to binary form 
    	if(inet_pton(AF_INET, CLIENT_ADDR, &serv_addr.sin_addr)<=0)  
    	{ 
        	printf("\nInvalid address/ Address not supported \n"); 
        	return;
    	} 
   
    	if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
    	{ 
        	printf("\nConnection Failed \n"); 
        	return;
    	}



  using namespace cv;

  std::cout << "Creting video stream" << std::endl;
  VideoCapture stream(0);
  std::cout << "Video stream created" << std::endl;

  if (!stream.isOpened())
  {
    std::cout << "Error: Cannot open video capture stream!" << std::endl;
    return;
  }
  stream.set(CV_CAP_PROP_FRAME_WIDTH, 640);
  stream.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

  while (true)
  {
    Mat frame;
    stream.read(frame);
    Mat gray;
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    std::vector<std::vector<Point>> contours;
    Mat bin;
    adaptiveThreshold(gray, bin, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 13, 2);
    findContours(bin, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    if (contours.empty())
      continue;

    Mat drawing = frame;
    int maxIdx = getKeyboardContourIdx(contours);
    std::vector<Point> vertices;
    getKeyboardVertices(contours, maxIdx, vertices);

    keyboardRegion = frame(boundingRect(contours[maxIdx]));
    
    setupBinKeyboard();
    keyboardVertexCorrection(vertices);
    drawKeyboardLines(drawing, vertices);
    
    //calculate median pixel value in all cells
    for (int i = 0; i < verticalPoints - 1; i++)
    {
      for (int j = 0; j < horizontalPoints - 1; j++)
      {
        Rect keyRect = Rect(keyboardPoints[i][j], keyboardPoints[i + 1][j + 1]);
        Mat keyRegion = gray(keyRect);
        medianKeyValues[i][j] = getMedianPixel(keyRegion);
      }
    }

    //try to finish calibration
    
    bool isCalibrated = true;
    for (int i = 0; i < 10; i++)
    {
      Mat frame;
      stream.read(frame);
      Mat gray;
      cvtColor(frame, gray, COLOR_BGR2GRAY);

      std::vector<std::vector<Point>> contours;
      Mat bin;
      adaptiveThreshold(gray, bin, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 13, 2);
      findContours(bin, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

      if (contours.empty())
        continue;

      Mat drawing = frame;
      int maxIdx = getKeyboardContourIdx(contours);
      std::vector<Point> vertices;
      getKeyboardVertices(contours, maxIdx, vertices);

      keyboardRegion = frame(boundingRect(contours[maxIdx]));
      
      setupBinKeyboard();
      keyboardVertexCorrection(vertices);
      drawKeyboardLines(drawing, vertices);

           
      //imshow("img", drawing);


      //calculate median pixel value in all cells
      
      for (int i = 0; i < verticalPoints - 1; i++)
      {
        for (int j = 0; j < horizontalPoints - 1; j++)
        {
          Rect keyRect = Rect(keyboardPoints[i][j], keyboardPoints[i + 1][j + 1]);
          Mat keyRegion = gray(keyRect);

          if (abs(medianKeyValues[i][j] - getMedianPixel(keyRegion)) > 5)
          {
            isCalibrated = false;
            break;
          }
        }
        if (!isCalibrated)
          break;
      }

    }



    if (isCalibrated)
    {
      std::cout << "Calibrated!" << std::endl;
      imwrite("calibrated_keyboard.jpg", drawing);
      while (true)
      {

        Mat frame;
        stream.read(frame);
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        bool keysPressed[5][10] = { false };
        bool isAKeyPressed = false;
        for (int i = 0; i < verticalPoints - 1; i++)
        {
          for (int j = 0; j < horizontalPoints - 1; j++)
          {
            if (isKeyPressed(i, j, gray))
            {
              //fill in key
              vector<Point> keyPoints(4);
              keyPoints[0] = keyboardPoints[i][j];
              keyPoints[1] = keyboardPoints[i][j + 1];
              keyPoints[2] = keyboardPoints[i + 1][j + 1];
              keyPoints[3] = keyboardPoints[i + 1][j];

              fillConvexPoly(frame, keyPoints, Scalar(50, 50, 50));
              keysPressed[i][j] = true;
              isAKeyPressed = true;
            }
          }
        }

        //imshow("pressedKeys", frame);

        static char lastKeyPressed;
        static int lastKeyRow = -1, lastKeyCol = -1;
        static bool lastKeyPrinted = true;
        
        if (!isAKeyPressed && !lastKeyPrinted)
        {
          std::cout << lastKeyPressed << std::endl;

		std::string s = "";
		s.push_back(lastKeyPressed);
		char const *pchar = s.c_str();
		send(sock , pchar , strlen(pchar) , 0 ); 
    		printf("Sent %s \n", pchar);

          lastKeyPrinted = true;
          lastKeyRow = -1;
        }
        else
        {
          for (int i = verticalPoints - 2; i >= 0; i--)
          {
            bool topKeyFound = false;
            for (int j = horizontalPoints - 2; j >= 0; j--)
            {
              if (keysPressed[i][j])
              {
                //std::cout << keySymbols[i][j];
                if (i > lastKeyRow || lastKeyRow == -1)
                {
                  lastKeyRow = i;
                  lastKeyPressed = keySymbols[i][j];
                  lastKeyPrinted = false;
                }
                topKeyFound = true;
                break;
              }
            }
            if (topKeyFound)
              break;
          }
        }
      }
    }
  }

}

int KeyboardDetector::getKeyboardContourIdx(vector<vector<Point>>& contours)
{
  int maxIdx = 0;
  double maxArea = contourArea(contours[0]);
  for (int i = 0; i < contours.size(); i++)
  {
    double currentArea = contourArea(contours[i]);
    if (currentArea > maxArea)
    {
      maxIdx = i;
      maxArea = currentArea;
    }
  }
  return maxIdx;
}

void KeyboardDetector::getKeyboardVertices(vector<vector<Point>>& contours, int index, vector<Point>& vertices)
{
  Rect r = boundingRect(contours[index]);
  double EpsH = r.height / 10;
  int upperY = r.y;
  int leftIdx = -1, rightIdx = -1;

  for (int i = 0; i < contours[index].size(); i++)
  {
    if (std::abs(contours[index][i].y - upperY) < EpsH)
    {
      if (leftIdx == -1 || contours[index][i].x < contours[index][leftIdx].x)
      {
        leftIdx = i;
      }
      if (rightIdx == -1 || contours[index][i].x > contours[index][rightIdx].x)
      {
        rightIdx = i;
      }
    }
  }

  vertices.resize(4);
  vertices[0] = Point(contours[index][leftIdx].x, upperY);
  vertices[1] = Point(contours[index][rightIdx].x, upperY);
  vertices[2] = Point(r.x, r.y + r.height);
  vertices[3] = Point(r.x + r.width, r.y + r.height);
}

void KeyboardDetector::drawKeyboardLines(Mat & dst, vector<Point>& vertices)
{
  const int keysInRow = 10, keysInCol = 5;

  int w1 = vertices[1].x - vertices[0].x;
  int dx1 = w1 / keysInRow;
  int w2 = vertices[3].x - vertices[2].x;
  int dx2 = w2 / keysInRow;

  //Draw vertical lines:
  for (int i = 0; i <= keysInRow; i++)
  {
    line(dst, Point(vertices[0].x + i * dx1, vertices[0].y),
      Point(vertices[2].x + i * dx2, vertices[2].y), Scalar(0, 0, 255));
  }

  int h1 = vertices[2].y - vertices[0].y;
  int dy1 = h1 / keysInCol;
  double k1 = (vertices[0].y - vertices[2].y) / (vertices[0].x - vertices[2].x + EPS);
  double k2 = (vertices[1].y - vertices[3].y) / (vertices[1].x - vertices[3].x + EPS);
  
  //Draw horizontal lines:
  double dy_sameHorizontal = vertices[0].y - vertices[1].y; 
  double dy_inLine = dy_sameHorizontal / (double)keysInRow;


  for (int i = 0; i <= keysInCol; i++)
  {
    int y = vertices[0].y + i * dy1 - (double)i*0.01*h1;

    int xLeft = (double)(y - vertices[0].y + k1 * vertices[0].x) / k1;
    int xRight = (double)(y - vertices[1].y + k2 * vertices[1].x) / k2;
    line(dst, Point(xLeft, y), Point(xRight, y), Scalar(0, 0, 255));

    int yOriginal = y;
    //draw intersection points
    int dx = (xRight - xLeft) / keysInRow;
    for (int j = 0; j <= keysInRow; j++)
    {
      int x = xLeft + j * dx;

      keyboardPoints[i][j] = Point(x, y);
    }
  }

  for (int i = 0; i <= keysInCol; i++)
    for (int j = 0; j <= keysInRow; j++)
      circle(dst, keyboardPoints[i][j], 3, Scalar(0, 0, 255), -1);
}

void KeyboardDetector::keyboardVertexCorrection(vector<Point> &vertices)
{
  vector<vector<Point>> innerContours;
  findContours(binKeyboard, innerContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  vector<Point> innerVertices;
  getKeyboardVertices(innerContours, getKeyboardContourIdx(innerContours), innerVertices);
  
  Size roiSize;
  Point offset;
  keyboardRegion.locateROI(roiSize, offset);
  vertices.resize(4);
  for (int i = 0; i < innerVertices.size(); i++)
  {
    vertices[i] = innerVertices[i] + offset;
  }
}

void KeyboardDetector::setupBinKeyboard()
{
  Mat keyboardRegionGray;
  cvtColor(keyboardRegion, keyboardRegionGray, COLOR_BGR2GRAY);

  threshold(keyboardRegionGray, binKeyboard, 200, 255, THRESH_OTSU | THRESH_BINARY);
  dilate(binKeyboard, binKeyboard, Mat());
  erode(binKeyboard, binKeyboard, Mat());
}

int KeyboardDetector::getMedianPixel(Mat & input)
{
  vector<uchar> pixels;
  pixels.reserve(input.rows*input.cols);
  for (int i = 0; i < input.rows; i++)
    for (int j = 0; j < input.cols; j++)
      pixels.push_back(input.at<uchar>(i, j));

  sort(pixels.begin(), pixels.end());
  return pixels[pixels.size() / 2];
}

bool KeyboardDetector::isKeyPressed(int row, int col, Mat & gray)
{
  Rect keyRect = Rect(keyboardPoints[row][col], keyboardPoints[row + 1][col + 1]);
  Mat keyRegion = gray(keyRect);

  uchar medianPixel = getMedianPixel(keyRegion);

  if (abs(medianPixel - medianKeyValues[row][col]) < 35)
  {
    return false;
  }
  return true;

}

void KeyboardDetector::rotateAroundPoint(Point & p, Point pivot, double angle)
{
  double s = sin(angle);
  double c = cos(angle);

  // translate point back to origin:
  p.x -= pivot.x;
  p.y -= pivot.y;

  // rotate point
  float xnew = p.x * c - p.y * s;
  float ynew = p.x * s + p.y * c;

  // translate point back:
  p.x = xnew + pivot.x;
  p.y = ynew + pivot.y;
}
