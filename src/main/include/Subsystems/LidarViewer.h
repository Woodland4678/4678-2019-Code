#pragma once
#include <atomic>
#include <memory>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../../include/Subsystems/Lidar.h"

class LidarViewer {
 public:
  ~LidarViewer();
  static LidarViewer *Get();
  
  void  setPoints(int numPoints, lidattp* lidarPts);
  void  setLines(int numLines, tpLine * lines);
  void addPoint(int x, int y);
 private:
  LidarViewer();
  void  CameraStreamThread();
  void  convertToXY();

 private:
  std::atomic<bool> running;
  std::atomic<bool> shouldRun;
  std::thread cameraThread;

  lidattp  m_lidarPts[1024];
  int      m_numLidarPts = 0;

 	tpPoint  m_cartPts[1024];
  int      m_numCartPts = 0;

  tpPoint  m_scoring[1024];
  int      m_numScoring = 0;
  
  tpLine   m_Lines[100];
  int      m_numLines  = 0;

  static LidarViewer *self;
};
