#include <vector>

#include <cameraserver/CameraServer.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

// #include "ApproxMath.h"
// #include "BallFinder.h"
//#include "../../include/Subsystems/Lidar.h"
#include "../../include/Subsystems/LidarViewer.h"
// #include "Log.h"
// #include "Settings.h"

// REGISTER_BOOL(EnableLidarViewer, "Lidar: Enable viewer", false)
// REGISTER_DOUBLE(LidarZoom, "Lidar: Zoom", 0.075)
// REGISTER_INT(LidarCameraWidth, "Lidar: Camera stream width", 480)
// REGISTER_INT(LidarCameraHeight, "Lidar: Camera stream height", 480)

static const bool ENABLE_LIDAR_VIEWER = true;
static const double LIDAR_ZOOM = 0.075;
static const int LIDAR_CAMERA_WIDTH = 480;
static const int LIDAR_CAMERA_HEIGHT = 480;



LidarViewer *LidarViewer::Get() {
  if (self == nullptr) {
    self = new LidarViewer();
  }
  return self;
}

LidarViewer::LidarViewer() {
  cameraThread = std::thread(&LidarViewer::CameraStreamThread, this);
}

LidarViewer::~LidarViewer() { shouldRun = false; }

void LidarViewer::CameraStreamThread() {
  
  shouldRun = true;
  running = true;
  // Log("Lidar viewer stream thread starting");

  // Lidar *lidar = Lidar::Get();
  
  const int cameraWidth = LIDAR_CAMERA_WIDTH;
  const int cameraHeight = LIDAR_CAMERA_HEIGHT;

  auto s = frc::CameraServer::GetInstance();
  cs::CvSource cameraStream = s->PutVideo("Lidar", cameraWidth, cameraHeight);

  double zoom = 2.0;
  static const cv::Scalar red(50, 50, 255);
  static const cv::Scalar blue(255, 100, 100);
  static const cv::Scalar green(50, 255, 50);
  static const cv::Scalar white(255, 255, 255);
  static const cv::Size size(cameraWidth, cameraHeight);
  static const int centerX = cameraWidth >> 1;
  static const int centerY = cameraHeight >> 1;
  static const auto type = CV_8UC3;

  int frames = 0;
  const double timeStarted = frc::Timer::GetFPGATimestamp();
  double lastFrameTime = timeStarted;
  bool createBaseFrame = true;

  cv::Mat baseFrame;
  //LidarData::Buffer nodes;
  double lastLidarViewerCheck = 0;
  bool enabled = true;

  while (shouldRun) {
    //  Reset the class variables needed for displaying game objects
    m_numLidarPts = 0;
    m_numCartPts = 0;
    m_numLines  = 0;

    const double now = frc::Timer::GetFPGATimestamp();

    if (now - lastLidarViewerCheck > 5) {
      lastLidarViewerCheck = now;
      enabled = ENABLE_LIDAR_VIEWER;
      const double changeZoom = LIDAR_ZOOM;
      if (zoom != changeZoom) {
        zoom = changeZoom;
        createBaseFrame = true;
        // Log("Changing LIDAR zoom level to %f", zoom);
      }
    }
    if (!enabled) {
      frc::Wait(5.1);
      continue;
    }

    if (createBaseFrame) {
      baseFrame = cv::Mat::zeros(size, type);
      cv::circle(baseFrame, cv::Point(centerX, centerY), 6, red, 1);
      cv::putText(baseFrame, "LIDAR", cv::Point(10, 15),
                  cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.5, white, 1);

      const int radii[] = {500, 1000, 2000, 3000, 4000, 5000};
      for (int i = 0; i < sizeof(radii) / sizeof(int); ++i) {
        const int radius = radii[i] * zoom;
        cv::circle(baseFrame, cv::Point(centerX, centerY), radius, blue, 1);
      }
      createBaseFrame = false;
    }

    if (0) {
      // Don't want this thread taking over everything.
      const static double minWait = 0.1;  // about 5 fps?
      const double elapsed = now - lastFrameTime;
      const double waitTime = elapsed < minWait ? minWait - elapsed : 0.01;
      frc::Wait(waitTime);
      lastFrameTime = now;
    } else {
      // The main thread is only going to update at most every 20ms.
      frc::Wait(0.1);
    }
    cv::Mat frame = baseFrame.clone();

//    lidar->GetDataPoints(nodes);

#ifdef SHOW_BOUNDARIES
    for (int i = 0; i < nodes.boundaryCount; ++i) {
      const double a = nodes.boundaryAngles[i];
      const double d = 2000;  //??
      const int x = centerX + (d * ApproxCos(a)) - 1;
      const int y = centerY + (d * ApproxSin(a)) - 1;
      cv::line(frame, cv::Point(centerX, centerY), cv::Point(x, y), red, 1);
    }
#endif

    // for (int i = 0; i < nodes.pointCount; i++) {
    //   const int x = centerX + nodes.points[i].x * zoom - 1;
    //   const int y = centerY + nodes.points[i].y * zoom - 1;
    //   if (x <= cameraWidth && x >= 0 && y <= cameraHeight && y >= 0) {
    //     cv::rectangle(frame, cv::Rect(x, y, 3, 3),
    //                   nodes.points[i].noise ? blue : white, 1);
    //   }
    // }

    // for (int i = 0; i < nodes.circleCount; i++) {
    //   const cv::Point c(centerX + nodes.circles[i].center.x * zoom,
    //                     centerY + nodes.circles[i].center.y * zoom);
    //   cv::circle(frame, c, nodes.circles[i].radius * zoom, red);
    // }

    // for (int i = 0; i < nodes.lineCount; i++) {
    //   const int x1 = centerX + nodes.lines[i].start.x * zoom;
    //   const int y1 = centerY + nodes.lines[i].start.y * zoom;
    //   const int x2 = centerX + nodes.lines[i].end.x * zoom;
    //   const int y2 = centerY + nodes.lines[i].end.y * zoom;
    //   cv::line(frame, cv::Point(x1, y1), cv::Point(x2, y2), red, 2);
    //   // Log("Line from %d, %d - %d, %d", x1, y1, x2, y2);
    // }

  convertToXY();
  for (int i = 0; i < m_numCartPts; i++) {
      const int x = centerX + m_cartPts[i].x * zoom - 1;
      const int y = centerY + m_cartPts[i].y * zoom - 1;
      if (x <= cameraWidth && x >= 0 && y <= cameraHeight && y >= 0) {
        cv::rectangle(frame, cv::Rect(x, y, 3, 3), white, 1);
      }
    }

   for (int i = 0; i < m_numLines; i++) {
      const int x1 = centerX + m_Lines[i].start.x * zoom;
      const int y1 = centerY + (-m_Lines[i].start.y) * zoom;
      const int x2 = centerX + m_Lines[i].end.x * zoom;
      const int y2 = centerY + (-m_Lines[i].end.y) * zoom;
      cv::line(frame, cv::Point(x1, y1), cv::Point(x2, y2), red, 2);
      // Log("Line from %d, %d - %d, %d", x1, y1, x2, y2);
    }

    for (int i = 0; i < m_numScoring; i++) {
      const int x = centerX + m_scoring[i].x * zoom - 1;
      const int y = centerY + (-m_scoring[i].y) * zoom - 1;
      if (x <= cameraWidth && x >= 0 && y <= cameraHeight && y >= 0) {
        cv::rectangle(frame, cv::Rect(x, y, 5, 5), blue, 1);
      }
    }

    cameraStream.PutFrame(frame);

    if (frames++ % 20 == 0) {
      const double elapsed = frc::Timer::GetFPGATimestamp() - timeStarted;
      if (elapsed > 1.0) {
        frc::SmartDashboard::PutNumber("Lidar FPS", frames / elapsed);
      }
      frc::SmartDashboard::PutNumber("Lidar lines found", m_numLines);
    }
  }
  running = false;
}

LidarViewer *LidarViewer::self = nullptr;

void  LidarViewer::setLines(int numLines, tpLine* lines){
  for (int i=0; i<(numLines+1); i++){
    m_Lines[i] = lines[i];
  }

  m_numLines = numLines+1;
  //printf("LidarViewer::setLines, numLines =%d\n", numLines);
}

void  LidarViewer::setPoints(int numPoints, lidattp* lidarPts){

  for (int i=0; i<numPoints; i++){
      m_lidarPts[i] = lidarPts[i];
	}

  m_numLidarPts = numPoints;
  //printf("LidarViewer::setPoints, numPoints =%d\n", numPoints);
}

void  LidarViewer::addPoint(int x, int y){
  if (m_numScoring >= 6)
    m_numScoring = 0;

  m_scoring[m_numScoring].x = x;
  m_scoring[m_numScoring].y = y;

  m_numScoring++;
}

void  LidarViewer::convertToXY()
	{
	int j = 0;

	for(int i=0; i < m_numLidarPts; i++)  {
		if(!m_lidarPts[i].dist)
			continue;

		double rad = M_PI * ((double)m_lidarPts[i].angle / 64.0) / 180;
 
		m_cartPts[j].x = -((((double)m_lidarPts[i].dist) * std::sin(rad)));
		m_cartPts[j].y = ((((double)m_lidarPts[i].dist) * std::cos(rad)));
		//printf(" %i,%i",lidatXY[j].x,lidatXY[j].y);
		j++;
	}

	m_numCartPts = j;
	}

