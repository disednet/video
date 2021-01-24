// important to include OpenCV before SDK to enable implicit conversion
// functions
#include <opencv2/opencv.hpp>
#include "parser.h"
#include "ssdk/ssdk_lpr.h"
#include "ssdk/ssdk_threads.h"
#include <atomic>
#include <chrono>
#include <deque>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>
#include <limits>
#include <memory>


template <typename T>
bool isEqual(T val, T est, T eps = std::numeric_limits<T>::epsilon()) {
  return std::abs(val - est) < eps;
}

struct StatisticData {
  double fps{ 0.0 };
  unsigned int wholeFrames{ 0 };
  unsigned int skipedFrames{ 0 };
};

//-------------------------------------------------------------------------------
int initialize_sdk() {
  try {
    std::cout << "Initializing LPRSDK" << std::endl;
    // the lprsdk.dat file contains trained models and other data required for
    // LPRSDK here we should give a path to this file
    std::string model_file_name = "/opt/ssdk/models/lprsdk.dat";
    bool ok = SSDK::Initialize(model_file_name);
    std::cout << "SDK Version " << SSDK::GetVersionName() << std::endl;
    std::cout << "License to " << SSDK::GetLicenseString("customer_name")
      << std::endl;
    std::string method = SSDK::GetLicenseString("method");
    std::cout << "License method " << method << std::endl;
    if (method == "dongle")
      std::cout << "USB dongle serial " << SSDK::GetLicenseString("dongle_serial")
      << std::endl;
    std::cout << "License serial " << SSDK::GetLicenseString("serial")
      << std::endl;
    std::cout << "License serial match " << SSDK::GetLicenseString("hwid_match")
      << std::endl;

    if (!ok) {
      std::cerr << "LPRSDK initialization failed" << std::endl;
      return EXIT_FAILURE;
    }
  }
  catch (std::exception& error) {
    std::cerr << error.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

using TimeLine = std::vector<std::chrono::time_point<std::chrono::system_clock>>;
//--------------------------------------------------------------------------
SSDK::PCarTracker initTracker(const std::string& inputParams, SSDK::pipe<SSDK::CarTrackerEvent>& events, TimeLine& times) {
  SSDK::CarTrackerParams ct_params;
  ct_params.min_recognition_confidence =
    0.4; // minimal license plate recognition confidence should be >= 0.4
  if (!SSDK::setParams(ct_params, input_params)) {
    throw std::runtime_error("Can't set params for car tracker.");
  }
  // Constructing Car Tracker algorithm which does license plate detection,
  // recognition, tracking and measuing speed
  SSDK::PCarTracker ct = SSDK::ICarTracker::Create(ct_params);
  ct->AddCarTrackerEventListener(
    [&](SSDK::CarTrackerEvent e) {
      times.push_back(std::chrono::system_clock::now());
      events.enqueue(e);
    },
    0);
  return ct;
}

//--------------------------------------------------------------------------
SSDK::PPassiveCamera initCamera(std::atomic<unsigned>& frames_in_processing, 
  std::atomic<unsigned>& whole_frames, std::atomic<unsigned>& skiped_frames) {
  SSDK::PPassiveCamera camera = SSDK::IPassiveCamera::Create();
  camera->AddCameraEventListener(
    [&](const SSDK::CameraEvent event) {
      frames_in_processing--;
      whole_frames++;
      // instead of simply decreasing the counter, here is a good place to
      // release memory array of video frame if it was not copied to SDK, see
      // below
      if (event.type == SSDK::CameraEventType::FrameProcessingSkipped)
        skiped_frames++;
    },
    0);
  return camera;
}

//--------------------------------------------------------------------------
void displayFunc(SSDK::pipe<SSDK::CarTrackerEvent>& events) {
  SSDK::CarTrackerEvent item;
  while (events.dequeue(item)) {
    cv::Mat mat = item.image->GetBGRMat();
    for (const SSDK::CarTrackerObject& o : item.objects) {
      std::cout << "Car found:" << o.plate->GetText() << std::endl;
    }
    item.image.reset();
  }
}

//----------------------------------------------------------------------------
int saveStatistic(const StatisticData& data, const std::string& outFileName) {
  try {
    std::ofstream file(outFileName);
    if (file.is_open()) {
      file << "fps|" << data.fps<<std::endl;
      file << "wholeFrames" << data.wholeFrames << std::endl;
      file << "skipedFrames" << data.skipedFrames << std::endl;
      file.close();
      return EXIT_SUCCESS;
    }
    else {
      std::cerr << "Can't open output file \'" << outFileName "\'.\n";
    }
  }
  catch (std::exception& error) {
    std::cerr << error.what() << std::endl;
  }
  return EXIT_FAILURE;
}

//----------------------------------------------------------------------------
class VideoStream {
public:
  VideoStream(const std::string& fileName) 
    : m_fileName(fileName)
  {
  }

  bool isOpen() {
    try {
      m_stream = std::make_unique<cv::VideoCapture>(m_fileName);
      if (m_stream->isOpened()) {
        m_fps = m_stream->get(cv::CAP_PROP_FPS);
        m_resolutionX = static_cast<unsigned>(m_stream->get(cv::CAP_PROP_FRAME_WIDTH));
        m_resolutionY = static_cast<unsigned>(m_stream->get(cv::CAP_PROP_FRAME_HEIGHT));
      }
    }
    catch (std::exception& error) {
      std::cerr << error.what() << std::endl;
      return false;
    }
    return false;
  }

  float getFps() const { return m_fps; }
  
  std::string getFileName() const { return m_fileName; }
  
  unsigned getResolutionX() const { return m_resolutionX; }
  
  unsigned getResolutionY() const { return m_resolutionY; }
  
  bool getNextFrame(cv::Mat& frame) const { return m_stream->read(frame); }

  bool isCorrectVideo() const {
    return !isEqual(getFps(), 0.0f) && getResolutionX() != 0 && getResolutionY() != 0;
  }
private:
  std::string m_fileName;
  std::unique_ptr<cv::VideoCapture> m_stream;
  unsigned m_resolutionX{0};
  unsigned m_resolutionY{0};
  float m_fps{0.0f};
};


int main(int argc, char **argv) try {
  if (argc < 4) {
    std::cout << "Usage: minimal_lpr <video_file_name> <input_params> <output_file>" << std::endl;
    return EXIT_FAILURE;
  }
  std::string input_video_file_name = argv[1];
  std::string input_params = argv[2];
  std::string output_filename = argv[3];
  // Camera can give an event when frame is released. This one tracks how many
  // frames are there inside SDK in processing
  std::atomic<unsigned> frames_in_processing{ 0 };
  std::atomic<unsigned> skiped_frames{ 0 };
  std::atomic<unsigned> whole_frames{ 0 };

  std::cout << "Starting minimal LPR example with video file name "
            << input_video_file_name << std::endl;
  if (initialize_sdk() != EXIT_SUCCESS) {
    return EXIT_FAILURE;
  }
  SSDK::pipe<SSDK::CarTrackerEvent> display_pipe(64);
  // Passive camera does not have any hardware ineracation and implements
  // interface to access video frames externally read into memory
  auto camera = initCamera(frames_in_processing, whole_frames, skiped_frames);
  TimeLine times_per_cycle;
  auto ct = initTracker(input_params, display_pipe, times_per_cycle);
  ct->SetCamera(camera);

  // opening video file for reading
  auto vid = std::make_unique<VideoStream>(input_video_file_name);
  if (!vid->isOpen() 
    || !vid->isCorrectVideo()) {
    throw std::runtime_error("Can't open file " + input_video_file_name +
      " as video for reading");
  }
  // all computations require absolute timestamp. In this example it starts from
  // zero
  SSDK::timestamp_type current_time = 0;
  SSDK::timestamp_type frame_interval = SSDK::msec(1000.0f / vid->getFps());
// potential cycle for tests----------------------------
  std::thread display_thread([&display_pipe]() {displayFunc(display_pipe); });
  cv::Mat frame;
  auto startTime = std::chrono::system_clock::now();
  while (!stopped && vid->getNextFrame(frame)) {
    // handling overload situation. distabling this delay will make CarTracker
    // to ignore some frames
    while (frames_in_processing > ct_params.thread_count && !stopped)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ++frames_in_processing;
    while (!camera->FeedFrame(current_time, frame.data, frame.cols,
                                    frame.rows, SSDK::ImageFormat::BGR, true)){
                                      std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                    }
    current_time += frame_interval;
  }

  std::cout << "Processing completed, releasing resources" << std::endl;
  display_pipe.wait_until_empty();
  display_pipe.disable();
  display_thread.join();
  auto endTime = std::chrono::system_clock::now();
  auto calcTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "Whole time = " << calcTime.count() << " ms.\n";
  std::cout << "-------------------------------------------\n";
  double duration = 0.0;
  for (std::size_t i = 1; i < times_per_cycle.size(); i++) {
    auto itTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        times_per_cycle[i] - times_per_cycle[i - 1]);
    std::cout << "Time [" << i - 1 << ", " << i
              << "] iteration =" << itTime.count() << " ms.\n";
    duration += static_cast<double>(itTime.count()) / 1000.0;
  }
  StatisticData outputData;
  if (times_per_cycle.size() > 1 && !isEqual(duration, 0.0)) {
    outputData.fps = static_cast<double>(whole_frames - skiped_frames - 1) / duration;
  }
  outputData.skipedFrames = skiped_frames;
  outputData.wholeFrames = whole_frames;
  if (saveStatistic(outputData) != EXIT_SUCCESS) {
    throw std::exception("Can't save statistic data.");
  }
  std::cout << "Whole frames num = " << whole_frames
            << " , skiped frames = " << skiped_frames << std::endl;
  //------------------------------------------------------
  std::cout << "Deinitializng the SDK" << std::endl;
  SSDK::Deinitialize();
  std::cout << "Successful completion" << std::endl;
  return EXIT_SUCCESS;
} catch (std::exception &e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
