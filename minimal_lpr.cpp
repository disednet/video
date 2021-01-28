// important to include OpenCV before SDK to enable implicit conversion
// functions
#include <opencv2/opencv.hpp>
#include "parser.h"
#include "ssdk/ssdk_lpr.h"
#include "ssdk/ssdk_threads.h"
#include <algorithm>
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
#include <iostream>
#include <set>

template <typename T>
bool isEqual(T val, T est, T eps = std::numeric_limits<T>::epsilon()) {
  return std::abs(val - est) < eps;
}

struct StatisticData {
  double fps{ 0.0 };
  unsigned int wholeFrames{ 0 };
  unsigned int skipedFrames{ 0 };
  double confidanceMed {0.0};
  double confidanceDisp {0.0};
  double errorMed{0.0};
  double errorDisp{0.0};
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
std::pair<SSDK::PCarTracker, SSDK::CarTrackerParams> initTracker(const std::string& inputParams, SSDK::pipe<SSDK::CarTrackerEvent>& events, TimeLine& times) {
  SSDK::CarTrackerParams ct_params;
  ct_params.min_recognition_confidence =
    0.4; // minimal license plate recognition confidence should be >= 0.4
  if (!SSDK::setParams(ct_params, inputParams)) {
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
  return std::make_pair(ct, ct_params);
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
void displayFunc(SSDK::pipe<SSDK::CarTrackerEvent>& events, std::set<std::string>& plates,
                 std::vector<double>& confidance) {
  SSDK::CarTrackerEvent item;
  while (events.dequeue(item)) {
    for (const SSDK::CarTrackerObject& o : item.recognized_objects/*objects*/) {
      std::cout << "Car found:" << o.plate->GetText() << std::endl;
      auto itemConfidance = o.plate->GetSymbolConfidences();
      itemConfidance.push_back(o.plate->GetRecognitionConfidence());
      std::sort(itemConfidance.begin(), itemConfidance.end());
      confidance.push_back(itemConfidance[itemConfidance.size()/2]);
      plates.insert(o.plate->GetText());
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
      file << "wholeFrames|" << data.wholeFrames << std::endl;
      file << "confidances|" << data.confidanceMed << std::endl;
      file << "error|" << data.errorMed << std::endl;
      file.close();
      return EXIT_SUCCESS;
    }
    else {
      std::cerr << "Can't open output file \'" << outFileName << "\'.\n";
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
  VideoStream(const std::string& fileName, unsigned scale = 100)
    : m_fileName(fileName)
  {
    m_scale = scale <= 100 ? scale : m_scale;
  }

  bool isOpen() {
    try {
      m_stream = std::make_unique<cv::VideoCapture>(m_fileName);
      if (m_stream->isOpened()) {
        m_fps = m_stream->get(cv::CAP_PROP_FPS);
        m_resolutionX = static_cast<unsigned>(m_stream->get(cv::CAP_PROP_FRAME_WIDTH));
        m_resolutionY = static_cast<unsigned>(m_stream->get(cv::CAP_PROP_FRAME_HEIGHT));
        return true;
      }
    }
    catch (std::exception& error) {
      std::cerr << error.what() << std::endl;
    }
    return false;
  }

  float getFps() const { return m_fps; }
  
  std::string getFileName() const { return m_fileName; }
  
  unsigned getResolutionX() const { return m_resolutionX; }
  
  unsigned getResolutionY() const { return m_resolutionY; }
  
  bool getNextFrame(cv::Mat& frame) {
    //static int index = 0;
    if (m_scale == 100) {
      auto res = m_stream->read(frame);
      /*auto number = std::to_string(index++);
      number.insert(0, 5-number.length(), '0');
      std::string name = "pic" + number + ".jpg";
      cv::imwrite(name, frame);*/
      return res;
    }
    else {
        auto resX = static_cast<unsigned>(static_cast<float>(m_resolutionX) * static_cast<float>(m_scale) / 100.0f);
        auto resY = static_cast<unsigned>(static_cast<float>(m_resolutionY) * static_cast<float>(m_scale) / 100.0f);
        auto result = m_stream->read(m_tmpFrame);
        if (result)
            cv::resize(m_tmpFrame, frame, {resX, resY}, 0 , 0, cv::INTER_CUBIC);
        return result;
    }
  }

  bool isCorrectVideo() {
    if (!isOpen())
        return false;
    return !isEqual(getFps(), 0.0f) && getResolutionX() != 0 && getResolutionY() != 0;
  }
private:
  std::string m_fileName;
  std::unique_ptr<cv::VideoCapture> m_stream;
  unsigned m_resolutionX{0};
  unsigned m_resolutionY{0};
  unsigned m_scale{100};
  float m_fps{0.0f};
  cv::Mat m_tmpFrame;
};

std::set<std::string> getCorrectPlates(const std::string& file_name){
  std::set<std::string> result;
  std::ifstream file(file_name);
  if (file.is_open()) {
    std::string line;
    while (std::getline(file, line)) {
      auto pos = line.rfind("\r");
      if (pos != std::string::npos) {
        line.replace(pos, 1, "");
      }
      result.insert(line);
    }
  }
  return result;
}

std::pair<double, double> getMidAndDisp(const std::vector<double>& data){
  if (!data.empty()) {
    double m = 0.0;
    for (auto el : data)
      m += el;
    m /= static_cast<double>(data.size());
    double d = 0.0;
    for (auto el : data)
      d += (el - m)*(el - m);
    d /= static_cast<double>(data.size());
    return std::make_pair(m, d);
  }
  return std::make_pair(0.0, 0.0);
}

int main(int argc, char **argv) try {
  if (argc < 6) {
    std::cout << "Usage: minimal_lpr <video_file_name> <input_params> <output_file> <video scale> <file_with_correct_plates>" << std::endl;
    return EXIT_FAILURE;
  }
  std::string input_video_file_name = argv[1];
  unsigned scale = std::stoul(argv[2]);
  std::string input_params = argv[3];
  std::string output_filename = argv[4];
  std::string correct_plates_file = argv[5];
  
  // Camera can give an event when frame is released. This one tracks how many
  // frames are there inside SDK in processing
  std::atomic<unsigned> frames_in_processing{ 0 };
  std::atomic<unsigned> skiped_frames{ 0 };
  std::atomic<unsigned> whole_frames{ 0 };
  auto correctPlates = getCorrectPlates(correct_plates_file);
  if (correctPlates.empty()){
    std::cerr << "Can't find data with correct car plates in \'" << correct_plates_file << "\'\n";
    return EXIT_FAILURE;
  }
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
  auto trackerData = initTracker(input_params, display_pipe, times_per_cycle);
  auto ct = trackerData.first;
  auto ct_params = trackerData.second;
  ct->SetCamera(camera);

  // opening video file for reading
  auto vid = std::make_unique<VideoStream>(input_video_file_name, scale);
  if (!vid->isCorrectVideo()) {
    throw std::runtime_error("Can't open file " + input_video_file_name +
      " as video for reading");
  }
  // all computations require absolute timestamp. In this example it starts from
  // zero
  SSDK::timestamp_type current_time = 0;
  SSDK::timestamp_type frame_interval = SSDK::msec(1000.0f / vid->getFps());
// potential cycle for tests----------------------------
  std::set<std::string> platesNum;
  std::vector<double> confidances;
  std::thread display_thread([&display_pipe, &confidances, &platesNum]() {displayFunc(display_pipe, platesNum, confidances); });
  cv::Mat frame;
  auto startTime = std::chrono::system_clock::now();
  while (vid->getNextFrame(frame)) {
    // handling overload situation. distabling this delay will make CarTracker
    // to ignore some frames
    while (frames_in_processing > ct_params.thread_count)
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
  auto confStat = getMidAndDisp(confidances);
  outputData.confidanceMed = confStat.first;
  outputData.confidanceDisp = confStat.second;
  std::vector<double> error;
  double errorVal = 0.0;
  for (const auto& plate : platesNum) {
    if (correctPlates.count(plate) == 0){
      errorVal += 1.0;
    }
  }
  if (!platesNum.empty()) {
    errorVal /= static_cast<double>(platesNum.size());
  }
  outputData.errorMed = errorVal;
  if (saveStatistic(outputData, output_filename) != EXIT_SUCCESS) {
    throw std::runtime_error("Can't save statistic data.");
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
