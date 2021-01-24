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


int main(int argc, char **argv) try {
  if (argc < 3) {
    std::cout << "Usage: minimal_lpr <video_file_name>" << std::endl;
    return EXIT_FAILURE;
  }


  std::string input_video_file_name = argv[1];
  std::cout << "Starting minimal LPR example with video file name "
            << input_video_file_name << std::endl;
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
    std::cout << "LPRSDK initialization failed" << std::endl;
    return EXIT_FAILURE;
  }

  // Passive camera does not have any hardware ineracation and implements
  // interface to access video frames externally read into memory
  SSDK::PPassiveCamera camera = SSDK::IPassiveCamera::Create();

  std::atomic<unsigned> frames_in_processing{0};
  std::atomic<unsigned> skiped_frames{0};
  std::atomic<unsigned> whole_frames{0};
  // Camera can give an event when frame is released. This one tracks how many
  // frames are there inside SDK in processing
  std::vector<std::chrono::time_point<std::chrono::system_clock>>
      times_per_cycle;
  std::vector<SSDK::CarTrackerEvent> allCarTrackerEvents;
  // display pipe is used to split the processing and displaying into separate
  // threads
  SSDK::pipe<SSDK::CarTrackerEvent> display_pipe(1024);

  // init params-------------------------------------------
  // Car Tracker parameters
  SSDK::CarTrackerParams ct_params;
  ct_params.min_recognition_confidence =
      0.4; // minimal license plate recognition confidence should be >= 0.4
  ct_params.thread_count = std::stoi(argv[2]);
  ct_params.disable_gpu = false;
  //------------------------------------------------------
  // Constructing Car Tracker algorithm which does license plate detection,
  // recognition, tracking and measuing speed
  SSDK::PCarTracker ct = SSDK::ICarTracker::Create(ct_params);
  ct->AddCarTrackerEventListener(
      [&](SSDK::CarTrackerEvent e) {
        times_per_cycle.push_back(std::chrono::system_clock::now());
        // allCarTrackerEvents.push_back(e);
        display_pipe.enqueue(e);
      },
      0);
  ct->SetCamera(camera);

  // opening video file for reading
  cv::VideoCapture vid(input_video_file_name);
  if (!vid.isOpened())
    throw std::runtime_error("Can't open file " + input_video_file_name +
                             " as video for reading");

  double fps = vid.get(cv::CAP_PROP_FPS);
  long resolution_x = (int)vid.get(cv::CAP_PROP_FRAME_WIDTH);
  long resolution_y = (int)vid.get(cv::CAP_PROP_FRAME_HEIGHT);
  std::cout << "Source video file has resolution " << resolution_x << "x"
            << resolution_y << " and fps=" << fps << std::endl;

  // all computations require absolute timestamp. In this example it starts from
  // zero
  SSDK::timestamp_type current_time = 0;
  SSDK::timestamp_type frame_interval = SSDK::msec(1000.0 / fps);
  bool stopped = false;
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

  // separate thread function for displaying processed data
  std::vector<double> avgPlateDetect;
  std::vector<double> avgPlateRecognize;
  auto display_fn = [&]() {
    SSDK::CarTrackerEvent item;
    while (display_pipe.dequeue(item)) {
      cv::Mat mat = item.image->GetBGRMat();
      double avg_detect_confidance = 0.0;
      double avg_recognize_confidance = 0.0;
      for (const SSDK::CarTrackerObject &o : item.objects) {
        // diagnose
        std::cout << "Car found:" << o.plate->GetText() << std::endl;
        avg_detect_confidance       += o.plate->GetDetectionConfidence();
        avg_recognize_confidance    += o.plate->GetRecognitionConfidence();
      }
      if (!item.objects.empty()) {
        avg_detect_confidance /=
            static_cast<double>(item.recognized_objects.size());
        avg_recognize_confidance /=
            static_cast<double>(item.recognized_objects.size());
      }
      avgPlateDetect.push_back(avg_detect_confidance);
      avgPlateRecognize.push_back(avg_recognize_confidance);
      item.image.reset();
    }
  };

  // potential cycle for tests----------------------------
  std::thread display_thread(display_fn);
  cv::Mat frame;
  auto startTime = std::chrono::system_clock::now();
  while (!stopped && vid.read(frame)) {
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
  std::cout << "all car tracker events have size  = "
            << allCarTrackerEvents.size() << std::endl;
  if (times_per_cycle.size() > 1 && duration > 0.0) {
    auto fps = static_cast<double>(whole_frames - skiped_frames - 1) / duration;
    std::cout << "FPS = " << fps << std::endl;
  }
  std::cout << "Whole frames num = " << whole_frames
            << " , skiped frames = " << skiped_frames << std::endl;
  for (std::size_t i = 0; i < avgPlateDetect.size(); i++) {
    std::cout << "Avg detect plate = " << avgPlateDetect[i] << std::endl;
    std::cout << "Avg recognize plate = " << avgPlateRecognize[i] << std::endl;
  }
  //------------------------------------------------------


  std::cout << "Deinitializng the SDK" << std::endl;
  SSDK::Deinitialize();
  std::cout << "Successful completion" << std::endl;
  return EXIT_SUCCESS;
} catch (std::exception &e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
