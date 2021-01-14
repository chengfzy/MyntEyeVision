#include <fmt/color.h>
#include <fmt/format.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <mynteyed/camera.h>
#include <mynteyed/utils.h>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace fmt;
using namespace cv;
using namespace mynteyed;

// get the section string
string section(const string& text) {
    return format(fg(color::cyan), "{:═^{}}", " " + text + " ", max(100, static_cast<int>(text.size() + 12)));
}

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    FLAGS_logtostderr = true;

    // get device information(list)
    cout << section("Device Information") << endl;
    Camera cam;
    DeviceInfo deviceInfo;
    if (!util::select(cam, &deviceInfo)) {
        LOG(FATAL) << "cannot get device information";
    }
    // print out device information
    util::print_stream_infos(cam, deviceInfo.index);

    // open camera
    cout << section("Open Camera") << endl;
    LOG(INFO) << format("open device, index = {}, name = {}", deviceInfo.index, deviceInfo.name) << endl;
    // set open parameters
    OpenParams openParams(deviceInfo.index);
    openParams.framerate = 30;
    openParams.dev_mode = DeviceMode::DEVICE_ALL;
    openParams.color_mode = ColorMode::COLOR_RAW;
    openParams.stream_mode = StreamMode::STREAM_2560x720;
    // camera setting
    cam.EnableImageInfo(true);
    cam.EnableProcessMode(ProcessMode::PROC_IMU_ALL);
    // open
    cam.Open(openParams);
    if (!cam.IsOpened()) {
        LOG(FATAL) << "open camera failed";
    } else {
        LOG(INFO) << "open device success";
    }

    // get camera intrinsics
    cout << section("Camera Intrinsics") << endl;
    StreamIntrinsics streamIntrinsics = cam.GetStreamIntrinsics(openParams.stream_mode);
    cout << "left: " << streamIntrinsics.left << endl;
    cout << "right: " << streamIntrinsics.right << endl;

    // get camera intrinsics
    cout << section("Camera Extrinsics") << endl;
    StreamExtrinsics streamExtrinsics = cam.GetStreamExtrinsics(openParams.stream_mode);
    cout << streamExtrinsics << endl;

    // get IMU intrinsics
    cout << section("IMU Intrinsics") << endl;
    MotionIntrinsics motionIntrinsics = cam.GetMotionIntrinsics();
    cout << motionIntrinsics << endl;

    // get IMU extrinsics
    cout << section("Camera(Left)-IMU Extrinsics") << endl;
    MotionExtrinsics motionExtrinsics = cam.GetMotionExtrinsics();
    cout << motionExtrinsics << endl;

    // create window and show image
    cout << section("Read Data") << endl;
    while (true) {
        // get left stream
        auto leftStream = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
        if (leftStream.img) {
            Mat img = leftStream.img->To(ImageFormat::COLOR_BGR)->ToMat();
            imshow("Left", img);
        }
        if (leftStream.img_info) {
            LOG(INFO) << format("frame ID = {}, timestamp = {}, exposure time = {}", leftStream.img_info->frame_id,
                                leftStream.img_info->timestamp, leftStream.img_info->exposure_time);
        }

        // get right stream
        auto rightStream = cam.GetStreamData(ImageType::IMAGE_RIGHT_COLOR);
        if (rightStream.img) {
            Mat img = rightStream.img->To(ImageFormat::COLOR_BGR)->ToMat();
            imshow("Right", img);
        }

        // exit
        auto key = static_cast<char>(waitKey(1));
        if (key == 27 || key == 'q' || key == 'Q' || key == 'x' || key == 'X') {
            break;
        }
    }

    cam.Close();

    google::ShutDownCommandLineFlags();
    google::ShutdownGoogleLogging();
    return 0;
}