#include <fmt/color.h>
#include <fmt/format.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <mynteyed/camera.h>
#include <mynteyed/utils.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace fmt;
using namespace boost::filesystem;
using namespace cv;
using namespace mynteyed;

DEFINE_bool(show, true, "show image or not");
DEFINE_string(folder, "./data", "save folder");

// get the section string
string section(const string& text) {
    return format(fg(color::cyan), "{:═^{}}", " " + text + " ", max(100, static_cast<int>(text.size() + 12)));
}

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    bool showImg = FLAGS_show;
    path rootFolder = absolute(FLAGS_folder);
    path leftFolder = rootFolder / path("left");
    path rightFolder = rootFolder / path("right");
    // remove old file and create save folder
    if (is_directory(rootFolder)) {
        remove_all(rootFolder);
    }
    create_directories(rootFolder);
    create_directories(leftFolder);
    create_directories(rightFolder);
    // create IMU file name
    path imuPath = rootFolder / path("imu.txt");
    std::fstream imuFile(imuPath.string(), ios::out);
    CHECK(imuFile.is_open()) << format("cannot create IMU file \"{}\"", imuPath.string());
    imuFile << "# Timestamp, AccX, AccY, AccZ, GyroX, GyroY, GyroZ" << endl;

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
    openParams.dev_mode = DeviceMode::DEVICE_COLOR;
    openParams.color_mode = ColorMode::COLOR_RAW;
    openParams.stream_mode = StreamMode::STREAM_2560x720;
    // open
    cam.Open(openParams);
    if (!cam.IsOpened()) {
        LOG(FATAL) << "open camera failed";
    } else {
        LOG(INFO) << "open device success";
    }

    // data enable
    cam.EnableImageInfo(true);
    cam.EnableProcessMode(ProcessMode::PROC_IMU_ALL);
    cam.EnableMotionDatas();
    LOG(INFO) << format("FPS = {} Hz", cam.GetOpenParams().framerate);

    // create window and show image
    cout << section("Read Data") << endl;
    size_t imgNum{0};
    while (true) {
        cam.WaitForStream();

        // get left stream
        auto leftStream = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
        if (leftStream.img && leftStream.img_info) {
            Mat img = leftStream.img->To(ImageFormat::COLOR_BGR)->ToMat();
            path fileName =
                leftFolder / path(format("{}_{}.jpg", leftStream.img_info->frame_id, leftStream.img_info->timestamp));
            imwrite(fileName.string(), img);

            // show
            if (showImg || imgNum / 10 == 0) {
                imshow("Left", img);
            }
        }

        // get right stream
        auto rightStream = cam.GetStreamData(ImageType::IMAGE_RIGHT_COLOR);
        if (rightStream.img && rightStream.img_info) {
            Mat img = rightStream.img->To(ImageFormat::COLOR_BGR)->ToMat();
            path fileName = rightFolder /
                            path(format("{}_{}.jpg", rightStream.img_info->frame_id, rightStream.img_info->timestamp));

            imwrite(fileName.string(), img);
            // show
            if (showImg || imgNum / 10 == 0) {
                imshow("Right", img);
            }
        }

        // get IMU
        auto motionData = cam.GetMotionDatas();
        for (auto& motion : motionData) {
            if (motion.imu) {
                imuFile << format("{},{},{}, {},{},{},{}", motion.imu->timestamp, motion.imu->accel[0],
                                  motion.imu->accel[1], motion.imu->accel[2], motion.imu->gyro[0], motion.imu->gyro[1],
                                  motion.imu->gyro[2])
                        << endl;
            }
        }

        // exit
        auto key = static_cast<char>(waitKey(1));
        if (key == 27 || key == 'q' || key == 'Q' || key == 'x' || key == 'X') {
            break;
        }
    }

    imuFile.close();
    cam.Close();

    google::ShutDownCommandLineFlags();
    google::ShutdownGoogleLogging();
    return 0;
}
