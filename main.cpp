#include <fmt/color.h>
#include <fmt/format.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <mynteyed/camera.h>
#include <mynteyed/utils.h>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace mynteyed;

// get the section string
string section(const string& text) {
    return fmt::format(fmt::fg(fmt::color::cyan), "{:═^{}}", " " + text + " ",
                       max(100, static_cast<int>(text.size() + 12)));
}

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    // get device information(list)
    cout << section("Device Information") << endl;
    Camera cam;
    DeviceInfo deviceInfo;
    if (!util::select(cam, &deviceInfo)) {
        LOG(FATAL) << "cannot get device information";
    }
    cout << "Print Device Information..." << endl;
    // print out device information
    util::print_stream_infos(cam, deviceInfo.index);

    // open camera
    cout << section("Open Camera") << endl;
    LOG(INFO) << fmt::format("open device, index = {}, name = {}", deviceInfo.index, deviceInfo.name) << endl;
    // set open parameters
    OpenParams openParams(deviceInfo.index);
    openParams.framerate = 30;
    openParams.dev_mode = DeviceMode::DEVICE_ALL;
    openParams.color_mode = ColorMode::COLOR_RAW;
    openParams.stream_mode = StreamMode::STREAM_2560x720;
    // open
    cam.Open(openParams);
    if (!cam.IsOpened()) {
        LOG(FATAL) << "open camera failed";
    } else {
        LOG(INFO) << "open device success";
    }
    LOG(INFO) << fmt::format("data support, image info = {}, motion = {}, distance = {}, location = {}",
                             cam.IsImageInfoSupported(), cam.IsMotionDatasSupported(), cam.IsDistanceDatasSupported(),
                             cam.IsLocationDatasSupported());
    // data enable
    cam.EnableImageInfo(true);
    cam.EnableProcessMode(ProcessMode::PROC_IMU_ALL);
    cam.EnableMotionDatas();
    cam.EnableDistanceDatas();
    cam.EnableLocationDatas();
    LOG(INFO) << fmt::format("data enable, image info = {}, motion = {}, distance = {}, location = {}",
                             cam.IsImageInfoEnabled(), cam.IsMotionDatasEnabled(), cam.IsDistanceDatasEnabled(),
                             cam.IsLocationDatasSupported());
    LOG(INFO) << fmt::format("left cam is enable = {}, right cam is enable = {}, depth is enable = {}",
                             cam.IsStreamDataEnabled(ImageType::IMAGE_LEFT_COLOR),
                             cam.IsStreamDataEnabled(ImageType::IMAGE_RIGHT_COLOR),
                             cam.IsStreamDataEnabled(ImageType::IMAGE_DEPTH));

    // get camera intrinsics
    cout << section("Camera Intrinsics") << endl;
    StreamIntrinsics streamIntrinsics = cam.GetStreamIntrinsics(openParams.stream_mode);
    cout << "left: " << streamIntrinsics.left << endl;
    cout << "right: " << streamIntrinsics.right << endl;

    // get camera extrinsics
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
        cam.WaitForStream();

        // get left stream
        auto leftStream = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
        if (leftStream.img) {
            // auto imgFormat = leftStream.img->format();  // the image format is COLOR_YUYV
            Mat img = leftStream.img->To(ImageFormat::COLOR_BGR)->ToMat();
            imshow("Left", img);
        }
        if (leftStream.img_info) {
            LOG(INFO) << fmt::format("left frame ID = {}, timestamp = {}, exposure time = {}",
                                     leftStream.img_info->frame_id, leftStream.img_info->timestamp,
                                     leftStream.img_info->exposure_time);
        }

        // get right stream, the image format is COLOR_YUYV
        auto rightStream = cam.GetStreamData(ImageType::IMAGE_RIGHT_COLOR);
        if (rightStream.img) {
            // auto imgFormat = rightStream.img->format();  // the image format is COLOR_YUYV
            Mat img = rightStream.img->To(ImageFormat::COLOR_BGR)->ToMat();
            imshow("Right", img);
        }
        if (rightStream.img_info) {
            LOG(INFO) << fmt::format("right frame ID = {}, timestamp = {}, exposure time = {}",
                                     rightStream.img_info->frame_id, rightStream.img_info->timestamp,
                                     rightStream.img_info->exposure_time);
        }

        // get depth
        auto depthStream = cam.GetStreamData(ImageType::IMAGE_DEPTH);
        if (depthStream.img) {
            // auto imgFormat = depthStream.img->format();  // the image format is IMAGE_GRAY_16
            Mat img = depthStream.img->ToMat();
            imshow("Depth", img);
        }
        if (depthStream.img_info) {
            LOG(INFO) << fmt::format("depth frame ID = {}, timestamp = {}, exposure time = {}",
                                     depthStream.img_info->frame_id, depthStream.img_info->timestamp,
                                     depthStream.img_info->exposure_time);
        }

        // get IMU
        auto motionData = cam.GetMotionDatas();
        for (auto& motion : motionData) {
            if (motion.imu) {
                if (motion.imu->flag == MYNTEYE_IMU_ACCEL) {
                    LOG(INFO) << fmt::format("IMU, timestamp = {}, temp = {}, acc = [{}, {}, {}]",
                                             motion.imu->timestamp, motion.imu->temperature, motion.imu->accel[0],
                                             motion.imu->accel[1], motion.imu->accel[2]);
                } else if (motion.imu->flag == MYNTEYE_IMU_GYRO) {
                    LOG(INFO) << fmt::format("IMU, timestamp = {}, temp = {}, gyro = [{}, {}, {}]",
                                             motion.imu->timestamp, motion.imu->temperature, motion.imu->gyro[0],
                                             motion.imu->gyro[1], motion.imu->gyro[2]);
                } else if (motion.imu->flag == MYNTEYE_IMU_ACCEL_GYRO_CALIB) {
                    LOG(INFO) << fmt::format("IMU, timestamp = {}, temp = {}, acc = [{}, {}, {}], gyro = [{}, {}, {}]",
                                             motion.imu->timestamp, motion.imu->temperature, motion.imu->accel[0],
                                             motion.imu->accel[1], motion.imu->accel[2], motion.imu->gyro[0],
                                             motion.imu->gyro[1], motion.imu->gyro[2]);
                } else {
                    LOG(ERROR) << "unknow IMU type";
                }
            }
        }

        /* // get distance
        auto distanceData = cam.GetDistanceDatas();
        for (auto& distance : distanceData) {
            if (distance.dis) {
                LOG(INFO) << format("distance, timestamp = {}, distance = {} mm", distance.dis->detection_time,
                                    distance.dis->distance);
            }
        }

        // get GPS
        auto gpsData = cam.GetLocationDatas();
        for (auto& v : gpsData) {
            if (v.gps) {
                LOG(INFO) << format("GPS, timestamp = {}, long = {}, lat = {}", v.gps->device_time, v.gps->longitude,
                                    v.gps->latitude);
            }
        } */

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
