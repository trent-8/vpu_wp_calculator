/*******************************************************************************
 * Copyright (c) 2012, Rockwell Automation, Inc.
 * All rights reserved.
 *
 ******************************************************************************/
#include <algorithm>
#include <chrono>
#include <cipster_api.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <ifm3d/device/device.h>
#include <ifm3d/device/o3r.h>
#include <ifm3d/fg.h>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <memory>
#include <stdlib.h>
#include <string.h>

using namespace std::chrono_literals;
using namespace ifm3d::literals;

#define INPUT_ASSEMBLY_NUM 101  // 0x065
#define OUTPUT_ASSEMBLY_NUM 102 // 0x066
#define CONFIG_ASSEMBLY_NUM 151 // 0x097

int32_t camera_status = 0;                      // status bits for all four cameras
int32_t message_count = 0;                      // number of ethernet/ip messages sent
float heights[4] = {0,0,0,0};                 // heights of the four corners of the flat
uint8_t g_assembly_data065[(sizeof(camera_status) + sizeof(message_count) + sizeof(heights))]; // input byte array
uint8_t g_assembly_data066[sizeof(int32_t)]; // output byte array
uint8_t g_assembly_data097[0];                  // Config
std::vector<std::chrono::_V2::system_clock::time_point> camera_frame_times;     // stores the last frame times of the 4 cameras

std::shared_ptr<ifm3d::O3R> vpu;           // IFM 3D camera
ifm3d::FrameGrabber::Ptr fgs[4]; // frame grabbers

struct point3d{ float x, y, z; };

bool comparePointZ(const point3d &a, const point3d &b) {
  return a.z > b.z;
}

std::string formatTimestamp(ifm3d::TimePointT timestamp) {
  using namespace std::chrono;
  std::time_t time = std::chrono::system_clock::to_time_t(
      std::chrono::time_point_cast<std::chrono::system_clock::duration>(
          timestamp));
  milliseconds milli = duration_cast<milliseconds>(
      timestamp.time_since_epoch() -
      duration_cast<seconds>(timestamp.time_since_epoch()));
  std::ostringstream s;
  s << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S") << ":"
    << std::setw(3) << std::setfill('0') << milli.count();
  return s.str();
}

/**
 * captures camera frames
 */
void getHeight(ifm3d::Frame::Ptr frame, int camera_index) {
  ifm3d::Buffer_<ifm3d::Point3D_32F> img =
      frame->GetBuffer(ifm3d::buffer_id::XYZ);
  auto confidence = frame->GetBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE);
  std::vector<point3d> points;
  std::vector<std::vector<point3d>> point_blocks{100};
  // create the point blocks and filter the points
  for (int index = 0; index < img.width() * img.height(); index++) {
    auto &point = img.at(index);
    if (point.val[2] != 0 &&
        point.val[0] > -0.1 && point.val[0] < 0.1 && point.val[1] > -101 &&
        point.val[1] < 100) {
      int block_index_x = (point.val[0] + 0.1) / 0.02;
      int block_index_y = (point.val[1] + 0.1) / 0.02;
      if (block_index_x > 9) block_index_x = 9;
      else if (block_index_x < 0) block_index_x = 0;
      if (block_index_y > 9) block_index_y = 9;
      else if (block_index_y < 0) block_index_y = 0;
      int block_index = block_index_y * 10 + block_index_x;
      point_blocks[block_index].push_back(point3d{
          point.val[0], point.val[1], point.val[2]});
    }
  }
  // filter the points to the highest ones
  for (auto &block : point_blocks) {
    sort(block.begin(), block.end(), comparePointZ);
    if (block.size() != 0) {
      block.erase(block.begin() + block.size() / 5, block.end());
      points.insert(points.end(), block.begin(), block.end());
    }
  }
  // find the average point height
  heights[camera_index] = 0;
  for (int index = 0; index < points.size(); index++) {
    heights[camera_index] += points[index].z;
  }
  heights[camera_index] /= points.size(); // get average point height
  // save the point array to a file
  std::ofstream outputFileStream;
  outputFileStream.open("points_for_camera" + std::to_string(camera_index), std::ios::out|std::ios::binary);
  if (points.size() != 0) {
    outputFileStream.write((char*) points.data(), sizeof(points.data()));
  }
  auto current_time = std::chrono::system_clock::now();
  if (camera_frame_times.empty()) {
    for (int i = 0; i<4; i++ ) {
      camera_frame_times.push_back(current_time);
    }
  }
  camera_frame_times[camera_index] = current_time;
}

void getCamera0Height(ifm3d::Frame::Ptr frame) {
  getHeight(frame, 0);
}

void getCamera1Height(ifm3d::Frame::Ptr frame) {
  getHeight(frame, 1);
}

void getCamera2Height(ifm3d::Frame::Ptr frame) {
  getHeight(frame, 2);
}

void getCamera3Height(ifm3d::Frame::Ptr frame) {
  getHeight(frame, 3);
}

void (*getCameraHeight[])(ifm3d::Frame::Ptr frame) = {
  getCamera0Height,
  getCamera1Height,
  getCamera2Height,
  getCamera3Height
};

EipStatus ApplicationInitialization(char *ipAddressCamera) {
  EipStatus status = kEipStatusOk;
  std::cout << "assigning VPU object...\n";
  vpu = std::make_shared<ifm3d::O3R>(std::string(ipAddressCamera));
  std::cout << "retrieving configuration...\n";
  ifm3d::json conf = vpu->Get();
  std::cout << "starting framegrabbers for the first 4 ports...\n";
  for (int port_num = 0; port_num < 4; port_num++) {
    std::string port_name = "port" + std::to_string(port_num);
    if (vpu->Port(port_name).type != "3D") {
      std::cerr << "wrong imager type plugged in at " << port_name << "\n";
      status = kEipStatusError;
    } else {
      std::cout << "3D camera found at " << port_name << "\n";
      uint16_t pcic_port = vpu->Port(port_name).pcic_port;
      if (pcic_port == 0) {
        std::cerr << "3D " << port_name << "PCIC port not found in the configuration\n";
        status = kEipStatusError;
      }
      auto fg = std::make_shared<ifm3d::FrameGrabber>(vpu, pcic_port);
      fg->Start({ifm3d::buffer_id::XYZ, ifm3d::buffer_id::CONFIDENCE_IMAGE});
      fgs[port_num] = fg;
      fgs[port_num]->OnNewFrame(getCameraHeight[port_num]);
    }
  }
  // INPUT
  CreateAssemblyInstance(
      INPUT_ASSEMBLY_NUM,
      ByteBuf(g_assembly_data065, sizeof(g_assembly_data065)));
  // OUTPUT
  CreateAssemblyInstance(
      OUTPUT_ASSEMBLY_NUM,
      ByteBuf(g_assembly_data066, sizeof(g_assembly_data066)));
  // CONFIG
  CreateAssemblyInstance(
      CONFIG_ASSEMBLY_NUM,
      ByteBuf(g_assembly_data097, sizeof(g_assembly_data097)));
  // Reserve some connection instances for the above assemblies:
  ConfigureExclusiveOwnerConnectionPoint(OUTPUT_ASSEMBLY_NUM, INPUT_ASSEMBLY_NUM, CONFIG_ASSEMBLY_NUM);
  // Reserve a connection instance that can connect without a config_path
  ConfigureExclusiveOwnerConnectionPoint(OUTPUT_ASSEMBLY_NUM, INPUT_ASSEMBLY_NUM, -1); // config path may be omitted

  return status;
}

void ApplicationTermination() {
  for (auto& fg : fgs) {
    if (fg != nullptr) {
      fg->Stop();
    }
  }
}

void HandleApplication() {
  // check if application needs to trigger a connection
}

void NotifyIoConnectionEvent(CipConn *aConn,
                             IoConnectionEvent io_connection_event) {
  // maintain a correct output state according to the connection state
  int consuming_id = aConn->ConsumingPath().GetInstanceOrConnPt();
  int producing_id = aConn->ProducingPath().GetInstanceOrConnPt();
}

EipStatus AfterAssemblyDataReceived(AssemblyInstance *aInstance, OpMode aMode,
                                    int aBytesReceivedCount) {
  EipStatus status = kEipStatusOk;
  if (aInstance->Id() == OUTPUT_ASSEMBLY_NUM) {}
  return status;
}

bool BeforeAssemblyDataSend(AssemblyInstance *instance) {
  if (camera_frame_times.empty())
  {
    for (int i = 0; i<4; i++) {
      camera_frame_times.push_back(std::chrono::system_clock::now());
    }
  }
  for (int i = 0; i<4; i++) {
    auto current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = current_time - camera_frame_times[i];
  }
  if (instance->Id() == INPUT_ASSEMBLY_NUM) {
    memcpy(g_assembly_data065, &camera_status, sizeof(camera_status));
    memcpy(g_assembly_data065 + sizeof(int32_t), heights, sizeof(heights));
  }
  return true;
}

EipStatus ResetDevice() {
  // add reset code here
  return kEipStatusOk;
}

EipStatus ResetDeviceToInitialConfiguration(bool also_reset_comm_params) {
  // reset the parameters

  // then perform device reset

  return kEipStatusOk;
}

void RunIdleChanged(uint32_t run_idle_value) { (void)run_idle_value; }