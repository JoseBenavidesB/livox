//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <algorithm>
#include <string.h>
#include "lvx_file.h"
#include "cmdline.h"
#include <stdio.h>
#include <time.h>

#include <iostream>
#include <chrono>
#include <iomanip>

// mavlink
#include <atomic>
#include <thread>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

// Si instalas los headers del sistema: sudo apt-get install -y libmavlink-dev
// y usa:
#include <mavlink/common/mavlink.h>  // MAVLink v2 common
// end mavlink

DeviceItem devices[kMaxLidarCount];
LvxFileHandle lvx_file_handler;
std::list<LvxBasePackDetail> point_packet_list;
std::vector<std::string> broadcast_code_rev;
std::condition_variable lidar_arrive_condition;
std::condition_variable extrinsic_condition;
std::condition_variable point_pack_condition;
std::mutex mtx;
int lvx_file_save_time = 10;
bool is_finish_extrinsic_parameter = false;
bool is_read_extrinsic_from_xml = false;
// uint8_t connected_lidar_count = 0;
PointCloudReturnMode g_return_mode = kTripleReturn;
LidarScanPattern g_scan_pattern = kNoneRepetitiveScanPattern;

// detect
uint8_t connected_lidar_count = 0;

// mavlink
std::string g_mav_port = "/dev/ttyAMA1";
int g_mav_baud = 115200;
int g_mav_sysid = 245;
int g_mav_compid = 190;
std::string g_ts_log_path = "pi5_timesync.csv";

std::atomic<bool> g_run_timesync{true};

// end mavlink

#define FRAME_RATE 20

using namespace std::chrono;

/** Connect all the broadcast device in default and connect specific device when use program options or broadcast_code_list is not empty. */
std::vector<std::string> broadcast_code_list = {
  "3JEDN5K001JF711"
};

/** Receiving error message from Livox Lidar. */
void OnLidarErrorStatusCallback(livox_status status, uint8_t handle, ErrorMessage *message) {
  static uint32_t error_message_count = 0;
  if (message != NULL) {
    ++error_message_count;
    if (0 == (error_message_count % 100)) {
      printf("handle: %u\n", handle);
      printf("temp_status : %u\n", message->lidar_error_code.temp_status);
      printf("volt_status : %u\n", message->lidar_error_code.volt_status);
      printf("motor_status : %u\n", message->lidar_error_code.motor_status);
      printf("dirty_warn : %u\n", message->lidar_error_code.dirty_warn);
      printf("firmware_err : %u\n", message->lidar_error_code.firmware_err);
      printf("pps_status : %u\n", message->lidar_error_code.device_status);
      printf("fan_status : %u\n", message->lidar_error_code.fan_status);
      printf("self_heating : %u\n", message->lidar_error_code.self_heating);
      printf("ptp_status : %u\n", message->lidar_error_code.ptp_status);
      printf("time_sync_status : %u\n", message->lidar_error_code.time_sync_status);
      printf("system_status : %u\n", message->lidar_error_code.system_status);
    }
  }
}

/** Receiving point cloud data from Livox LiDAR. */
void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data) {
  if (data) {
    if (handle < connected_lidar_count && is_finish_extrinsic_parameter) {
      std::unique_lock<std::mutex> lock(mtx);
      LvxBasePackDetail packet;
      packet.device_index = handle;
      lvx_file_handler.BasePointsHandle(data, packet);
      point_packet_list.push_back(packet);
    }
  }
}

/** Callback function of starting sampling. */
void OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {
  printf("OnSampleCallback statues %d handle %d response %d \n", status, handle, response);
  if (status == kStatusSuccess) {
    if (response != 0) {
      devices[handle].device_state = kDeviceStateConnect;
    }
  } else if (status == kStatusTimeout) {
    devices[handle].device_state = kDeviceStateConnect;
  }
}

/** Callback function of setting return mode. */
void OnSetReturnMode(livox_status status, uint8_t handle, uint8_t response, void *data) {
  printf("OnSetReturnMode status %d handle %u response %u\n", status, handle, response);
    printf("************************************************************\n");

}

/** Callback function of setting return mode. */
void OnSetScanPattern(livox_status status, uint8_t handle, DeviceParameterResponse *response, void *client_data) {
    if (status != kStatusNotSupported && response != NULL) {
      printf("SCAN ==============> status=> %d || handle=> %u || response=> %u\n", status, handle, response->ret_code);

      printf("\n");
      printf("************************************************************\n");
    } else {
      printf("SCAN ==============> NOOOOOO soportado");
    printf("************************************************************\n");
    }
}

void OnGetImu(livox_status status, uint8_t handle, LidarGetImuPushFrequencyResponse *response, void *client_data) {
  if (status == kStatusSuccess && response != NULL) {
    printf("IMU ==============> status=> %d || handle=> %u || response=> %u\n", status, handle, response->freq);
    printf("************************************************************\n");

  } else {
    printf("IMU ==============> NOOOOOO soportado");
    printf("************************************************************\n");

  }
}

void OnGetScan(livox_status status, uint8_t handle, GetDeviceParameterResponse *response, void *client_dataa) {
    if (status != kStatusNotSupported && response != NULL) {
      printf("SCAN ==============> status=> %d || handle=> %u || response=> %u\n", status, handle, response->kv.value);

      // Imprimir datos básicos de respuesta
      printf("Device Parameter Response:\n");
      printf("  Return Code: %u\n", response->rsp.ret_code);
      printf("  Error Param Key: %u\n", response->rsp.error_param_key);
      printf("  Error Code: %u\n", response->rsp.error_code);

      // Imprimir KeyValueParam (kv)
      printf("  KeyValueParam:\n");
      printf("    Key: %u\n", response->kv.key);
      printf("    Length: %u\n", response->kv.length);

      // Imprimir valor (array)
      printf("    Value: ");
      for (int i = 0; i < response->kv.length; i++) {
        printf("%02X ", response->kv.value[i]);  // en hexadecimal
      }
      printf("\n");
      printf("************************************************************\n");
    } else {
      printf("SCAN ==============> NOOOOOO soportado");
    printf("************************************************************\n");
    }
}


/** Callback function of stopping sampling. */
void OnStopSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {
}

/** Callback function of get LiDARs' extrinsic parameter. */
void OnGetLidarExtrinsicParameter(livox_status status, uint8_t handle, LidarGetExtrinsicParameterResponse *response, void *data) {
  if (status == kStatusSuccess) {
    if (response != 0) {
      printf("OnGetLidarExtrinsicParameter statue %d handle %d response %d \n", status, handle, response->ret_code);
      std::unique_lock<std::mutex> lock(mtx);
      LvxDeviceInfo lidar_info;
      strncpy((char *)lidar_info.lidar_broadcast_code, devices[handle].info.broadcast_code, kBroadcastCodeSize);
      memset(lidar_info.hub_broadcast_code, 0, kBroadcastCodeSize);
      lidar_info.device_index = handle;
      lidar_info.device_type = devices[handle].info.type;
      lidar_info.extrinsic_enable = true;
      lidar_info.pitch = response->pitch;
      lidar_info.roll = response->roll;
      lidar_info.yaw = response->yaw;
      lidar_info.x = static_cast<float>(response->x / 1000.0);
      lidar_info.y = static_cast<float>(response->y / 1000.0);
      lidar_info.z = static_cast<float>(response->z / 1000.0);
      lvx_file_handler.AddDeviceInfo(lidar_info);
      if (lvx_file_handler.GetDeviceInfoListSize() == connected_lidar_count) {
        is_finish_extrinsic_parameter = true;
        extrinsic_condition.notify_one();
      }
    }
  }
  else if (status == kStatusTimeout) {
    printf("GetLidarExtrinsicParameter timeout! \n");
  }
}

/** Get LiDARs' extrinsic parameter from file named "extrinsic.xml". */
void LidarGetExtrinsicFromXml(uint8_t handle) {
  LvxDeviceInfo lidar_info;
  ParseExtrinsicXml(devices[handle], lidar_info);
  lvx_file_handler.AddDeviceInfo(lidar_info);
  lidar_info.extrinsic_enable = true;
  if (lvx_file_handler.GetDeviceInfoListSize() == broadcast_code_list.size()) {
    is_finish_extrinsic_parameter = true;
    extrinsic_condition.notify_one();
  }
}

/** Query the firmware version of Livox LiDAR. */
void OnDeviceInformation(livox_status status, uint8_t handle, DeviceInformationResponse *ack, void *data) {
  if (status != kStatusSuccess) {
    printf("Device Query Informations Failed %d\n", status);
  }
  if (ack) {
    printf("firm ver: %d.%d.%d.%d\n",
           ack->firmware_version[0],
           ack->firmware_version[1],
           ack->firmware_version[2],
           ack->firmware_version[3]);
  }
}

void LidarConnect(const DeviceInfo *info) {
  uint8_t handle = info->handle;
  QueryDeviceInformation(handle, OnDeviceInformation, NULL);
  if (devices[handle].device_state == kDeviceStateDisconnect) {
    devices[handle].device_state = kDeviceStateConnect;
    devices[handle].info = *info;
  }
}

void LidarDisConnect(const DeviceInfo *info) {
  uint8_t handle = info->handle;
  devices[handle].device_state = kDeviceStateDisconnect;
}

void LidarStateChange(const DeviceInfo *info) {
  uint8_t handle = info->handle;
  devices[handle].info = *info;
}

/** Callback function of changing of device state. */
void OnDeviceInfoChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == nullptr) {
    return;
  }
  printf("OnDeviceChange broadcast code %s update type %d\n", info->broadcast_code, type);
  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }

  if (type == kEventConnect) {
    LidarConnect(info);
    printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
  } else if (type == kEventDisconnect) {
    LidarDisConnect(info);
    printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
  } else if (type == kEventStateChange) {
    LidarStateChange(info);
    printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
  }

  if (devices[handle].device_state == kDeviceStateConnect) {
    printf("Device Working State %d\n", devices[handle].info.state);
    if (devices[handle].info.state == kLidarStateInit) {
      printf("Device State Change Progress %u\n", devices[handle].info.status.progress);
    } else {
      printf("Device State Error Code 0X%08x\n", devices[handle].info.status.status_code.error_code);
    }
    printf("Device feature %d\n", devices[handle].info.feature);
    SetErrorMessageCallback(handle, OnLidarErrorStatusCallback);
    if (devices[handle].info.state == kLidarStateNormal) {

      // 👇 Enviar return-mode primero
      LidarSetPointCloudReturnMode(handle, g_return_mode, OnSetReturnMode, nullptr);

      // Set scan pattern
      LidarSetScanPattern(handle, g_scan_pattern, OnSetScanPattern, nullptr);

      if (!is_read_extrinsic_from_xml) {
        LidarGetExtrinsicParameter(handle, OnGetLidarExtrinsicParameter, nullptr);
      } else {
        LidarGetExtrinsicFromXml(handle);
      }
      
      LidarStartSampling(handle, OnSampleCallback, nullptr);
      devices[handle].device_state = kDeviceStateSampling;
    }
  }
}

/** Callback function when broadcast message received.
 * You need to add listening device broadcast code and set the point cloud data callback in this function.
 */
void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == nullptr || info->dev_type == kDeviceTypeHub) {
    return;
  }

  printf("Receive Broadcast Code %s\n", info->broadcast_code);
  if ((broadcast_code_rev.size() == 0) ||
      (std::find(broadcast_code_rev.begin(), broadcast_code_rev.end(), info->broadcast_code) == broadcast_code_rev.end())) {
    broadcast_code_rev.push_back(info->broadcast_code);
    lidar_arrive_condition.notify_one();
  }
}

/** Wait until no new device arriving in 2 second. */
void WaitForDevicesReady( ) {
  bool device_ready = false;
  seconds wait_time = seconds(2);
  steady_clock::time_point last_time = steady_clock::now();
  while (!device_ready) {
    std::unique_lock<std::mutex> lock(mtx);
    lidar_arrive_condition.wait_for(lock,wait_time);
    if ((steady_clock::now() - last_time + milliseconds(50)) >= wait_time) {
      device_ready = true;
    } else {
      last_time = steady_clock::now();
    }
  }
}

void WaitForExtrinsicParameter() {
  std::unique_lock<std::mutex> lock(mtx);
  extrinsic_condition.wait(lock);
}

void AddDevicesToConnect() {
  if (broadcast_code_rev.size() == 0)
    return;

  for (int i = 0; i < broadcast_code_rev.size(); ++i) {
    if ((broadcast_code_list.size() != 0) &&
        (std::find(broadcast_code_list.begin(), broadcast_code_list.end(), broadcast_code_rev[i]) == broadcast_code_list.end())) {
      continue;
    }
    uint8_t handle = 0;
    livox_status result = AddLidarToConnect(broadcast_code_rev[i].c_str(), &handle);
    if (result == kStatusSuccess) {
      /** Set the point cloud data for a specific Livox LiDAR. */
      SetDataCallback(handle, GetLidarData, nullptr);
      devices[handle].handle = handle;
      devices[handle].device_state = kDeviceStateDisconnect;
      connected_lidar_count++;
    }
  }
}

/** Set the program options.
* You can input the registered device broadcast code and decide whether to save the log file.
*/
void SetProgramOption(int argc, const char *argv[]) {
  cmdline::parser cmd;
  cmd.add<std::string>("code", 'c', "Register device broadcast code", false);
  cmd.add("log", 'l', "Save the log file");
  cmd.add<int>("time", 't', "Time to save point cloud to the lvx file", false);
  cmd.add("param", 'p', "Get the extrinsic parameter from extrinsic.xml file");
 
  // 👇 NUEVO: return-mode
  cmd.add<int>("return-mode", 'r', "Set point cloud return mode (0:first, 1:strongest, 2:dual, 3:triple)", false);

  cmd.add<int>("scan", 's', "Set point cloud scan pattern (0:nonRepetitive, 1:repetitive)", false);

  // Mavlink
  cmd.add<std::string>("mav-port", 0, "Serial port to Cube (e.g. /dev/ttyAMA1 or /dev/ttyS0)", false, "/dev/ttyAMA1");
  cmd.add<int>("mav-baud", 0, "Baudrate for serial MAVLink (e.g. 115200 or 921600)", false, 115200);
  cmd.add<int>("mav-sysid", 0, "MAVLink system id for PI5", false, 245);
  cmd.add<int>("mav-compid", 0, "MAVLink component id for PI5", false, 190);
  cmd.add<std::string>("ts-log", 0, "Path for local timestamp log", false, "pi5_timesync.csv");
  // end mavlink

  
  cmd.add("help", 'h', "Show help");
  cmd.parse_check(argc, const_cast<char **>(argv));
  if (cmd.exist("code")) {
    std::string sn_list = cmd.get<std::string>("code");
    printf("Register broadcast code: %s\n", sn_list.c_str());
    size_t pos = 0;
    broadcast_code_list.clear();
    while ((pos = sn_list.find("&")) != std::string::npos) {
      broadcast_code_list.push_back(sn_list.substr(0, pos));
      sn_list.erase(0, pos + 1);
    }
    broadcast_code_list.push_back(sn_list);
  }
  if (cmd.exist("log")) {
    printf("Save the log file.\n");
    SaveLoggerFile();
  }
  if (cmd.exist("time")) {
    printf("Time to save point cloud to the lvx file:%d.\n", cmd.get<int>("time"));
    lvx_file_save_time = cmd.get<int>("time");
  }

  // 👇 NUEVO: parsear return-mode
  if (cmd.exist("return-mode")) {
    int mode_val = cmd.get<int>("return-mode");

    switch (mode_val) {
      case 0: g_return_mode = kFirstReturn; break;
      case 1: g_return_mode = kStrongestReturn; break;
      case 2: g_return_mode = kDualReturn; break;
      case 3: g_return_mode = kTripleReturn; break;
      default:
        printf("Unknown return-mode %d, using strongest.\n", mode_val);
        g_return_mode = kTripleReturn;
        break;
    }

    printf("Selected return mode: %d\n", mode_val);
  }

    // 👇 NUEVO: parsear return-mode
  if (cmd.exist("scan")) {
    int mode_val = cmd.get<int>("scan");

    switch (mode_val) {
      case 0: g_scan_pattern = kNoneRepetitiveScanPattern; break;
      case 1: g_scan_pattern = kRepetitiveScanPattern; break;
      default:
        printf("Unknown scan pattern %d, using non repetetive.\n", mode_val);
        g_scan_pattern = kNoneRepetitiveScanPattern;
        break;
    }

    printf("Selected scan pattern: %d\n", mode_val);
  }

  if (cmd.exist("param")) {
    printf("Get the extrinsic parameter from extrinsic.xml file.\n");
    is_read_extrinsic_from_xml = true;
  }
  
  if (cmd.exist("mav-port")) g_mav_port = cmd.get<std::string>("mav-port");
  if (cmd.exist("mav-baud")) g_mav_baud = cmd.get<int>("mav-baud");
  if (cmd.exist("mav-sysid")) g_mav_sysid = cmd.get<int>("mav-sysid");
  if (cmd.exist("mav-compid")) g_mav_compid = cmd.get<int>("mav-compid");
  if (cmd.exist("ts-log")) g_ts_log_path = cmd.get<std::string>("ts-log");
  
  return;
}

void print_system_time() {
    using namespace std::chrono;

    // Tiempo actual
    auto now = system_clock::now();

    // Convertir a time_t para obtener fecha y hora legible
    auto time_t_now = system_clock::to_time_t(now);
    auto local_time = *std::localtime(&time_t_now);

    // Extraer milisegundos
    auto duration_since_epoch = now.time_since_epoch();
    auto millis = duration_cast<milliseconds>(duration_since_epoch) % 1000;

    // Extraer nanosegundos
    auto nanos = duration_cast<nanoseconds>(duration_since_epoch).count();

    // Imprimir formato legible + timestamp en ns
    std::cout << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S")
              << '.' << std::setfill('0') << std::setw(3) << millis.count()
              << " | ns since epoch: " << nanos
              << std::endl;
}


// Mavlink
int open_serial_port(const std::string& port, int baud) {
  int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) return -1;

  termios tty{};
  if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }

  cfmakeraw(&tty);

  speed_t br = B115200;
  switch (baud) {
    case 57600: br = B57600; break;
    case 115200: br = B115200; break;
    case 230400: br = B230400; break;
    case 460800: br = B460800; break;
    case 921600: br = B921600; break;
    default: br = B115200; break;
  }
  cfsetispeed(&tty, br);
  cfsetospeed(&tty, br);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;        // 1 stop bit
  tty.c_cflag &= ~PARENB;        // no parity
  tty.c_cflag &= ~CRTSCTS;       // no HW flow

  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN]  = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return -1; }

  // pasar a blocking write
  int flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

  return fd;
}

bool mav_write_msg(int fd, const mavlink_message_t& msg) {
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
  ssize_t w = ::write(fd, buffer, len);
  return (w == (ssize_t)len);
}

void send_system_time(int fd, uint8_t sysid, uint8_t compid, int64_t ns) {
  mavlink_message_t msg{};
  const uint64_t usec = (uint64_t)(ns / 1000);
  const uint32_t boot_ms = (uint32_t)(std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count() & 0xFFFFFFFFu);
  mavlink_msg_system_time_pack(sysid, compid, &msg, usec, boot_ms);
  mav_write_msg(fd, msg);
}

void send_timesync(int fd, uint8_t sysid, uint8_t compid, int64_t ns_pi5) {
  // En el "handshake" clásico, el companion manda tc1=0, ts1=local.
  // Aquí queremos LOG estructurado con ns, por lo que:
  //   tc1 = 0 (solicitud), ts1 = ns del PI5
  mavlink_message_t msg{};
  mavlink_msg_timesync_pack(sysid, compid, &msg, 0 /*tc1*/, ns_pi5 /*ts1*/);
  mav_write_msg(fd, msg);
}

void send_statustext_ns(int fd, uint8_t sysid, uint8_t compid, int64_t ns) {
  mavlink_message_t msg{};
  char txt[50];
  snprintf(txt, sizeof(txt), "PI5_NS=%lld", (long long)ns);
  mavlink_msg_statustext_pack(sysid, compid, &msg, MAV_SEVERITY_INFO, txt, 0, 0);
  mav_write_msg(fd, msg);
}

void timesync_worker() {
  // Abrir puerto
  const int fd = open_serial_port(g_mav_port, g_mav_baud);
  if (fd < 0) {
    fprintf(stderr, "[Timesync] ERROR: no pude abrir %s @ %d\n", g_mav_port.c_str(), g_mav_baud);
    return;
  }

  // Abrir/crear log CSV y escribir encabezado si está vacío
  std::ofstream ofs;
  ofs.open(g_ts_log_path, std::ios::out | std::ios::app);
  if (!ofs) {
    fprintf(stderr, "[Timesync] WARNING: no pude abrir log %s\n", g_ts_log_path.c_str());
  } else if (ofs.tellp() == 0) {
    ofs << "unix_time_ns\n";
    ofs.flush();
  }

  // Programar “cada 60 s” relativo al inicio (estable y sin drift perceptible)
  auto t0 = std::chrono::steady_clock::now();
  int n = 0;

  while (g_run_timesync.load()) {
    auto target = t0 + std::chrono::seconds(60 * n);
    std::this_thread::sleep_until(target);

    // Timestamp ns (CLOCK_REALTIME ~ epoch)
    int64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::system_clock::now().time_since_epoch()).count();

    // 1) Log local
    if (ofs) {
      ofs << ns << "\n";
      ofs.flush();
    }

    // 2) Envío MAVLink inmediato
    send_system_time(fd, (uint8_t)g_mav_sysid, (uint8_t)g_mav_compid, ns);
    send_timesync(fd,    (uint8_t)g_mav_sysid, (uint8_t)g_mav_compid, ns);
    send_statustext_ns(fd,(uint8_t)g_mav_sysid, (uint8_t)g_mav_compid, ns);

    ++n;
  }

  if (ofs) ofs.close();
  ::close(fd);
}

// End Mavlink

int main(int argc, const char *argv[]) {
/** Set the program options. */
  SetProgramOption(argc, argv);

/** Initialize Livox-SDK. */
  if (!Init()) {
    return -1;
  }
  printf("Livox SDK has been initialized.\n");

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  printf("Livox SDK version %d.%d.%d .\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

  memset(devices, 0, sizeof(devices));

/** Set the callback function receiving broadcast message from Livox LiDAR. */
  SetBroadcastCallback(OnDeviceBroadcast);

/** Set the callback function called when device state change */
  SetDeviceStateUpdateCallback(OnDeviceInfoChange);

/** Start the device discovering routine. */
  if (!Start()) {
    Uninit();
    return -1;
  }
  printf("Start discovering device.\n");

  WaitForDevicesReady();

  AddDevicesToConnect();

  if (connected_lidar_count == 0) {
    printf("No device will be connected.\n");
    Uninit();
    return -1;
  }

  WaitForExtrinsicParameter();

  printf("Start initialize lvx file.\n");
  if (!lvx_file_handler.InitLvxFile()) {
    Uninit();
    return -1;
  }

  lvx_file_handler.InitLvxFileHeader();

  // mavlink
  // Lanzar hilo de timesync (log + MAVLink por minuto)
  std::thread ts_thread(timesync_worker);
  // end mavlink

  int i = 0;
  print_system_time();
  steady_clock::time_point last_time = steady_clock::now();
  for (i = 0; i < lvx_file_save_time * FRAME_RATE; ++i) {
    std::list<LvxBasePackDetail> point_packet_list_temp;
    {
      std::unique_lock<std::mutex> lock(mtx);
      point_pack_condition.wait_for(lock, milliseconds(kDefaultFrameDurationTime) - (steady_clock::now() - last_time));
      last_time = steady_clock::now();
      point_packet_list_temp.swap(point_packet_list);
    }
    if(point_packet_list_temp.empty()) {
      printf("Point cloud packet is empty.\n");
      break;
    }

    lvx_file_handler.SaveFrameToLvxFile(point_packet_list_temp);
  }

  printf("Closing File.\n");
  lvx_file_handler.CloseLvxFile();

  for (i = 0; i < kMaxLidarCount; ++i) {
    if (devices[i].device_state == kDeviceStateSampling) {
/** Stop the sampling of Livox LiDAR. */
      LidarStopSampling(devices[i].handle, OnStopSampleCallback, nullptr);
    }
  }

  // mavlink
  g_run_timesync.store(false);
  if (ts_thread.joinable()) ts_thread.join();
  // end mavlink

  /** Uninitialize Livox-SDK. */
  Uninit();
}