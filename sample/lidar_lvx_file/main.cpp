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

// ===================== BEGIN GPIO (libgpiod) =====================
#include <gpiod.hpp>
#include <atomic>
#include <thread>
#include <fstream>
#include <mutex>
#include <sys/stat.h>

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

      // Get IMU uint8_t handle, LidarGetImuPushFrequencyCallback cb, void * data
      // LidarGetImuPushFrequency(handle, OnGetImu, nullptr);

      // Get pattern scan
      // LidarGetScanPattern(handle, OnGetScan, nullptr);

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
  return;
}

// void print_system_time() {
//     // Obtener tiempo actual
//     auto now = std::chrono::system_clock::now();

//     // Convertir a tiempo con precisión de segundos
//     std::time_t t = std::chrono::system_clock::to_time_t(now);
//     auto tm_info = *std::localtime(&t);

//     // Calcular milisegundos (o microsegundos)
//     auto duration = now.time_since_epoch();
//     auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() % 1000;

//     // Imprimir con milisegundos
//     std::cout << "Inicio a las ======================> "
//               << std::put_time(&tm_info, "%Y-%m-%d %H:%M:%S")
//               << "." << std::setfill('0') << std::setw(3) << millis
//               << std::endl;
// }

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

// ===================== BEGIN GPIO (libgpiod) =====================
namespace gpio_marker {

static std::atomic<bool> g_run{false};
static std::thread g_thread;
static std::ofstream g_log;
static std::mutex g_log_mtx;

// Convierte timespec a ns
inline uint64_t to_ns(const timespec& ts) {
  return uint64_t(ts.tv_sec) * 1000000000ull + uint64_t(ts.tv_nsec);
}

// ISO legible con milisegundos usando CLOCK_REALTIME
inline std::string iso_from_timespec(const timespec& ts_rt) {
  std::time_t s = ts_rt.tv_sec;
  std::tm tm{};
  localtime_r(&s, &tm);
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
                tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                tm.tm_hour, tm.tm_min, tm.tm_sec);
  int ms = ts_rt.tv_nsec / 1000000;
  char out[80];
  std::snprintf(out, sizeof(out), "%s.%03d", buf, ms);
  return std::string(out);
}

inline void now_timespec(clockid_t clk, timespec& ts) {
  clock_gettime(clk, &ts);
}

struct Config {
  std::string chip = "/dev/gpiochip0"; // Verifica con: gpiodetect
  unsigned line = 23;                   // BCM23 = pin físico 16
  std::string csv_path = "/var/log/livox_markers.csv";
  bool falling = false;                 // escuchamos ambos bordes
};

static Config cfg;

static bool file_exists_and_nonempty(const std::string& p) {
  struct stat st{};
  return stat(p.c_str(), &st) == 0 && st.st_size > 0;
}

static void write_header_if_new(std::ofstream& ofs) {
  // no escribir si ya existía con contenido
  // (abre primero g_log, luego chequea tamaño real del archivo con stat)
  // asumiendo que ya revisaste antes:
  ofs << "seq,edge,kernel_ts_ns,realtime_iso,realtime_ns,monotonic_ns\n";
  ofs.flush();
}

static void loop() {
  uint64_t seq = 0;
  try {
    gpiod::chip chip(cfg.chip);
    auto line = chip.get_line(cfg.line);

    gpiod::line_request req;
    req.consumer = "livox_marker";
    req.request_type = gpiod::line_request::EVENT_BOTH_EDGES;
    req.flags = gpiod::line_request::FLAG_BIAS_PULL_DOWN; // pull-down interno (si el SoC lo soporta)

    line.request(req);

    // Abrir CSV (append)
    {
      std::lock_guard<std::mutex> lk(g_log_mtx);
      bool existed = file_exists_and_nonempty(cfg.csv_path);
      g_log.open(cfg.csv_path, std::ios::out | std::ios::app);
      if (!g_log.is_open()) {
        std::cerr << "[GPIO] No se pudo abrir " << cfg.csv_path << "\n";
        return;
      }
      if (!existed) write_header_if_new(g_log);
    }

    std::cerr << "[GPIO] Escuchando en " << cfg.chip << " línea " << cfg.line
              << " -> log: " << cfg.csv_path << "\n";

    while (g_run.load()) {
      // espera con timeout
      if (!line.event_wait(std::chrono::milliseconds(200))) {
        continue; // timeout: reintenta y permite salida limpia
      }
      gpiod::line_event ev = line.event_read();

      timespec rt{}, mono{};
      now_timespec(CLOCK_REALTIME, rt);
      now_timespec(CLOCK_MONOTONIC_RAW, mono);

      const bool rising = (ev.event_type == gpiod::line_event::RISING_EDGE);

      const auto kern_ts = ev.timestamp;                  // chrono::nanoseconds
      const uint64_t kern_ns = static_cast<uint64_t>(kern_ts.count());

      {
        std::lock_guard<std::mutex> lk(g_log_mtx);
        g_log << ++seq << ","
              << (rising ? "RISING" : "FALLING") << ","
              << kern_ns << ","
              << iso_from_timespec(rt) << ","
              << to_ns(rt) << ","
              << to_ns(mono) << "\n";
        g_log.flush();
      } 
    }

  } catch (const std::exception& e) {
    std::cerr << "[GPIO] Excepción: " << e.what() << "\n";
  }
}

inline void start(const std::string& chip, unsigned line, const std::string& csv_path) {
  cfg.chip = chip;
  cfg.line = line;
  cfg.csv_path = csv_path;
  g_run.store(true);
  g_thread = std::thread(loop);
}

inline void stop() {
  g_run.store(false);
  // Desbloquear event_read: una forma limpia es enviar un pulso externo; si no,
  // deja que el proceso continúe hasta recibir el siguiente evento.
  if (g_thread.joinable()) g_thread.join();
  std::lock_guard<std::mutex> lk(g_log_mtx);
  if (g_log.is_open()) g_log.close();
}

} // namespace gpio_marker
// ===================== END GPIO (libgpiod) =====================


int main(int argc, const char *argv[]) {
/** Set the program options. */
  SetProgramOption(argc, argv);

  const std::string gpio_chip = "/dev/gpiochip0";
  const unsigned gpio_line = 23; // BCM23
  const std::string gpio_log_path = "/var/log/livox_markers.csv";
  gpio_marker::start(gpio_chip, gpio_line, gpio_log_path);

  // printf("Livox SDK initializing.\n");
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

/** Set the callback function called when device state change,
 * which means connection/disconnection and changing of LiDAR state.
 */
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

  // ====== BEGIN GPIO STOP ======
  gpio_marker::stop();
  // ====== END GPIO STOP ======

/** Uninitialize Livox-SDK. */
  Uninit();
}