#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <cstring>
#include <cmath>
#include <cstdio>
#include <cctype>
#include <cstdlib>
#include <algorithm>

namespace esphome {
namespace hlk_ld2402 {

static const uint8_t FRAME_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t FRAME_FOOTER[] = {0x04, 0x03, 0x02, 0x01};

// Data frame format
static const uint8_t DATA_FRAME_HEADER[] = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t DATA_FRAME_FOOTER[] = {0xF8, 0xF7, 0xF6, 0xF5};
static const uint8_t DATA_FRAME_TYPE_DISTANCE = 0x83;
static const uint8_t DATA_FRAME_TYPE_ENGINEERING = 0x84;

// Commands
static const uint16_t CMD_GET_VERSION = 0x0000;
static const uint16_t CMD_ENABLE_CONFIG = 0x00FF;
static const uint16_t CMD_DISABLE_CONFIG = 0x00FE;
static const uint16_t CMD_GET_SN_HEX = 0x0016;
static const uint16_t CMD_GET_SN_CHAR = 0x0011;
static const uint16_t CMD_GET_PARAMS = 0x0008;
static const uint16_t CMD_SET_PARAMS = 0x0007;
static const uint16_t CMD_SET_MODE = 0x0012;
static const uint16_t CMD_START_CALIBRATION = 0x0009;
static const uint16_t CMD_GET_CALIBRATION_STATUS = 0x000A;
static const uint16_t CMD_CALIBRATION_INTERFERENCE = 0x0014;
static const uint16_t CMD_SAVE_PARAMS = 0x00FD;
static const uint16_t CMD_AUTO_GAIN = 0x00EE;
static const uint16_t CMD_AUTO_GAIN_COMPLETE = 0x00F0;

// Parameters
static const uint16_t PARAM_MAX_DISTANCE = 0x0001;
static const uint16_t PARAM_TIMEOUT = 0x0004;
static const uint16_t PARAM_POWER_INTERFERENCE = 0x0005;
static const uint16_t PARAM_TRIGGER_THRESHOLD = 0x0010;
static const uint16_t PARAM_MICRO_THRESHOLD = 0x0030;

// Work modes
static const uint32_t MODE_PRODUCTION = 0x00000064;
static const uint32_t MODE_NORMAL = 0x00000064;
static const uint32_t MODE_CONFIG = 0x00000001;
static const uint32_t MODE_ENGINEERING = 0x00000004;

// Ranges / limits (from manual)
static constexpr float MAX_THEORETICAL_RANGE = 10.0f;
static constexpr float MOVEMENT_RANGE = 10.0f;
static constexpr float MICROMOVEMENT_RANGE = 6.0f;
static constexpr float STATIC_RANGE = 5.0f;
static constexpr float DISTANCE_PRECISION = 0.15f;
static constexpr float DISTANCE_GATE_SIZE = 0.7f;
static const uint8_t MAX_GATES = 32;
static const uint8_t DEFAULT_GATES = 16;

// Calibration coefficients
static const uint8_t DEFAULT_COEFF = 0x1E;  // 3.0
static const float MIN_COEFF = 1.0f;
static const float MAX_COEFF = 20.0f;

// --- 非阻塞引擎常量 ---
static const uint16_t RX_BUF_SIZE = 320;      // 工程数据帧约 278 字节
static const uint16_t TEXT_BUF_SIZE = 128;    // 文本行缓冲
static const uint32_t FRAME_TIMEOUT_MS = 500; // 半帧/半行超时
static const uint8_t MAX_OP_STEPS = 8;        // 单个操作脚本最大步骤数

// 命令响应处理返回值
static const uint8_t RESP_WAIT = 0;   // 收到响应但继续等待更多
static const uint8_t RESP_OK = 1;     // 命令成功
static const uint8_t RESP_FAIL = 2;   // 命令失败

// 操作脚本特殊步骤命令码
static const uint16_t OP_CMD_ENTER_CONFIG = 0xFFFF;
static const uint16_t OP_CMD_EXIT_CONFIG = 0xFFFE;
static const uint16_t OP_CMD_WAIT_CALIB = 0xFFFD;  // 等待校准完成（由轮询推进）
static const uint16_t OP_CMD_FINISH = 0xFFFC;

// 操作类型
enum class OpType : uint8_t {
  NONE,
  STARTUP,         // 启动初始化（进config -> 设Normal -> 出config）
  GET_VERSION,     // 主动读取固件版本
  POWER_CHECK,     // 电源干扰检查
  SET_THRESHOLD,   // 设置单门阈值
  READ_THRESHOLDS, // 批量读取阈值
  CALIBRATE,       // 自动校准
  SAVE_CONFIG,     // 保存参数到 flash
  AUTO_GAIN,       // 自动增益
  FACTORY_RESET,   // 恢复出厂（带参数 + 校准）
  SET_MODE,        // 切换工作模式
  GET_SN,          // 读取串号
};

// RX 帧组装状态机
enum class RxState : uint8_t { IDLE, CMD_FRAME, DATA_FRAME, TEXT };

class HLKLD2402Component;

// 响应处理器：解析收到的命令帧 payload（去掉帧头尾），
// 返回 RESP_WAIT / RESP_OK / RESP_FAIL
typedef uint8_t (*RespHandler)(HLKLD2402Component *self, const uint8_t *payload, size_t len);

// 操作脚本中的一步
struct OpStep {
  uint16_t cmd{OP_CMD_FINISH};
  RespHandler handler{nullptr};
  uint32_t timeout_ms{1000};
  uint8_t max_attempts{2};     // 命令响应超时后的重发次数
  bool continue_on_fail{false};  // 命令失败时继续下一步（如 SN 双格式回退）
  uint8_t data_len{0};
  uint8_t data[40];
};

class HLKLD2402Component : public Component, public uart::UARTDevice {
public:
  void set_distance_sensor(sensor::Sensor *distance_sensor) { distance_sensor_ = distance_sensor; }
  void set_distance_throttle(uint32_t throttle_ms) { distance_throttle_ms_ = throttle_ms; }
  void set_presence_binary_sensor(binary_sensor::BinarySensor *presence) { presence_binary_sensor_ = presence; }
  void set_micromovement_binary_sensor(binary_sensor::BinarySensor *micro) { micromovement_binary_sensor_ = micro; }
  void set_power_interference_binary_sensor(binary_sensor::BinarySensor *power_interference) {
    power_interference_binary_sensor_ = power_interference;
  }
  void set_max_distance(float max_distance) { max_distance_ = max_distance; }
  void set_timeout(uint32_t timeout) { timeout_ = timeout; }

  void set_firmware_version_text_sensor(text_sensor::TextSensor *version_sensor) {
    this->firmware_version_text_sensor_ = version_sensor;
  }

  void set_operating_mode_text_sensor(text_sensor::TextSensor *mode_sensor) {
    this->operating_mode_text_sensor_ = mode_sensor;
  }

  void set_calibration_progress_sensor(sensor::Sensor *calibration_progress) {
    calibration_progress_sensor_ = calibration_progress;
  }

  // 定长数组存储（ESP8266 防堆碎片）
  void set_energy_gate_sensor(uint8_t gate_index, sensor::Sensor *energy_sensor) {
    if (gate_index < MAX_GATES) {
      energy_gate_sensors_[gate_index] = energy_sensor;
      engineering_data_enabled_ = true;
    }
  }

  void set_still_energy_gate_sensor(uint8_t gate_index, sensor::Sensor *still_energy_sensor) {
    if (gate_index < MAX_GATES) {
      still_energy_gate_sensors_[gate_index] = still_energy_sensor;
      engineering_data_enabled_ = true;
    }
  }

  void set_motion_threshold_sensor(uint8_t gate_index, sensor::Sensor *threshold_sensor) {
    if (gate_index < MAX_GATES) {
      motion_threshold_sensors_[gate_index] = threshold_sensor;
    }
  }

  void set_micromotion_threshold_sensor(uint8_t gate_index, sensor::Sensor *threshold_sensor) {
    if (gate_index < MAX_GATES) {
      micromotion_threshold_sensors_[gate_index] = threshold_sensor;
    }
  }

  void setup() override;
  void loop() override;
  void dump_config() override;

  // 公共方法（签名全部保持不变，YAML/services.yaml 兼容）
  void calibrate();
  void save_config();
  void enable_auto_gain();
  void check_power_interference();
  void factory_reset();
  void factory_reset_with_params(float max_distance, int timeout);
  void set_engineering_mode_direct();
  void set_normal_mode_direct();
  void set_engineering_mode();
  void set_normal_mode();
  void get_serial_number();

  bool set_motion_threshold(uint8_t gate, float db_value);
  bool set_micromotion_threshold(uint8_t gate, float db_value);
  bool calibrate_with_coefficients(float trigger_coeff, float hold_coeff, float micromotion_coeff);

  void set_gate_motion_threshold(int gate, float db_value) {
    set_motion_threshold(gate, db_value);
  }

  void set_gate_micromotion_threshold(int gate, float db_value) {
    set_micromotion_threshold(gate, db_value);
  }

  bool get_all_motion_thresholds();
  bool get_all_micromotion_thresholds();

  void read_motion_thresholds() { get_all_motion_thresholds(); }
  void read_micromotion_thresholds() { get_all_micromotion_thresholds(); }

protected:
  // === 异步命令引擎 ===
  void start_command_(uint16_t command, const uint8_t *data, size_t len, RespHandler handler,
                      uint32_t timeout_ms, uint8_t max_attempts = 2);
  void run_command_();  // loop 中推进：发送 / 超时重试
  void command_finished_(bool success);

  // === 操作引擎（命令脚本）===
  bool queue_op_(OpType type);
  void advance_op_();
  void op_finish_(bool success);
  void op_fail_();  // 失败路径：尝试清理 config 状态后结束
  void build_script_();

  // === UART 接收 ===
  void process_uart_();
  void on_cmd_frame_(const uint8_t *frame, size_t len);
  void on_data_frame_(const uint8_t *frame, size_t len);
  void process_text_line_(const char *line, size_t len);
  void process_line_(const char *line, size_t len);
  void fallback_to_text_(uint16_t n);
  void reset_rx_();

  // === 帧/行处理 ===
  bool process_engineering_from_frame_(const uint8_t *frame_data, size_t frame_size);
  void update_binary_sensors_(float distance_cm);
  void publish_operating_mode_();
  void get_firmware_version_();
  uint32_t db_to_threshold_(float db_value);
  float threshold_to_db_(uint32_t threshold);

  // === 周期任务 ===
  void run_scheduled_();

  // === 响应处理器（静态成员，可访问私有状态）===
  static uint8_t resp_config_enter_(HLKLD2402Component *self, const uint8_t *p, size_t len);
  static uint8_t resp_config_exit_(HLKLD2402Component *self, const uint8_t *p, size_t len);
  static uint8_t resp_set_mode_(HLKLD2402Component *self, const uint8_t *p, size_t len);
  static uint8_t resp_set_param_(HLKLD2402Component *self, const uint8_t *p, size_t len);
  static uint8_t resp_get_param_(HLKLD2402Component *self, const uint8_t *p, size_t len);
  static uint8_t resp_calib_status_(HLKLD2402Component *self, const uint8_t *p, size_t len);
  static uint8_t resp_version_(HLKLD2402Component *self, const uint8_t *p, size_t len);
  static uint8_t resp_save_(HLKLD2402Component *self, const uint8_t *p, size_t len);
  static uint8_t resp_auto_gain_(HLKLD2402Component *self, const uint8_t *p, size_t len);
  static uint8_t resp_sn_hex_(HLKLD2402Component *self, const uint8_t *p, size_t len);
  static uint8_t resp_sn_char_(HLKLD2402Component *self, const uint8_t *p, size_t len);
  static uint8_t resp_batch_params_(HLKLD2402Component *self, const uint8_t *p, size_t len);
  static uint8_t resp_trigger_data_(HLKLD2402Component *self, const uint8_t *p, size_t len);

private:
  // 命令引擎状态
  bool cmd_busy_{false};
  uint8_t tx_frame_[64];
  uint8_t tx_len_{0};
  bool cmd_sent_{false};
  uint8_t cmd_attempt_{0};
  uint8_t cmd_max_attempts_{2};
  uint32_t cmd_deadline_{0};
  uint32_t cmd_timeout_ms_{1000};
  RespHandler cmd_handler_{nullptr};

  // 操作引擎状态
  OpType op_type_{OpType::NONE};
  uint8_t op_index_{0};
  bool op_entered_config_{false};
  bool op_result_{false};
  OpStep op_script_[MAX_OP_STEPS];
  float op_f1_{0.0f}, op_f2_{0.0f}, op_f3_{0.0f};
  uint8_t op_gate_{0};
  uint8_t op_sub_{0};   // 子类型：0=motion, 1=micromotion
  uint8_t op_mode_{0};  // 工作模式低字节

  // 接收状态机
  RxState rx_state_{RxState::IDLE};
  uint8_t rx_buf_[RX_BUF_SIZE];
  uint16_t rx_pos_{0};
  uint16_t rx_len_{0};
  char text_buf_[TEXT_BUF_SIZE];
  uint16_t text_pos_{0};
  uint32_t last_rx_byte_{0};
  uint32_t rx_count_{0};

  // 周期任务时间戳
  uint32_t startup_time_{0};
  bool firmware_check_done_{false};
  bool power_check_done_{false};
  uint32_t firmware_check_time_{0};
  uint32_t last_status_log_{0};
  uint32_t last_debug_log_{0};
  uint32_t last_calib_check_{0};
  bool calib_polling_{false};
  uint32_t last_eng_frame_{0};
  uint32_t eng_mode_start_{0};
  uint32_t last_eng_retry_{0};
  uint32_t last_eng_warn_{0};
  uint8_t eng_retry_count_{0};

  // 设备状态
  char firmware_version_[32];
  char operating_mode_[16];
  char serial_number_[32];
  bool config_mode_{false};
  bool power_interference_detected_{false};
  uint32_t calibration_progress_{0};
  bool calibration_in_progress_{false};
  uint32_t last_distance_update_{0};
  uint32_t distance_throttle_ms_{2000};
  uint32_t last_engineering_update_{0};
  uint32_t engineering_throttle_ms_{2000};

  // 实体指针
  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *calibration_progress_sensor_{nullptr};
  binary_sensor::BinarySensor *presence_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *micromovement_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *power_interference_binary_sensor_{nullptr};
  text_sensor::TextSensor *firmware_version_text_sensor_{nullptr};
  text_sensor::TextSensor *operating_mode_text_sensor_{nullptr};

  // 定长传感器指针数组（替代 std::vector）
  sensor::Sensor *energy_gate_sensors_[MAX_GATES] = {};
  sensor::Sensor *still_energy_gate_sensors_[MAX_GATES] = {};
  sensor::Sensor *motion_threshold_sensors_[MAX_GATES] = {};
  sensor::Sensor *micromotion_threshold_sensors_[MAX_GATES] = {};
  bool engineering_data_enabled_{false};

  // 阈值缓存（定长数组）
  float motion_threshold_values_[DEFAULT_GATES] = {};
  float micromotion_threshold_values_[DEFAULT_GATES] = {};

  float max_distance_{5.0};
  uint32_t timeout_{5};
};

}  // namespace hlk_ld2402
}  // namespace esphome
