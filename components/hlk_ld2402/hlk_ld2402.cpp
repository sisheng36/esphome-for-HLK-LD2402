#include "hlk_ld2402.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hlk_ld2402 {

static const char *const TAG = "hlk_ld2402";

// 构造一个操作脚本步骤
static OpStep step(uint16_t cmd, RespHandler handler = nullptr, uint32_t timeout_ms = 1000,
                   uint8_t max_attempts = 2, bool continue_on_fail = false,
                   const uint8_t *data = nullptr, uint8_t data_len = 0) {
  OpStep s;
  s.cmd = cmd;
  s.handler = handler;
  s.timeout_ms = timeout_ms;
  s.max_attempts = max_attempts;
  s.continue_on_fail = continue_on_fail;
  if (data != nullptr && data_len > 0) {
    memcpy(s.data, data, data_len);
    s.data_len = data_len;
  }
  return s;
}

// ============================== setup / loop ==============================

void HLKLD2402Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HLK-LD2402...");

  // UART 参数已由 YAML 配置，此处不再重复设置；setup 不进行任何通信
  startup_time_ = millis();
  last_status_log_ = startup_time_;
  last_debug_log_ = startup_time_;
  last_eng_frame_ = startup_time_;

  // 发布默认版本与模式（实际版本由 20s 后的异步检查更新）
  snprintf(firmware_version_, sizeof(firmware_version_), "%s", "HLK-LD2402");
  if (firmware_version_text_sensor_ != nullptr) {
    firmware_version_text_sensor_->publish_state(firmware_version_);
  }
  snprintf(operating_mode_, sizeof(operating_mode_), "%s", "Normal");
  publish_operating_mode_();

  // 延迟启动初始化（进config -> 设Normal -> 出config），全程非阻塞
  // 注意：本类 set_timeout(uint32_t) 是雷达超时参数 setter，会遮蔽
  // Component::set_timeout(ms, fn)，因此显式调用基类版本
  this->Component::set_timeout(5000, [this]() { queue_op_(OpType::STARTUP); });
}

void HLKLD2402Component::loop() {
  process_uart_();
  run_command_();
  run_scheduled_();
}

// ============================== 命令引擎 ==============================

void HLKLD2402Component::start_command_(uint16_t command, const uint8_t *data, size_t len,
                                        RespHandler handler, uint32_t timeout_ms,
                                        uint8_t max_attempts) {
  // 构造命令帧: FD FC FB FA | len(LE) | cmd(LE) | data | 04 03 02 01
  tx_len_ = 0;
  memcpy(tx_frame_ + tx_len_, FRAME_HEADER, 4);
  tx_len_ += 4;
  uint16_t payload_len = 2 + len;
  tx_frame_[tx_len_++] = payload_len & 0xFF;
  tx_frame_[tx_len_++] = (payload_len >> 8) & 0xFF;
  tx_frame_[tx_len_++] = command & 0xFF;
  tx_frame_[tx_len_++] = (command >> 8) & 0xFF;
  if (data != nullptr && len > 0) {
    memcpy(tx_frame_ + tx_len_, data, len);
    tx_len_ += len;
  }
  memcpy(tx_frame_ + tx_len_, FRAME_FOOTER, 4);
  tx_len_ += 4;

  cmd_busy_ = true;
  cmd_sent_ = false;
  cmd_attempt_ = 0;
  cmd_max_attempts_ = max_attempts;
  cmd_handler_ = handler;
  cmd_timeout_ms_ = timeout_ms;
}

void HLKLD2402Component::run_command_() {
  if (!cmd_busy_)
    return;
  uint32_t now = millis();

  if (!cmd_sent_) {
    // 发送前丢弃陈旧接收数据，避免陈旧帧被误判为响应
    reset_rx_();
    write_array(tx_frame_, tx_len_);
    cmd_sent_ = true;
    cmd_deadline_ = now + cmd_timeout_ms_;
    char hex_buf[128] = {0};
    for (size_t i = 0; i < tx_len_ && i * 3 + 2 < sizeof(hex_buf); i++) {
      sprintf(hex_buf + (i * 3), "%02X ", tx_frame_[i]);
    }
    ESP_LOGD(TAG, "Sent command (%u bytes): %s", tx_len_, hex_buf);
    return;
  }

  if (now >= cmd_deadline_) {
    if (cmd_attempt_ < cmd_max_attempts_) {
      cmd_attempt_++;
      cmd_sent_ = false;
      ESP_LOGD(TAG, "Response timeout, resending (attempt %u/%u)", cmd_attempt_,
               cmd_max_attempts_ + 1);
    } else {
      ESP_LOGW(TAG, "Command failed: no response within %u ms", cmd_timeout_ms_);
      command_finished_(false);
    }
  }
}

void HLKLD2402Component::command_finished_(bool success) {
  cmd_busy_ = false;
  cmd_handler_ = nullptr;

  if (op_type_ == OpType::NONE)
    return;  // 独立命令（如工程模式重触发），无需推进操作

  OpStep &s = op_script_[op_index_];
  if (s.cmd == OP_CMD_ENTER_CONFIG) {
    if (!success) {
      op_fail_();
      return;
    }
    config_mode_ = true;
    op_index_++;
  } else if (s.cmd == OP_CMD_EXIT_CONFIG) {
    if (!success)
      ESP_LOGW(TAG, "Exit config command failed, assuming device left config mode anyway");
    config_mode_ = false;
    op_entered_config_ = false;
    op_index_++;
  } else if (s.cmd == OP_CMD_WAIT_CALIB || s.cmd == OP_CMD_FINISH) {
    // 校准轮询命令完成，回到脚本检查校准状态
    advance_op_();
    return;
  } else {
    if (!success) {
      if (s.continue_on_fail) {
        op_index_++;
      } else {
        op_fail_();
        return;
      }
    } else {
      op_result_ = true;
      op_index_++;
    }
  }
  advance_op_();
}

// ============================== 操作引擎 ==============================

bool HLKLD2402Component::queue_op_(OpType type) {
  if (op_type_ != OpType::NONE || cmd_busy_) {
    ESP_LOGW(TAG, "Another operation is in progress, ignoring request");
    return false;
  }
  op_type_ = type;
  op_index_ = 0;
  op_entered_config_ = false;
  op_result_ = false;
  build_script_();
  advance_op_();
  return true;
}

void HLKLD2402Component::build_script_() {
  uint8_t i = 0;
  switch (op_type_) {
    case OpType::STARTUP:
      op_script_[i++] = step(OP_CMD_ENTER_CONFIG);
      {
        uint8_t d[6] = {0, 0, (uint8_t) (MODE_NORMAL & 0xFF), (uint8_t) ((MODE_NORMAL >> 8) & 0xFF),
                        0, 0};
        op_script_[i++] = step(CMD_SET_MODE, resp_set_mode_, 2000, 2, false, d, 6);
      }
      op_script_[i++] = step(OP_CMD_EXIT_CONFIG);
      op_script_[i++] = step(OP_CMD_FINISH);
      break;
    case OpType::GET_VERSION:
      op_script_[i++] = step(OP_CMD_ENTER_CONFIG);
      op_script_[i++] = step(CMD_GET_VERSION, resp_version_, 1500);
      op_script_[i++] = step(OP_CMD_EXIT_CONFIG);
      op_script_[i++] = step(OP_CMD_FINISH);
      break;
    case OpType::POWER_CHECK: {
      uint8_t d[2] = {(uint8_t) (PARAM_POWER_INTERFERENCE & 0xFF),
                      (uint8_t) ((PARAM_POWER_INTERFERENCE >> 8) & 0xFF)};
      op_script_[i++] = step(OP_CMD_ENTER_CONFIG);
      op_script_[i++] = step(CMD_GET_PARAMS, resp_get_param_, 3000, 2, false, d, 2);
      op_script_[i++] = step(OP_CMD_EXIT_CONFIG);
      op_script_[i++] = step(OP_CMD_FINISH);
      break;
    }
    case OpType::SET_THRESHOLD: {
      uint32_t threshold = db_to_threshold_(op_f1_);
      uint16_t param_id =
          (op_sub_ == 0) ? (PARAM_TRIGGER_THRESHOLD + op_gate_) : (PARAM_MICRO_THRESHOLD + op_gate_);
      uint8_t d[6] = {(uint8_t) (param_id & 0xFF), (uint8_t) ((param_id >> 8) & 0xFF),
                      (uint8_t) (threshold & 0xFF), (uint8_t) ((threshold >> 8) & 0xFF),
                      (uint8_t) ((threshold >> 16) & 0xFF), (uint8_t) ((threshold >> 24) & 0xFF)};
      op_script_[i++] = step(OP_CMD_ENTER_CONFIG);
      op_script_[i++] = step(CMD_SET_PARAMS, resp_set_param_, 1500, 2, false, d, 6);
      op_script_[i++] = step(OP_CMD_EXIT_CONFIG);
      op_script_[i++] = step(OP_CMD_FINISH);
      break;
    }
    case OpType::READ_THRESHOLDS: {
      // 批量读取 16 个门阈值: count(2 LE) + 16 * param_id(2 LE)
      uint8_t d[34];
      d[0] = DEFAULT_GATES;
      d[1] = 0;
      uint16_t base = (op_sub_ == 0) ? PARAM_TRIGGER_THRESHOLD : PARAM_MICRO_THRESHOLD;
      for (uint8_t g = 0; g < DEFAULT_GATES; g++) {
        d[2 + g * 2] = (base + g) & 0xFF;
        d[3 + g * 2] = (base + g) >> 8;
      }
      op_script_[i++] = step(OP_CMD_ENTER_CONFIG);
      op_script_[i++] = step(CMD_GET_PARAMS, resp_batch_params_, 2000, 2, false, d, 34);
      op_script_[i++] = step(OP_CMD_EXIT_CONFIG);
      op_script_[i++] = step(OP_CMD_FINISH);
      break;
    }
    case OpType::CALIBRATE: {
      uint16_t tv = (uint16_t) (op_f1_ * 10.0f);
      uint16_t hv = (uint16_t) (op_f2_ * 10.0f);
      uint16_t mv = (uint16_t) (op_f3_ * 10.0f);
      uint8_t d[6] = {(uint8_t) (tv & 0xFF), (uint8_t) (tv >> 8), (uint8_t) (hv & 0xFF),
                      (uint8_t) (hv >> 8), (uint8_t) (mv & 0xFF), (uint8_t) (mv >> 8)};
      op_script_[i++] = step(OP_CMD_ENTER_CONFIG);
      op_script_[i++] = step(CMD_START_CALIBRATION, resp_set_param_, 1500, 2, false, d, 6);
      op_script_[i++] = step(OP_CMD_WAIT_CALIB);
      op_script_[i++] = step(OP_CMD_EXIT_CONFIG);
      op_script_[i++] = step(OP_CMD_FINISH);
      break;
    }
    case OpType::SAVE_CONFIG:
      op_script_[i++] = step(OP_CMD_ENTER_CONFIG);
      op_script_[i++] = step(CMD_SAVE_PARAMS, resp_save_, 3000);
      op_script_[i++] = step(OP_CMD_EXIT_CONFIG);
      op_script_[i++] = step(OP_CMD_FINISH);
      break;
    case OpType::AUTO_GAIN:
      op_script_[i++] = step(OP_CMD_ENTER_CONFIG);
      op_script_[i++] = step(CMD_AUTO_GAIN, resp_auto_gain_, 10000, 0, false);  // 不重试，10s 超时
      op_script_[i++] = step(OP_CMD_EXIT_CONFIG);
      op_script_[i++] = step(OP_CMD_FINISH);
      break;
    case OpType::FACTORY_RESET: {
      uint32_t max_distance_dm = (uint32_t) (op_f1_ * 10.0f);
      uint32_t timeout_s = (uint32_t) op_f2_;
      uint8_t d1[6] = {(uint8_t) (PARAM_MAX_DISTANCE & 0xFF),
                       (uint8_t) ((PARAM_MAX_DISTANCE >> 8) & 0xFF),
                       (uint8_t) (max_distance_dm & 0xFF), (uint8_t) ((max_distance_dm >> 8) & 0xFF),
                       (uint8_t) ((max_distance_dm >> 16) & 0xFF),
                       (uint8_t) ((max_distance_dm >> 24) & 0xFF)};
      uint8_t d2[6] = {(uint8_t) (PARAM_TIMEOUT & 0xFF), (uint8_t) ((PARAM_TIMEOUT >> 8) & 0xFF),
                       (uint8_t) (timeout_s & 0xFF), (uint8_t) ((timeout_s >> 8) & 0xFF),
                       (uint8_t) ((timeout_s >> 16) & 0xFF), (uint8_t) ((timeout_s >> 24) & 0xFF)};
      uint8_t d3[6] = {DEFAULT_COEFF, 0, DEFAULT_COEFF, 0, DEFAULT_COEFF, 0};
      op_script_[i++] = step(OP_CMD_ENTER_CONFIG);
      op_script_[i++] = step(CMD_SET_PARAMS, resp_set_param_, 1500, 2, true, d1, 6);
      op_script_[i++] = step(CMD_SET_PARAMS, resp_set_param_, 1500, 2, true, d2, 6);
      op_script_[i++] = step(CMD_START_CALIBRATION, resp_set_param_, 1500, 2, false, d3, 6);
      op_script_[i++] = step(OP_CMD_WAIT_CALIB);
      op_script_[i++] = step(OP_CMD_EXIT_CONFIG);
      op_script_[i++] = step(OP_CMD_FINISH);
      break;
    }
    case OpType::SET_MODE: {
      uint8_t d[6] = {0, 0, op_mode_, 0, 0, 0};
      op_script_[i++] = step(OP_CMD_ENTER_CONFIG);
      op_script_[i++] = step(CMD_SET_MODE, resp_set_mode_, 2000, 2, false, d, 6);
      op_script_[i++] = step(OP_CMD_EXIT_CONFIG);
      op_script_[i++] = step(OP_CMD_FINISH);
      break;
    }
    case OpType::GET_SN:
      op_script_[i++] = step(OP_CMD_ENTER_CONFIG);
      op_script_[i++] = step(CMD_GET_SN_HEX, resp_sn_hex_, 1500, 2, true);
      op_script_[i++] = step(CMD_GET_SN_CHAR, resp_sn_char_, 1500);
      op_script_[i++] = step(OP_CMD_EXIT_CONFIG);
      op_script_[i++] = step(OP_CMD_FINISH);
      break;
    case OpType::NONE:
      break;
  }
  // 清除多余步骤
  for (; i < MAX_OP_STEPS; i++)
    op_script_[i] = OpStep();
}

void HLKLD2402Component::advance_op_() {
  if (cmd_busy_ || op_type_ == OpType::NONE)
    return;

  while (op_index_ < MAX_OP_STEPS) {
    OpStep &s = op_script_[op_index_];
    switch (s.cmd) {
      case OP_CMD_ENTER_CONFIG:
        if (config_mode_) {
          op_index_++;
          continue;
        }
        op_entered_config_ = true;
        start_command_(CMD_ENABLE_CONFIG, nullptr, 0, resp_config_enter_, 1500);
        return;
      case OP_CMD_EXIT_CONFIG:
        if (!config_mode_) {
          op_index_++;
          continue;
        }
        start_command_(CMD_DISABLE_CONFIG, nullptr, 0, resp_config_exit_, 1000);
        return;
      case OP_CMD_WAIT_CALIB:
        // 等待校准轮询将 calibration_in_progress_ 置 false
        if (calibration_in_progress_)
          return;
        op_index_++;
        continue;
      case OP_CMD_FINISH:
        op_finish_(op_result_);
        return;
      default:
        op_result_ = false;
        start_command_(s.cmd, s.data, s.data_len, s.handler, s.timeout_ms, s.max_attempts);
        return;
    }
  }
  op_finish_(op_result_);
}

void HLKLD2402Component::op_fail_() {
  // 清理：若本操作进入了 config 模式，直接发送退出命令（不等待响应）
  if (op_entered_config_ && config_mode_) {
    uint8_t frame[12];  // 帧头4 + 长度2 + 命令2 + 帧尾4
    size_t n = 0;
    memcpy(frame + n, FRAME_HEADER, 4);
    n += 4;
    frame[n++] = 0x02;
    frame[n++] = 0x00;
    frame[n++] = CMD_DISABLE_CONFIG & 0xFF;
    frame[n++] = (CMD_DISABLE_CONFIG >> 8) & 0xFF;
    memcpy(frame + n, FRAME_FOOTER, 4);
    n += 4;
    write_array(frame, n);
    config_mode_ = false;
    ESP_LOGW(TAG, "Operation failed, sent fire-and-forget exit config");
  }
  op_finish_(false);
}

void HLKLD2402Component::op_finish_(bool success) {
  OpType finished = op_type_;
  op_type_ = OpType::NONE;
  op_entered_config_ = false;

  const char *result = success ? "OK" : "FAILED";
  switch (finished) {
    case OpType::STARTUP:
      ESP_LOGI(TAG, "Startup initialization %s", success ? "complete" : "failed (continuing anyway)");
      break;
    case OpType::GET_VERSION:
      ESP_LOGI(TAG, "Firmware version check %s", result);
      break;
    case OpType::POWER_CHECK:
      ESP_LOGI(TAG, "Power interference check %s", result);
      break;
    case OpType::SET_THRESHOLD:
      ESP_LOGI(TAG, "Set gate %u threshold %s", op_gate_, result);
      break;
    case OpType::READ_THRESHOLDS:
      ESP_LOGI(TAG, "Read thresholds %s", result);
      break;
    case OpType::CALIBRATE:
      ESP_LOGI(TAG, "Calibration %s", result);
      calibration_in_progress_ = false;
      break;
    case OpType::SAVE_CONFIG:
      ESP_LOGI(TAG, "Save config %s", result);
      break;
    case OpType::AUTO_GAIN:
      ESP_LOGI(TAG, "Auto gain %s", result);
      break;
    case OpType::FACTORY_RESET:
      ESP_LOGI(TAG, "Factory reset %s", result);
      calibration_in_progress_ = false;
      break;
    case OpType::SET_MODE:
      ESP_LOGI(TAG, "Set mode %s", result);
      break;
    case OpType::GET_SN:
      ESP_LOGI(TAG, "Serial number read %s", result);
      break;
    case OpType::NONE:
      break;
  }
}

// ============================== 公共方法 ==============================

void HLKLD2402Component::calibrate() { calibrate_with_coefficients(3.0f, 3.0f, 3.0f); }

bool HLKLD2402Component::calibrate_with_coefficients(float trigger_coeff, float hold_coeff,
                                                     float micromotion_coeff) {
  trigger_coeff = std::max(MIN_COEFF, std::min(MAX_COEFF, trigger_coeff));
  hold_coeff = std::max(MIN_COEFF, std::min(MAX_COEFF, hold_coeff));
  micromotion_coeff = std::max(MIN_COEFF, std::min(MAX_COEFF, micromotion_coeff));

  if (op_type_ != OpType::NONE || cmd_busy_) {
    ESP_LOGW(TAG, "Busy, calibration request ignored");
    return false;
  }
  ESP_LOGI(TAG, "Starting calibration: trigger %.1f, hold %.1f, micro %.1f", trigger_coeff,
           hold_coeff, micromotion_coeff);
  op_type_ = OpType::CALIBRATE;
  op_f1_ = trigger_coeff;
  op_f2_ = hold_coeff;
  op_f3_ = micromotion_coeff;
  op_index_ = 0;
  op_entered_config_ = false;
  op_result_ = false;
  build_script_();
  advance_op_();
  return true;
}

void HLKLD2402Component::save_config() {
  if (queue_op_(OpType::SAVE_CONFIG)) {
    ESP_LOGI(TAG, "Save config queued");
  }
}

void HLKLD2402Component::enable_auto_gain() {
  if (queue_op_(OpType::AUTO_GAIN)) {
    ESP_LOGI(TAG, "Auto gain queued");
  }
}

void HLKLD2402Component::check_power_interference() {
  if (queue_op_(OpType::POWER_CHECK)) {
    ESP_LOGI(TAG, "Power interference check queued");
  }
}

void HLKLD2402Component::factory_reset() { factory_reset_with_params(5.0f, 5); }

void HLKLD2402Component::factory_reset_with_params(float max_distance, int timeout) {
  if (op_type_ != OpType::NONE || cmd_busy_) {
    ESP_LOGW(TAG, "Busy, factory reset ignored");
    return;
  }
  ESP_LOGI(TAG, "Factory reset with params: max_distance=%.1fm, timeout=%ds", max_distance, timeout);
  op_type_ = OpType::FACTORY_RESET;
  op_f1_ = max_distance;
  op_f2_ = (float) timeout;
  op_index_ = 0;
  op_entered_config_ = false;
  op_result_ = false;
  build_script_();
  advance_op_();
}

void HLKLD2402Component::set_engineering_mode_direct() {
  if (strcmp(operating_mode_, "Engineering") == 0) {
    ESP_LOGI(TAG, "Already in engineering mode, no action needed");
    return;
  }
  if (op_type_ != OpType::NONE || cmd_busy_) {
    ESP_LOGW(TAG, "Busy, mode switch ignored");
    return;
  }
  ESP_LOGI(TAG, "Switching to engineering mode...");
  op_type_ = OpType::SET_MODE;
  op_mode_ = (uint8_t) (MODE_ENGINEERING & 0xFF);
  op_index_ = 0;
  op_entered_config_ = false;
  op_result_ = false;
  build_script_();
  advance_op_();
}

void HLKLD2402Component::set_normal_mode_direct() {
  if (strcmp(operating_mode_, "Normal") == 0) {
    ESP_LOGI(TAG, "Already in normal mode, no action needed");
    return;
  }
  if (op_type_ != OpType::NONE || cmd_busy_) {
    ESP_LOGW(TAG, "Busy, mode switch ignored");
    return;
  }
  ESP_LOGI(TAG, "Switching to normal mode...");
  op_type_ = OpType::SET_MODE;
  op_mode_ = (uint8_t) (MODE_NORMAL & 0xFF);
  op_index_ = 0;
  op_entered_config_ = false;
  op_result_ = false;
  build_script_();
  advance_op_();
}

void HLKLD2402Component::set_engineering_mode() {
  if (strcmp(operating_mode_, "Engineering") == 0) {
    ESP_LOGI(TAG, "Already in engineering mode, switching back to normal");
    set_normal_mode_direct();
    return;
  }
  set_engineering_mode_direct();
}

void HLKLD2402Component::set_normal_mode() { set_normal_mode_direct(); }

void HLKLD2402Component::get_serial_number() {
  if (queue_op_(OpType::GET_SN)) {
    ESP_LOGI(TAG, "Serial number read queued");
  }
}

bool HLKLD2402Component::set_motion_threshold(uint8_t gate, float db_value) {
  if (gate >= DEFAULT_GATES) {
    ESP_LOGE(TAG, "Invalid gate index %u (must be 0-%u)", gate, DEFAULT_GATES - 1);
    return false;
  }
  db_value = std::max(0.0f, std::min(95.0f, db_value));
  if (op_type_ != OpType::NONE || cmd_busy_) {
    ESP_LOGW(TAG, "Busy, threshold setting ignored");
    return false;
  }
  ESP_LOGD(TAG, "Setting motion threshold for gate %u to %.1f dB", gate, db_value);
  op_type_ = OpType::SET_THRESHOLD;
  op_gate_ = gate;
  op_sub_ = 0;
  op_f1_ = db_value;
  op_index_ = 0;
  op_entered_config_ = false;
  op_result_ = false;
  build_script_();
  advance_op_();
  return true;
}

bool HLKLD2402Component::set_micromotion_threshold(uint8_t gate, float db_value) {
  if (gate >= DEFAULT_GATES) {
    ESP_LOGE(TAG, "Invalid gate index %u (must be 0-%u)", gate, DEFAULT_GATES - 1);
    return false;
  }
  db_value = std::max(0.0f, std::min(95.0f, db_value));
  if (op_type_ != OpType::NONE || cmd_busy_) {
    ESP_LOGW(TAG, "Busy, threshold setting ignored");
    return false;
  }
  ESP_LOGD(TAG, "Setting micromotion threshold for gate %u to %.1f dB", gate, db_value);
  op_type_ = OpType::SET_THRESHOLD;
  op_gate_ = gate;
  op_sub_ = 1;
  op_f1_ = db_value;
  op_index_ = 0;
  op_entered_config_ = false;
  op_result_ = false;
  build_script_();
  advance_op_();
  return true;
}

bool HLKLD2402Component::get_all_motion_thresholds() {
  if (op_type_ != OpType::NONE || cmd_busy_) {
    ESP_LOGW(TAG, "Busy, threshold read ignored");
    return false;
  }
  ESP_LOGI(TAG, "Reading all motion thresholds");
  op_type_ = OpType::READ_THRESHOLDS;
  op_sub_ = 0;
  op_index_ = 0;
  op_entered_config_ = false;
  op_result_ = false;
  build_script_();
  advance_op_();
  return true;
}

bool HLKLD2402Component::get_all_micromotion_thresholds() {
  if (op_type_ != OpType::NONE || cmd_busy_) {
    ESP_LOGW(TAG, "Busy, threshold read ignored");
    return false;
  }
  ESP_LOGI(TAG, "Reading all micromotion thresholds");
  op_type_ = OpType::READ_THRESHOLDS;
  op_sub_ = 1;
  op_index_ = 0;
  op_entered_config_ = false;
  op_result_ = false;
  build_script_();
  advance_op_();
  return true;
}

void HLKLD2402Component::get_firmware_version_() { queue_op_(OpType::GET_VERSION); }

// ============================== UART 接收状态机 ==============================

void HLKLD2402Component::reset_rx_() {
  rx_state_ = RxState::IDLE;
  rx_pos_ = 0;
  rx_len_ = 0;
  text_pos_ = 0;
}

void HLKLD2402Component::fallback_to_text_(uint16_t n) {
  // 帧头验证失败：已收到的字节按文本行处理
  if (text_pos_ + n < TEXT_BUF_SIZE - 1) {
    memcpy(text_buf_ + text_pos_, rx_buf_, n);
    text_pos_ += n;
  } else {
    text_pos_ = 0;
  }
  rx_state_ = RxState::TEXT;
}

void HLKLD2402Component::process_uart_() {
  while (available()) {
    uint8_t c;
    read_byte(&c);
    rx_count_++;
    last_rx_byte_ = millis();

    switch (rx_state_) {
      case RxState::IDLE:
        if (c == FRAME_HEADER[0]) {
          rx_state_ = RxState::CMD_FRAME;
          rx_pos_ = 1;
          rx_len_ = 0;
          rx_buf_[0] = c;
        } else if (c == DATA_FRAME_HEADER[0]) {
          rx_state_ = RxState::DATA_FRAME;
          rx_pos_ = 1;
          rx_len_ = 0;
          rx_buf_[0] = c;
        } else {
          rx_state_ = RxState::TEXT;
          text_pos_ = 0;
          text_buf_[text_pos_++] = (char) c;
        }
        break;

      case RxState::CMD_FRAME: {
        if (rx_pos_ < RX_BUF_SIZE)
          rx_buf_[rx_pos_++] = c;
        else {
          reset_rx_();
          break;
        }
        if (rx_pos_ == 4) {
          if (memcmp(rx_buf_, FRAME_HEADER, 4) != 0) {
            fallback_to_text_(4);
            break;
          }
        } else if (rx_pos_ == 6) {
          uint16_t len_field = rx_buf_[4] | (rx_buf_[5] << 8);
          if (len_field > RX_BUF_SIZE - 10) {
            reset_rx_();
            break;
          }
          rx_len_ = 10 + len_field;
        } else if (rx_pos_ == rx_len_) {
          on_cmd_frame_(rx_buf_, rx_pos_);
          reset_rx_();
          break;
        }
        break;
      }

      case RxState::DATA_FRAME: {
        if (rx_pos_ < RX_BUF_SIZE)
          rx_buf_[rx_pos_++] = c;
        else {
          ESP_LOGW(TAG, "Data frame exceeds %u bytes, discarding", RX_BUF_SIZE);
          reset_rx_();
          break;
        }
        if (rx_pos_ == 4) {
          if (memcmp(rx_buf_, DATA_FRAME_HEADER, 4) != 0) {
            fallback_to_text_(4);
            break;
          }
        } else if (rx_pos_ >= 4 && c == DATA_FRAME_FOOTER[3] &&
                   memcmp(rx_buf_ + rx_pos_ - 4, DATA_FRAME_FOOTER, 4) == 0) {
          // 帧尾 F8 F7 F6 F5 匹配（设备帧的 frame[5..6] 不是长度字段，
          // 与原版实现一致地使用头/尾定位完整帧）
          on_data_frame_(rx_buf_, rx_pos_);
          reset_rx_();
          break;
        }
        break;
      }

      case RxState::TEXT:
        if (c == '\n') {
          text_buf_[text_pos_] = '\0';
          process_text_line_(text_buf_, text_pos_);
          text_pos_ = 0;
          rx_state_ = RxState::IDLE;
        } else if (c != '\r') {
          if (text_pos_ < TEXT_BUF_SIZE - 1) {
            text_buf_[text_pos_++] = (char) c;
          } else {
            text_pos_ = 0;  // 溢出，丢弃该行
            rx_state_ = RxState::IDLE;
          }
        }
        break;
    }
  }
}

void HLKLD2402Component::on_cmd_frame_(const uint8_t *frame, size_t len) {
  if (len < 8)
    return;
  const uint8_t *payload = frame + 4;  // 去掉 4 字节帧头
  size_t payload_len = len - 8;        // 去掉 4 字节帧尾

  if (cmd_busy_ && cmd_sent_ && cmd_handler_ != nullptr) {
    uint8_t r = cmd_handler_(this, payload, payload_len);
    if (r == RESP_OK) {
      command_finished_(true);
    } else if (r == RESP_FAIL) {
      command_finished_(false);
    }
    // RESP_WAIT: 继续等待更多响应
  } else {
    ESP_LOGV(TAG, "Unexpected command frame (%u bytes)", payload_len);
  }
}

void HLKLD2402Component::on_data_frame_(const uint8_t *frame, size_t len) {
  last_eng_frame_ = millis();
  eng_retry_count_ = 0;  // 收到数据即视为数据流恢复
  process_engineering_from_frame_(frame, len);
}

void HLKLD2402Component::process_text_line_(const char *line, size_t len) {
  if (len == 0)
    return;
  ESP_LOGD(TAG, "Received line [%d bytes]: '%s'", len, line);

  process_line_(line, len);

  // 被动版本检测：默认版本（无 v 前缀）时从文本流提取版本号
  if (firmware_version_[0] != '\0' && strchr(firmware_version_, 'v') == nullptr &&
      strchr(firmware_version_, 'V') == nullptr) {
    for (size_t i = 0; i + 2 < len; i++) {
      bool is_version_start =
          (line[i] == 'v' || line[i] == 'V') ||
          (isdigit((unsigned char) line[i]) && line[i + 1] == '.' && isdigit((unsigned char) line[i + 2]));
      if (!is_version_start)
        continue;
      size_t start = (line[i] == 'v' || line[i] == 'V') ? i + 1 : i;
      size_t end = start;
      while (end < len && (isdigit((unsigned char) line[end]) || line[end] == '.'))
        end++;
      if (end > start) {
        size_t vlen = std::min(end - start, sizeof(firmware_version_) - 2);
        firmware_version_[0] = 'v';
        memcpy(firmware_version_ + 1, line + start, vlen);
        firmware_version_[1 + vlen] = '\0';
        if (firmware_version_text_sensor_ != nullptr) {
          firmware_version_text_sensor_->publish_state(firmware_version_);
        }
        ESP_LOGI(TAG, "Updated firmware version from passive detection: %s", firmware_version_);
        break;
      }
    }
  }
}

void HLKLD2402Component::process_line_(const char *line, size_t len) {
  // Handle OFF status
  if (len == 3 && strncmp(line, "OFF", 3) == 0) {
    ESP_LOGD(TAG, "No target detected");
    if (this->presence_binary_sensor_ != nullptr)
      this->presence_binary_sensor_->publish_state(false);
    if (this->micromovement_binary_sensor_ != nullptr)
      this->micromovement_binary_sensor_->publish_state(false);

    uint32_t now = millis();
    bool throttled =
        (this->distance_sensor_ != nullptr && now - last_distance_update_ < distance_throttle_ms_);
    if (!throttled && this->distance_sensor_ != nullptr) {
      this->distance_sensor_->publish_state(0);
      last_distance_update_ = now;
    }
    return;
  }

  // 提取距离数值：优先 "distance:" 前缀，其次整行纯数字
  const char *num_start = nullptr;
  const char *dist = strstr(line, "distance:");
  if (dist != nullptr) {
    num_start = dist + 9;
  } else {
    bool numeric = len > 0;
    for (size_t i = 0; i < len; i++) {
      if (!isdigit((unsigned char) line[i]) && line[i] != '.') {
        numeric = false;
        break;
      }
    }
    if (numeric)
      num_start = line;
  }

  if (num_start == nullptr)
    return;
  // "distance:" 后无数字时跳过（与原实现 strtof endptr 检查等效）
  if (!isdigit((unsigned char) num_start[0]) && num_start[0] != '.')
    return;

  float distance_cm = strtof(num_start, nullptr);

  // 更新二进制传感器
  update_binary_sensors_(distance_cm);

  // 距离传感器按节流发布
  uint32_t now = millis();
  bool throttled =
      (this->distance_sensor_ != nullptr && now - last_distance_update_ < distance_throttle_ms_);
  if (!throttled && this->distance_sensor_ != nullptr) {
    ESP_LOGV(TAG, "Detected distance (text): %.1f cm", distance_cm);
    this->distance_sensor_->publish_state(distance_cm);
    last_distance_update_ = now;
  }
}

void HLKLD2402Component::update_binary_sensors_(float distance_cm) {
  if (this->presence_binary_sensor_ != nullptr) {
    this->presence_binary_sensor_->publish_state(distance_cm <= (STATIC_RANGE * 100));
  }
  if (this->micromovement_binary_sensor_ != nullptr) {
    this->micromovement_binary_sensor_->publish_state(distance_cm <= (MICROMOVEMENT_RANGE * 100));
  }
}

void HLKLD2402Component::publish_operating_mode_() {
  if (operating_mode_text_sensor_ != nullptr) {
    operating_mode_text_sensor_->publish_state(operating_mode_);
    ESP_LOGI(TAG, "Published operating mode: %s", operating_mode_);
  }
}

void HLKLD2402Component::dump_config() {
  ESP_LOGCONFIG(TAG, "HLK-LD2402:");
  ESP_LOGCONFIG(TAG, "  Firmware Version: %s", firmware_version_);
  ESP_LOGCONFIG(TAG, "  Max Distance: %.1f m", max_distance_);
  ESP_LOGCONFIG(TAG, "  Timeout: %u s", timeout_);
}

// ============================== 工程数据帧解析 ==============================

bool HLKLD2402Component::process_engineering_from_frame_(const uint8_t *frame_data,
                                                         size_t frame_size) {
  if (!engineering_data_enabled_) {
    return false;
  }
  if (energy_gate_sensors_[0] == nullptr && still_energy_gate_sensors_[0] == nullptr) {
    return false;
  }
  if (frame_size < 10) {
    ESP_LOGW(TAG, "Engineering frame too short: %d bytes", frame_size);
    return false;
  }

  uint32_t now = millis();
  bool throttled = (now - last_engineering_update_ < engineering_throttle_ms_);

  // 验证帧格式（头尾）
  if (memcmp(frame_data, DATA_FRAME_HEADER, 4) != 0 ||
      memcmp(frame_data + frame_size - 4, DATA_FRAME_FOOTER, 4) != 0) {
    ESP_LOGW(TAG, "Invalid frame format (%u bytes)", frame_size);
    return false;
  }

  uint16_t data_length = frame_data[5] | (frame_data[6] << 8);
  bool is_complete_frame = (frame_size >= (9 + data_length));
  if (!throttled) {
    ESP_LOGV(TAG, "Engineering frame %s (%u bytes, reported length %u)",
             is_complete_frame ? "COMPLETE" : "TRUNCATED", frame_size, data_length);
  }

  // 解析检测结果与距离（偏移与原实现一致）
  if (frame_size >= 9) {
    uint8_t detection_status = frame_data[6];
    uint16_t target_distance = frame_data[7] | (frame_data[8] << 8);

    if (!throttled) {
      const char *status_text = "unknown";
      switch (detection_status) {
        case 0:
          status_text = "no person";
          break;
        case 1:
          status_text = "person";
          break;
        case 2:
          status_text = "stationary person";
          break;
      }
      ESP_LOGI(TAG, "Engineering mode - Detection: %s, Distance: %d cm", status_text,
               target_distance);

      if (this->distance_sensor_ != nullptr) {
        this->distance_sensor_->publish_state(target_distance);
      }
      if (this->presence_binary_sensor_ != nullptr) {
        this->presence_binary_sensor_->publish_state(detection_status == 1 || detection_status == 2);
      }
      if (this->micromovement_binary_sensor_ != nullptr) {
        this->micromovement_binary_sensor_->publish_state(detection_status == 2);
      }
    }
  }

  // 运动能量（前 16 门，从字节 9 开始，每门 4 字节）
  const size_t motion_energy_start = 9;
  for (uint8_t i = 0; i < DEFAULT_GATES; i++) {
    size_t offset = motion_energy_start + (i * 4);
    if (offset + 3 >= frame_size)
      break;
    uint32_t raw_energy = frame_data[offset] | (frame_data[offset + 1] << 8) |
                          (frame_data[offset + 2] << 16) | (frame_data[offset + 3] << 24);
    float db_energy = (raw_energy > 0) ? 10.0f * log10f((float) raw_energy) : 0.0f;
    if (!throttled && energy_gate_sensors_[i] != nullptr) {
      energy_gate_sensors_[i]->publish_state(db_energy);
    }
  }

  // 静止能量（后 16 门，从字节 73 开始）
  const size_t still_energy_start = motion_energy_start + (DEFAULT_GATES * 4);
  if (frame_size < still_energy_start + (DEFAULT_GATES * 4)) {
    ESP_LOGV(TAG, "Frame size %u insufficient for still energy data", frame_size);
  }
  for (uint8_t i = 0; i < DEFAULT_GATES; i++) {
    size_t offset = still_energy_start + (i * 4);
    if (offset + 3 >= frame_size)
      break;
    uint32_t raw_still_energy = frame_data[offset] | (frame_data[offset + 1] << 8) |
                                (frame_data[offset + 2] << 16) | (frame_data[offset + 3] << 24);
    float db_still_energy = (raw_still_energy > 0) ? 10.0f * log10f((float) raw_still_energy) : 0.0f;
    if (!throttled && still_energy_gate_sensors_[i] != nullptr) {
      still_energy_gate_sensors_[i]->publish_state(db_still_energy);
    }
  }

  if (!throttled) {
    last_engineering_update_ = now;
  }
  return true;
}

// ============================== 响应处理器 ==============================

uint8_t HLKLD2402Component::resp_config_enter_(HLKLD2402Component *self, const uint8_t *p,
                                               size_t len) {
  if (len >= 6 && ((p[0] == 0xFF && p[1] == 0x01 && p[2] == 0x00 && p[3] == 0x00) ||
                   (p[4] == 0x00 && p[5] == 0x00))) {
    self->config_mode_ = true;
    snprintf(self->operating_mode_, sizeof(self->operating_mode_), "%s", "Config");
    self->publish_operating_mode_();
    ESP_LOGI(TAG, "Entered config mode");
    return RESP_OK;
  }
  ESP_LOGW(TAG, "Invalid config mode response (%u bytes)", len);
  return RESP_FAIL;
}

uint8_t HLKLD2402Component::resp_config_exit_(HLKLD2402Component *self, const uint8_t *,
                                              size_t) {
  self->config_mode_ = false;
  if (strcmp(self->operating_mode_, "Config") == 0) {
    snprintf(self->operating_mode_, sizeof(self->operating_mode_), "%s", "Normal");
    // 退出配置模式回到 Normal 时同时禁用工程数据处理，
    // 避免正常模式下的 0x83 帧被误按工程帧解析
    self->engineering_data_enabled_ = false;
    self->publish_operating_mode_();
  }
  ESP_LOGI(TAG, "Left config mode");
  return RESP_OK;
}

uint8_t HLKLD2402Component::resp_set_mode_(HLKLD2402Component *self, const uint8_t *p, size_t len) {
  // 从脚本步骤数据中取请求的模式字节
  uint8_t mode_byte = self->op_script_[self->op_index_].data[2];

  bool success = false;
  if (len >= 2 && p[0] == 0x00 && p[1] == 0x00) {
    success = true;
    ESP_LOGI(TAG, "Work mode set successfully (standard ACK)");
  } else if (mode_byte == (MODE_ENGINEERING & 0xFF) && len >= 3 && p[0] == mode_byte &&
             p[2] == (CMD_SET_MODE & 0xFF)) {
    success = true;
    ESP_LOGI(TAG, "Engineering mode set successfully (device-specific response)");
  } else if (len >= 6 && p[0] == 0x04 && p[2] == (CMD_SET_MODE & 0xFF) && p[3] == 0x01 &&
             p[4] == 0x00 && p[5] == 0x00) {
    success = true;
    ESP_LOGI(TAG, "Normal mode set successfully (engineering exit response)");
  }

  if (success) {
    if (mode_byte == (MODE_ENGINEERING & 0xFF)) {
      snprintf(self->operating_mode_, sizeof(self->operating_mode_), "%s", "Engineering");
      self->engineering_data_enabled_ = true;
      // 重置工程模式看门狗计时，避免刚进入就触发重试
      self->last_eng_frame_ = millis();
      self->eng_mode_start_ = millis();
      self->eng_retry_count_ = 0;
    } else {
      snprintf(self->operating_mode_, sizeof(self->operating_mode_), "%s", "Normal");
      self->engineering_data_enabled_ = false;
    }
    self->publish_operating_mode_();
    return RESP_OK;
  }
  ESP_LOGE(TAG, "Invalid response to set work mode");
  return RESP_FAIL;
}

uint8_t HLKLD2402Component::resp_set_param_(HLKLD2402Component *self, const uint8_t *p,
                                            size_t len) {
  if (len < 2) {
    ESP_LOGE(TAG, "Set parameter response too short");
    return RESP_FAIL;
  }
  if (p[0] == 0xFF && p[1] == 0xFF) {
    ESP_LOGE(TAG, "Parameter setting failed with error response");
    return RESP_FAIL;
  }
  return RESP_OK;
}

uint8_t HLKLD2402Component::resp_get_param_(HLKLD2402Component *self, const uint8_t *p,
                                            size_t len) {
  bool has_interference = false;
  if (len >= 10) {
    uint32_t value = p[6] | (p[7] << 8) | (p[8] << 16) | (p[9] << 24);
    ESP_LOGI(TAG, "Power interference value: %u", value);
    if (value == 0) {
      ESP_LOGI(TAG, "Power interference check not performed");
    } else if (value == 1) {
      ESP_LOGI(TAG, "No power interference detected");
    } else if (value == 2) {
      ESP_LOGI(TAG, "Power interference detected");
      has_interference = true;
    } else {
      ESP_LOGW(TAG, "Unknown power interference value: %u", value);
      has_interference = (value != 1);
    }
  } else {
    ESP_LOGW(TAG, "Invalid power interference parameter response format");
    has_interference = true;
  }

  if (self->power_interference_binary_sensor_ != nullptr) {
    self->power_interference_binary_sensor_->publish_state(has_interference);
    ESP_LOGI(TAG, "Set power interference to %s", has_interference ? "ON" : "OFF");
  }
  return RESP_OK;
}

uint8_t HLKLD2402Component::resp_calib_status_(HLKLD2402Component *self, const uint8_t *p,
                                               size_t len) {
  bool handled = false;
  uint16_t progress = 0;

  // 实际设备格式: 06 00 0A 01 00 00 XX 00，进度在字节 6（原始值 0-0x64）
  if (!handled && len >= 8 && p[0] == 0x06 && p[1] == 0x00 && p[2] == 0x0A && p[3] == 0x01) {
    progress = p[6];
    uint16_t percentage = (progress * 100) / 0x64;
    if (percentage > 100)
      percentage = 100;
    ESP_LOGI(TAG, "Calibration progress: %u%% (raw value: %u)", percentage, progress);
    self->calibration_progress_ = percentage;
    if (self->calibration_progress_sensor_ != nullptr) {
      self->calibration_progress_sensor_->publish_state(percentage);
    }
    if (progress >= 0x64) {
      ESP_LOGI(TAG, "Calibration complete");
      self->calibration_in_progress_ = false;
    }
    handled = true;
  }

  // 文档格式: 00 00 + 2 字节百分比
  if (!handled && len >= 4 && p[0] == 0x00 && p[1] == 0x00) {
    progress = p[2] | (p[3] << 8);
    if (progress > 100) {
      ESP_LOGW(TAG, "Invalid calibration progress value: %u, capping to 100", progress);
      progress = 100;
    }
    ESP_LOGI(TAG, "Calibration progress: %u%% (standard format)", progress);
    self->calibration_progress_ = progress;
    if (self->calibration_progress_sensor_ != nullptr) {
      self->calibration_progress_sensor_->publish_state(progress);
    }
    if (progress >= 100) {
      ESP_LOGI(TAG, "Calibration complete");
      self->calibration_in_progress_ = false;
    }
    handled = true;
  }

  if (!handled) {
    ESP_LOGW(TAG, "Unrecognized calibration status response format");
  }
  return RESP_OK;
}

uint8_t HLKLD2402Component::resp_version_(HLKLD2402Component *self, const uint8_t *p, size_t len) {
  if (len >= 2) {
    uint16_t version_length = p[0] | (p[1] << 8);
    if (len >= 2 + version_length && version_length > 0) {
      size_t copy_len = std::min((size_t) version_length, sizeof(self->firmware_version_) - 1);
      memcpy(self->firmware_version_, p + 2, copy_len);
      self->firmware_version_[copy_len] = '\0';
      ESP_LOGI(TAG, "Got firmware version: %s", self->firmware_version_);
      if (self->firmware_version_text_sensor_ != nullptr) {
        self->firmware_version_text_sensor_->publish_state(self->firmware_version_);
      }
      return RESP_OK;
    }
    ESP_LOGW(TAG, "Invalid version string length in response");
    if (self->firmware_version_text_sensor_ != nullptr) {
      self->firmware_version_text_sensor_->publish_state("Invalid Response");
    }
    return RESP_OK;
  }
  ESP_LOGW(TAG, "Response too short for version data");
  if (self->firmware_version_text_sensor_ != nullptr) {
    self->firmware_version_text_sensor_->publish_state("Invalid Response Format");
  }
  return RESP_OK;
}

uint8_t HLKLD2402Component::resp_save_(HLKLD2402Component *self, const uint8_t *p, size_t len) {
  // 标准 ACK: 00 00
  if (len >= 2 && p[0] == 0x00 && p[1] == 0x00) {
    ESP_LOGI(TAG, "Save configuration acknowledged with standard ACK");
    return RESP_OK;
  }
  // 设备实际格式: [04 00][FD 01][00 00]
  if (len >= 6 && p[0] == 0x04 && p[1] == 0x00 && p[2] == 0xFD && p[4] == 0x00 && p[5] == 0x00) {
    ESP_LOGI(TAG, "Save configuration acknowledged with device-specific format");
    return RESP_OK;
  }
  // 其他非空响应：宽松处理
  if (len >= 2) {
    ESP_LOGW(TAG, "Received non-standard save response format but continuing");
    return RESP_OK;
  }
  ESP_LOGW(TAG, "Unrecognized save configuration response format");
  return RESP_FAIL;
}

uint8_t HLKLD2402Component::resp_auto_gain_(HLKLD2402Component *self, const uint8_t *p,
                                            size_t len) {
  // 等待完成通知帧 (F0)
  if (len >= 2 && p[0] == (CMD_AUTO_GAIN_COMPLETE & 0xFF) && p[1] == 0x00) {
    ESP_LOGI(TAG, "Auto gain adjustment completed");
    return RESP_OK;
  }
  return RESP_WAIT;  // 继续等待完成通知，超时由命令引擎处理
}

uint8_t HLKLD2402Component::resp_sn_hex_(HLKLD2402Component *self, const uint8_t *p, size_t len) {
  if (len >= 4 && p[0] == 0x00 && p[1] == 0x00) {
    uint16_t sn_length = p[2] | (p[3] << 8);
    if (len >= 4 + sn_length) {
      size_t max_bytes = (sizeof(self->serial_number_) - 1) / 2;
      size_t n = std::min((size_t) sn_length, max_bytes);
      size_t pos = 0;
      for (size_t i = 0; i < n; i++) {
        pos += sprintf(self->serial_number_ + pos, "%02X", p[4 + i]);
      }
      self->serial_number_[pos] = '\0';
      ESP_LOGI(TAG, "Serial number (hex): %s", self->serial_number_);
      return RESP_OK;
    }
  }
  return RESP_FAIL;
}

uint8_t HLKLD2402Component::resp_sn_char_(HLKLD2402Component *self, const uint8_t *p, size_t len) {
  if (len >= 4 && p[0] == 0x00 && p[1] == 0x00) {
    uint16_t sn_length = p[2] | (p[3] << 8);
    if (len >= 4 + sn_length) {
      size_t n = std::min((size_t) sn_length, sizeof(self->serial_number_) - 1);
      memcpy(self->serial_number_, p + 4, n);
      self->serial_number_[n] = '\0';
      ESP_LOGI(TAG, "Serial number (char): %s", self->serial_number_);
      return RESP_OK;
    }
  }
  return RESP_FAIL;
}

uint8_t HLKLD2402Component::resp_batch_params_(HLKLD2402Component *self, const uint8_t *p,
                                               size_t len) {
  const size_t data_offset = 10;  // 命令响应 4 字节 + 前导字段 6 字节
  if (len < data_offset + DEFAULT_GATES * 4) {
    ESP_LOGE(TAG, "Invalid batch parameter response format or insufficient data");
    return RESP_FAIL;
  }
  for (uint8_t i = 0; i < DEFAULT_GATES; i++) {
    size_t offset = data_offset + (i * 4);
    uint32_t value = p[offset] | (p[offset + 1] << 8) | (p[offset + 2] << 16) | (p[offset + 3] << 24);
    float db_value = self->threshold_to_db_(value);
    if (self->op_sub_ == 0) {
      self->motion_threshold_values_[i] = db_value;
      if (self->motion_threshold_sensors_[i] != nullptr) {
        self->motion_threshold_sensors_[i]->publish_state(db_value);
      }
    } else {
      self->micromotion_threshold_values_[i] = db_value;
      if (self->micromotion_threshold_sensors_[i] != nullptr) {
        self->micromotion_threshold_sensors_[i]->publish_state(db_value);
      }
    }
    ESP_LOGI(TAG, "Gate %u: %u (%.1f dB)", i, value, db_value);
  }
  return RESP_OK;
}

uint32_t HLKLD2402Component::db_to_threshold_(float db_value) {
  return static_cast<uint32_t>(pow(10, db_value / 10));
}

float HLKLD2402Component::threshold_to_db_(uint32_t threshold) {
  return 10 * log10(threshold);
}

// ============================== 周期任务 ==============================

void HLKLD2402Component::run_scheduled_() {
  uint32_t now = millis();

  // 一致性自愈：非工程模式下不应启用工程数据处理（如设备异常退出工程模式时）
  if (strcmp(operating_mode_, "Engineering") != 0 && engineering_data_enabled_) {
    ESP_LOGW(TAG, "Detected inconsistent state: engineering data enabled but not in engineering mode. Fixing...");
    engineering_data_enabled_ = false;
  }

  // 启动后 20s 执行固件版本检查
  if (!firmware_check_done_ && (now - startup_time_) > 20000) {
    firmware_check_done_ = true;
    firmware_check_time_ = now;
    ESP_LOGI(TAG, "Performing firmware version check");
    queue_op_(OpType::GET_VERSION);
  }

  // 版本检查完成后 3s 执行电源干扰检查（等上一个操作结束）
  if (firmware_check_done_ && !power_check_done_ && op_type_ == OpType::NONE &&
      !cmd_busy_ && (now - firmware_check_time_) > 3000) {
    power_check_done_ = true;
    ESP_LOGI(TAG, "Performing power interference check");
    queue_op_(OpType::POWER_CHECK);
  }

  // 校准进行中：每 5s 轮询一次进度
  if (calibration_in_progress_ && !cmd_busy_ && (now - last_calib_check_) >= 5000) {
    last_calib_check_ = now;
    start_command_(CMD_GET_CALIBRATION_STATUS, nullptr, 0, resp_calib_status_, 1500);
  }

  // 工程模式无数据看门狗：15s 无帧则重触发数据流，最多 3 次
  if (strcmp(operating_mode_, "Engineering") == 0) {
    if (eng_mode_start_ == 0)
      eng_mode_start_ = now;
    if ((now - last_eng_frame_) > 15000) {
      if (eng_retry_count_ < 3 && (now - last_eng_retry_) > 15000) {
        // fire-and-forget：直接发送参数读取命令触发数据流。
        // 不进入命令引擎 —— 数据帧流会淹没命令响应，等待只会超时
        uint8_t frame[12];  // 帧头4 + 长度2 + 命令2 + 参数2 + 帧尾4
        size_t n = 0;
        memcpy(frame + n, FRAME_HEADER, 4);
        n += 4;
        frame[n++] = 0x04;  // payload: 命令2 + 参数2
        frame[n++] = 0x00;
        frame[n++] = CMD_GET_PARAMS & 0xFF;
        frame[n++] = (CMD_GET_PARAMS >> 8) & 0xFF;
        frame[n++] = PARAM_MAX_DISTANCE & 0xFF;
        frame[n++] = (PARAM_MAX_DISTANCE >> 8) & 0xFF;
        memcpy(frame + n, FRAME_FOOTER, 4);
        n += 4;
        write_array(frame, n);
        eng_retry_count_++;
        last_eng_retry_ = now;
        ESP_LOGW(TAG, "No data in engineering mode, re-triggering data flow (attempt %u/3)",
                 eng_retry_count_);
      } else if (eng_retry_count_ >= 3 && (now - eng_mode_start_) > 60000 &&
                 (now - last_eng_warn_) > 60000) {
        ESP_LOGW(TAG, "Engineering mode not producing data after retries. Try power cycling the device.");
        last_eng_warn_ = now;
      }
    }
  } else {
    eng_mode_start_ = 0;
    eng_retry_count_ = 0;
    last_eng_retry_ = 0;
  }

  // 每 10s 状态日志
  if ((now - last_status_log_) > 10000) {
    ESP_LOGI(TAG, "Status: received %u bytes in last 10 seconds", rx_count_);
    rx_count_ = 0;
    last_status_log_ = now;
  }

  // 每 30s 调试日志
  if ((now - last_debug_log_) > 30000) {
    ESP_LOGD(TAG, "Waiting for data. Available bytes: %d", available());
    last_debug_log_ = now;
  }

  // 半帧/半行超时：丢弃不完整帧，未换行的文本行按完整行处理
  if (rx_state_ != RxState::IDLE && (now - last_rx_byte_) > FRAME_TIMEOUT_MS) {
    if (rx_state_ == RxState::TEXT && text_pos_ > 0) {
      text_buf_[text_pos_] = '\0';
      process_text_line_(text_buf_, text_pos_);
    }
    reset_rx_();
  }
}

}  // namespace hlk_ld2402
}  // namespace esphome
