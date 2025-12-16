import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import (
    CONF_ID,
    CONF_TIMEOUT,
    ENTITY_CATEGORY_DIAGNOSTIC,
    CONF_ENGINEERING_MODE,
)

# 依赖与常量定义（移除 automation 依赖）
DEPENDENCIES = ["uart", "text_sensor", "sensor", "binary_sensor"]
AUTO_LOAD = ["sensor", "binary_sensor", "text_sensor"]
MULTI_CONF = True

# 配置项常量
CONF_MAX_DISTANCE = "max_distance"
CONF_HLK_LD2402_ID = "hlk_ld2402_id"
CONF_CALIBRATION_SENSITIVITY = "calibration_sensitivity"
CONF_ENGINEERING_MODE = "engineering_mode"

# 命名空间与核心类
hlk_ld2402_ns = cg.esphome_ns.namespace("hlk_ld2402")
HLKLD2402Component = hlk_ld2402_ns.class_(
    "HLKLD2402Component", cg.Component, uart.UARTDevice
)

# 配置校验 Schema
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HLKLD2402Component),
    cv.Optional(CONF_MAX_DISTANCE, default=5.0): cv.float_range(
        min=0.0, max=11.2,
        description="最大检测距离（米），对应雷达距离门：0=0.0-0.7m...15=10.5-11.2m"
    ),
    cv.Optional(CONF_TIMEOUT, default=5): cv.int_range(
        min=0, max=65535,
        description="目标消失延迟时间（秒），超出此时间判定为无人"
    ),
    cv.Optional(CONF_CALIBRATION_SENSITIVITY, default=1.0): cv.float_range(
        min=1.0,
        description="校准灵敏度乘数（1.0-4.0+），值越高抗干扰性越强"
    ),
    cv.Optional(CONF_ENGINEERING_MODE, default=False): cv.boolean(
        description="是否启用工程模式，启用后输出原始信号强度数据"
    ),
}).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

# 代码生成逻辑
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    
    # 传递配置参数到 C++ 层
    cg.add(var.set_max_distance(config[CONF_MAX_DISTANCE]))
    cg.add(var.set_timeout(config[CONF_TIMEOUT]))
    cg.add(var.set_calibration_sensitivity(config[CONF_CALIBRATION_SENSITIVITY]))
    cg.add(var.set_engineering_mode(config[CONF_ENGINEERING_MODE]))

# 【关键】删除所有 automation 相关代码（避免导入错误）
# 自动化功能可后续通过 lambda 调用，无需注册 action
