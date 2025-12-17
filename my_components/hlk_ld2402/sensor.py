import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_DISTANCE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CENTIMETER,
    UNIT_PERCENT,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

from . import HLKLD2402Component, CONF_HLK_LD2402_ID

CONF_THROTTLE = "throttle"
CONF_CALIBRATION_PROGRESS = "calibration_progress"
CONF_ENERGY_GATE = "energy_gate"  # Motion energy gate sensors (前16个gate)
CONF_STILL_ENERGY_GATE = "still_energy_gate"  # Still energy gate sensors (后16个gate)
CONF_GATE_INDEX = "gate_index"     # Gate number (0-15)
CONF_MOTION_THRESHOLD = "motion_threshold"  # Motion threshold sensors
CONF_MICROMOTION_THRESHOLD = "micromotion_threshold"  # Micromotion threshold sensors

# Update schema to include still energy gate sensors
CONFIG_SCHEMA = sensor.sensor_schema().extend({
    cv.GenerateID(): cv.declare_id(sensor.Sensor),
    cv.Required(CONF_HLK_LD2402_ID): cv.use_id(HLKLD2402Component),
    cv.Optional(CONF_THROTTLE): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_CALIBRATION_PROGRESS, default=False): cv.boolean,
    cv.Optional(CONF_ENERGY_GATE): cv.Schema({
        cv.Required(CONF_GATE_INDEX): cv.int_range(0, 15),
    }),
    # 添加静止能量门配置
    cv.Optional(CONF_STILL_ENERGY_GATE): cv.Schema({
        cv.Required(CONF_GATE_INDEX): cv.int_range(0, 15),
    }),
    cv.Optional(CONF_MOTION_THRESHOLD): cv.Schema({
        cv.Required(CONF_GATE_INDEX): cv.int_range(0, 15),
    }),
    cv.Optional(CONF_MICROMOTION_THRESHOLD): cv.Schema({
        cv.Required(CONF_GATE_INDEX): cv.int_range(0, 15),
    }),
})

async def to_code(config):
    parent = await cg.get_variable(config[CONF_HLK_LD2402_ID])
    var = await sensor.new_sensor(config)
    
    if CONF_ENERGY_GATE in config:
        gate_index = config[CONF_ENERGY_GATE][CONF_GATE_INDEX]
        cg.add(parent.set_energy_gate_sensor(gate_index, var))
    # 添加静止能量门传感器处理
    elif CONF_STILL_ENERGY_GATE in config:
        gate_index = config[CONF_STILL_ENERGY_GATE][CONF_GATE_INDEX]
        cg.add(parent.set_still_energy_gate_sensor(gate_index, var))
    elif CONF_MOTION_THRESHOLD in config:
        gate_index = config[CONF_MOTION_THRESHOLD][CONF_GATE_INDEX]
        cg.add(parent.set_motion_threshold_sensor(gate_index, var))
    elif CONF_MICROMOTION_THRESHOLD in config:
        gate_index = config[CONF_MICROMOTION_THRESHOLD][CONF_GATE_INDEX]
        cg.add(parent.set_micromotion_threshold_sensor(gate_index, var))
    elif config.get(CONF_CALIBRATION_PROGRESS):
        # This is a calibration progress sensor
        cg.add(parent.set_calibration_progress_sensor(var))
    else:
        # This is a regular distance sensor
        cg.add(parent.set_distance_sensor(var))
        if CONF_THROTTLE in config:
            cg.add(parent.set_distance_throttle(config[CONF_THROTTLE]))
