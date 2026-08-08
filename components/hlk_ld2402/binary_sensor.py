import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (
    CONF_ID,
    CONF_DEVICE_CLASS,
    DEVICE_CLASS_PRESENCE,
    DEVICE_CLASS_MOTION,
    DEVICE_CLASS_PROBLEM,
)

from . import HLKLD2402Component, CONF_HLK_LD2402_ID

# Define sensor types
CONF_POWER_INTERFERENCE = "power_interference"


def validate_config(config):
    # device_class "problem" must be a power interference sensor, otherwise
    # no setter would be registered and the entity would silently never update
    if (
        config.get(CONF_DEVICE_CLASS) == DEVICE_CLASS_PROBLEM
        and not config.get(CONF_POWER_INTERFERENCE, False)
    ):
        raise cv.Invalid(
            f"device_class '{DEVICE_CLASS_PROBLEM}' requires power_interference: true"
        )
    return config


# Define the schema for binary sensors - START BY EXTENDING THE BASE SCHEMA
CONFIG_SCHEMA = cv.All(
    binary_sensor.binary_sensor_schema().extend({
        cv.GenerateID(): cv.declare_id(binary_sensor.BinarySensor),
        cv.Required(CONF_HLK_LD2402_ID): cv.use_id(HLKLD2402Component),
        cv.Optional(CONF_DEVICE_CLASS): cv.one_of(DEVICE_CLASS_PRESENCE, DEVICE_CLASS_MOTION, DEVICE_CLASS_PROBLEM),
        cv.Optional(CONF_POWER_INTERFERENCE, default=False): cv.boolean,
    }),
    validate_config,
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_HLK_LD2402_ID])
    var = await binary_sensor.new_binary_sensor(config)
    
    if config.get(CONF_POWER_INTERFERENCE, False):
        cg.add(parent.set_power_interference_binary_sensor(var))
    elif CONF_DEVICE_CLASS in config:
        if config[CONF_DEVICE_CLASS] == DEVICE_CLASS_PRESENCE:
            cg.add(parent.set_presence_binary_sensor(var))
        elif config[CONF_DEVICE_CLASS] == DEVICE_CLASS_MOTION:
            cg.add(parent.set_micromovement_binary_sensor(var))
