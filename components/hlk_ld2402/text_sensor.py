import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

from . import HLKLD2402Component, CONF_HLK_LD2402_ID

# Define text sensor types
CONF_FIRMWARE_VERSION = "firmware_version"
CONF_OPERATING_MODE = "operating_mode"

# Define schema with optional sensor types
CONFIG_SCHEMA = text_sensor.text_sensor_schema(
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC
).extend({
    cv.GenerateID(): cv.declare_id(text_sensor.TextSensor),
    cv.Required(CONF_HLK_LD2402_ID): cv.use_id(HLKLD2402Component),
    cv.Optional(CONF_FIRMWARE_VERSION, default=False): cv.boolean,
    cv.Optional(CONF_OPERATING_MODE, default=False): cv.boolean,
})

async def to_code(config):
    parent = await cg.get_variable(config[CONF_HLK_LD2402_ID])
    var = await text_sensor.new_text_sensor(config)
    
    if config.get(CONF_FIRMWARE_VERSION):
        cg.add(parent.set_firmware_version_text_sensor(var))
    elif config.get(CONF_OPERATING_MODE):
        cg.add(parent.set_operating_mode_text_sensor(var))
