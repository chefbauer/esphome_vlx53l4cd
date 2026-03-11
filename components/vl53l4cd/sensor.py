import esphome.codegen as cg
from esphome.components import i2c, sensor
import esphome.config_validation as cv
from esphome.const import (
    ICON_ARROW_EXPAND_VERTICAL,
    STATE_CLASS_MEASUREMENT,
    UNIT_METER,
)

CODEOWNERS = ["@your-github-username"]
DEPENDENCIES = ["i2c"]

vl53l4cd_ns = cg.esphome_ns.namespace("vl53l4cd")
VL53L4CDSensor = vl53l4cd_ns.class_(
    "VL53L4CDSensor", sensor.Sensor, cg.PollingComponent, i2c.I2CDevice
)

CONF_TIMING_BUDGET_MS = "timing_budget_ms"
CONF_INTER_MEASUREMENT_MS = "inter_measurement_ms"
CONF_TIMEOUT_MS = "timeout_ms"


def validate_timing(config):
    if (
        config[CONF_INTER_MEASUREMENT_MS] != 0
        and config[CONF_INTER_MEASUREMENT_MS] <= config[CONF_TIMING_BUDGET_MS]
    ):
        raise cv.Invalid(
            "inter_measurement_ms must be 0 (continuous mode) or greater than timing_budget_ms"
        )
    return config


CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(
        VL53L4CDSensor,
        unit_of_measurement=UNIT_METER,
        icon=ICON_ARROW_EXPAND_VERTICAL,
        accuracy_decimals=3,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            cv.Optional(CONF_TIMING_BUDGET_MS, default=50): cv.int_range(
                min=10, max=200
            ),
            cv.Optional(CONF_INTER_MEASUREMENT_MS, default=0): cv.positive_int,
            cv.Optional(CONF_TIMEOUT_MS, default=500): cv.positive_int,
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x29)),
    validate_timing,
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_timing_budget_ms(config[CONF_TIMING_BUDGET_MS]))
    cg.add(var.set_inter_measurement_ms(config[CONF_INTER_MEASUREMENT_MS]))
    cg.add(var.set_timeout_ms(config[CONF_TIMEOUT_MS]))
