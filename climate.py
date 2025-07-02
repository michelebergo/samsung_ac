import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, uart, binary_sensor, sensor
from esphome.const import CONF_ID, CONF_UART_ID

samsung_ac_ns = cg.esphome_ns.namespace("samsung_ac")
SamsungAcComponent = samsung_ac_ns.class_("SamsungAcComponent", cg.Component, climate.Climate, uart.UARTDevice)

CONFIG_SCHEMA = climate.climate_schema(SamsungAcComponent).extend({
    cv.Optional("quiet_mode"): cv.use_id(binary_sensor.BinarySensor),
    cv.Optional("reset_clean_filter_msg"): cv.use_id(binary_sensor.BinarySensor),
    cv.Optional("blade_position"): cv.use_id(sensor.Sensor),
    cv.Optional("unit", default=0): cv.uint8_t,
    cv.Optional("source_address", default=0): cv.uint8_t,
}).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    uart_dev = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID], uart_dev)
    await cg.register_component(var, config)
    await climate.register_climate(var, config)
    await uart.register_uart_device(var, config)

    if "quiet_mode" in config:
        sens = await cg.get_variable(config["quiet_mode"])
        cg.add(var.set_quiet_mode(sens))

    if "reset_clean_filter_msg" in config:
        sens = await cg.get_variable(config["reset_clean_filter_msg"])
        cg.add(var.set_reset_clean_filter_msg(sens))

    if "blade_position" in config:
        sens = await cg.get_variable(config["blade_position"])
        cg.add(var.set_blade_position(sens))

