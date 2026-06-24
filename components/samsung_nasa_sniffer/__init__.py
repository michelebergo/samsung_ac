import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, text_sensor, sensor
from esphome.const import CONF_ID, CONF_UART_ID

samsung_nasa_sniffer_ns = cg.esphome_ns.namespace("samsung_nasa_sniffer")
SamsungNasaSniffer = samsung_nasa_sniffer_ns.class_(
    "SamsungNasaSniffer", cg.Component, uart.UARTDevice
)

CONF_SAMSUNG_NASA_SNIFFER_ID = "samsung_nasa_sniffer_id"
CONF_LAST_PACKET = "last_packet"
CONF_TOTAL_PACKETS = "total_packets"
CONF_VALID_PACKETS = "valid_packets"
CONF_INVALID_PACKETS = "invalid_packets"

SNIFFER_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SAMSUNG_NASA_SNIFFER_ID): cv.use_id(SamsungNasaSniffer),
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SamsungNasaSniffer),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    }
).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)


async def to_code(config):
    uart_dev = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID], uart_dev)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)


@sensor.register_sensor("samsung_nasa_sniffer", SNIFFER_SCHEMA)
async def samsung_nasa_sniffer_sensor_to_code(config, var_id):
    parent = await cg.get_variable(config[CONF_SAMSUNG_NASA_SNIFFER_ID])
    var = cg.new_Pvariable(var_id, parent)
    return var


@text_sensor.register_text_sensor("samsung_nasa_sniffer", SNIFFER_SCHEMA)
async def samsung_nasa_sniffer_text_sensor_to_code(config, var_id):
    parent = await cg.get_variable(config[CONF_SAMSUNG_NASA_SNIFFER_ID])
    var = cg.new_Pvariable(var_id, parent)
    return var
