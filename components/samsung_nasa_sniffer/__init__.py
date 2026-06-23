import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, text_sensor, sensor
from esphome.const import CONF_ID, CONF_UART_ID

samsung_nasa_sniffer_ns = cg.esphome_ns.namespace("samsung_nasa_sniffer")
SamsungNasaSniffer = samsung_nasa_sniffer_ns.class_(
    "SamsungNasaSniffer", cg.Component, uart.UARTDevice
)

CONF_LAST_PACKET = "last_packet"
CONF_TOTAL_PACKETS = "total_packets"
CONF_VALID_PACKETS = "valid_packets"
CONF_INVALID_PACKETS = "invalid_packets"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SamsungNasaSniffer),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Optional(CONF_TOTAL_PACKETS): sensor.sensor_schema(),
        cv.Optional(CONF_VALID_PACKETS): sensor.sensor_schema(),
        cv.Optional(CONF_INVALID_PACKETS): sensor.sensor_schema(),
        cv.Optional(CONF_LAST_PACKET): text_sensor.text_sensor_schema(),
    }
).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)


async def to_code(config):
    uart_dev = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID], uart_dev)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_TOTAL_PACKETS in config:
        sens = await sensor.new_sensor(config[CONF_TOTAL_PACKETS])
        cg.add(var.set_total_packets_sensor(sens))
    if CONF_VALID_PACKETS in config:
        sens = await sensor.new_sensor(config[CONF_VALID_PACKETS])
        cg.add(var.set_valid_packets_sensor(sens))
    if CONF_INVALID_PACKETS in config:
        sens = await sensor.new_sensor(config[CONF_INVALID_PACKETS])
        cg.add(var.set_invalid_packets_sensor(sens))
    if CONF_LAST_PACKET in config:
        sens = await text_sensor.new_text_sensor(config[CONF_LAST_PACKET])
        cg.add(var.set_last_packet_sensor(sens))
