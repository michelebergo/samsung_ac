# Samsung NASA Protocol Sniffer

Passive ESPHome component that logs every NASA packet seen on the Samsung
F1/F2 bus. Useful to confirm baud rate, polarity, and to discover device
addresses before enabling the full `esphome_samsung_hvac_bus` component.

## What it does
- Syncs on the NASA start byte `0x32`
- Reads the 16-bit length field
- Validates end byte `0x34` and CRC16
- Logs source/destination addresses, command, and message sets as hex
- Exposes counters for total/valid/invalid packets

## What it does NOT do
- Does not decode message numbers into temperatures
- Does not write to the bus (passive only)

## Wiring
Connect an ESP32 UART RX to the F1/F2 bus through an optocoupler or
RS-485 transceiver. Leave TX unconnected for passive sniffing.

## Configuration

```yaml
external_components:
  - source:
      type: local
      path: components
    components: [samsung_nasa_sniffer]

uart:
  id: samsung_bus
  tx_pin: GPIO17   # required by ESPHome UART but unused
  rx_pin: GPIO16
  baud_rate: 9600
  parity: EVEN

sensor:
  - platform: samsung_nasa_sniffer
    uart_id: samsung_bus
    total_packets:
      name: "NASA Total Packets"
    valid_packets:
      name: "NASA Valid Packets"
    invalid_packets:
      name: "NASA Invalid Packets"

text_sensor:
  - platform: samsung_nasa_sniffer
    uart_id: samsung_bus
    last_packet:
      name: "NASA Last Packet"
```

## Next step
Once you see valid packets, note the addresses (e.g. `20.00.00`, `10.00.00`)
and switch to `samsung_ehs_nasa_example.yaml` with the full NASA component.
