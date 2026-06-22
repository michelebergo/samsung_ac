# Samsung Heat Pump / Hydronic Module Sniffer Notes

## Goal
Read the **inlet water temperature** (and possibly outlet / flow / compressor / pump status) of a Samsung EHS heat pump connected to a hydronic module over the F1/F2 bus.

## What we know
- The physical layer is a **2400 baud, 8E1, 14-byte frame**:
  - `0x32` start
  - byte 1: source address
  - byte 2: destination address
  - byte 3: command
  - bytes 4-11: data
  - byte 12: XOR checksum (bytes 1-11)
  - `0x34` end
- The existing `samsung_ac` component uses addresses `0x84` (controller) and `0x20` (indoor unit) and commands `0x52`, `0x53`, `0x54`, `0x64`, `0xD1`, `0xA0`.
- Heat pumps / hydronic modules use **different addresses** and additional commands. Known Samsung EHS addresses from community work:
  - Outdoor unit: `0x20` or `0x40`
  - Indoor / hydro unit: `0x60` - `0x6F`
  - Wired remote / MIM: `0x80` - `0x8F`
  - Broadcast: `0xAD`

## Sniffer strategy
1. Flash `samsung_ac_sniffer.yaml` to an ESP32 connected to the F1/F2 bus.
2. Let it log every valid frame for several minutes while the heat pump runs.
3. Correlate frame bytes with the inlet water temperature shown on the official controller / app.
4. Identify the command byte and data offset for the inlet temperature.

## Common temperature encoding
Samsung HVAC temperatures in the existing component are encoded as:

```cpp
float temp_c = raw_byte - 55.0f;
```

or, for 16-bit values:

```cpp
float temp_c = ((high_byte << 8) + low_byte - 553) / 10.0f;
```

The hydronic module likely uses the 16-bit form for water temperatures (higher precision).

## Next steps after sniffing
Once the inlet temperature command/offset is identified, extend the sniffer or create a new `samsung_ehs` component that exposes:
- `sensor` for inlet water temperature
- `sensor` for outlet water temperature (if available)
- `sensor` for flow rate / pump speed (if available)
- `binary_sensor` for compressor / pump / defrost status

## Hardware notes
- The F1/F2 bus is typically 5V logic. Use a level-shifter or optocoupler to connect to a 3.3V ESP32.
- For passive sniffing, connect only RX through an optocoupler or voltage divider.
- Do not connect TX unless you intend to send commands; a passive sniffer is safer.

## References
- Existing component: `components/samsung_ac/`
- Sniffer component: `components/samsung_ac_sniffer/`
- Example config: `samsung_ac_sniffer.yaml`
