# libsigrok with Rigol VS5000 support

This fork of [libsigrok](https://sigrok.org/wiki/Libsigrok) adds support for
the **Rigol VS5000** series of PC oscilloscopes (tested with a VS5042) to the
`rigol-ds` driver. The work is on the branch
[`rigol-vs5000`](https://github.com/FluffyPotatoKitten/libsigrok/tree/rigol-vs5000); `master` follows upstream.
The upstream README is in [`README`](../README).

The changes are split into commits that can be reviewed and merged upstream
one at a time:

| Commit | What it does |
| --- | --- |
| `output/analog` | Ignore analog packets without samples (fix, not VS5000 specific) |
| `output/csv` | Count only the channels that send samples (fix, not VS5000 specific) |
| `scpi_tcp` | tcp-rigol mode: send each command with CR LF in one TCP segment |
| `SR_CONF_TRIGGER_SWEEP` | New config key for the trigger sweep (Auto / Normal / Single) |
| `scpi` | New SCPI transport `usb-vs5000` for the vendor specific USB protocol |
| `contrib` | udev rule for USB ID 0400:03e8 |
| `rigol-ds` | The VS5000 support itself |

## What works

- **USB** (`usb-vs5000`, the scope does not implement USBTMC) and **LAN**
  (`tcp-rigol`).
- **Live** capture of the displayed waveform (2048 samples per channel,
  continuous frames).
- **Memory** capture of the full acquisition memory (512K samples per
  channel on the VS5042). Over USB always; over LAN only on firmware with
  the community fix, see below.
- Timebase, V/div, coupling, probe factor, offset, trigger source, slope,
  level and sweep. The input range that V/div and offset allow is reported
  per channel.

## Connection strings

    sigrok-cli -d rigol-ds:conn=usb-vs5000 --scan
    sigrok-cli -d rigol-ds:conn=usb-vs5000/<bus>.<address> ...
    sigrok-cli -d rigol-ds:conn=tcp-rigol/<scope-ip>/19 --scan

Example, 5 frames of the full memory over USB, as CSV:

    sigrok-cli -d rigol-ds:conn=usb-vs5000 --config data_source=Memory \
        --frames 5 -O csv -o capture.csv

For USB access without root, install `contrib/60-libsigrok.rules` (or the
whole `contrib` rules set) to `/etc/udev/rules.d/` and replug the scope.

## Firmware notes

- Firmware 03.00.00.01.00 (stock) was tested. Over LAN, the stock firmware
  corrupts or drops `:WAVEFORM:ALL?` replies and can lock up its LAN
  interface until a power cycle, so the driver offers **Memory** over LAN
  only when the scope reports a patched firmware (`*IDN?` version
  `03.00.00.01.P<n>`).
- The stock firmware can also hang over USB (power cycle needed) when a USB
  memory read coincides with its own display readout. This was found with
  the vendor software; the firmware fix removes it.
- The firmware fix (sources and tools, applied to your own flash backup over
  JTAG) is in [vs5000-fwpatch](https://github.com/FluffyPotatoKitten/vs5000-fwpatch).
- Numbers written in exponent notation (`4e-3`) are silently ignored by the
  firmware; the driver always sends plain decimals.

## Known limitations

- The horizontal trigger position is sent with 6 decimals, so offsets below
  1 µs are rounded away.
- With a large horizontal offset the live update rate drops (one frame per
  offset time plus about 35 ms). This is how the scope works.

## Building

    git clone -b rigol-vs5000 https://github.com/FluffyPotatoKitten/libsigrok.git
    cd libsigrok
    ./autogen.sh
    ./configure --prefix=$HOME/.local/sigrok
    make -j$(nproc) install

To build only what is needed for the VS5000 add
`--disable-all-drivers --enable-rigol-ds`, and `--enable-cxx` if you want to
build PulseView against it. Use the
[PulseView fork](https://github.com/FluffyPotatoKitten/pulseview/tree/rigol-vs5000)
for the trigger sweep, offset and input range controls.

## License

GPL-3.0 or later.
