# libsigrok with Rigol VS5000 support and DS1000Z fixes

This fork of [libsigrok](https://sigrok.org/wiki/Libsigrok) extends the
`rigol-ds` driver:

- **Rigol VS5000** series of PC oscilloscopes (tested with a VS5042): new
  support over USB and LAN.
- **Rigol DS1000Z** series (tested with a DS1054Z): fixes for USB at high
  speed, memory captures and the trigger sweep, and faster downloads.

The upstream README is in [`README`](../README).

## Branches

| Branch | Contents |
| --- | --- |
| [`rigol-combined`](https://github.com/FluffyPotatoKitten/libsigrok/tree/rigol-combined) (default) | Everything below, merged: **build this one** |
| [`rigol-vs5000`](https://github.com/FluffyPotatoKitten/libsigrok/tree/rigol-vs5000) | VS5000 support only |
| [`rigol-ds1000z`](https://github.com/FluffyPotatoKitten/libsigrok/tree/rigol-ds1000z) | DS1000Z fixes only |
| `master` | Follows upstream |

The two topic branches start from upstream and are split into commits that
can be reviewed and merged upstream one at a time.

## Building

    git clone https://github.com/FluffyPotatoKitten/libsigrok.git
    cd libsigrok
    ./autogen.sh
    ./configure --prefix=$HOME/.local/sigrok
    make -j$(nproc) install

To build only the Rigol driver add `--disable-all-drivers --enable-rigol-ds`,
and `--enable-cxx` if you want to build PulseView against it. Use the
[PulseView fork](https://github.com/FluffyPotatoKitten/pulseview/tree/rigol-vs5000)
for the trigger sweep, offset and input range controls.

For USB access without root, install `contrib/60-libsigrok.rules` (or the
whole `contrib` rules set) to `/etc/udev/rules.d/` and replug the scope.

## Rigol VS5000

| Commit (`rigol-vs5000`) | What it does |
| --- | --- |
| `output/analog` | Ignore analog packets without samples (not VS5000 specific) |
| `output/csv` | Count only the channels that send samples (not VS5000 specific) |
| `scpi_tcp` | tcp-rigol mode: send each command with CR LF in one TCP segment |
| `SR_CONF_TRIGGER_SWEEP` | New config key for the trigger sweep (Auto / Normal / Single) |
| `scpi` | New SCPI transport `usb-vs5000` for the vendor specific USB protocol |
| `contrib` | udev rule for USB ID 0400:03e8 |
| `rigol-ds` | The VS5000 support itself |
| `rigol-ds`: memory depth | 4K or 512K points for memory captures (SR_CONF_BUFFERSIZE) |

**What works:**

- **USB** (`usb-vs5000`, the scope does not implement USBTMC) and **LAN**
  (`tcp-rigol`).
- **Live** capture of the displayed waveform (2048 samples per channel).
- **Memory** capture of the full acquisition memory (512K samples per
  channel on the VS5042). Over USB always; over LAN only on firmware with
  the community fix, see below.
- Timebase, V/div, coupling, probe factor, offset, trigger source, slope,
  level and sweep. The input range that V/div and offset allow is reported
  per channel.
- Memory depth of memory captures: 512K points at the full rate (default)
  or 4K, the display record.

**Connection strings:**

    sigrok-cli -d rigol-ds:conn=usb-vs5000 --scan
    sigrok-cli -d rigol-ds:conn=usb-vs5000/<bus>.<address> ...
    sigrok-cli -d rigol-ds:conn=tcp-rigol/<scope-ip>/19 --scan

Example, 5 frames of the full memory over USB, as CSV:

    sigrok-cli -d rigol-ds:conn=usb-vs5000 --config data_source=Memory \
        --frames 5 -O csv -o capture.csv

**Firmware notes:**

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

**Known limitations:**

- The horizontal trigger position is sent with 6 decimals, so offsets below
  1 µs are rounded away.
- With a large horizontal offset the live update rate drops (one frame per
  offset time plus about 35 ms). This is how the scope works.
- The trigger level is limited by the scope to the screen range of the
  trigger channel.

## Rigol DS1000Z

| Commit (`rigol-ds1000z`) | What it does |
| --- | --- |
| `scpi_usbtmc`: limit message size | At high speed the DS1000Z declares 64 byte bulk endpoints (512 are required) and silently sends stale data in messages longer than 512 bytes: request at most 500 data bytes per message |
| `scpi_usbtmc`: fill the read buffer | Read across transfers and messages until the buffer is full (memory downloads were about 1 KB/s) |
| `scpi_usbtmc`: empty bulk in start | Log the empty transfer the DS1000Z sends before many messages at debug level |
| `scpi_usbtmc`: transfers in flight | For such devices, keep up to 16 one-packet transfers in flight while a message is read |
| `rigol-ds`: single shot per memory frame | Memory captures take a new shot per frame again (broken since 2020: they waited forever or read old memory) |
| `rigol-ds`: memory depth and sample rate | Read `:ACQ:MDEP?` (the driver assumed 12M points, read junk beyond the real depth) and `:ACQ:SRAT?` |
| `rigol-ds`: wait for the new shot | Wait until the new shot really runs, restore the trigger sweep and run state afterwards, force a trigger when the sweep was Auto |
| `rigol-ds`: live with the Single sweep | One new single shot per live frame instead of the same waveform again |
| `rigol-ds`: 250k point blocks | Read memory in blocks of 250000 points and poll fast |
| `rigol-ds`: fewer `*OPC?` | No `*OPC?` after the block commands, fuller reads (much faster over LAN) |
| `SR_CONF_TRIGGER_SWEEP` | The new config key (the same commit as on `rigol-vs5000`) |
| `rigol-ds`: trigger sweep for V3 and later | Auto / Normal / Single for the DS2000, DS1000Z, DS4000 and MSO5000 (tested with the DS1000Z) |
| `rigol-ds`: sample rate as meta | Send the sample rate of each capture: PulseView showed memory taken after live with the live rate |
| `rigol-ds`: no "read should have been completed" over raw TCP | The error appeared after every block although the data was complete |
| `rigol-ds`: input range and memory depth | Input range per channel (read-only); memory depth Auto, 12k to 24M points |
| `rigol-ds`: memory depth as a level | The depth follows the enabled channels like on the scope and is applied when a capture starts |
| `rigol-ds`: memory depth level from the scope | Also before a depth is chosen |

**Connection strings:**

    sigrok-cli -d rigol-ds:conn=usbtmc --scan
    sigrok-cli -d rigol-ds:conn=tcp-raw/<scope-ip>/5555 --scan

Example, 3 frames of the full memory of CH1, as CSV:

    sigrok-cli -d rigol-ds:conn=usbtmc --channels CH1 \
        --config data_source=Memory --frames 3 -O csv -o capture.csv

The trigger sweep (Auto, Normal, Single) can be set like the other options,
also in PulseView. With Single, live capture takes a new shot per frame.

The memory depth (`buffersize`, 0 = Auto) works like the scope's *Mem Depth*
menu: 12k, 120k, 1.2M, 12M or 24M points with one channel, half of that with
two, a quarter with three or four. A chosen depth follows the enabled
channels and is applied when a capture starts. 24M needs the memory option;
if the scope refuses a depth, the driver warns and uses the scope's.

Each channel reports its input range (`range`), the voltages that V/div and
offset allow without clipping.

A memory capture takes one single shot per frame. With the Normal or Single
sweep it waits for a trigger; with the Auto sweep a trigger is forced when
none comes. The sweep and run state are restored afterwards. The memory depth
(menu setting, 12M points, 24M with the memory option) is read from the scope.

**Things to know about the DS1000Z firmware** (00.04.05.SP2 tested):

- **USB at high speed:** the 64 byte endpoints are a firmware bug. Some USB
  host controllers do not work with it at all, others deliver corrupted data
  for long messages. With this fork the data is correct at high speed (tested
  in a VirtualBox VM). At full speed (e.g. behind a USB isolator) 64 bytes is
  valid.
- **LAN only without USB:** while USB is connected, the scope does not answer
  SCPI over LAN (neither port 5555 nor VXI-11). Unplug USB and restart the
  scope to use LAN.
- **One LAN client:** port 5555 serves one client at a time; close other
  programs (e.g. DSRemote) first. Right after another client closed its
  connection, the next one can occasionally get no answers; start again.
- Over LAN the scope needs about 200 ms per command.

## Speed

Tested with this branch at 1 ms/div; times include starting sigrok-cli and
taking the shot.

| Scope | Interface | Live, 3 frames | Memory, 1 frame | Memory, each further frame |
| --- | --- | --- | --- | --- |
| VS5042 | USB | 2.2 s | 4.8 s | 0.65 s (2 × 512K) |
| VS5042 | LAN (patched firmware) | 4.8 s | 15.5 s | about 9 s (2 × 512K) |
| DS1054Z | USB, high speed | 1.2 s | 2.6 s | 2.5 s (1 × 1.2M) |
| DS1054Z | LAN | 4.6 s | 3.2 to 4.4 s | 3.3 s (1 × 1.2M) |

## License

GPL-3.0 or later.
