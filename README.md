# AMBIT firmware

AMBIT is an ESP32-C3 leaf-sensor platform developed by the Jan IngenHousz
Institute. This repository contains the firmware that coordinates three sensing
paths on the AMBIT board:

- ADPD6100 optical measurements for fluorescence and PAM-style experiments;
- AS7341 spectral measurements and PAR calculation; and
- MLX90632 leaf and ambient temperature measurements.

This repository is the public JII continuation of
[`hjc2023/ambit`](https://github.com/hjc2023/ambit). Its public upstream ancestry
and the attributed JII development record are preserved explicitly; see
[`ORIGIN.md`](ORIGIN.md), [`MIGRATION_MAP.md`](MIGRATION_MAP.md), and
[`PUBLICATION_AUDIT.md`](PUBLICATION_AUDIT.md) for the provenance boundary.

One firmware image serves the supported hosts at runtime. It can communicate
with an Ambyte datalogger over the binary UART protocol, with the openJII app and
Calibratron over the legacy text console, or with an openJII JSON-envelope host.
It also stores device identity, calibration, and measurement metadata in NVS.

This README is an operator and integrator orientation. The source and linked
plans remain authoritative for protocol bytes, measurement behavior, and
hardware-conformance requirements.

## Hardware and interface boundaries

The PlatformIO target is `esp32-c3-devkitm-1`, used to build firmware for the
AMBIT ESP32-C3 board. The attached sensors and firmware buses are:

| Function | Device | Firmware interface |
| --- | --- | --- |
| Fluorescence / optical acquisition | ADPD6100 | SPI |
| Spectral channels / PAR | AS7341 | I2C, address `0x39` |
| Leaf and ambient temperature | MLX90632 | I2C, address `0x3A` |
| Host protocol | Ambyte, openJII, or Calibratron | UART0, 115200 8N1 |

UART0 is mapped to GPIO20 RX and GPIO21 TX. The Ambyte uses that UART over the
board's FFC connection. Native USB CDC is disabled in the firmware build;
USB-Serial-JTAG on GPIO18/19 is a recovery/programming path, not an additional
live host-protocol port.

> [!CAUTION]
> This firmware repository does not define the board connector pinout, supply
> voltage limits, or a safe external-powering procedure. Do not infer those
> electrical limits from the generic PlatformIO board name or the GPIO numbers
> above. Use the AMBIT hardware documentation and approved programmer/fixture
> for the exact board revision. Disconnect the device from measurement hosts
> before attaching programming hardware unless the fixture instructions
> explicitly support that arrangement.

Flashing is also a data boundary: never erase NVS on a calibrated device merely
to troubleshoot communication. A full flash must use a mutually matching set
of release assets and the offsets in that release's `manifest.json`.

## Use a release or build from source

For field devices, prefer a published [GitHub release](https://github.com/Jan-IngenHousz-Institute/ambit/releases).
Release assets are produced together, hashed in a manifest, and intended for
use by release-aware tooling such as the Calibratron. Do not combine binaries
from different releases.

Build from source when developing or validating a change. The project uses the
Arduino framework through PlatformIO and has one environment, `ambit`.

1. Install Git and PlatformIO Core (or a PlatformIO-enabled IDE).
2. Clone this repository and enter its root directory.
3. On Linux or macOS, provide a writable `LOCALAPPDATA` because the build output
   is deliberately redirected outside the working tree:

   ```sh
   export LOCALAPPDATA="$HOME/.cache"
   ```

4. Build the firmware:

   ```sh
   pio run
   ```

For a development board connected through a PlatformIO-detected upload port,
add the upload target:

```sh
pio run -t upload
```

The serial monitor settings are 115200 baud, 8 data bits, no parity, one stop
bit. Uploading directly is a development/recovery operation; field upgrades
should follow the release and OTA guidance below.

### Build-time version identity

Do not edit a version literal in the source. [`tools/version.py`](tools/version.py)
injects one version using this precedence:

1. `AMBIT_RELEASE_VERSION`, set by CI for a release or PR candidate;
2. `git describe --tags --always --dirty` for a local build; or
3. `0.0.0-dev` when neither source is available.

The complete string is reported by text `hello`, the JSON identity, and is
embedded in the application image. The frozen binary firmware-info response
(command 33, subtype 2) also exposes legacy `uint8` major/minor/batch fields.
Those three bytes are parsed only when the version begins with
`<number>.<number>.<number>`; otherwise they are `0.0.0`.

This matters for non-release images:

- a local build described as `1.2.3-4-gabcdef` reports that complete string and
  numeric identity `1.2.3`;
- a PR candidate such as `pr-42-abcdef123456` reports that complete string but
  numeric identity `0.0.0`; and
- the current release automation publishes stable semantic versions from
  `main`; it does not define a separate prerelease channel.

Use the complete text/JSON version (or the release manifest version) when
distinguishing candidate builds. Numeric command 33/2 identity remains useful
for compatibility with existing Ambyte readers, but is not unique for every
development build.

## One image, three host frontends

There are no current `ambyte` and `cloud` firmware variants. `platformio.ini`
builds one image, and [`loop()`](src/ambit-1.ino) routes UART0 by the first byte:

| First input | Frontend | Intended host | Orientation |
| --- | --- | --- | --- |
| Binary framing byte (`> 127`) | Frozen Ambyte adapter / FSM | Ambyte datalogger over FFC | [`src/run_esp.cpp`](src/run_esp.cpp), [`src/data_utils.cpp`](src/data_utils.cpp), [wire-contract notes](plans/MERGE_PLAN.md) |
| `{` or `[` | openJII JSON envelope | JSON-capable openJII integration | [`src/frontend_json.cpp`](src/frontend_json.cpp), [`lib/openjii_proto/`](lib/openjii_proto/) |
| Other printable text | Legacy line console | openJII app and Calibratron | [`src/do_command.h`](src/do_command.h) |

Unknown and text hosts remain awake so their first request is not lost. A valid
Ambyte wake latches the binary host behavior, including its light-sleep and
idle-heartbeat contract. The text latch is released after two minutes without
traffic so the same device can later be attached to an Ambyte without rebooting.

All three frontends share the same command core and staged measurement
configuration. Transport-specific encodings are normalized in their adapter;
they are not part of the core measurement API.

### Text console

The text console accepts the established openJII/Calibratron dialect. Send a
printable command line over UART0. For example, `hello` replies in the form:

```text
NEW <device-name> Ready FW:<full-version>
```

`NEW` and `Ready` are compatibility sentinels used by existing hosts. The
complete command dispatch, argument reads, and response shapes are defined in
[`src/do_command.h`](src/do_command.h). Some commands mutate calibration or NVS;
integrators should not probe unfamiliar verbs on a calibrated field device.

### openJII JSON envelope

Only requests beginning with `{` or `[` reach the JSON-envelope frontend. The
currently registered surface includes identity, temperature, raw/calibrated
PAR, staged current/gain accessors, and streamed array runs. See
[`frontend_json_register()`](src/frontend_json.cpp) for the exact command set
and [`openjii_proto`](lib/openjii_proto/openjii_proto.cpp) for accepted envelope
forms, response framing, size limits, and errors.

Bare printable lines do **not** enter the protocol library's generic LINE mode
in this firmware; they belong to the legacy text console above.

### Ambyte binary protocol

The binary path is a compatibility contract with deployed Ambyte firmware.
Framing values, struct sizes and offsets, checksums, gain indexing, and length
encodings must remain byte-compatible. Start with the frozen-contract section
of [`plans/MERGE_PLAN.md`](plans/MERGE_PLAN.md) and validate any change with
[`plans/HW_CONFORMANCE.md`](plans/HW_CONFORMANCE.md). The Ambyte implementation
is the final source of truth for the bytes it sends and accepts.

UART0 is both the protocol stream and the only configured serial console. Extra
debug text can corrupt a binary session, so production code deliberately
suppresses ESP/Arduino logging on that port. The existing `BOOT` line and
startup metadata/calibration/firmware dump predate the frozen contract and are
tolerated by deployed hosts; expect them after reset or OTA reboot, but do not
add further unsolicited output.

## Releases and flashing

Every PR is designed to build one candidate asset set. When a semantic release
is required after merge, the release workflow retrieves and republishes that
exact PR-built artifact rather than rebuilding it. The release contains:

| Asset | Role | Full-flash offset |
| --- | --- | --- |
| `ambit-fw-v<version>.bin` | Application / OTA image | `0x10000` |
| `bootloader.bin` | ESP32-C3 bootloader | `0x0` |
| `partitions.bin` | Pinned dual-OTA partition table | `0x8000` |
| `boot_app0.bin` | OTA data initialization | `0xe000` |
| `manifest.json` | Flash metadata and SHA-256 hashes | n/a |

Treat a published release as immutable. If any firmware bit or companion asset
must change, publish a new semantic version; do not replace an asset under an
existing tag. Before flashing, verify each selected file against the SHA-256 in
the same manifest.

### OTA versus full flash

Use an **OTA update** for a normally operating installed device whose partition
layout is already compatible. The Ambyte sends only the application image in
sequenced, CRC-checked chunks to the inactive OTA slot. The firmware verifies
the completed image, boots it pending confirmation, and retains the previous
slot for rollback if the Ambyte cannot confirm the new image.

Use a **full flash** for first installation, recovery, or an explicitly planned
bootloader/partition migration. Release-aware tooling must write all four
regions at the manifest offsets. A field device's partition table should be
read and verified before rewriting it; the build pins Arduino-ESP32's 4 MB
dual-OTA `default.csv` layout specifically to prevent an unnoticed layout
change.

An application-only OTA does not rewrite NVS. A full flash using the same
partition layout also leaves the NVS offset unchanged, but calibration is not
protected from an erase operation or an incompatible partition-table change.
Back up or record device calibration before recovery work.

## Calibration, metadata, and NVS

At boot, [`src/nvs1.cpp`](src/nvs1.cpp) loads the AMBIT name, calibration
factors, emissivity, actinic set points, ADPD baselines, and hardware revision
from the `config` NVS namespace. Site and measurement metadata use a separate
`metadata` namespace. MLX90632 coefficients are read from the sensor and
included in the calibration record.

The field-facing paths apply the stored calibration consistently:

- text `PAR`, JSON-envelope `PAR`, and binary command 31 multiply `get_PAR()` by
  `spec_coef`; text/JSON `get_par` deliberately remain raw diagnostic endpoints;
- the six ADPD baselines map to `s_630`, `r_630`, `sun`, `leaf`, `s_730`, and
  `r_730`, and are subtracted with saturation at zero exactly once before data
  enters a result buffer;
- older devices with no stored sun/leaf offsets retain the historical 65,000
  correction until a complete baseline vector is calibrated; and
- both text and binary baseline operations save all six values in one NVS
  commit, verify readback, and update runtime state only after success.

Calibration acquisition and the raw PAR endpoints remain uncorrected on
purpose: the Calibratron needs those raw observations to derive coefficients.

Binary command 6 retains its deployed silent acknowledgement contract: it
always returns `ESP_CMD_DONE` followed by `ESP_CMD_END`, with no payload or
success/failure status byte. Acquisition, sensor, timeout, validation, or NVS
failures therefore cannot be diagnosed from that reply. In particular, an
acquired `s_630` baseline above 400 is rejected and not persisted. The
authoritative calibration workflow is the Calibratron's text-console
`baseline` capture, which prints the six acquired values and explicitly reports
acquisition, range, and save failures.

Binary commands and selected text-console operations can update this state.
Metadata is persisted only when its frozen end marker is valid and its
longitude, latitude, and altitude pass the source's range checks. The binary
command acknowledges the transfer before those persistence checks, so its
framing acknowledgement alone does not prove the metadata was saved.
Calibration and metadata structures also travel over the deployed binary
protocol, so changing their layouts is a compatibility change, not ordinary
refactoring.

> [!WARNING]
> The text command `clean_nvs` erases the NVS partition. Do not use it as a
> general reset on a calibrated device. Reflashing firmware is not a substitute
> for restoring calibration.

## Validation

The minimum software gate is a clean PlatformIO build:

```sh
pio run
```

Host-side unit tests under [`test/adpd6000/`](test/adpd6000/) and
[`test/calibration/`](test/calibration/) verify the transport boundary and
baseline math, including legacy defaults and saturating subtraction. Run them
before approving a release candidate. Hardware remains the release-quality
regression gate for transport and measurement changes.

Any change touching the binary adapter, FSM framing, or first-byte router must
run the real-device procedure in
[`plans/HW_CONFORMANCE.md`](plans/HW_CONFORMANCE.md). That procedure exercises a
real Ambyte, verifies framing/length/checksum behavior, and separates stable
wire bytes from sensor values that naturally vary with measurement noise.
Record the hardware, image identity, scope, and result in the conformance log.

## Repository layout

| Path | Purpose |
| --- | --- |
| [`src/ambit-1.ino`](src/ambit-1.ino) | Boot, sensor initialization, host latch, and runtime router |
| [`src/core.*`](src/core.h) | Transport-neutral configuration and command operations |
| [`src/PAM.cpp`](src/PAM.cpp) | Measurement engine, array/MPF runs, triggers, and output selection |
| [`src/run_esp.cpp`](src/run_esp.cpp) | Frozen Ambyte binary command adapter and OTA receiver |
| [`src/data_utils.cpp`](src/data_utils.cpp) | Binary data FSM, framing, and checksums |
| [`src/do_command.h`](src/do_command.h) | Legacy text-console command dispatch |
| [`src/frontend_json.cpp`](src/frontend_json.cpp) | AMBIT handlers for the openJII JSON envelope |
| [`lib/openjii_proto/`](lib/openjii_proto/) | Device-independent openJII protocol module |
| [`src/nvs1.*`](src/nvs1.h) | Persistent calibration, metadata, hardware revision, and firmware identity |
| [`src/src/`](src/src/) | Sensor drivers and board-level sensor interfaces |
| [`plans/`](plans/) | Merge rationale and reusable hardware-conformance procedure |
| [`.github/workflows/`](.github/workflows/) | PR validation/build and immutable-artifact release pipeline |
| [`tools/version.py`](tools/version.py) | Single-source build version injection |

[`plans/MERGE_PLAN.md`](plans/MERGE_PLAN.md) is a historical design record. Its
old build-variant proposal is superseded by the single runtime-routed image, but
its frozen-wire rationale remains relevant.

## Troubleshooting

### PlatformIO cannot create the build directory

`platformio.ini` uses `${sysenv.LOCALAPPDATA}` for a Windows OneDrive/antivirus
workaround. On Linux or macOS, set it to a writable location before building:

```sh
export LOCALAPPDATA="$HOME/.cache"
pio run
```

### A text or JSON host gets no reply

- Confirm 115200 8N1 on UART0 and that TX/RX are connected in the correct
  directions for the approved interface.
- Native USB CDC is disabled; attaching to an unrelated USB serial port will
  not expose the runtime protocol.
- JSON envelopes must begin with `{` or `[`. Bare printable input is interpreted
  as a legacy text-console command.
- After a partial JSON request, allow its one-second receive timeout or reset the
  device before switching protocols.

### An Ambyte session reports framing or checksum errors

- After reset or OTA reboot, expect and skip the firmware's legacy boot and NVS
  text before starting a binary exchange. Unexpected ASCII during an active
  exchange still indicates a writer corrupting the binary stream; remove any
  terminal monitor or external debug writer from UART0.
- Confirm the Ambyte is using the deployed framing and struct definitions
  referenced by the merge plan.
- Reproduce the failure with the capture and byte-level checks in
  [`plans/HW_CONFORMANCE.md`](plans/HW_CONFORMANCE.md).

### The reported version is `0.0.0` or differs between fields

Compare the full `FW:`/JSON version first. A non-semantic development or PR
identifier intentionally maps to `0.0.0` in the legacy numeric binary fields.
For a published release, verify the running image against that release's
`manifest.json` and retrieve binary command 33/2 after an OTA reboot.

### Calibration appears to be missing

Stop before writing more flash. Check whether NVS was erased or whether a
different partition table was installed. Restore calibration using the
approved calibration workflow; do not guess coefficient values from defaults.

## Contributing

Keep changes focused and preserve the deployed transport contracts. Before a
pull request:

1. read [`AGENTS.md`](AGENTS.md) and the relevant plan/source files;
2. build with `pio run`;
3. run hardware conformance when the router or binary path is affected;
4. use a Conventional Commit-style PR title, for example
   `fix(router): preserve binary wake framing`; and
5. include verification evidence and call out any operator-visible or wire-level
   effect.

PR titles drive semantic release: `feat` requests a minor release;
`fix`, `perf`, and `revert` request a patch; breaking changes request a major;
and documentation, tests, refactors, build, CI, style, and chores do not publish
firmware on their own.

Do not add ordinary logging to UART0, hand-edit version fields, reformat vendored
drivers wholesale, or change frozen structs/bytes without coordinating the host
side and completing conformance testing.

## License and attribution

The repository's covered source is provided under the
[CERN Open Hardware Licence Version 2 – Strongly Reciprocal](LICENSE.txt)
(`CERN-OHL-S-2.0`). Retain copyright, license, and source notices when modifying
or redistributing it.

Vendored or adapted sensor code retains its upstream terms and attribution:

- the AS7341 driver is from Adafruit Industries under the
  [BSD 3-Clause License](src/src/as7341/license.txt);
- the MLX90632 driver retains Melexis N.V. copyright and Apache-2.0 notices in
  [`src/src/mlx90632/`](src/src/mlx90632/); and
- the ADPD configuration wrapper retains its MSU-PRL Kramer Lab and Jan
  IngenHousz Institute attribution in [`src/src/adpd/`](src/src/adpd/).

The main repository license does not replace those third-party notices. Keep
them with redistributed source and review all applicable terms for a binary or
hardware distribution.
