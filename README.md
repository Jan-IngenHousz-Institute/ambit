# AMBIT firmware

Firmware for the Jan IngenHousz Institute AMBIT instrument.

This repository is the public JII continuation of
[`hjc2023/ambit`](https://github.com/hjc2023/ambit). The GitHub fork relationship and
upstream commit history are intentionally preserved; the JII continuation is imported as
a separate, truthful snapshot. See [ORIGIN.md](ORIGIN.md) for the boundary and attribution.

## Build

The supported build uses PlatformIO:

```sh
AMBIT_RELEASE_VERSION=1.1.3-rc1 pio run -e ambit
```

Release bundles contain `manifest.json`, the OTA application image named by
`manifest.ota.file`, and the three boot/partition images used by the Calibratron. Stable
firmware is selected through GitHub's `/releases/latest`; prereleases must always be chosen
explicitly.

## Licensing

JII-authored firmware is provided under CERN-OHL-S-2.0; see [LICENSE.txt](LICENSE.txt).
Third-party components retain their own copyright and license notices in their source
directories.
