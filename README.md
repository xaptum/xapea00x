# xapea00x

A Linux kernel driver for the Xaptum ENF Access card, a TPM 2.0-based
hardware card used to authenticate IoT devices and gateways to the
Xaptum Edge Network Fabric.

## Supported Product IDs

| Model # | Form Factor | USB Vendor ID | USB Product ID |
|---------|-------------|---------------|----------------|
| XAP-EA-001 | USB | 0x10C4 | 0x8BDE |
| XAP-EA-002 | USB | 0x2FE0 | 0x8BDE |
| XAP-EA-003 | Mini PCI-e | 0x2FE0 | 0x8BEE |
| XAP-EA-004 | MultiTech MTAC | 0x2FE0 | 0x8BFE |

## Runtime Dependencies

The necessary TPM 2.0 support was added to the 4.9 kernel, so this is
the earliest supported version.

The kernel must have been compiled with support for USB
(`USB_SUPPORT=y`), SPI (`SPI=y`), and TPM 2.0 (`TCG_TPM=y|m` and
`TCG_TIS_CORE=y|m`).

The `TCG_TIS_SPI` kernel module is also required.  Most Linux
distributions do not include it, so it must be built separately.
Further, kernels 4.9 through 4.11 contain a buggy version anyway.

This repo contains the source for a working version of `TCG_TIS_SPI`
to make out-of-tree building easy.

## Installation

### Debian (Stretch)

``` bash
# Install the Xaptum API repo GPG signing key.
apt-key adv --keyserver keyserver.ubuntu.com --recv-keys c615bfaa7fe1b4ca

# Add the repository to your APT sources.
echo "deb http://dl.bintray.com/xaptum/deb stretch main" > /etc/apt/sources.list.d/xaptum.list

# Install the library
sudo apt-get install xapea00x-dkms
```

### From Source

#### Build Dependencies

* `make`
* `gcc`
* Linux kernel headers for your distribution

#### xapea00x

```bash
make
sudo make modules_install
```

#### TCG_TIS_SPI (if needed)

If your system does not include a working `TCG_TIS_SPI` module, build
and install the copy in this repo:

```bash
cd tpm_tis_spi
make
sudo make modules_install
```

## License
Copyright (c) 2017-2020 Xaptum, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
