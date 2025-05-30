# INDI-ADS1x15

An [INDI](https://indilib.org/) driver for the Analog Devices ADS1x15 series of analog-to-digital converters (ADCs), including the ADS1015 and ADS1115. This driver enables integration of high-resolution ADCs into INDI-compatible systems, facilitating precise analog signal acquisition for applications such as radio astronomy and environmental monitoring.

## Features

- Supports ADS1015 and ADS1115 4-channel ADCs
- I²C communication interface
- Configurable integration time
- Integration with INDI clients for real-time data acquisition

## Requirements

- Linux-based system (e.g., Raspberry Pi)
- [INDI Library](https://indilib.org/) installed
- CMake 3.5 or higher
- C++ compiler supporting C++17 standard

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/Pi-Radio-Telescope/indi-ads1x15.git
cd indi-ads1x15
```

### 2. Build the Driver

```bash
mkdir build
cd build
cmake ..
make
sudo make install
```

### 3. Configure INDI

After installation, the driver should be available in your INDI server. You can configure it using your preferred INDI client (e.g., KStars/Ekos).

## Usage

1. Connect the ADS1x15 device to your system via I²C.
2. Start the INDI server:
   ```bash
   indiserver indi_ads1x15
   ```
3. Use an INDI client to connect to the server and interact with the ADS1x15 device.

## Configuration

The driver allows configuration of following parameter:

- **Integration Time**: Configure the buffer depth for required integration time.

These settings can be adjusted through your INDI client interface.

## Planned Extensions

- **Gain**: Set the programmable gain amplifier (PGA) to adjust the input voltage range.
- **AGC**: Automatically adjust the input voltage range depending on the current input signal.
- **Sampling Rate**: Configure the data rate for conversions.
- **Measurement Mode**: Choose between continuous and single-shot modes.


## Contributing

Contributions are welcome! Please fork the repository and submit pull requests for any enhancements or bug fixes.

## License

This project is licensed under the [LGPL-2.1](https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html) license.

## Acknowledgments

This driver is developed and maintained by the [Pi-Radio-Telescope](https://github.com/Pi-Radio-Telescope) team.

---

For more information and support, please visit the [INDI Library website](https://indilib.org/).
