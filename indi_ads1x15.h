/*
   INDI Raspberry Pi based mount driver.
   The driver itself acts as a telescope mount reading the positions from SSI-based absolute encoders from
   the on-board SPI interfaces and driving DC motors via PWM over GPIO pins
   "Pi Radiotelescope Driver"

*/

#pragma once

#include "defaultdevice.h"
#include "connection_i2c.h"
#include <ads1115_measurement.h>
// #include <voltage_monitor.h>
#include <map>

class i2cDevice;

namespace PiRaTe {
}
class ADS1115;

/** \brief A customized INDI device for readout of Analog Devices ADC ADS1x15.
    \author Hans-Georg Zaunick
*/
class IndiADS1x15 : public INDI::DefaultDevice {
public:
    using INDI::DefaultDevice::saveConfig;

    IndiADS1x15();

    bool Connect() override;
    bool Disconnect() override;
    void TimerHit() override;
    virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
    virtual bool ISNewSwitch(const char* dev, const char* name, ISState* states, char* names[], int n) override;
    virtual bool ISNewNumber(const char* dev, const char* name, double values[], char* names[], int n) override;
//     virtual bool ISSnoopDevice(XMLEle* root) override;
    virtual bool saveConfigItems(FILE *fp) override;
    virtual bool saveConfig(bool silent = false, const char *property = nullptr);

protected:
    virtual bool Handshake();
    const char* getDefaultName() override;
    bool initProperties() override;
    bool updateProperties() override;

private:
    void updateMonitoring();
    void updateTime();
    auto upTime() const -> std::chrono::duration<long, std::ratio<1>>;

    // Device physical bus
//     ITextVectorProperty BusTP;
//     IText BusT {};

//     ISwitch AddressS[6];
//     ISwitchVectorProperty AddressSP;

//     ISwitch *SystemBusS = nullptr;
//     ISwitchVectorProperty SystemBusSP;

    ILight StatusL;
    ILightVectorProperty StatusLP;

    INumber VoltageMeasurementN[16];
    INumberVectorProperty VoltageMeasurementNP;
    INumber MeasurementIntTimeN;
    INumberVectorProperty MeasurementIntTimeNP;

    INumber DriverUpTimeN;
    INumberVectorProperty DriverUpTimeNP;

    ISwitch ErrorResetS;
    ISwitchVectorProperty ErrorResetSP;

    uint8_t DBG_DEVICE { INDI::Logger::DBG_IGNORE };

    std::shared_ptr<ADS1115> m_adc { nullptr };
    constexpr static std::size_t m_num_channels { 4 };
//     std::shared_ptr<Connection::I2C> m_connection { nullptr };
    
    std::map<std::uint8_t, std::shared_ptr<i2cDevice>> i2cDeviceMap {};

//     std::vector<std::shared_ptr<PiRaTe::Ads1115VoltageMonitor>> voltageMonitors {};
    std::array<std::shared_ptr<PiRaTe::Ads1115Measurement>, m_num_channels> voltageMeasurements {};
    std::chrono::time_point<std::chrono::system_clock> fStartTime {};
    std::string m_ConfigBus;
    std::uint8_t m_ConfigAddress { 0xff };
    std::vector<std::string> m_SystemBuses;
    std::shared_ptr<Connection::I2C> m_interface { nullptr };

};
