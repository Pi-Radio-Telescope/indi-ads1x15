/*
   INDI driver for I2C Analog-to-Digital-Converter (ADC).
*/

#pragma once

#include "defaultdevice.h"
#include "connection_i2c.h"
#include <ads1115_measurement.h>
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

    constexpr static std::size_t m_num_channels { 4 };

    ILight StatusL;
    ILightVectorProperty StatusLP;

    INumber VoltageMeasurementN[m_num_channels];
    INumberVectorProperty VoltageMeasurementNP;
    INumber VoltageMeasurementErrorN[m_num_channels];
    INumberVectorProperty VoltageMeasurementErrorNP;
    INumber MeasurementBufSizeN[m_num_channels];
    INumberVectorProperty MeasurementBufSizeNP;
    INumber MeasurementGlobalIntTimeN;
    INumberVectorProperty MeasurementGlobalIntTimeNP;
    INumber MeasurementIntTimeN[m_num_channels];
    INumberVectorProperty MeasurementIntTimeNP;
    INumber MeasurementFactorN[m_num_channels];
    INumberVectorProperty MeasurementFactorNP;
    IText MeasurementUnitT[m_num_channels];
    ITextVectorProperty MeasurementUnitTP;
    
    static constexpr const char* GAIN_SWITCH_DESCRIPTORS[] = { "6V", "4V", "2V", "1V", "512MV", "256MV" };
    static constexpr std::size_t N_GAINS { 6 };

    std::array<ISwitchVectorProperty, m_num_channels> GainSwitchPropertyArray;
    std::array<ISwitch[N_GAINS], m_num_channels> GainSwitchArray;
//     ISwitch GainSwitchS[m_num_channels*7];
//     ISwitchVectorProperty GainSwitchSP[m_num_channels*7];

    INumber DriverUpTimeN;
    INumberVectorProperty DriverUpTimeNP;

    ISwitch ErrorResetS;
    ISwitchVectorProperty ErrorResetSP;

    uint8_t DBG_DEVICE { INDI::Logger::DBG_IGNORE };

    std::shared_ptr<ADS1115> m_adc { nullptr };
    std::array<std::shared_ptr<PiRaTe::Ads1115Measurement>, m_num_channels> voltageMeasurements {};
    std::chrono::time_point<std::chrono::system_clock> fStartTime {};
    std::shared_ptr<Connection::I2C> m_interface { nullptr };
};
