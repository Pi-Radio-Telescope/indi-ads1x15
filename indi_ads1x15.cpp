/*
   INDI device driver for I2C ADC ADS1x1x family
*/

#include "indi_ads1x15.h"

#include <connectionplugins/connectioninterface.h>

#include <cmath>
#include <map>
#include <memory>

#include <libnova.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ads1115.h"
#include "config.h"

namespace Connection {
class Interface;
}

constexpr unsigned int POLL_INTERVAL_MS { 500 }; //< polling interval of this driver
constexpr std::uint8_t DEFAULT_ADC_ADDRESS { 0x48 }; //< default I2C address of ADS1115 ADCs to be read-out
constexpr std::chrono::milliseconds DEFAULT_INT_TIME { 1000 };
constexpr std::chrono::seconds MAX_INT_TIME { 1800 };

struct I2cVoltageDef {
    std::string name;
    double nominal;
    double divider_ratio;
    std::uint8_t adc_address;
    std::uint8_t adc_channel;
    std::string unit;
};


const I2cVoltageDef default_measurement_voltage_def { 
    "Analog", 0., 55.5556, DEFAULT_ADC_ADDRESS, 0, "dB"
};


// the server will handle one unique instance of the driver
static std::unique_ptr<IndiADS1x15> ads(new IndiADS1x15());

/**************************************************************************************
** Return properties of device.
***************************************************************************************/
void ISGetProperties(const char* dev)
{
    ads->ISGetProperties(dev);
}

/**************************************************************************************
** Process new switch from client
***************************************************************************************/
void ISNewSwitch(const char* dev, const char* name, ISState* states, char* names[], int n)
{
    ads->ISNewSwitch(dev, name, states, names, n);
}

/**************************************************************************************
** Process new text from client
***************************************************************************************/
void ISNewText(const char* dev, const char* name, char* texts[], char* names[], int n)
{
    ads->ISNewText(dev, name, texts, names, n);
}

/**************************************************************************************
** Process new number from client
***************************************************************************************/
void ISNewNumber(const char* dev, const char* name, double values[], char* names[], int n)
{
    ads->ISNewNumber(dev, name, values, names, n);
}

/**************************************************************************************
** Process new blob from client
***************************************************************************************/
void ISNewBLOB(const char* dev, const char* name, int sizes[], int blobsizes[], char* blobs[], char* formats[],
    char* names[], int n)
{
    ads->ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

/**************************************************************************************
** Process snooped property from another driver
***************************************************************************************/
void ISSnoopDevice(XMLEle* root)
{
    ads->ISSnoopDevice(root);
}

/*
 * class IndiADS1x15
 * 
 */
IndiADS1x15::IndiADS1x15()
{
    DBG_DEVICE = INDI::Logger::DBG_SESSION;
    // We add an additional debug level so we can log verbose scope status
//     DBG_DEVICE = INDI::Logger::getInstance().addDebugLevel("Device Verbose", "DEVICE");

    // set driver version from cmake-set constants
    setVersion(CDRIVER_VERSION_MAJOR, CDRIVER_VERSION_MINOR);
    // set driver interface to AUX_INTERFACE
    setDriverInterface(INDI::BaseDevice::AUX_INTERFACE);
}

/**************************************************************************************
** We init our properties here. The only thing we want to init are the Debug controls
***************************************************************************************/
bool IndiADS1x15::initProperties()
{
    // ALWAYS call initProperties() of parent first
    INDI::DefaultDevice::initProperties();

    m_interface = std::make_shared<Connection::I2C>(this);
    registerConnection(m_interface.get());
    setActiveConnection(m_interface.get());
    
    setDefaultPollingPeriod(POLL_INTERVAL_MS);

    IUFillLight(&StatusL, "STATUS", "Status", IPS_OK);
    IUFillLightVector(&StatusLP, &StatusL, 1, getDeviceName(), "STATUS", "Connection", MAIN_CONTROL_TAB, IPS_IDLE);

    IUFillNumber(&MeasurementGlobalIntTimeN, "INT_TIME", "Int Time", "%5.2f s", 0, MAX_INT_TIME.count(), 0.1, DEFAULT_INT_TIME.count() / 1000.);
    IUFillNumberVector(&MeasurementGlobalIntTimeNP, &MeasurementGlobalIntTimeN, 1, getDeviceName(), "INT_TIME", "Integration Time", OPTIONS_TAB,
        IP_RW, 60, IPS_IDLE);
    IUGetConfigNumber(getDeviceName(), "INT_TIME", "INT_TIME", &MeasurementGlobalIntTimeN.value);
    
    IUFillNumber(&DriverUpTimeN, "UPTIME", "Uptime", "%5.2f h", 0, 0, 0, 0);
    IUFillNumberVector(&DriverUpTimeNP, &DriverUpTimeN, 1, getDeviceName(), "DRIVER_UPTIME", "Driver Uptime", OPTIONS_TAB,
        IP_RO, 60, IPS_IDLE);

    for (std::size_t i { 0 }; i < m_num_channels; ++i) {
        IUFillNumber(&VoltageMeasurementN[i], ("MEASUREMENT" + std::to_string(i)).c_str(), (default_measurement_voltage_def.name + std::to_string(i)).c_str(), ("%4.4f " + default_measurement_voltage_def.unit).c_str(), 0, 0, 0, 0.);

        IUFillNumber(&VoltageMeasurementErrorN[i], ("STDDEV" + std::to_string(i)).c_str(), ("StdDev" + std::to_string(i)).c_str(), ("%4.4f " + default_measurement_voltage_def.unit).c_str(), 0, 0, 0, 0.);

        IUFillNumber(&MeasurementBufSizeN[i], ("BUFSIZE" + std::to_string(i)).c_str(), ("Entries" + std::to_string(i)).c_str(), "%4.0f ", 0, 0, 0, 0.);

        IUFillNumber(&MeasurementFactorN[i], ("FACTOR" + std::to_string(i)).c_str(), ("Factor " + std::to_string(i)).c_str(), "%4.4f ", -1e6, 1e6, 0, default_measurement_voltage_def.divider_ratio);
        
        double factor{};
        if (IUGetConfigNumber(getDeviceName(), "FACTORS", ("FACTOR"+std::to_string(i)).c_str(), &factor)==0) {
            MeasurementFactorN[i].value = factor;
        }

        IUFillNumber(&MeasurementIntTimeN[i], ("INT_TIME"+std::to_string(i)).c_str(), ("Int Time "+std::to_string(i)).c_str(), "%5.2f s", 0.01, MAX_INT_TIME.count(), 0.1, DEFAULT_INT_TIME.count() / 1000.);

        double int_time{};
        if (IUGetConfigNumber(getDeviceName(), "CHANNEL_INT_TIME", ("INT_TIME"+std::to_string(i)).c_str(), &int_time)==0) {
            MeasurementIntTimeN[i].value = int_time;
        }
    }

    IUFillNumberVector(&MeasurementIntTimeNP, MeasurementIntTimeN, m_num_channels, getDeviceName(), "CHANNEL_INT_TIME", "Integration Time", OPTIONS_TAB,
        IP_RW, 60, IPS_IDLE);

    IUFillNumberVector(&VoltageMeasurementNP, VoltageMeasurementN, m_num_channels, getDeviceName(), "MEASUREMENTS", "Measurements", MAIN_CONTROL_TAB,
            IP_RO, 60, IPS_IDLE);

    IUFillNumberVector(&VoltageMeasurementErrorNP, VoltageMeasurementErrorN, m_num_channels, getDeviceName(), "ERRORS", "Measurement Errors", MAIN_CONTROL_TAB,
            IP_RO, 60, IPS_IDLE);

    IUFillNumberVector(&MeasurementBufSizeNP, MeasurementBufSizeN, m_num_channels, getDeviceName(), "BUFSIZE", "Buffer Sizes", MAIN_CONTROL_TAB,
            IP_RO, 60, IPS_IDLE);

    IUFillNumberVector(&MeasurementFactorNP, MeasurementFactorN, m_num_channels, getDeviceName(), "FACTORS", "Calibration", OPTIONS_TAB,
            IP_RW, 60, IPS_IDLE);

    for (auto& gainSwitchVector : GainSwitchPropertyArray) {
        std::size_t ichannel = std::distance(std::begin(GainSwitchPropertyArray), &gainSwitchVector);
        for (std::size_t switch_index { 0 }; switch_index < N_GAINS; ++switch_index) {
            IUFillSwitch(&GainSwitchArray[ichannel][switch_index], GAIN_SWITCH_DESCRIPTORS[switch_index], GAIN_SWITCH_DESCRIPTORS[switch_index], ISS_OFF);
        }
        IUFillSwitchVector(&gainSwitchVector, GainSwitchArray[ichannel], N_GAINS, getDeviceName(), std::string("GAIN"+std::to_string(ichannel)).c_str(), std::string("Gain Ch"+std::to_string(ichannel)).c_str(), OPTIONS_TAB,
        IP_RW, ISR_1OFMANY, 60, IPS_IDLE);
    }

    IUFillSwitch(&AgcSwitchS[0], "AGC0", "AGC Ch0", ISS_ON);
    IUFillSwitch(&AgcSwitchS[1], "AGC1", "AGC Ch1", ISS_ON);
    IUFillSwitch(&AgcSwitchS[2], "AGC2", "AGC Ch2", ISS_ON);
    IUFillSwitch(&AgcSwitchS[3], "AGC3", "AGC Ch3", ISS_ON);
    IUFillSwitchVector(&AgcSwitchSP, AgcSwitchS, m_num_channels, getDeviceName(), "AGC", "AGC", OPTIONS_TAB,
    IP_RW, ISR_NOFMANY, 60, IPS_IDLE);

    addDebugControl();
    addPollPeriodControl();
    return true;
}

bool IndiADS1x15::updateProperties()
{
    // ALWAYS call updateProperties() of parent first
    INDI::DefaultDevice::updateProperties();

    if (isConnected()) {
        defineProperty(&StatusLP);
        defineProperty(&VoltageMeasurementNP);
        defineProperty(&VoltageMeasurementErrorNP);
        defineProperty(&MeasurementBufSizeNP);
        defineProperty(&MeasurementGlobalIntTimeNP);
        defineProperty(&MeasurementIntTimeNP);
        defineProperty(&MeasurementFactorNP);
        defineProperty(&DriverUpTimeNP);
        for (auto& gainSwitchVector : GainSwitchPropertyArray) {
            defineProperty(&gainSwitchVector);
        }
        defineProperty(&AgcSwitchSP);
        if (m_interface) m_interface->Deactivated();
    } else {
        deleteProperty(StatusLP.name);
        deleteProperty(VoltageMeasurementNP.name);
        deleteProperty(VoltageMeasurementErrorNP.name);
        deleteProperty(MeasurementBufSizeNP.name);
        deleteProperty(MeasurementGlobalIntTimeNP.name);
        deleteProperty(MeasurementIntTimeNP.name);
        deleteProperty(MeasurementFactorNP.name);
        deleteProperty(DriverUpTimeNP.name);
        for (auto& gainSwitchVector : GainSwitchPropertyArray) {
            deleteProperty(gainSwitchVector.name);
        }
        deleteProperty(AgcSwitchSP.name);
        if (m_interface) m_interface->Activated();
    }
    return true;
}

/**************************************************************************************
**
***************************************************************************************/

bool IndiADS1x15::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    //  Nobody has claimed this, so forward it to the base class method
    return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n);
}

bool IndiADS1x15::ISNewSwitch(const char* dev, const char* name, ISState* states, char* names[], int n)
{
    if (strcmp(dev, getDeviceName()) == 0) {
        // set AGC switch
        if (!strcmp(name, AgcSwitchSP.name)) {
            if (n < 0)
                return false;
            for (int index = 0; index < n; index++) {
                ISwitch* sw = IUFindSwitch(&AgcSwitchSP, names[index]);
                std::size_t ichannel = std::distance(std::begin(AgcSwitchS), sw);
                if (sw != nullptr) {
                    if (sw->s != states[index]) {
                        m_adc->setAGC(ichannel, (states[index] == ISS_ON)?true:false);
                        GainSwitchPropertyArray[ichannel].p = (states[index] == ISS_ON)?IP_RO:IP_RW;
                        IDSetSwitch(&GainSwitchPropertyArray[ichannel], nullptr);
                    }
                }
            }
            AgcSwitchSP.s = IPS_IDLE;
            IUUpdateSwitch(&AgcSwitchSP, states, names, n);
            IDSetSwitch(&AgcSwitchSP, nullptr);
            return true;
        }
        //  Set gain switch
        for (auto& gainSwitchVector : GainSwitchPropertyArray) {
            std::size_t ichannel = std::distance(std::begin(GainSwitchPropertyArray), &gainSwitchVector);
            if (!strcmp(name, gainSwitchVector.name)) {
                if (n < 0)
                    return false;
                if (AgcSwitchS[ichannel].s == ISS_OFF) {
                    IUUpdateSwitch(&gainSwitchVector, states, names, n);
                    int index = IUFindOnSwitchIndex(&gainSwitchVector);
                    if (index >= 0)
                    {
                        if (m_adc) {
                            m_adc->setPga(ichannel, index);
                        }
                    }

                    gainSwitchVector.s = IPS_OK;
                    IDSetSwitch(&gainSwitchVector, nullptr);
                }
                return true;
            }
        }
    }

    //  Nobody has claimed this, so forward it to the base class' method
    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool IndiADS1x15::ISNewNumber(const char* dev, const char* name, double values[], char* names[], int n)
{
    if (!strcmp(dev, getDeviceName())) {
        //  This one is for us
        if (!strcmp(name, MeasurementGlobalIntTimeNP.name)) {
            if (!voltageMeasurements.empty()) {
                for (auto meas : voltageMeasurements) {
                    meas->setIntTime(std::chrono::milliseconds(static_cast<long int>(values[0] * 1000)));
                }
                MeasurementGlobalIntTimeN.value = values[0];
                MeasurementIntTimeN[0].value = values[0];
                MeasurementIntTimeN[1].value = values[0];
                MeasurementIntTimeN[2].value = values[0];
                MeasurementIntTimeN[3].value = values[0];
                IDSetNumber(&MeasurementGlobalIntTimeNP, nullptr);
                IDSetNumber(&MeasurementIntTimeNP, nullptr);
                MeasurementGlobalIntTimeNP.s = IPS_OK;
            } else {
                MeasurementIntTimeNP.s = IPS_ALERT;
            }
        } else if (!strcmp(name, MeasurementIntTimeNP.name)) {
            for (int index{0}; index < n; ++index) {
                INumber* nr = IUFindNumber(&MeasurementIntTimeNP, names[index]);
                if (nr != nullptr) {
                    std::size_t nr_pos = std::distance(MeasurementIntTimeN, nr);
                    if (voltageMeasurements[nr_pos]) {
                        voltageMeasurements[nr_pos]->setIntTime(std::chrono::milliseconds(static_cast<long int>(values[index] * 1000)));
                    } else {
                        MeasurementIntTimeNP.s = IPS_ALERT;
                    }
                }
            }
            IDSetNumber(&MeasurementIntTimeNP, nullptr);
            MeasurementIntTimeNP.s = IPS_OK;
        } 
        else if (!strcmp(name, MeasurementFactorNP.name)) {
            for (int index{0}; index < n; ++index) {
                INumber* nr = IUFindNumber(&MeasurementFactorNP, names[index]);
                if (nr != nullptr) {
                    std::size_t nr_pos = std::distance(MeasurementFactorN, nr);
                    if (voltageMeasurements[nr_pos]) {
                        voltageMeasurements[nr_pos]->setFactor(values[index]);
                        MeasurementFactorN[nr_pos].value = values[index];
                    } else {
                        MeasurementFactorNP.s = IPS_ALERT;
                    }
                }
            }
            IDSetNumber(&MeasurementFactorNP, nullptr);
            MeasurementFactorNP.s = IPS_OK;
        }
    }

    //  Nobody has claimed this, so forward it to the base class method
    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool IndiADS1x15::saveConfigItems(FILE *fp) {
    // Save custom setting
    IUSaveConfigNumber(fp, &MeasurementGlobalIntTimeNP);
    IUSaveConfigNumber(fp, &MeasurementIntTimeNP);
    IUSaveConfigNumber(fp, &MeasurementFactorNP);
    // Save base device config
    return INDI::DefaultDevice::saveConfigItems(fp);
}

bool IndiADS1x15::Connect()
{
    if (!m_interface->Connect()) return false;
    fStartTime = std::chrono::system_clock::now();
    for (auto channel : voltageMeasurements) {
        channel.reset();
    }
    
    uint8_t addr = m_interface->address();

    // search for the ADS1115 ADC at the specified addresses and initialize it
    m_adc = std::make_shared<ADS1115>(m_interface->bus(), m_interface->address());
    // the following line demonstrates, how you would like to do the upcast from
    // i2cDevice to ADS1115 but unfortunately it doesn't work
    // unless the i2cDevice-type object was created as ADS1115
    // this could in principle be achieved by templating the Connection::I2C class
//    m_adc = std::dynamic_pointer_cast<ADS1115>(m_interface->get_device());
    
    if (!m_adc || !m_adc->devicePresent()) {
        DEBUGF(INDI::Logger::DBG_ERROR, "Connection to ADS1x15 at %s, addr 0x%02X failed", m_interface->bus(), addr);
        return false;
    }
    DEBUGF(INDI::Logger::DBG_SESSION, "Connected to ADS1x15 at %s, addr 0x%02x.", m_interface->bus(), addr);
    
    StatusL.s = IPS_OK;
    StatusLP.s = IPS_OK;
    IDSetLight(&StatusLP, nullptr);
            
    m_adc->setPga(ADS1115::PGA4V);
    m_adc->setRate(ADS1115::RATE860);
    m_adc->setAGC(0, true);
    m_adc->setAGC(1, true);
    m_adc->setAGC(2, true);
    m_adc->setAGC(3, true);
    double v1 = m_adc->readVoltage(0);
    double v2 = m_adc->readVoltage(1);
    double v3 = m_adc->readVoltage(2);
    double v4 = m_adc->readVoltage(3);

    DEBUGF(INDI::Logger::DBG_SESSION, "ADC1 values ch0: %f V ch1: %f ch3: %f V ch4: %f", v1, v2, v3, v4);

    // set up the measurement voltages to be monitored
    int voltage_index = 0;
    for (auto &channel : voltageMeasurements) {
        I2cVoltageDef item = default_measurement_voltage_def;
        item.name += std::to_string(voltage_index);
        
        std::shared_ptr<PiRaTe::Ads1115Measurement> meas(
            new PiRaTe::Ads1115Measurement(
                item.name,
                m_adc,
                voltage_index,
                MeasurementFactorN[voltage_index].value,
                std::chrono::milliseconds(static_cast<std::size_t>(MeasurementIntTimeN[voltage_index].value*1000))
            )
        );
        channel = std::move(meas);
        
        voltage_index++;
    }

    return INDI::DefaultDevice::Connect();
}

bool IndiADS1x15::Disconnect()
{
    for (auto &channel : voltageMeasurements) {
        channel.reset();
    }
    m_adc.reset();
    return true;
}

void IndiADS1x15::TimerHit()
{
    if (isConnected()) {
//         DEBUG(INDI::Logger::DBG_SESSION, "Timer hit");
        SetTimer(getCurrentPollingPeriod());
        updateMonitoring();
    }
}

/**************************************************************************************
** INDI is asking us to check communication with the device via a handshake
***************************************************************************************/
bool IndiADS1x15::Handshake()
{
    // When communicating with a real device, we check here if commands are receieved  
    // and acknolowedged by the device.
    if (isConnected()) {
        return true;
    }
    return false;
}

/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char* IndiADS1x15::getDefaultName()
{
    return "ADS1x15_ADC";
}

void IndiADS1x15::updateMonitoring()
{
    // update uptime
    DriverUpTimeN.value = upTime().count() / 3600.;
    IDSetNumber(&DriverUpTimeNP, nullptr);
    
    // update voltage measurements
    int voltage_index = 0;
    if (!voltageMeasurements.empty()) {
        VoltageMeasurementNP.s = IPS_IDLE;
        for (auto meas : voltageMeasurements) {
            if (!meas->isInitialized()) {
                VoltageMeasurementN[voltage_index].value = 0.;
                VoltageMeasurementNP.s = IPS_ALERT;
            } else {
                double meanVoltage = meas->meanValue();
                double stddev = meas->stddev();
                auto nEntries = meas->nSamples();
                VoltageMeasurementN[voltage_index].value = meanVoltage;
                VoltageMeasurementErrorN[voltage_index].value = stddev;
                MeasurementBufSizeN[voltage_index].value = nEntries;
            }
            voltage_index++;
        }
        if (VoltageMeasurementNP.s != IPS_ALERT) {
            VoltageMeasurementNP.s = IPS_OK;
            VoltageMeasurementErrorNP.s = IPS_OK;
            MeasurementBufSizeNP.s = IPS_OK;
        }
        IDSetNumber(&VoltageMeasurementNP, nullptr);
        IDSetNumber(&VoltageMeasurementErrorNP, nullptr);
        IDSetNumber(&MeasurementBufSizeNP, nullptr);
    }
    for (auto& gainSwitchVector : GainSwitchPropertyArray) {
        IUResetSwitch(&gainSwitchVector);
        std::size_t ichannel = std::distance(std::begin(GainSwitchPropertyArray), &gainSwitchVector);
        if (m_adc) {
            int sw_index = static_cast<int>(m_adc->getPga(ichannel));
            GainSwitchArray[ichannel][sw_index].s = ISS_ON;
        } else {
            gainSwitchVector.s = IPS_ALERT;
        }
        IDSetSwitch(&gainSwitchVector, nullptr);
    }
}

bool IndiADS1x15::saveConfig(bool silent, const char *property)
{
    return INDI::DefaultDevice::saveConfig(silent, property);
}

auto IndiADS1x15::upTime() const -> std::chrono::duration<long, std::ratio<1>>
{
    auto now { std::chrono::system_clock::now() };
    auto difftime { now - fStartTime };
    return std::chrono::duration_cast<std::chrono::seconds>(difftime);
}
