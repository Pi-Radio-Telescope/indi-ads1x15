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

namespace Connection {
class Interface;
}

constexpr unsigned int POLL_INTERVAL_MS { 500 }; //< polling interval of this driver
constexpr std::uint8_t DEFAULT_ADC_ADDRESS { 0x48 }; //< default I2C address of ADS1115 ADCs to be read-out
constexpr std::chrono::milliseconds DEFAULT_INT_TIME { 1000 };

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
}

/**************************************************************************************
** We init our properties here. The only thing we want to init are the Debug controls
***************************************************************************************/
bool IndiADS1x15::initProperties()
{
    // ALWAYS call initProperties() of parent first
    INDI::DefaultDevice::initProperties();

    // We add an additional debug level so we can log verbose scope status
    DBG_DEVICE = INDI::Logger::getInstance().addDebugLevel("Device Verbose", "DEVICE");
    m_interface = std::make_shared<Connection::I2C>(this);
    registerConnection(m_interface.get());
    setActiveConnection(m_interface.get());
    
//     m_interface->Activated();

    setDefaultPollingPeriod(POLL_INTERVAL_MS);

    IUFillLight(&StatusL, "STATUS", "Status", IPS_OK);
    IUFillLightVector(&StatusLP, &StatusL, 1, getDeviceName(), "STATUS", "Connection", MAIN_CONTROL_TAB, IPS_IDLE);

    IUFillNumber(&MeasurementIntTimeN, "INT_TIME", "Time", "%5.2f s", 0, 60, 0.1, DEFAULT_INT_TIME.count() / 1000.);
    IUFillNumberVector(&MeasurementIntTimeNP, &MeasurementIntTimeN, 1, getDeviceName(), "INT_TIME", "Integration Time", OPTIONS_TAB,
        IP_RW, 60, IPS_IDLE);
    IUGetConfigNumber(getDeviceName(), "INT_TIME", "INT_TIME", &MeasurementIntTimeN.value);
    
    IUFillNumber(&DriverUpTimeN, "UPTIME", "Uptime", "%5.2f h", 0, 0, 0, 0);
    IUFillNumberVector(&DriverUpTimeNP, &DriverUpTimeN, 1, getDeviceName(), "DRIVER_UPTIME", "Driver Uptime", OPTIONS_TAB,
        IP_RO, 60, IPS_IDLE);

    for (std::size_t i { 0 }; i < m_num_channels; ++i) {
        IUFillNumber(&VoltageMeasurementN[i], ("MEASUREMENT" + std::to_string(i)).c_str(), (default_measurement_voltage_def.name + std::to_string(i)).c_str(), ("%4.3f " + default_measurement_voltage_def.unit).c_str(), 0, 0, 0, 0.);

        IUFillNumber(&MeasurementFactorN[i], ("FACTOR" + std::to_string(i)).c_str(), ("Factor " + std::to_string(i)).c_str(), "%4.3f ", -1e6, 1e6, 0, default_measurement_voltage_def.divider_ratio);
        
        double factor{};
        if (IUGetConfigNumber(getDeviceName(), "FACTORS", ("FACTOR"+std::to_string(i)).c_str(), &factor)==0) {
            MeasurementFactorN[i].value = factor;
        }
    }
    IUFillNumberVector(&VoltageMeasurementNP, VoltageMeasurementN, m_num_channels, getDeviceName(), "MEASUREMENTS", "Measurements", MAIN_CONTROL_TAB,
            IP_RO, 60, IPS_IDLE);

    IUFillNumberVector(&MeasurementFactorNP, MeasurementFactorN, m_num_channels, getDeviceName(), "FACTORS", "Calibration", OPTIONS_TAB,
            IP_RW, 60, IPS_IDLE);

    addDebugControl();
    return true;
}

bool IndiADS1x15::updateProperties()
{
    // ALWAYS call updateProperties() of parent first
    INDI::DefaultDevice::updateProperties();

    if (isConnected()) {
        defineProperty(&StatusLP);
        defineProperty(&VoltageMeasurementNP);
        defineProperty(&MeasurementIntTimeNP);
        defineProperty(&MeasurementFactorNP);
        defineProperty(&DriverUpTimeNP);
        if (m_interface) m_interface->Deactivated();
    } else {
        deleteProperty(StatusLP.name);
        deleteProperty(VoltageMeasurementNP.name);
        deleteProperty(MeasurementIntTimeNP.name);
        deleteProperty(MeasurementFactorNP.name);
        deleteProperty(DriverUpTimeNP.name);
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
    //  Nobody has claimed this, so forward it to the base class' method
    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool IndiADS1x15::ISNewNumber(const char* dev, const char* name, double values[], char* names[], int n)
{
    if (!strcmp(dev, getDeviceName())) {
        //  This one is for us
        if (!strcmp(name, MeasurementIntTimeNP.name)) {
            if (!voltageMeasurements.empty() && values[0] > 0. && values[0] < 1000.) {
                for (auto meas : voltageMeasurements) {
                    meas->setIntTime(std::chrono::milliseconds(static_cast<long int>(values[0] * 1000)));
                }
                MeasurementIntTimeN.value = values[0];
                IDSetNumber(&MeasurementIntTimeNP, nullptr);
                MeasurementIntTimeNP.s = IPS_OK;
            } else {
                MeasurementIntTimeNP.s = IPS_ALERT;
            }
        } else if (!strcmp(name, MeasurementFactorNP.name)) {
            if (!voltageMeasurements.empty()) {
                for (std::size_t i { 0 }; i < std::min(m_num_channels, static_cast<std::size_t>(n)); ++i) {
                    voltageMeasurements[i]->setFactor(values[i]);
                    MeasurementFactorN[i].value = values[i];
                }
                IDSetNumber(&MeasurementFactorNP, nullptr);
                MeasurementFactorNP.s = IPS_OK;
            } else {
                MeasurementFactorNP.s = IPS_ALERT;
            }
        }
    }

    //  Nobody has claimed this, so forward it to the base class method
    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool IndiADS1x15::saveConfigItems(FILE *fp) {
    // Save custom setting
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
    m_adc->setAGC(true);
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
                std::chrono::milliseconds(static_cast<std::size_t>(MeasurementIntTimeN.value*1000))
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
                VoltageMeasurementN[voltage_index].value = meanVoltage;
            }
            voltage_index++;
        }
        if (VoltageMeasurementNP.s != IPS_ALERT) {
            VoltageMeasurementNP.s = IPS_OK;
        }
        IDSetNumber(&VoltageMeasurementNP, nullptr);
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
