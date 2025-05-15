/*******************************************************************************
  Copyright(c) 2017 Jasem Mutlaq. All rights reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/
 
#include "connection_i2c.h"
#include "indistandardproperty.h"
#include "indilogger.h"
 
#include <cerrno>
#include <cstring>
#include <algorithm>
#include <thread>
#include <chrono>
#include <regex>
#include <string>
#include <sstream>
#include <iomanip>
#include <cctype>
#include <stdexcept>

/// Parses a hex string (e.g., "0x48", "48", "0X1A") into an int.
/// Returns true on success, false on failure.
/// On success, stores result in `out`.
/// source: CGPT
bool parseHexString(const std::string& hexStr, int& out)
{
    try {
        // Skip leading/trailing whitespace
        size_t start = hexStr.find_first_not_of(" \t\n\r");
        size_t end   = hexStr.find_last_not_of(" \t\n\r");
        if (start == std::string::npos || end == std::string::npos)
            return false;

        std::string trimmed = hexStr.substr(start, end - start + 1);

        // Detect invalid characters (allow only digits and A-F/a-f or 0x)
        for (char c : trimmed) {
            if (!std::isxdigit(c) && c != 'x' && c != 'X' && c != '0')
                return false;
        }

        // std::stoi with base 0 auto-detects 0x for hex or falls back to decimal
        int value = std::stoi(trimmed, nullptr, 0);

        if (value < 0 || value > 0xFFFF)
            return false;

        out = value;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

namespace Connection
{
constexpr char DEFAULT_I2C_ADDRESS[] { "0x7f" }; //< default I2C address

I2C::I2C(INDI::DefaultDevice *dev, IPerm permission) : Interface(dev, CONNECTION_CUSTOM), m_Permission(permission)
{
    char buffer[256] = {0};
    // Try to load the bus from the config file. If that fails, use default bus.
    if (IUGetConfigText(dev->getDeviceName(), "I2C_BUS", "I2C_BUS", buffer, 256) == 0)
    {
        IUFillText(&BusT, "I2C_BUS", "I2C Bus", buffer);
    }
    else
    {
#ifdef __APPLE__
        IUFillText(&BusT, "I2C_BUS", "I2C Bus", "/dev/cu.i2c-0");
#else
        IUFillText(&BusT, "I2C_BUS", "I2C Bus", "/dev/i2c-0");
#endif
    }
    IUFillTextVector(&BusTP, &BusT, 1, dev->getDeviceName(), "I2C_BUS", "I2C Bus", MAIN_CONTROL_TAB, m_Permission, 60,
                     IPS_IDLE);
    
    // Try to load the device address from the config file. If that fails, use default address.
    if (IUGetConfigText(dev->getDeviceName(), "I2C_ADDRESS", "I2C_ADDRESS", buffer, 256) == 0)
    {
        IUFillText(&DevAddrT, "I2C_ADDRESS", "Device Address", buffer);
        DEBUGF(INDI::Logger::DBG_SESSION, "read i2c address from config: %s", "0x02");
    } else {
        IUFillText(&DevAddrT, "I2C_ADDRESS", "Device Address", DEFAULT_I2C_ADDRESS);
        DEBUGF(INDI::Logger::DBG_ERROR, "error reading i2c address from config: %s", buffer);
    }

    IUFillTextVector(&DevAddrTP, &DevAddrT, 1, dev->getDeviceName(), "I2C_ADDRESS", "Address", MAIN_CONTROL_TAB,
        m_Permission, 60, IPS_IDLE);
    int addr_ { 0 };
    bool conv_success_ = parseHexString(std::string(DevAddrT.text), addr_);
    if (conv_success_) {
        m_DevAddress = static_cast<std::uint8_t>(addr_);
    } else {
        m_DevAddress = 0x7f;
    }
}
 
I2C::~I2C()
{
}

std::uint8_t I2C::address()
{
    return m_DevAddress;
}

bool I2C::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (!strcmp(dev, m_Device->getDeviceName()))
    {
        // I2C Bus
        if (!strcmp(name, BusTP.name)) {
            IUUpdateText(&BusTP, texts, names, n);
            BusTP.s = IPS_OK;
            IDSetText(&BusTP, nullptr);
        } else if (!strcmp(name, DevAddrTP.name)) {
            int i2cAddress { 0 };
            if (parseHexString(std::string(texts[0]), i2cAddress)) {
                std::cout << "Parsed I2C address: 0x" 
                    << std::hex << std::uppercase << i2cAddress 
                    << " (" << std::dec << i2cAddress << ")" << std::endl;
                if (i2cAddress > 0x00 && i2cAddress < 0x80) {
                    IUUpdateText(&DevAddrTP, texts, names, n);
                    DevAddrTP.s = IPS_OK;
                    IDSetText(&DevAddrTP, nullptr);
                    m_DevAddress = static_cast<std::uint8_t>(i2cAddress);
                } else {
                    DevAddrTP.s = IPS_ALERT;
                    DEBUGF(INDI::Logger::DBG_ERROR, "Device address 0x%02x out of range (0x01..0x7f)", i2cAddress);
                }
            } else {
                DevAddrTP.s = IPS_ALERT;
                DEBUGF(INDI::Logger::DBG_ERROR, "Device address not parsable: %s", texts[0]);
                std::cerr << "Invalid I2C address: '" << texts[0] << "'" << std::endl;
            }
        }
        return true;
    }
    return false;
}

bool I2C::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    return Connection::Interface::ISNewSwitch(dev,name,states,names,n);
}

bool I2C::ISNewNumber(const char* dev, const char* name, double values[], char* names[], int n)
{
    //  Nobody has claimed this, so forward it to the base class method
    return Connection::Interface::ISNewNumber(dev, name, values, names, n);
}

bool I2C::Connect()
{
    DEBUG(INDI::Logger::DBG_SESSION,"Connecting...");
    if (Connect(BusT.text, m_DevAddress) /* && processHandshake() */)
        return true;
    return false;
}
 
bool I2C::processHandshake()
{
    DEBUG(INDI::Logger::DBG_SESSION,"Connection successful, attempting handshake...");
    bool rc = Handshake();
    if (rc)
    {
        DEBUGF(INDI::Logger::DBG_SESSION,"%s is online.", getDeviceName());
    }
    else
        DEBUG(INDI::Logger::DBG_ERROR,"Handshake failed.");
    return rc;
}
 
bool I2C::Connect(const char *bus, uint8_t addr)
{
//     if (m_Device->isSimulation())
//         return true;
    DEBUGF(INDI::Logger::DBG_SESSION,"Probing connection to %s @ %d", bus, addr);
    std::unique_ptr<i2cDevice> i2c_device = std::make_unique<i2cDevice>(bus, addr);
    if (i2c_device && i2c_device->devicePresent()) { 
        DEBUG(INDI::Logger::DBG_SESSION,"Probe successful.");
        return true;
    }
    DEBUG(INDI::Logger::DBG_ERROR,"Probe failed.");
    return false;
}
 
bool I2C::Disconnect()
{
    return true;
}
 
void I2C::Activated()
{
    m_Device->defineProperty(&BusTP);
    m_Device->defineProperty(&DevAddrTP);
}
 
void I2C::Deactivated()
{
    m_Device->deleteProperty(BusTP.name);
    m_Device->deleteProperty(DevAddrTP.name);
}
 
bool I2C::saveConfigItems(FILE *fp)
{
    if (m_Permission != IP_RO)
    {
        DEBUGF(INDI::Logger::DBG_SESSION,"Saving connection config to file %d", fp);
        IUSaveConfigText(fp, &BusTP);
        IUSaveConfigText(fp, &DevAddrTP);
    }
    return Connection::Interface::saveConfigItems(fp);
}

void I2C::setDefaultBus(const char *bus)
{
    IUFillText(&BusT, "I2C_BUS", "I2C Bus", bus);
}

void I2C::setDefaultAddress(std::uint8_t newAddress)
{
}

}
