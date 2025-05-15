
/*******************************************************************************
  Copyright(c) 2017 Jasem Mutlaq. All rights reserved.
 
 Connection Plugin Interface
 
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
 
#pragma once
 
#include <connectionplugins/connectioninterface.h>
 
#include <string>
#include <vector>
#include <cstdint>
#include <memory>

#include "i2cdevice.h"

namespace Connection
{
class I2C : public Interface
{
    public:
        I2C(INDI::DefaultDevice *dev, IPerm permission = IP_RW);
        virtual ~I2C();
 
        virtual bool Connect() override;
 
        virtual bool Disconnect() override;
 
        virtual void Activated() override;
 
        virtual void Deactivated() override;
 
        virtual std::string name() override
        {
            return "CONNECTION_I2C";
        }
 
        virtual std::string label() override
        {
            return "I2C";
        }
 
        virtual const char *bus()
        {
            return BusT.text;
        }
 
        virtual uint8_t address();
 
        void setDefaultBus(const char *bus);
 
        void setDefaultAddress(std::uint8_t newAddress);
 
        virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
        virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        virtual bool ISNewNumber(const char* dev, const char* name, double values[], char* names[], int n) override;
        virtual bool saveConfigItems(FILE *fp) override;
 
        bool Refresh(bool silent = false);
 
    protected:
        virtual bool Connect(const char *bus, uint8_t addr);
 
        virtual bool processHandshake();
 
        // Device physical bus
        IText BusT {};
        ITextVectorProperty BusTP;
 
        // Device's bus address
        IText DevAddrT {};
        ITextVectorProperty DevAddrTP;
 
        IPerm m_Permission { IP_RW };
 
        std::string m_ConfigBus;
        std::uint8_t m_DevAddress { 0x7f };
//         std::shared_ptr<i2cDevice> m_i2c_device { nullptr };
};
}
