#include <cassert>
#include <chrono>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <string>
#include <unistd.h>

#include "ads1115.h"
#include "ads1115_measurement.h"
#include "utility.h"

#define DEFAULT_VERBOSITY 1

namespace PiRaTe {

constexpr std::chrono::microseconds loop_delay { 10000L };

Ads1115Measurement::Ads1115Measurement(std::string name,
    std::shared_ptr<ADS1115> adc,
    std::uint8_t adc_channel,
    double factor,
    std::chrono::milliseconds integration_time)
    : fName { std::move(name) }
    , fAdc { adc }
    , fAdcChannel { adc_channel }
    , fFactor { factor }
    , fIntTime { integration_time }
{
    // initialize ADC if one was supplied in the argument list
    if (fAdc != nullptr && fAdc->devicePresent()) {
        //fAdc->setPga(ADS1115::PGA4V);
        //fAdc->setRate(ADS1115::RATE860);
        //fAdc->setAGC(true);
    } else {
        return;
    }
    fActiveLoop = true;
    // since C++14 using std::make_unique
    fThread = std::make_unique<std::thread>( [this]() { this->threadLoop(); } );
    // C++11 is unfortunately more unconvenient with move from a locally generated pointer
//     std::unique_ptr<std::thread> thread(new std::thread([this]() { this->threadLoop(); }));
//     fThread = std::move(thread);
}

Ads1115Measurement::~Ads1115Measurement()
{
    if (!fActiveLoop)
        return;
    fActiveLoop = false;
    if (fThread != nullptr)
        fThread->join();
}

// this is the background thread loop
void Ads1115Measurement::threadLoop()
{
    while (fActiveLoop) {
        if (hasAdc() && !fInhibited) {
            double conv_time { 0. };
            if ([[maybe_unused]] bool readout_guard = true) {
                // read current voltage from adc
                fMutex.lock();
                fValue = fAdc->readVoltage(fAdcChannel) * fFactor;
                auto currentTime = std::chrono::system_clock::now();
                conv_time = fAdc->getLastConvTime();
                pruneOldSamples(currentTime);
//                 while (!fIntegrationBuffer.empty() && fIntegrationBuffer.front().time < (currentTime - fIntTime)) {
//                     fIntegrationBuffer.pop_front();
//                 }
                fRunningSum += fValue;
                fRunningSumSq += fValue * fValue;
                fIntegrationBuffer.push_back({ std::move(currentTime), fValue });

                fUpdated = true;
                fMutex.unlock();
            }
            if (fVoltageReadyFn)
                fVoltageReadyFn(fValue);
            auto actual_loop_delay = std::chrono::microseconds(std::max(loop_delay.count() - static_cast<long long int>(conv_time * 1000), 1000LL));
            std::this_thread::sleep_for(actual_loop_delay);
        } else {
            pruneOldSamples(std::chrono::system_clock::now());
            std::this_thread::sleep_for(loop_delay);
        }
    }
}

void Ads1115Measurement::pruneOldSamples(std::chrono::time_point<std::chrono::system_clock> now) {
    while (!fIntegrationBuffer.empty() && now - fIntegrationBuffer.front().time > fIntTime) {
        const Sample& old = fIntegrationBuffer.front();
        fRunningSum -= old.value;
        fRunningSumSq -= old.value * old.value;
        fIntegrationBuffer.pop_front();
    }
}

auto Ads1115Measurement::currentValue() -> double
{
    std::lock_guard<std::mutex> lock(fMutex);
    fUpdated = false;
    return fValue;
}

auto Ads1115Measurement::meanValue() -> double
{
    std::lock_guard<std::mutex> lock(fMutex);
    fUpdated = false;
    if (fIntegrationBuffer.empty())
        return 0.;
    if (fIntegrationBuffer.size() == 1)
        return fValue;
    return fRunningSum / fIntegrationBuffer.size();
//     double mean = std::accumulate(fIntegrationBuffer.begin(), fIntegrationBuffer.end(), 0.0, [](double sum, Sample s) {
//         return sum + s.value;
//     }) / fIntegrationBuffer.size();
//     return mean;
}

auto Ads1115Measurement::stddev() -> double
{
    if (fIntegrationBuffer.size() < 2) return 0.0;
    double mean_val = meanValue();
    std::lock_guard<std::mutex> lock(fMutex);
    return std::sqrt(fRunningSumSq / fIntegrationBuffer.size() - mean_val * mean_val);
}

auto Ads1115Measurement::nSamples() -> std::size_t
{
    std::lock_guard<std::mutex> lock(fMutex);
    return fIntegrationBuffer.size();
}
    
void Ads1115Measurement::setIntTime(std::chrono::milliseconds ms)
{
    std::lock_guard<std::mutex> lock(fMutex);
    fIntTime = ms;
}

void Ads1115Measurement::setFactor(double factor)
{
    std::lock_guard<std::mutex> lock(fMutex);
    fFactor = factor;
}

void Ads1115Measurement::inhibit(bool state)
{
    if (fInhibited == state) return;
    std::lock_guard<std::mutex> lock(fMutex);
    fInhibited = state;
}

} // namespace PiRaTe
