#include <SoapySDR/Version.hpp>
#include <SoapySDR/Modules.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/ConverterRegistry.hpp>

#include <algorithm>
#include <cstdlib>
#include <cstddef>
#include <iostream>
#include <iomanip>
#include <csignal>
#include <chrono>
#include <thread>
#include <getopt.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "modes.h"

#define OUTPUT_FILENAME "iq_dump.m"


/***********************************************************************
 * Find devices and print args
 **********************************************************************/
static int findDevices(const std::string &argStr, const bool sparse)
{
    const auto results = SoapySDR::Device::enumerate(argStr);
    std::cout << "Found " << results.size() << " devices" << std::endl;

    for (size_t i = 0; i < results.size(); i++)
    {
        std::cout << "Found device " << i << std::endl;
        for (const auto &it : results[i])
        {
            std::cout << "  " << it.first << " = " << it.second << std::endl;
        }
        std::cout << std::endl;
    }
    if (results.empty()) std::cerr << "No devices found! " << argStr << std::endl;
    else std::cout << std::endl;

    return results.empty()?EXIT_FAILURE:EXIT_SUCCESS;
}

/***********************************************************************
 * Run the stream function
 **********************************************************************/
static sig_atomic_t loopDone = false;
static void sigIntHandler(const int)
{
    loopDone = true;
}

void runSoapyProcess(
    SoapySDR::Device *device,
    SoapySDR::Stream *stream,
    const int direction,
    const size_t numChans,
    const size_t elemSize)
{
    //allocate buffers for the stream read/write
    const size_t numElems = device->getStreamMTU(stream);
    int16_t* buff = (int16_t*)malloc (2*sizeof(int16_t)*numElems);	// complex 16 bit samples

    std::cout << "Starting stream loop, press Ctrl+C to exit..." << std::endl;
    device->activateStream(stream);
    signal(SIGINT, sigIntHandler);
    while (not loopDone)
    {

        int ret = 0;
        int flags = 0;
        long long timeUS = numElems;
        ret = device->readStream(stream, (void* const*)&buff, numElems, flags, timeUS);

	if (ret != numElems) {
		printf("ret %d\n", ret);
	}

	break;
    }

    device->deactivateStream(stream);

    // export results
    //
    FILE * fid = fopen(OUTPUT_FILENAME,"w");
    fprintf(fid,"%% %s : auto-generated file\n", OUTPUT_FILENAME);
    fprintf(fid,"clear all\n");
    fprintf(fid,"close all\n");
    int num_samples = numElems;
    fprintf(fid,"num_samples = %u;\n", num_samples);

    fprintf(fid,"y = zeros(1,num_samples);\n");
    for (int i=0; i<num_samples; i++)
        fprintf(fid,"y(%4u) = %12.8f + j*%12.8f;\n", i+1, ((float) buff[2*i])/4096, ((float)buff[2*i+1])/4096);

    fprintf(fid,"t=[0:(num_samples-1)];\n");
    fprintf(fid,"figure;\n");
    fprintf(fid,"subplot(4,1,1);\n");
    fprintf(fid,"  plot(t,real(y), t,imag(y));\n");
    //fprintf(fid,"  plot(t,real(y));\n");
    fprintf(fid,"  grid on;\n");
    fprintf(fid,"  xlabel('time');\n");
    fprintf(fid,"  ylabel('received signal');\n");

    fclose(fid);
    printf("results written to '%s'\n", OUTPUT_FILENAME);
}


/***********************************************************************
 * Main entry point
 **********************************************************************/
int main(int argc, char *argv[])
{
    //SoapySDR::ModuleManager mm(false);
    SoapySDR::Device *device(nullptr);
    std::vector<size_t> channels;
    std::string argStr;
    double fullScale = 0.0;
    double freq = 870e6;

    findDevices("Cariboulite", false);

    try
    {
        device = SoapySDR::Device::make(argStr);

        // push the 6GHZ channel into the channel list
        channels.push_back(0);

        // set the sample rate
        device->setSampleRate(SOAPY_SDR_RX, channels[0], 4e6);
	device->setBandwidth(SOAPY_SDR_RX, channels[0], 0.5e6);
	device->setGainMode(SOAPY_SDR_RX, channels[0], false);
	device->setGain(SOAPY_SDR_RX, channels[0], 70);
	device->setFrequency(SOAPY_SDR_RX, channels[0], freq);

        //create the stream, use the native format   
        const auto format = device->getNativeStreamFormat(SOAPY_SDR_RX, channels.front(), fullScale);
        const size_t elemSize = SoapySDR::formatToSize(format);
        auto stream = device->setupStream(SOAPY_SDR_RX, format, channels);

        //run the rate test one setup is complete
        std::cout << "Stream format: " << format << std::endl;
        std::cout << "Num channels: " << channels.size() << std::endl;
        std::cout << "Element size: " << elemSize << " bytes" << std::endl;
        runSoapyProcess(device, stream, SOAPY_SDR_RX, channels.size(), elemSize);

        //cleanup stream and device
        device->closeStream(stream);
        SoapySDR::Device::unmake(device);
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Error " << ex.what() << std::endl;
        SoapySDR::Device::unmake(device);
        return EXIT_FAILURE;
    }
    std::cout << std::endl;


    return 0;
}
