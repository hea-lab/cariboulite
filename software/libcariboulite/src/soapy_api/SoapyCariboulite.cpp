#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include "Cariboulite.hpp"

/***********************************************************************
 * Find available devices
 **********************************************************************/
SoapySDR::KwargsList findCariboulite(const SoapySDR::Kwargs &args)
{
    std::vector<SoapySDR::Kwargs> results;

    SoapySDR::Kwargs soapyInfo;

    soapyInfo["device_id"] = "112233";
    soapyInfo["label"] = "mySDR";
    soapyInfo["serial"] = "0123456789ABCDEF";
    soapyInfo["name"] = "mysdr";
    soapyInfo["vendor"] = "product_vendor";
    soapyInfo["uuid"] = "board_info.product_uuid";
    soapyInfo["version"] = "board_info.product_version";

    results.push_back(soapyInfo);

    return results;
}

/***********************************************************************
 * Make device instance
 **********************************************************************/
SoapySDR::Device *makeCariboulite(const SoapySDR::Kwargs &args)
{
    return new Cariboulite(args);
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry registerCariboulite("MySDR", &findCariboulite, &makeCariboulite, SOAPY_SDR_ABI_VERSION);
