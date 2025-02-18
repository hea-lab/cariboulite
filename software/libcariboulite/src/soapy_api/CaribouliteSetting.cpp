#include <math.h>
#include "Cariboulite.hpp"

SoapyCaribouliteSession Cariboulite::sess;

/*******************************************************************
 * Constructor / Destructor
 ******************************************************************/
Cariboulite::Cariboulite(const SoapySDR::Kwargs &args)
{
    cariboulite_init_radios(&radios, &sess.cariboulite_sys);

	_current_mode=HACKRF_TRANSCEIVER_MODE_OFF;

    // TODO: Exception when error
}

//========================================================
Cariboulite::~Cariboulite()
{
    cariboulite_dispose_radios(&radios);
}

/*******************************************************************
 * Identification API
 ******************************************************************/
SoapySDR::Kwargs Cariboulite::getHardwareInfo() const
{
    //key/value pairs for any useful information
    //this also gets printed in --probe
    SoapySDR::Kwargs args;

	/* TODO */

#if 0
    uint32_t serial_number = 0;
    uint32_t deviceId = 0;
    int count = 0;
    cariboulite_get_serial_number((cariboulite_st*)&sess.cariboulite_sys, &serial_number, &count) ;

    args["device_id"] = std::to_string(deviceId);
    args["serial_number"] = std::to_string(serial_number);
    args["hardware_revision"] = sess.cariboulite_sys.board_info.product_version;
    args["fpga_revision"] = std::to_string(1);
    args["vendor_name"] = sess.cariboulite_sys.board_info.product_vendor;
    args["product_name"] = sess.cariboulite_sys.board_info.product_name;
#endif

    return args;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/
std::vector<std::string> Cariboulite::listAntennas( const int direction, const size_t channel ) const
{
	std::vector<std::string> options;
    if (channel == cariboulite_channel_s1g) options.push_back( "TX/RX Sub1GHz" );
	return(options);
}

//========================================================
std::string Cariboulite::getAntenna( const int direction, const size_t channel ) const
{
	if (channel == cariboulite_channel_s1g) return "TX/RX Sub1GHz";
    return "";
}

/*******************************************************************
 * Gain API
 ******************************************************************/
std::vector<std::string> Cariboulite::listGains(const int direction, const size_t channel) const
{
    std::vector<std::string> gains;
    if (direction == SOAPY_SDR_RX)
    {
        gains.push_back("Modem AGC");
    }
    else if (direction == SOAPY_SDR_TX)
    {
        gains.push_back("Modem PA");
    }

    return (gains);
}

//========================================================
/*!
* Set the overall amplification in a chain.
* The gain will be distributed automatically across available element.
* \param direction the channel direction RX or TX
* \param channel an available channel on the device
* \param value the new amplification value in dB
*/
void Cariboulite::setGain(const int direction, const size_t channel, const double value)
{
    cariboulite_radio_state_st* rad = channel == cariboulite_channel_s1g? &radios.radio_sub1g : &radios.radio_6g;
    bool cur_agc_mode = rad->rx_agc_on;

    if (direction == SOAPY_SDR_RX)
    {
        cariboulite_set_rx_gain_control(&radios, (cariboulite_channel_en)channel, cur_agc_mode, value);
    }
    else if (direction == SOAPY_SDR_TX)
    {
        // base if -18dBm output so, given a gain of 0dB we should have -18 dBm
        cariboulite_set_tx_power(&radios, (cariboulite_channel_en)channel, value - 18.0);
    }
}

//========================================================
void Cariboulite::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
    // no amplification device names - redirect to  the regular setGain function
    Cariboulite::setGain(direction, channel, value);
}

//========================================================
double Cariboulite::getGain(const int direction, const size_t channel) const
{
    int value = 0;
   
    if (direction == SOAPY_SDR_RX)
    {
        cariboulite_get_rx_gain_control((cariboulite_radios_st*)&radios, (cariboulite_channel_en)channel, NULL, &value);
    }
    else if (direction == SOAPY_SDR_TX)
    {
        int temp = 0;
        cariboulite_get_tx_power((cariboulite_radios_st*)&radios, (cariboulite_channel_en)channel, &temp);
        value = temp + 18.0;
    }
    SoapySDR_logf(SOAPY_SDR_INFO, "getGain dir: %d, channel: %ld, value: %d", direction, channel, value);
    return (double)value;
}

//========================================================
double Cariboulite::getGain(const int direction, const size_t channel, const std::string &name) const
{
    return getGain(direction, channel);
}

//========================================================
SoapySDR::Range Cariboulite::getGainRange(const int direction, const size_t channel) const
{
    if (direction == SOAPY_SDR_RX)
    {
        return SoapySDR::Range(0.0, 23.0*3.0);
    }

    // and for TX
    return SoapySDR::Range(0.0, 31.0);
}

//========================================================
SoapySDR::Range Cariboulite::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
    return getGainRange(direction, channel);
}


//========================================================
bool Cariboulite::hasGainMode(const int direction, const size_t channel) const
{
    return (direction==SOAPY_SDR_RX?true:false);
}

//========================================================
/*!
* Set the automatic gain mode on the chain.
* \param direction the channel direction RX or TX
* \param channel an available channel on the device
* \param automatic true for automatic gain setting
*/
void Cariboulite::setGainMode( const int direction, const size_t channel, const bool automatic )
{
    cariboulite_radio_state_st* rad = channel == cariboulite_channel_s1g? &radios.radio_sub1g : &radios.radio_6g;
    bool rx_gain = rad->rx_gain_value_db;

    if (direction == SOAPY_SDR_RX)
    {
        cariboulite_set_rx_gain_control(&radios, (cariboulite_channel_en)channel, automatic, rx_gain);
    }
}

//========================================================
/*!
* Get the automatic gain mode on the chain.
* \param direction the channel direction RX or TX
* \param channel an available channel on the device
* \return true for automatic gain setting
*/
bool Cariboulite::getGainMode( const int direction, const size_t channel ) const
{
    bool mode = false;

    if (direction == SOAPY_SDR_RX)
    {
        cariboulite_get_rx_gain_control((cariboulite_radios_st*)&radios, (cariboulite_channel_en)channel, 
                                            &mode, NULL);
        SoapySDR_logf(SOAPY_SDR_INFO, "getGainMode dir: %d, channel: %ld, auto: %d", direction, channel, mode);
        return mode;
    }
    
    return (false);
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/
void Cariboulite::setSampleRate( const int direction, const size_t channel, const double rate )
{
    cariboulite_radio_state_st* rad = channel == cariboulite_channel_s1g? &radios.radio_sub1g : &radios.radio_6g;
    at86rf215_radio_f_cut_en rx_cuttof = rad->rx_fcut;
    at86rf215_radio_f_cut_en tx_cuttof = rad->tx_fcut;

    if (direction == SOAPY_SDR_RX)
    {
        cariboulite_set_rx_samp_cutoff(&radios, 
                                   (cariboulite_channel_en)channel,
                                   at86rf215_radio_rx_sample_rate_4000khz,
                                   rx_cuttof);
    }
    else if (direction == SOAPY_SDR_TX)
    {
        cariboulite_set_tx_samp_cutoff(&radios, 
                                   (cariboulite_channel_en)channel,
                                   at86rf215_radio_rx_sample_rate_4000khz,
                                   tx_cuttof);
    }
}

//========================================================
double Cariboulite::getSampleRate( const int direction, const size_t channel ) const
{
    // A single option for now. then we will add more options
    return 4000000;
}

//========================================================
std::vector<double> Cariboulite::listSampleRates( const int direction, const size_t channel ) const
{
    std::vector<double> options;
	options.push_back( 4000000 );
	return(options);
}

#define BW_SHIFT_FACT   (1.25)

//========================================================
static at86rf215_radio_rx_bw_en convertRxBandwidth(double bw_numeric)
{
    float fact = BW_SHIFT_FACT;
    if (fabs(bw_numeric - (160000*fact)) < 1) return at86rf215_radio_rx_bw_BW160KHZ_IF250KHZ;
    if (fabs(bw_numeric - (200000*fact)) < 1) return at86rf215_radio_rx_bw_BW200KHZ_IF250KHZ;
    if (fabs(bw_numeric - (250000*fact)) < 1) return at86rf215_radio_rx_bw_BW250KHZ_IF250KHZ;
    if (fabs(bw_numeric - (320000*fact)) < 1) return at86rf215_radio_rx_bw_BW320KHZ_IF500KHZ;
    if (fabs(bw_numeric - (400000*fact)) < 1) return at86rf215_radio_rx_bw_BW400KHZ_IF500KHZ;
    if (fabs(bw_numeric - (500000*fact)) < 1) return at86rf215_radio_rx_bw_BW500KHZ_IF500KHZ;
    if (fabs(bw_numeric - (630000*fact)) < 1) return at86rf215_radio_rx_bw_BW630KHZ_IF1000KHZ;
    if (fabs(bw_numeric - (800000*fact)) < 1) return at86rf215_radio_rx_bw_BW800KHZ_IF1000KHZ;
    if (fabs(bw_numeric - (1000000*fact)) < 1) return at86rf215_radio_rx_bw_BW1000KHZ_IF1000KHZ;
    if (fabs(bw_numeric - (1250000*fact)) < 1) return at86rf215_radio_rx_bw_BW1250KHZ_IF2000KHZ;
    if (fabs(bw_numeric - (1600000*fact)) < 1) return at86rf215_radio_rx_bw_BW1600KHZ_IF2000KHZ;
    if (fabs(bw_numeric - (2000000*fact)) < 1) return at86rf215_radio_rx_bw_BW2000KHZ_IF2000KHZ;

    //if (fabs(bw_numeric - (2500000*fact)) < 1) return at86rf215_radio_rx_bw_BW2000KHZ_IFCCKHZ;
    //if (fabs(bw_numeric - (3000000*fact)) < 1) return at86rf215_radio_rx_bw_BW2000KHZ_IFDDHZ;
    //if (fabs(bw_numeric - (4000000*fact)) < 1) return at86rf215_radio_rx_bw_BW2000KHZ_IFEEKHZ;
    //if (fabs(bw_numeric - (5000000*fact)) < 1) return at86rf215_radio_rx_bw_BW2000KHZ_IFFFKHZ;
    
    return at86rf215_radio_rx_bw_BW2000KHZ_IF2000KHZ;
}

//========================================================
static double convertRxBandwidth(at86rf215_radio_rx_bw_en bw_en)
{
    float fact = BW_SHIFT_FACT;
    if (at86rf215_radio_rx_bw_BW160KHZ_IF250KHZ == bw_en) return 160000 * fact;
    if (at86rf215_radio_rx_bw_BW200KHZ_IF250KHZ == bw_en) return 200000 * fact;
    if (at86rf215_radio_rx_bw_BW250KHZ_IF250KHZ == bw_en) return 250000 * fact;
    if (at86rf215_radio_rx_bw_BW320KHZ_IF500KHZ == bw_en) return 320000 * fact;
    if (at86rf215_radio_rx_bw_BW400KHZ_IF500KHZ == bw_en) return 400000 * fact;
    if (at86rf215_radio_rx_bw_BW500KHZ_IF500KHZ == bw_en) return 500000 * fact;
    if (at86rf215_radio_rx_bw_BW630KHZ_IF1000KHZ == bw_en) return 630000 * fact;
    if (at86rf215_radio_rx_bw_BW800KHZ_IF1000KHZ == bw_en) return 800000 * fact;
    if (at86rf215_radio_rx_bw_BW1000KHZ_IF1000KHZ == bw_en) return 1000000 * fact;
    if (at86rf215_radio_rx_bw_BW1250KHZ_IF2000KHZ == bw_en) return 1250000 * fact;
    if (at86rf215_radio_rx_bw_BW1600KHZ_IF2000KHZ == bw_en) return 1600000 * fact;
    if (at86rf215_radio_rx_bw_BW2000KHZ_IF2000KHZ == bw_en) return 2000000 * fact;

    //if (at86rf215_radio_rx_bw_BW2000KHZ_IFCCKHZ == bw_en) return 2500000 * fact;
    //if (at86rf215_radio_rx_bw_BW2000KHZ_IFDDHZ == bw_en) return 3000000 * fact;
    //if (at86rf215_radio_rx_bw_BW2000KHZ_IFEEKHZ == bw_en) return 4000000 * fact;
    //if (at86rf215_radio_rx_bw_BW2000KHZ_IFFFKHZ == bw_en) return 5000000 * fact;
    
    return 2000000 * fact;
}

//========================================================
static at86rf215_radio_tx_cut_off_en convertTxBandwidth(double bw_numeric)
{
    if (fabs(bw_numeric - 80000) < 1) return at86rf215_radio_tx_cut_off_80khz;
    if (fabs(bw_numeric - 100000) < 1) return at86rf215_radio_tx_cut_off_100khz;
    if (fabs(bw_numeric - 125000) < 1) return at86rf215_radio_tx_cut_off_125khz;
    if (fabs(bw_numeric - 160000) < 1) return at86rf215_radio_tx_cut_off_160khz;
    if (fabs(bw_numeric - 200000) < 1) return at86rf215_radio_tx_cut_off_200khz;
    if (fabs(bw_numeric - 250000) < 1) return at86rf215_radio_tx_cut_off_250khz;
    if (fabs(bw_numeric - 315000) < 1) return at86rf215_radio_tx_cut_off_315khz;
    if (fabs(bw_numeric - 400000) < 1) return at86rf215_radio_tx_cut_off_400khz;
    if (fabs(bw_numeric - 500000) < 1) return at86rf215_radio_tx_cut_off_500khz;
    if (fabs(bw_numeric - 625000) < 1) return at86rf215_radio_tx_cut_off_625khz;
    if (fabs(bw_numeric - 800000) < 1) return at86rf215_radio_tx_cut_off_800khz;
    if (fabs(bw_numeric - 1000000) < 1) return at86rf215_radio_tx_cut_off_1000khz;
    return at86rf215_radio_tx_cut_off_1000khz;
}

//========================================================
static double convertTxBandwidth(at86rf215_radio_tx_cut_off_en bw_en)
{
    if (at86rf215_radio_tx_cut_off_80khz == bw_en) return 80000;
    if (at86rf215_radio_tx_cut_off_100khz == bw_en) return 100000;
    if (at86rf215_radio_tx_cut_off_125khz == bw_en) return 125000;
    if (at86rf215_radio_tx_cut_off_160khz == bw_en) return 160000;
    if (at86rf215_radio_tx_cut_off_200khz == bw_en) return 200000;
    if (at86rf215_radio_tx_cut_off_250khz == bw_en) return 250000;
    if (at86rf215_radio_tx_cut_off_315khz == bw_en) return 315000;
    if (at86rf215_radio_tx_cut_off_400khz == bw_en) return 400000;
    if (at86rf215_radio_tx_cut_off_500khz == bw_en) return 500000;
    if (at86rf215_radio_tx_cut_off_625khz == bw_en) return 625000;
    if (at86rf215_radio_tx_cut_off_800khz == bw_en) return 800000;
    if (at86rf215_radio_tx_cut_off_1000khz == bw_en) return 1000000;
    return 1000000;
}


//========================================================
void Cariboulite::setBandwidth( const int direction, const size_t channel, const double bw )
{
	double setbw = bw;

	if (setbw < 2000000.0) setbw = 2000000.0;

    if (direction == SOAPY_SDR_RX)
    {
		cariboulite_set_rx_bandwidth(&radios,(cariboulite_channel_en)channel, convertRxBandwidth(setbw));
    }
    else if (direction == SOAPY_SDR_TX)
    {
        cariboulite_set_tx_bandwidth(&radios,(cariboulite_channel_en)channel, 
                                convertTxBandwidth(setbw));
    }
}

//========================================================
double Cariboulite::getBandwidth( const int direction, const size_t channel ) const
{
    if (direction == SOAPY_SDR_RX)
    {
        at86rf215_radio_rx_bw_en bw;
        cariboulite_get_rx_bandwidth((cariboulite_radios_st*)&radios,(cariboulite_channel_en)channel, &bw);
        return convertRxBandwidth(bw);
    }
    else if (direction == SOAPY_SDR_TX)
    {
        at86rf215_radio_tx_cut_off_en bw;
        cariboulite_get_tx_bandwidth((cariboulite_radios_st*)&radios,(cariboulite_channel_en)channel, &bw);
        return convertTxBandwidth(bw);
    }
    return 0.0;
}

//========================================================
std::vector<double> Cariboulite::listBandwidths( const int direction, const size_t channel ) const
{
    std::vector<double> options;
    if (direction == SOAPY_SDR_RX)
    {
        float fact = BW_SHIFT_FACT;
		options.push_back( 20000 );
		options.push_back( 50000 );
		options.push_back( 100000 );
        options.push_back( 160000*fact );
        options.push_back( 200000*fact );
        options.push_back( 250000*fact );
        options.push_back( 320000*fact );
        options.push_back( 400000*fact );
        options.push_back( 500000*fact );
        options.push_back( 630000*fact );
        options.push_back( 800000*fact );
        options.push_back( 1000000*fact );
        options.push_back( 1250000*fact );
        options.push_back( 1600000*fact );
        options.push_back( 2000000*fact );

        //options.push_back( 2500000*fact );
        //options.push_back( 3000000*fact );
        //options.push_back( 4000000*fact );
        //options.push_back( 5000000*fact );
    }
    else
    {
        options.push_back( 80000 );
        options.push_back( 100000 );
        options.push_back( 125000 );
        options.push_back( 160000 );
        options.push_back( 200000 );
        options.push_back( 250000 );
        options.push_back( 315000 );
        options.push_back( 400000 );
        options.push_back( 500000 );
        options.push_back( 625000 );
        options.push_back( 800000 );
        options.push_back( 1000000 );
    }
    return(options);
}

/*******************************************************************
 * Frequency API
 ******************************************************************/
void Cariboulite::setFrequency( const int direction, const size_t channel, const std::string &name, 
                                const double frequency, const SoapySDR::Kwargs &args )
{
    int err = 0;
    if (name != "RF")
    {
        return;
    }

    err = cariboulite_set_frequency(&radios, (cariboulite_channel_en)channel, true, (double *)&frequency);
    if (err == 0) SoapySDR_logf(SOAPY_SDR_INFO, "setFrequency dir: %d, channel: %ld, freq: %.2f", direction, channel, frequency);
    else SoapySDR_logf(SOAPY_SDR_ERROR, "setFrequency dir: %d, channel: %ld, freq: %.2f FAILED", direction, channel, frequency);
}

//========================================================
double Cariboulite::getFrequency( const int direction, const size_t channel, const std::string &name ) const
{
    double freq;
    if (name != "RF")
    {
        return 0.0;
    }

    cariboulite_get_frequency((cariboulite_radios_st*)&radios,(cariboulite_channel_en)channel, &freq, NULL, NULL);
    return freq;
}

//========================================================
SoapySDR::ArgInfoList Cariboulite::getFrequencyArgsInfo(const int direction, const size_t channel) const
{
    SoapySDR::ArgInfoList freqArgs;
	// TODO: frequency arguments
	return freqArgs;
}

//========================================================
std::vector<std::string> Cariboulite::listFrequencies( const int direction, const size_t channel ) const
{
    // on both sub1ghz and the wide channel, the RF frequency is controlled
    std::vector<std::string> names;
	names.push_back( "RF" );
	return(names);
}

//========================================================
SoapySDR::RangeList Cariboulite::getFrequencyRange( const int direction, const size_t channel, const std::string &name ) const
{
    if (name != "RF" ) {
		throw std::runtime_error( "getFrequencyRange(" + name + ") unknown name" );
    }

    if (channel == cariboulite_channel_s1g) {
        SoapySDR::RangeList list;
        list.push_back(SoapySDR::Range( 389.5e6, 510e6 ));
        list.push_back(SoapySDR::Range( 779e6, 1020e6 ));
        return list;
    }
    throw std::runtime_error( "getFrequencyRange - unknown channel" );
}
