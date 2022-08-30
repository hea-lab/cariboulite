#include "Cariboulite.hpp"
#include "cariboulite_config/cariboulite_config_default.h"
#include <sys/ioctl.h>
#include "shibu.h"

//=================================================================
static void caribou_stream_data_event( void *ctx, 
                                        void *service_context,
                                        caribou_smi_stream_type_en type,
                                        caribou_smi_channel_en ch,
                                        size_t sample_count,
                                        caribou_smi_sample_complex_int16 *cmplx_vec,
										caribou_smi_sample_meta *meta_vec,
                                        size_t buffers_capacity_samples)
{
#if 0
    cariboulite_st* sys = (cariboulite_st*)ctx;
    Cariboulite *obj = (Cariboulite*)service_context;

    int dir = (type == caribou_smi_stream_type_read) ? SOAPY_SDR_RX : SOAPY_SDR_TX;
    int channel = (ch == caribou_smi_channel_900) ? cariboulite_channel_s1g : cariboulite_channel_6g;

    switch(type)
    {
        //-------------------------------------------------------
        case caribou_smi_stream_type_read:
            {
                int sample_queue_index = CARIBOU_SMI_GET_STREAM_ID(type, ch);
                obj->sample_queues[sample_queue_index]->Write(cmplx_vec, sample_count, 0, 10000L);
            }
            break;

        //-------------------------------------------------------
        case caribou_smi_stream_type_write:
            {
                int sample_queue_index = CARIBOU_SMI_GET_STREAM_ID(dir, channel);
                printf("Wrote to sample_queue_index %d\n", sample_queue_index);
            }
            break;

        //-------------------------------------------------------
        case caribou_smi_stream_start:
            {
                SoapySDR_logf(SOAPY_SDR_DEBUG, "start event: stream channel %d, batch length: %d samples", 
                                    ch, buffers_capacity_samples);
            }
            break;

        //-------------------------------------------------------
        case caribou_smi_stream_end:
            {
                SoapySDR_logf(SOAPY_SDR_DEBUG, "end event: stream channel %d, batch length: %d sample", 
                                    ch, buffers_capacity_samples);
            }
            break;

        //-------------------------------------------------------
        default:
            break;
    }
#endif
}

//========================================================
/*!
* Query a list of the available stream formats.
* \param direction the channel direction RX or TX
* \param channel an available channel on the device
* \return a list of allowed format strings. See setupStream() for the format syntax.
*/
std::vector<std::string> Cariboulite::getStreamFormats(const int direction, const size_t channel) const
{
    //printf("getStreamFormats\n");
    std::vector<std::string> formats;
    formats.push_back(SOAPY_SDR_CS16);
    formats.push_back(SOAPY_SDR_CS8);
    formats.push_back(SOAPY_SDR_CF32);
    formats.push_back(SOAPY_SDR_CF64);
	return formats;
}

//========================================================
/*!
* Get the hardware's native stream format for this channel.
* This is the format used by the underlying transport layer,
* and the direct buffer access API calls (when available).
* \param direction the channel direction RX or TX
* \param channel an available channel on the device
* \param [out] fullScale the maximum possible value
* \return the native stream buffer format string
*/
std::string Cariboulite::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const
{
    //printf("getNativeStreamFormat\n");
    fullScale = (double)((1<<12)-1);
    return SOAPY_SDR_CS16;
}

//========================================================
/*!
* Query the argument info description for stream args.
* \param direction the channel direction RX or TX
* \param channel an available channel on the device
* \return a list of argument info structures
*/
SoapySDR::ArgInfoList Cariboulite::getStreamArgsInfo(const int direction, const size_t channel) const
{
    //printf("getStreamArgsInfo start\n");
	SoapySDR::ArgInfoList streamArgs;
	return streamArgs;
}

//========================================================
int Cariboulite::findSampleQueue(const int direction, const size_t channel)
{
#if 0
    for (uint32_t i = 0; i < 4; i++)
    {
        if (sample_queues[i]->stream_dir == direction &&
            sample_queues[i]->stream_channel == (int)channel)
            return i;
    }
#endif
    return -1;
}

//========================================================
int Cariboulite::findSampleQueueById(int stream_id)
{
#if 0
    for (uint32_t i = 0; i < 4; i++)
    {
        if (sample_queues[i]->stream_id == stream_id)
            return i;
    }
#endif
    return -1;
}

//========================================================
/*!
* Initialize a stream given a list of channels and stream arguments.
* The implementation may change switches or power-up components.
* All stream API calls should be usable with the new stream object
* after setupStream() is complete, regardless of the activity state.
*
* The API allows any number of simultaneous TX and RX streams, but many dual-channel
* devices are limited to one stream in each direction, using either one or both channels.
* This call will throw an exception if an unsupported combination is requested,
* or if a requested channel in this direction is already in use by another stream.
*
* When multiple channels are added to a stream, they are typically expected to have
* the same sample rate. See setSampleRate().
*
* \param direction the channel direction (`SOAPY_SDR_RX` or `SOAPY_SDR_TX`)
* \param format A string representing the desired buffer format in read/writeStream()
* \parblock
*
* The first character selects the number type:
*   - "C" means complex
*   - "F" means floating point
*   - "S" means signed integer
*   - "U" means unsigned integer
*
* The type character is followed by the number of bits per number (complex is 2x this size per sample)
*
*  Example format strings:
*   - "CF32" -  complex float32 (8 bytes per element)
*   - "CS16" -  complex int16 (4 bytes per element)
*   - "CS12" -  complex int12 (3 bytes per element)
*   - "CS4" -  complex int4 (1 byte per element)
*   - "S32" -  int32 (4 bytes per element)
*   - "U8" -  uint8 (1 byte per element)
*
* \endparblock
* \param channels a list of channels or empty for automatic.
* \param args stream args or empty for defaults.
* \parblock
*
*   Recommended keys to use in the args dictionary:
*    - "WIRE" - format of the samples between device and host
* \endparblock
* \return an opaque pointer to a stream handle.
* \parblock
*
* The returned stream is not required to have internal locking, and may not be used
* concurrently from multiple threads.
* \endparblock
*/
int demo;

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>  
SoapySDR::Stream *Cariboulite::setupStream(const int direction, 
                            const std::string &format, 
                            const std::vector<size_t> &channels, 
                            const SoapySDR::Kwargs &args)
{

    //cariboulite_radio_state_st* rad_s1g = GET_RADIO_PTR(radios,cariboulite_channel_s1g);
    
    std::vector<size_t> channels_internal = channels;
    // default channel - sub1GHz
    if ( channels_internal.size() == 0 )
    {
        channels_internal.push_back(cariboulite_channel_s1g);
    }

    // currently we take only the first channel
    size_t ch = channels_internal[0];

    cariboulite_channel_en channel = (ch == cariboulite_channel_s1g) ? cariboulite_channel_s1g : cariboulite_channel_6g;
    cariboulite_channel_dir_en channel_dir = (direction == SOAPY_SDR_RX) ? cariboulite_channel_dir_rx : cariboulite_channel_dir_tx;

    cariboulite_setup_stream(&radios, channel, channel_dir);
	demo = open("/dev/mychardev", O_RDWR);

	struct setup_stream setup_stream;

    setup_stream.is_rx = direction == SOAPY_SDR_RX;
    ioctl(demo, SETUP_STREAM, (unsigned long)&setup_stream);
    SoapySDR_logf(SOAPY_SDR_INFO, "setupStream %x", demo);

	printf("FORMAT %s\n", format.c_str());

    return (SoapySDR::Stream *)((void*)(demo));
}

void Cariboulite::closeStream(SoapySDR::Stream *stream)
{
    SoapySDR_logf(SOAPY_SDR_INFO, "closeStream %x", stream);
	if (stream) {
		close(demo);
    }
}

//========================================================
/*!
     * Get the stream's maximum transmission unit (MTU) in number of elements.
     * The MTU specifies the maximum payload transfer in a stream operation.
     * This value can be used as a stream buffer allocation size that can
     * best optimize throughput given the underlying stream implementation.
     *
     * \param stream the opaque pointer to a stream handle
     * \return the MTU in number of stream elements (never zero)
     */
size_t Cariboulite::getStreamMTU(SoapySDR::Stream *stream) const
{
    //printf("getStreamMTU\n");
    return 1024 * 1024 / 2 / 4;
}


struct stream_config stream_config;

int Cariboulite::activateStream(SoapySDR::Stream *stream,
                                    const int flags,
                                    const long long timeNs,
                                    const size_t numElems)
{
    printf("activateStream flags %d timeNs %lld numElems %ld\n", flags, timeNs, numElems);

    cariboulite_activate_channel(&radios, 
                                cariboulite_channel_s1g, 
                                true);

	stream_config.activation_time = timeNs;
	stream_config.flags = flags;
	stream_config.num_elements = numElems;

    return ioctl(demo, ACTIVATE_STREAM, (unsigned long)&stream_config);
}

//========================================================
/*!
     * Deactivate a stream.
     * Call deactivate when not using using read/write().
     * The implementation control switches or halt data flow.
     *
     * The timeNs is only valid when the flags have SOAPY_SDR_HAS_TIME.
     * Not all implementations will support the full range of options.
     * In this case, the implementation returns SOAPY_SDR_NOT_SUPPORTED.
     *
     * \param stream the opaque pointer to a stream handle
     * \param flags optional flag indicators about the stream
     * \param timeNs optional deactivation time in nanoseconds
     * \return 0 for success or error code on failure
     */
int Cariboulite::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs)
{
	//printf("deactivateStream\n");

    int res = cariboulite_deactivate_channel(&radios, 
                                cariboulite_channel_s1g, 
                                true);
    return res;
    }

int Cariboulite::writeStream(
            SoapySDR::Stream *stream,
            const void * const *buffs,
            const size_t numElems,
            int &flags,
            const long long timeNs,
            const long timeoutUs)
{
	int n = write(demo, (char*) buffs[0], numElems*sizeof(uint32_t)); 
	return n;
}

int Cariboulite::readStream(
            SoapySDR::Stream *stream,
            void * const *buffs,
            const size_t numElems,
            int &flags,
            long long &timeNs,
            const long timeoutUs)
{
	struct md md;
	int n;

	/* TODO timeoutUs not supported yet */

	/* we need to retreive metadata */
	printf("readStream\n");

    long ret = ioctl(demo, GET_METADATA, (unsigned long)&md);

	printf("readStream_ioctl %ld\n", ret);
	sample_complex_float *bam = (sample_complex_float*) buffs[0];

	if (ret == 0) {
		flags = 0;
		timeNs = md.timestamp;
		uint8_t temp_buf[numElems*4];
		
		//printf("readStream %lld\n", numElems);
		n = read(demo, temp_buf, numElems*sizeof(uint32_t)); 

		for (int i = 0; i<numElems; i++) {
			bam[i].i = ((float)((temp_buf[4*i] << 8) + temp_buf[4*i+1]));
			bam[i].q = ((float)((temp_buf[4*i+2] << 8) + temp_buf[4*i+3]));
			if (bam[i].i >= (int16_t)0x1000) bam[i].i -= (int16_t)0x2000;
			if (bam[i].q >= (int16_t)0x1000) bam[i].q -= (int16_t)0x2000;
			bam[i].i /= 4095.0;
			bam[i].q /= 4095.0;
    }

	} else {
		//printf("ret %ld %d\n", ret, errno);
		flags = 0;
		n = -errno;
    }

    return n;
}
