//TODO review  activateStream
//TODO review  readStream
//TODO review  writeStream

#include "Cariboulite.hpp"
#include "cariboulite_config/cariboulite_config_default.h"
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>  
#include "stream/smi_kernel.h"

/*!
* Query a list of the available stream formats.
* \param direction the channel direction RX or TX
* \param channel an available channel on the device
* \return a list of allowed format strings. See setupStream() for the format syntax.
*/
std::vector<std::string> Cariboulite::getStreamFormats(const int direction, const size_t channel) const
{
    std::vector<std::string> formats;
    formats.push_back(SOAPY_SDR_CS16);
    formats.push_back(SOAPY_SDR_CS8);
    formats.push_back(SOAPY_SDR_CF32);
    formats.push_back(SOAPY_SDR_CF64);
	return formats;
}

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
    fullScale = (double)((1<<12)-1);
    return SOAPY_SDR_CS16;
}

/*!
* Query the argument info description for stream args.
* \param direction the channel direction RX or TX
* \param channel an available channel on the device
* \return a list of argument info structures
*/
SoapySDR::ArgInfoList Cariboulite::getStreamArgsInfo(const int direction, const size_t channel) const
{
	SoapySDR::ArgInfoList streamArgs;
	return streamArgs;
}

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
SoapySDR::Stream *Cariboulite::setupStream(const int direction, 
                            const std::string &format, 
                            const std::vector<size_t> &channels, 
                            const SoapySDR::Kwargs &args)
{

	std::lock_guard<std::mutex> lock(_device_mutex);

	int fd;

	if (direction==SOAPY_SDR_RX){
		if (_rx_stream.opened) {
			throw std::runtime_error("RX stream already opened");
		}

		fd = open("/dev/mychardev", O_RDWR);
		_rx_stream.opened = true;
		_rx_stream.stream = (SoapySDR::Stream *)((long)(fd));
		ioctl(fd, SETUP_STREAM_RX);

	} else if (direction==SOAPY_SDR_TX){
		if (_tx_stream.opened) {
			throw std::runtime_error("TX stream already opened");
		}

		fd = open("/dev/mychardev", O_RDWR);
		_tx_stream.opened = true;
		_tx_stream.stream = (SoapySDR::Stream *)((long)(fd));
		ioctl(fd, SETUP_STREAM_TX);

	} else {
		throw std::runtime_error("Invalid direction");
	}

	SoapySDR_logf(SOAPY_SDR_INFO, "setupStream %x", fd);

	return (SoapySDR::Stream *)((long)(fd));
}

/*!
 * Close an open stream created by setupStream
 * The implementation may change switches or power-down components.
 * \param stream the opaque pointer to a stream handle
 */
void Cariboulite::closeStream(SoapySDR::Stream *stream)
{
	std::lock_guard<std::mutex> lock(_device_mutex);

    SoapySDR_logf(SOAPY_SDR_INFO, "closeStream %x", stream);

	if (stream == _rx_stream.stream) {
		_rx_stream.opened = false;
		_rx_stream.stream = (SoapySDR::Stream *)((long)(0));
		ioctl((long)stream, CLOSE_STREAM_RX);
		close((long)stream);
	} else if (stream == _tx_stream.stream) {
		_tx_stream.opened = false;
		_tx_stream.stream = (SoapySDR::Stream *)((long)(0));
		ioctl((long)stream, CLOSE_STREAM_TX);
		close((long)stream);
	}
}

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
    return 1024 * 1024 / 2 / 4;
}

/*!
 * Activate a stream.
 * Call activate to prepare a stream before using read/write().
 * The implementation control switches or stimulate data flow.
 *
 * The timeNs is only valid when the flags have SOAPY_SDR_HAS_TIME.
 * The numElems count can be used to request a finite burst size.
 * The SOAPY_SDR_END_BURST flag can signal end on the finite burst.
 * Not all implementations will support the full range of options.
 * In this case, the implementation returns SOAPY_SDR_NOT_SUPPORTED.
 *
 * \param stream the opaque pointer to a stream handle
 * \param flags optional flag indicators about the stream
 * \param timeNs optional activation time in nanoseconds
 * \param numElems optional element count for burst control
 * \return 0 for success or error code on failure
 */
int Cariboulite::activateStream(SoapySDR::Stream *stream,
                                    const int flags,
                                    const long long timeNs,
                                    const size_t numElems)
{

    printf("activateStream flags %d timeNs %lld numElems %ld\n", flags, timeNs, numElems);

	/* TODO check what is going on */
	if (stream == _rx_stream.stream) {
		std::lock_guard<std::mutex> lock(_device_mutex);

		if(_current_mode==HACKRF_TRANSCEIVER_MODE_RX)
			return 0;

		if(_current_mode==HACKRF_TRANSCEIVER_MODE_TX) {
			/* 1) TODO wait if burst_end  == true */

			ioctl((long)stream, DEACTIVATE_STREAM_TX);
		}

		cariboulite_setup_stream(&radios, cariboulite_channel_s1g, cariboulite_channel_dir_rx);

		_current_mode = HACKRF_TRANSCEIVER_MODE_RX;

		/* 3) start RX */
	} else if (stream == _tx_stream.stream) {
		std::lock_guard<std::mutex> lock(_device_mutex);

		if(_current_mode==HACKRF_TRANSCEIVER_MODE_TX)
			return 0;

		if(_current_mode==HACKRF_TRANSCEIVER_MODE_RX) {
			ioctl((long)stream, DEACTIVATE_STREAM_RX);
		}

		cariboulite_setup_stream(&radios, cariboulite_channel_s1g, cariboulite_channel_dir_tx);

		_current_mode = HACKRF_TRANSCEIVER_MODE_TX;
	}

    cariboulite_activate_channel(&radios, cariboulite_channel_s1g, true);

	if (stream == _rx_stream.stream) {
		struct stream_config stream_config;

		stream_config.activation_time = timeNs;
		stream_config.flags = flags;
		stream_config.num_elements = numElems;

		/* TODO introduce ACIVATE_STREAM_RX/TX, DEACTIVATE_STREAM_RX/TX */
		/* introduire parametre _current_mode _stream_rx, or stream_tx */
		/* il faudra surement un dev_mutex */

		/* stocker la freq, le sampling rate, le gain etc et configurer au besoin */

		return ioctl((long)stream, ACTIVATE_STREAM_RX, (unsigned long)&stream_config);
	} else {
		return 0;
	}
}

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
	printf("deactivateStream\n");

	int res = cariboulite_deactivate_channel(&radios, cariboulite_channel_s1g, true);

	if (stream == _rx_stream.stream) {
		std::lock_guard<std::mutex> lock(_device_mutex);
		if(_current_mode==HACKRF_TRANSCEIVER_MODE_RX) {
			ioctl((long)stream, DEACTIVATE_STREAM_RX);
			_current_mode = HACKRF_TRANSCEIVER_MODE_OFF;
		}
	} else if (stream == _tx_stream.stream) {
		std::lock_guard<std::mutex> lock(_device_mutex);
		if(_current_mode==HACKRF_TRANSCEIVER_MODE_TX) {
			ioctl((long)stream, DEACTIVATE_STREAM_TX);
			_current_mode = HACKRF_TRANSCEIVER_MODE_OFF;
		}
	}

	return res;
}

/*!
 * Write elements to a stream for transmission.
 * This is a multi-channel call, and buffs should be an array of void *,
 * where each pointer will be filled with data for a different channel.
 *
 * **Client code compatibility:**
 * Client code relies on writeStream() for proper back-pressure.
 * The writeStream() implementation must enforce the timeout
 * such that the call blocks until space becomes available
 * or timeout expiration.
 *
 * \param stream the opaque pointer to a stream handle
 * \param buffs an array of void* buffers num chans in size
 * \param numElems the number of elements in each buffer
 * \param flags optional input flags and output flags
 * \param timeNs the buffer's timestamp in nanoseconds
 * \param timeoutUs the timeout in microseconds
 * \return the number of elements written per buffer or error
 */
int Cariboulite::writeStream(
            SoapySDR::Stream *stream,
            const void * const *buffs,
            const size_t numElems,
            int &flags,
            const long long timeNs,
            const long timeoutUs)
{
	int n = write((long) stream, (char*) buffs[0], numElems*sizeof(uint32_t)); 
	return n;
}

/*!
 * Read elements from a stream for reception.
 * This is a multi-channel call, and buffs should be an array of void *,
 * where each pointer will be filled with data from a different channel.
 *
 * **Client code compatibility:**
 * The readStream() call should be well defined at all times,
 * including prior to activation and after deactivation.
 * When inactive, readStream() should implement the timeout
 * specified by the caller and return SOAPY_SDR_TIMEOUT.
 *
 * \param stream the opaque pointer to a stream handle
 * \param buffs an array of void* buffers num chans in size
 * \param numElems the number of elements in each buffer
 * \param flags optional flag indicators about the result
 * \param timeNs the buffer's timestamp in nanoseconds
 * \param timeoutUs the timeout in microseconds
 * \return the number of elements read per buffer or error code
 */
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

    long ret = ioctl((long) stream, GET_METADATA, (unsigned long)&md);

	printf("readStream_ioctl %ld\n", ret);
	sample_complex_float *bam = (sample_complex_float*) buffs[0];

	if (ret == 0) {
		flags = 0;
		timeNs = md.timestamp;
		uint8_t temp_buf[numElems*4];
		
		//printf("readStream %lld\n", numElems);
		n = read((long) stream, temp_buf, numElems*sizeof(uint32_t)); 

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
