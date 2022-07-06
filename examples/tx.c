#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
# define M_PI (3.14159265358979323846)
#endif

#define MTU 5000 

typedef struct {
        int16_t i;
        int16_t q;
} sample_complex_int16;

sample_complex_int16 samples[MTU];

int fs = 4e6; /* 4Msps */
int freq = 4e4; /* 40khz */
double target_freq = 871e6; /* 871 Mhz */

int main(void)
{
	size_t length;

	SoapySDRKwargs args = {};
	SoapySDRKwargs_set(&args, "driver", "MySDR");

	SoapySDRDevice *sdr = SoapySDRDevice_make(&args);
	SoapySDRKwargs_clear(&args);

	if (sdr == NULL) {
		printf("SoapySDRDevice_make fail: %s\n", SoapySDRDevice_lastError());
		return EXIT_FAILURE;
	}

	SoapySDRRange *ranges = SoapySDRDevice_getFrequencyRange(sdr, SOAPY_SDR_TX, 0, &length);
	printf("Tx freq ranges: ");
	for (size_t i = 0; i < length; i++) printf("[%g Hz -> %g Hz], ", ranges[i].minimum, ranges[i].maximum);
	printf("\n");
	free(ranges);


	if (SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_TX, 0, fs) != 0) {
		printf("setSampleRate fail: %s\n", SoapySDRDevice_lastError());
		SoapySDRDevice_unmake(sdr);
		return EXIT_FAILURE;
	}

	if (SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, 0, target_freq, NULL) != 0) {
		printf("setFrequency fail: %s\n", SoapySDRDevice_lastError());
		SoapySDRDevice_unmake(sdr);
		return EXIT_FAILURE;
	}

	//setup a stream (complex floats)
	SoapySDRStream *txStream;
	if (SoapySDRDevice_setupStream(sdr, &txStream, SOAPY_SDR_TX, SOAPY_SDR_CF32, NULL, 0, NULL) != 0) {
		printf("setupStream fail: %s\n", SoapySDRDevice_lastError());
		SoapySDRDevice_unmake(sdr);
		return EXIT_FAILURE;
	}

	int ret = SoapySDRDevice_activateStream(sdr, txStream, 0, 0, 0); /* CONTINUOUS STREAMING */

	int num_samples = sizeof(samples)/sizeof(sample_complex_int16);

	for (int i = 0; i< num_samples; i++) {
		samples[i].i = cos((float)(i)/fs*2*M_PI*freq) * 4095;
		samples[i].q = sin((float)(i)/fs*2*M_PI*freq) * 4095;

		if (samples[i].i < 0) 
		       samples[i].i += 0x2000;

		if (samples[i].q < 0) 
		       samples[i].q += 0x2000;
	}

	for (int i = 0; i < 5; i++) {

		size_t remaining_samples = num_samples;

		int j;
		int flags; //flags set by receive operation

		for (j = 0; j< num_samples/MTU; j++) {

			const void *buffs[] = {&samples[j*MTU]}; //array of buffers
			ret = SoapySDRDevice_writeStream(sdr, txStream, buffs, MTU, &flags, 0, 100000);
			remaining_samples -= MTU;
		}

		if (remaining_samples > 0) {
			const void *buffs[] = {&samples[j*MTU]}; //array of buffers
			ret = SoapySDRDevice_writeStream(sdr, txStream, buffs, remaining_samples, &flags, 0, 100000);
			remaining_samples = 0;
		}

		printf("remaining_samples %d %d %ld\n", i, j, remaining_samples);
	}

	//shutdown the stream
	SoapySDRDevice_deactivateStream(sdr, txStream, 0, 0); //stop streaming
	SoapySDRDevice_closeStream(sdr, txStream);

	//cleanup device handle
	SoapySDRDevice_unmake(sdr);

	return EXIT_SUCCESS;
}