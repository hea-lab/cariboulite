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

	int num_samples = sizeof(samples)/sizeof(sample_complex_int16);

	for (int i = 0; i< num_samples; i++) {
		samples[i].i = cos((float)(i)/fs*2*M_PI*freq) * 4095;
		samples[i].q = sin((float)(i)/fs*2*M_PI*freq) * 4095;

		if (samples[i].i < 0) 
			samples[i].i += 0x2000;

		if (samples[i].q < 0) 
			samples[i].q += 0x2000;
	}


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

	ranges = SoapySDRDevice_getFrequencyRange(sdr, SOAPY_SDR_RX, 0, &length);
	printf("Rx freq ranges: ");
	for (size_t i = 0; i < length; i++) printf("[%g Hz -> %g Hz], ", ranges[i].minimum, ranges[i].maximum);
	printf("\n");
	free(ranges);

	if (SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_TX, 0, fs) != 0) {
		printf("Tx setSampleRate fail: %s\n", SoapySDRDevice_lastError());
		SoapySDRDevice_unmake(sdr);
		return EXIT_FAILURE;
	}

	if (SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, 0, target_freq, NULL) != 0) {
		printf("Tx setFrequency fail: %s\n", SoapySDRDevice_lastError());
		SoapySDRDevice_unmake(sdr);
		return EXIT_FAILURE;
	}

	if (SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_RX, 0, fs) != 0) {
		printf("Rx setSampleRate fail: %s\n", SoapySDRDevice_lastError());
		SoapySDRDevice_unmake(sdr);
		return EXIT_FAILURE;
	}

	/* FIXME attention ca switch le mode */
	if (SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_RX, 0, target_freq, NULL) != 0) {
		printf("Rx setFrequency fail: %s\n", SoapySDRDevice_lastError());
		SoapySDRDevice_unmake(sdr);
		return EXIT_FAILURE;
	}

	/* TODO verify */

	//setup streams (complex floats)
	SoapySDRStream *txStream, *rxStream;

	if (SoapySDRDevice_setupStream(sdr, &txStream, SOAPY_SDR_TX, SOAPY_SDR_CF32, NULL, 0, NULL) != 0) {
		printf("setupStream fail: %s\n", SoapySDRDevice_lastError());
		SoapySDRDevice_unmake(sdr);
		return EXIT_FAILURE;
	}

	if (SoapySDRDevice_setupStream(sdr, &rxStream, SOAPY_SDR_RX, SOAPY_SDR_CF32, NULL, 0, NULL) != 0) {
		printf("setupStream fail: %s\n", SoapySDRDevice_lastError());
		SoapySDRDevice_unmake(sdr);
		return EXIT_FAILURE;
	}

	/* TODO en fonction de ce que l on active, on switche d un mode a l autre */
	int ret = SoapySDRDevice_activateStream(sdr, txStream, 0, 0, 0); /* CONTINUOUS STREAMING */

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

#if 0 /* activer ca dans un 2eme temps */

	/* TODO en fonction de ce que l on active, on switche d un mode a l autre */
	ret = SoapySDRDevice_activateStream(sdr, rxStream, 0, 0, 0); /* CONTINUOUS STREAMING */

	/* TODO refaire un rx */
	for (int i = 0; i< 10; i++) {

		uint8_t *temp;

		void *buffs[] = {samples}; //array of buffers
		int flags; //flags set by receive operation
		long long timeNs; //timestamp for receive buffer
		ret = SoapySDRDevice_readStream(sdr, rxStream, buffs, num_samples, &flags, &timeNs, 100000);
		printf("ret=%d, flags=%d, timeNs=%lld\n", ret, flags, timeNs);

		if (ret > 0) {
			for (size_t j = 0; j < num_samples; j++) {
				if (samples[j].i >= (int16_t)0x1000) samples[j].i -= (int16_t)0x2000;
				if (samples[j].q >= (int16_t)0x1000) samples[j].q -= (int16_t)0x2000;
				samples[j].i /= 4095.0;
				samples[j].q /= 4095.0;
				printf("I = %f Q = %f\n", samples[j].i, samples[j].q );
			}
		}

		sleep(5);
	}
#endif

	//shutdown the stream
	SoapySDRDevice_deactivateStream(sdr, txStream, 0, 0); //stop streaming
	SoapySDRDevice_closeStream(sdr, txStream);

	SoapySDRDevice_deactivateStream(sdr, rxStream, 0, 0); //stop streaming
	SoapySDRDevice_closeStream(sdr, rxStream);

	//cleanup device handle
	SoapySDRDevice_unmake(sdr);

	return EXIT_SUCCESS;
}
