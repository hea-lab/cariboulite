#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define OUTPUT_FILENAME "test.m"

typedef struct
{
        float i;
        float q;
} sample_complex_float;


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

	SoapySDRRange *ranges = SoapySDRDevice_getFrequencyRange(sdr, SOAPY_SDR_RX, 0, &length);
	printf("Rx freq ranges: ");
	for (size_t i = 0; i < length; i++) printf("[%g Hz -> %g Hz], ", ranges[i].minimum, ranges[i].maximum);
	printf("\n");
	free(ranges);

	//setup a stream (complex floats)
	SoapySDRStream *rxStream;

	if (SoapySDRDevice_setupStream(sdr, &rxStream, SOAPY_SDR_RX, SOAPY_SDR_CF32, NULL, 0, NULL) != 0) {
		printf("setupStream fail: %s\n", SoapySDRDevice_lastError());
		SoapySDRDevice_unmake(sdr);
		return EXIT_FAILURE;
	}

	int ret = SoapySDRDevice_activateStream(sdr, rxStream, 0, 0, 0); /* CONTINUOUS STREAMING */

	sample_complex_float samples[4096];

	memset(samples, 0, sizeof(samples));

	int num_samples = sizeof(samples)/sizeof(sample_complex_float);

	for (int i = 0; i< 10; i++) {

#if 1
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
#endif

		sleep(5);
	}

	//shutdown the stream
	SoapySDRDevice_deactivateStream(sdr, rxStream, 0, 0); //stop streaming
	SoapySDRDevice_closeStream(sdr, rxStream);

	//cleanup device handle
	SoapySDRDevice_unmake(sdr);

	// write results to output file
	FILE * fid = fopen(OUTPUT_FILENAME,"w");
	fprintf(fid,"%% %s : auto-generated file\n", OUTPUT_FILENAME);
	fprintf(fid,"graphics_toolkit(\'gnuplot\')\n");
	fprintf(fid,"clear all\n");
	fprintf(fid,"close all\n");
	fprintf(fid,"num_samples = %u;\n", num_samples);

	fprintf(fid,"x = zeros(1,num_samples);\n");
	for (int i=0; i<num_samples; i++) {
		fprintf(fid,"x(%4u) = %12.8f + j*%12.8f;\n", i+1, samples[i].i, samples[i].q);
	}
	fprintf(fid,"t=[0:(num_samples-1)];\n");
	fprintf(fid,"figure;\n");
	fprintf(fid,"plot(t,real(x),t,imag(x));\n");
	fprintf(fid,"grid on;\n");

	fclose(fid);
	printf("results written to '%s'\n", OUTPUT_FILENAME);

	printf("Done\n");
	return EXIT_SUCCESS;
}
