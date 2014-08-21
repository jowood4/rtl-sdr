/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 * Copyright (C) 2012 by Kyle Keen <keenerd@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


/*
 * rtl_power: general purpose FFT integrator
 * -f low_freq:high_freq:max_bin_size
 * -i seconds
 * outputs CSV
 * time, low, high, step, db, db, db ...
 * db optional?  raw output might be better for noise correction
 * todo:
 *	threading
 *	randomized hopping
 *	noise correction
 *	continuous IIR
 *	general astronomy usefulness
 *	multiple dongles
 *	multiple FFT workers
 *	check edge cropping for off-by-one and rounding errors
 *	1.8MS/s for hiding xtal harmonics
 */

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#include "getopt/getopt.h"
#define usleep(x) Sleep(x/1000)
#ifdef _MSC_VER
#define round(x) (x > 0.0 ? floor(x + 0.5): ceil(x - 0.5))
#endif
#define _USE_MATH_DEFINES
#endif

#include <math.h>
#include <pthread.h>
#include <libusb.h>

#include "rtl-sdr.h"
#include "convenience/convenience.h"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))

#define DEFAULT_num_samplesGTH		(1 * 16384)
#define AUTO_GAIN			-100
#define BUFFER_DUMP			(1<<12)

#define MAXIMUM_RATE			2800000
#define MINIMUM_RATE			1000000

struct tuning_state
/* one per tuning range */
{
	int freq;  //Tuner frequency
	int rate;  //Sample Rate
	int num_samples;  //Number of samples
	int gain;  //AUTO_GAIN; // tenths of a dB
	
	int direct_sampling;
	int offset_tuning;
	double crop;
	int ppm_error;
	int custom_ppm;
	
	int bin_e;
	long *avg;  /* length == 2^bin_e */
	int samples;
	int downsample;
	int downsample_passes;  /* for the recursive filter */

};

/* 3000 is enough for 3GHz b/w worst case */
#define MAX_TUNES	3000
struct tuning_state tunes[MAX_TUNES];
static rtlsdr_dev_t *dev = NULL;

int tune_count = 0;

int boxcar = 1;
int comp_fir_size = 0;
int peak_hold = 0;

void usage(void)
{
	fprintf(stderr,
		"rtl_power, a simple FFT logger for RTL2832 based DVB-T receivers\n\n"
		"Use:\trtl_power -f freq_range [-options] [filename]\n"
		"\t-f lower:upper:bin_size [Hz]\n"
		"\t (bin size is a maximum, smaller more convenient bins\n"
		"\t  will be used.  valid range 1Hz - 2.8MHz)\n"
		"\t[-i integration_interval (default: 10 seconds)]\n"
		"\t (buggy if a full sweep takes longer than the interval)\n"
		"\t[-1 enables single-shot mode (default: off)]\n"
		"\t[-e exit_timer (default: off/0)]\n"
		//"\t[-s avg/iir smoothing (default: avg)]\n"
		//"\t[-t threads (default: 1)]\n"
		"\t[-d device_index (default: 0)]\n"
		"\t[-g tuner_gain (default: automatic)]\n"
		"\t[-p ppm_error (default: 0)]\n"
		"\tfilename (a '-' dumps samples to stdout)\n"
		"\t (omitting the filename also uses stdout)\n"
		"\n"
		"Experimental options:\n"
		"\t[-w window (default: rectangle)]\n"
		"\t (hamming, blackman, blackman-harris, hann-poisson, bartlett, youssef)\n"
		// kaiser
		"\t[-c crop_percent (default: 0%%, recommended: 20%%-50%%)]\n"
		"\t (discards data at the edges, 100%% discards everything)\n"
		"\t (has no effect for bins larger than 1MHz)\n"
		"\t[-F fir_size (default: disabled)]\n"
		"\t (enables low-leakage downsample filter,\n"
		"\t  fir_size can be 0 or 9.  0 has bad roll off,\n"
		"\t  try with '-c 50%%')\n"
		"\t[-P enables peak hold (default: off)]\n"
		"\t[-D direct_sampling_mode, 0 (default/off), 1 (I), 2 (Q), 3 (no-mod)]\n"
		"\t[-O enable offset tuning (default: off)]\n"
		"\n"
		"CSV FFT output columns:\n"
		"\tdate, time, Hz low, Hz high, Hz step, samples, dbm, dbm, ...\n\n"
		"Examples:\n"
		"\trtl_power -f 88M:108M:125k fm_stations.csv\n"
		"\t (creates 160 bins across the FM band,\n"
		"\t  individual stations should be visible)\n"
		"\trtl_power -f 100M:1G:1M -i 5m -1 survey.csv\n"
		"\t (a five minute low res scan of nearly everything)\n"
		"\trtl_power -f ... -i 15m -1 log.csv\n"
		"\t (integrate for 15 minutes and exit afterwards)\n"
		"\trtl_power -f ... -e 1h | gzip > log.csv.gz\n"
		"\t (collect data for one hour and compress it on the fly)\n\n"
		"Convert CSV to a waterfall graphic with:\n"
		"\t http://kmkeen.com/tmp/heatmap.py.txt \n");
	exit(1);
}

#ifdef _MSC_VER
double log2(double n)
{
	return log(n) / log(2.0);
}
#endif


void rms_power(int ts_index, uint8_t *buf, double *rms_pow_val, double *rms_pow_dc_val)
/* for bins between 1MHz and 2MHz */
{
	struct tuning_state *ts = &tunes[ts_index];

	int i, s1, s2;
	//uint8_t *buf = ts->buf8;
	int num_samples = ts->num_samples;
	double rms_sum, dc_sum, s1_2, s2_2;
	double rms, dc;

	//for (i=0; i<10; i++) {
	//	fprintf(file, "%i\n", buf[i]-127);
	//}

	//p = t = 0L;
	rms_sum = 0;
	dc_sum = 0;
	//for (i=0; i<10; i=i+2) {
	for (i=0; i<num_samples; i=i+2) {

		s1 = (int)buf[i] - 127;
		s2 = (int)buf[i+1] - 127;
		s1_2 = s1 * s1;
		s2_2 = s2 * s2;

		dc_sum += sqrt(s1_2 + s2_2);
		rms_sum += s1_2 + s2_2;
		//fprintf(file, "%.2f\n", dc_sum);
		//fprintf(file, "%.2f\n", rms_sum);

	}

	rms = sqrt(rms_sum/ (num_samples/2));
	dc = dc_sum / (num_samples/2);

	*rms_pow_val = 20*log10(rms/181);
	*rms_pow_dc_val = 20*log10((rms-dc)/181);  //256/sqrt(2)
}

void read_data(int index, uint8_t *buf8)
{
	int n_read;
	struct tuning_state *ts;
	ts = &tunes[index];

	verbose_reset_buffer(dev);
	
	//Get data
	rtlsdr_read_sync(dev, buf8, ts->num_samples, &n_read);

	if (n_read != ts->num_samples) {
		fprintf(stderr, "Error: dropped samples.\n");}

}

void initialize_tuner_values(int index)
{
	struct tuning_state *ts;
	int j;

	ts = &tunes[index];

	//Tuner Properties
	ts->freq = 1e9;
	ts->rate = 2.048e6;
	ts->num_samples = DEFAULT_num_samplesGTH;
	ts->gain = AUTO_GAIN;  //254
	ts->direct_sampling = 0;
	ts->offset_tuning = 0;
	ts->crop = 0.0;
	ts->ppm_error = 0;
	ts->custom_ppm = 0;
	
	//FFT Properties
	ts->bin_e = 10;
	ts->samples = 0;
	ts->crop = 0;
	ts->downsample = 1;//downsample;
	ts->downsample_passes = 0;//downsample_passes;
	
	ts->avg = (long*)malloc((1<<ts->bin_e) * sizeof(long));
	if (!ts->avg) {
		fprintf(stderr, "Error: malloc.\n");
		exit(1);
	}
	for (j=0; j<(1<<ts->bin_e); j++) {
		ts->avg[j] = 0L;
	}
}

void find_and_open_dev(void)
{
	int dev_index, r = 0;

	dev_index = verbose_device_search("0");

	if (dev_index < 0) {
		exit(1);
	}

	r = rtlsdr_open(&dev, (uint32_t)dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
		exit(1);
	}
}

void close_dev(void)
{
	rtlsdr_close(dev);
}

void set_tuner(int index)
{
	uint8_t dump[BUFFER_DUMP];
	int n_read;
	struct tuning_state *ts;
	ts = &tunes[index];
	
	if (ts->direct_sampling) {
		verbose_direct_sampling(dev, ts->direct_sampling);
	}

	if (ts->offset_tuning) {
		verbose_offset_tuning(dev);
	}

	/* Set the tuner gain */
	if (ts->gain == AUTO_GAIN) {
		verbose_auto_gain(dev);
	} else {
		ts->gain = nearest_gain(dev, ts->gain);
		verbose_gain_set(dev, ts->gain);
	}

	if (!ts->custom_ppm) {
		verbose_ppm_eeprom(dev, &ts->ppm_error);
	}
	verbose_ppm_set(dev, ts->ppm_error);

	/* Reset endpoint before we start reading from it (mandatory) */
	verbose_reset_buffer(dev);
	
	/* actually do stuff */
	rtlsdr_set_sample_rate(dev, (uint32_t)ts->rate);
	
	rtlsdr_set_center_freq(dev, (uint32_t)ts->freq);
	/* wait for settling and flush buffer */
	usleep(5000);
	rtlsdr_read_sync(dev, &dump, BUFFER_DUMP, &n_read);
	if (n_read != BUFFER_DUMP) {
		fprintf(stderr, "Error: bad retune.\n");}

}

void set_value(int index, char param, double value)
{
	struct tuning_state *ts;
	ts = &tunes[index];

	switch (param) {
	case 'f': // lower:upper:bin_size
		ts->freq = (int)value;
		break;
	case 'r':
		ts->rate = (int)value;
		break;
	case 'b': 
		ts->num_samples = pow(2,(int)value);
		break;
	case 'g': 
		ts->gain = (int)value;
		break;
	default:
		break;
	}
}

uint32_t get_value(char param)
{
	uint32_t value = 0;
	switch (param) {
	case 'f': // lower:upper:bin_size
		value = rtlsdr_get_center_freq(dev);
		break;
	case 'r':
		value = rtlsdr_get_sample_rate(dev);
		break;
	case 'g': 
		value = rtlsdr_get_tuner_gain(dev);
		break;
	default:
		break;
	}
	return value;
}

int main(int argc, char **argv)
{
	char *filename = NULL;
	int opt = 0;
	int bw2, bin_count;
	double rms_pow_val, rms_pow_dc_val;
	int r = 0;
	int index = 0;
	FILE *file;
	uint8_t *buf8;

	time_t time_now;
	char t_str[50];
	struct tm *cal_time;
	
	struct tuning_state *ts;

	file = stdout;
	
	ts = &tunes[index];

	//Set initial state for tuner
	initialize_tuner_values(index);

	//Change tuner values based on input options
	while ((opt = getopt(argc, argv, "f:r:b:g")) != -1) {
		switch (opt) {
		case 'f': // lower:upper:bin_size
			//ts->freq = atof(optarg);
			set_value(index, 'f', atof(optarg));
			break;
		case 'r':
			//ts->rate = atof(optarg);
			set_value(index, 'r', atof(optarg));
			break;
		case 'b': 
			//ts->num_samples = pow(2,atoi(optarg));
			set_value(index, 'b', atoi(optarg));
			break;
		case 'g':
			set_value(index, 'g', atof(optarg)*10);
		default:
			usage();
			break;
		}
	}

	if (argc <= optind) {
		filename = "-";
	} else {
		filename = argv[optind];
	}
	
	find_and_open_dev();

	//Configure Tuner settings, if necessary
	set_tuner(index);

	//Get data from tuner
	buf8 = (uint8_t*)malloc(ts->num_samples * sizeof(uint8_t));
	if (!buf8) {
		fprintf(stderr, "Error: malloc.\n");
		exit(1);
	}
	read_data(index, buf8);
	
	/* rms */
	rms_power(index, buf8, &rms_pow_val, &rms_pow_dc_val);
	
	//Print Time
	time_now = time(NULL);
	cal_time = localtime(&time_now);
	strftime(t_str, 50, "%Y-%m-%d, %H:%M:%S", cal_time);
	fprintf(file, "%s\n", t_str);
	
	bin_count = (int)((double)ts->num_samples * (1.0 - ts->crop));
	bw2 = (int)(((double)ts->rate * (double)bin_count) / (ts->num_samples * 2));
		
	//Print all info
	fprintf(stderr, "Number of frequency hops: %i\n", tune_count);
	fprintf(stderr, "Dongle bandwidth: %iHz\n", ts->rate);
	fprintf(stderr, "Downsampling by: %ix\n", ts->downsample);
	fprintf(stderr, "Cropping by: %0.2f%%\n", ts->crop*100);
	fprintf(stderr, "Buffer size: %i bytes (%0.2fms)\n", ts->num_samples, 1000 * 0.5 * (float)ts->num_samples / (float)ts->rate);
	fprintf(file, "Lowest Frequency is %.2f MHz\n", (double)(ts->freq - bw2)/1e6);
	fprintf(file, "Highest Frequency is %.2f MHz\n", (double)(ts->freq + bw2)/1e6);
	fprintf(file, "FFT Bin Size would be %.2f kHz\n", (double)(ts->rate / ts->num_samples)/1e3);
	fprintf(file, "Number of Samples is %i\n", ts->num_samples);
	fprintf(file, "RMS Voltage with DC is %.2f dBFS\n", rms_pow_val);
	fprintf(file, "RMS Voltage without DC is %.2f dBFS\n", rms_pow_dc_val);
	
	fflush(file);

	free(buf8);
		
	close_dev();

	return r >= 0 ? r : -r;
}
