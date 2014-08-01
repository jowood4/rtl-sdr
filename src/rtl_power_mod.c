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

#define DEFAULT_BUF_LENGTH		(1 * 16384)
#define AUTO_GAIN			-100
#define BUFFER_DUMP			(1<<12)

#define MAXIMUM_RATE			2800000
#define MINIMUM_RATE			1000000

static volatile int do_exit = 0;
static rtlsdr_dev_t *dev = NULL;
FILE *file;

int16_t* Sinewave;
double* power_table;
int N_WAVE, LOG2_N_WAVE;
int next_power;
int16_t *fft_buf;
int *window_coefs;

struct tuning_state
/* one per tuning range */
{
	int freq;
	int rate;
	int bin_e;
	long *avg;  /* length == 2^bin_e */
	int samples;
	int downsample;
	int downsample_passes;  /* for the recursive filter */
	double crop;
	//pthread_rwlock_t avg_lock;
	//pthread_mutex_t avg_mutex;
	/* having the iq buffer here is wasteful, but will avoid contention */
	uint8_t *buf8;
	int buf_len;
	//int *comp_fir;
	//pthread_rwlock_t buf_lock;
	//pthread_mutex_t buf_mutex;
};

/* 3000 is enough for 3GHz b/w worst case */
#define MAX_TUNES	3000
struct tuning_state tunes[MAX_TUNES];
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

void multi_bail(void)
{
	if (do_exit == 1)
	{
		fprintf(stderr, "Signal caught, finishing scan pass.\n");
	}
	if (do_exit >= 2)
	{
		fprintf(stderr, "Signal caught, aborting immediately.\n");
	}
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		do_exit++;
		multi_bail();
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	do_exit++;
	multi_bail();
}
#endif

/* more cond dumbness */
#define safe_cond_signal(n, m) pthread_mutex_lock(m); pthread_cond_signal(n); pthread_mutex_unlock(m)
#define safe_cond_wait(n, m) pthread_mutex_lock(m); pthread_cond_wait(n, m); pthread_mutex_unlock(m)

/* {length, coef, coef, coef}  and scaled by 2^15
   for now, only length 9, optimal way to get +85% bandwidth */
#define CIC_TABLE_MAX 10
int cic_9_tables[][10] = {
	{0,},
	{9, -156,  -97, 2798, -15489, 61019, -15489, 2798,  -97, -156},
	{9, -128, -568, 5593, -24125, 74126, -24125, 5593, -568, -128},
	{9, -129, -639, 6187, -26281, 77511, -26281, 6187, -639, -129},
	{9, -122, -612, 6082, -26353, 77818, -26353, 6082, -612, -122},
	{9, -120, -602, 6015, -26269, 77757, -26269, 6015, -602, -120},
	{9, -120, -582, 5951, -26128, 77542, -26128, 5951, -582, -120},
	{9, -119, -580, 5931, -26094, 77505, -26094, 5931, -580, -119},
	{9, -119, -578, 5921, -26077, 77484, -26077, 5921, -578, -119},
	{9, -119, -577, 5917, -26067, 77473, -26067, 5917, -577, -119},
	{9, -199, -362, 5303, -25505, 77489, -25505, 5303, -362, -199},
};

#ifdef _MSC_VER
double log2(double n)
{
	return log(n) / log(2.0);
}
#endif

/* FFT based on fix_fft.c by Roberts, Slaney and Bouras
   http://www.jjj.de/fft/fftpage.html
   16 bit ints for everything
   -32768..+32768 maps to -1.0..+1.0
*/

void sine_table(int size)
{
	int i;
	double d;
	LOG2_N_WAVE = size;
	N_WAVE = 1 << LOG2_N_WAVE;
	Sinewave = malloc(sizeof(int16_t) * N_WAVE*3/4);
	power_table = malloc(sizeof(double) * N_WAVE);
	for (i=0; i<N_WAVE*3/4; i++)
	{
		d = (double)i * 2.0 * M_PI / N_WAVE;
		Sinewave[i] = (int)round(32767*sin(d));
		//printf("%i\n", Sinewave[i]);
	}
}

inline int16_t FIX_MPY(int16_t a, int16_t b)
/* fixed point multiply and scale */
{
	int c = ((int)a * (int)b) >> 14;
	b = c & 0x01;
	return (c >> 1) + b;
}

void rms_power(struct tuning_state *ts)
/* for bins between 1MHz and 2MHz */
{
	int i, s;
	uint8_t *buf = ts->buf8;
	int buf_len = ts->buf_len;
	long p, t;
	int ln, lp;
	double dc, err;

	p = t = 0L;
	for (i=0; i<buf_len; i++) {
		s = (int)buf[i] - 127;
		t += (long)s;
		p += (long)(s * s);
	}
	/* correct for dc offset in squares */
	dc = (double)t / (double)buf_len;
	err = t * 2 * dc - dc * dc * buf_len;
	p -= (long)round(err);

	if (!peak_hold) {
		ts->avg[0] += p;
	} else {
		ts->avg[0] = MAX(ts->avg[0], p);
	}
	ts->samples += 1;
}

void frequency_range(char *arg)
/* flesh out the tunes[] for scanning */
// do we want the fewest ranges (easy) or the fewest bins (harder)?
{
	char *start, *stop, *step;
	int i, j, buf_len;
	//double bin_size;
	struct tuning_state *ts;

	tune_count = 1;

	/* build the array */
	//for (i=0; i<tune_count; i++) {

		i = 0;
		ts = &tunes[i];
		ts->freq = 1e9;//lower + i*bw_seen + bw_seen/2;
		ts->rate = 2.56e6;//bw_used;
		ts->bin_e = 10;//bin_e;
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

		buf_len = 2 * (1<<ts->bin_e) * ts->downsample;
		if (buf_len < DEFAULT_BUF_LENGTH) {
			buf_len = DEFAULT_BUF_LENGTH;
		}

		ts->buf8 = (uint8_t*)malloc(buf_len * sizeof(uint8_t));
		if (!ts->buf8) {
			fprintf(stderr, "Error: malloc.\n");
			exit(1);
		}
		ts->buf_len = buf_len;

	//}
	/* report */
	fprintf(stderr, "Number of frequency hops: %i\n", tune_count);
	fprintf(stderr, "Dongle bandwidth: %iHz\n", ts->rate);
	fprintf(stderr, "Downsampling by: %ix\n", ts->downsample);
	fprintf(stderr, "Cropping by: %0.2f%%\n", ts->crop*100);
	fprintf(stderr, "Total FFT bins: %i\n", tune_count * (1<<ts->bin_e));
	fprintf(stderr, "Logged FFT bins: %i\n", \
	  (int)((double)(tune_count * (1<<ts->bin_e)) * (1.0-ts->crop)));
	//fprintf(stderr, "FFT bin size: %0.2fHz\n", bin_size);
	fprintf(stderr, "Buffer size: %i bytes (%0.2fms)\n", buf_len, 1000 * 0.5 * (float)buf_len / (float)ts->rate);
}

void retune(rtlsdr_dev_t *d, int freq)
{
	uint8_t dump[BUFFER_DUMP];
	int n_read;
	rtlsdr_set_center_freq(d, (uint32_t)freq);
	/* wait for settling and flush buffer */
	usleep(5000);
	rtlsdr_read_sync(d, &dump, BUFFER_DUMP, &n_read);
	if (n_read != BUFFER_DUMP) {
		fprintf(stderr, "Error: bad retune.\n");}
}

void fifth_order(int16_t *data, int length)
/* for half of interleaved data */
{
	int i;
	int a, b, c, d, e, f;
	a = data[0];
	b = data[2];
	c = data[4];
	d = data[6];
	e = data[8];
	f = data[10];
	/* a downsample should improve resolution, so don't fully shift */
	/* ease in instead of being stateful */
	data[0] = ((a+b)*10 + (c+d)*5 + d + f) >> 4;
	data[2] = ((b+c)*10 + (a+d)*5 + e + f) >> 4;
	data[4] = (a + (b+e)*5 + (c+d)*10 + f) >> 4;
	for (i=12; i<length; i+=4) {
		a = c;
		b = d;
		c = e;
		d = f;
		e = data[i-2];
		f = data[i];
		data[i/2] = (a + (b+e)*5 + (c+d)*10 + f) >> 4;
	}
}

void remove_dc(int16_t *data, int length)
/* works on interleaved data */
{
	int i;
	int16_t ave;
	long sum = 0L;
	for (i=0; i < length; i+=2) {
		sum += data[i];
	}
	ave = (int16_t)(sum / (long)(length));
	if (ave == 0) {
		return;}
	for (i=0; i < length; i+=2) {
		data[i] -= ave;
	}
}

void generic_fir(int16_t *data, int length, int *fir)
/* Okay, not at all generic.  Assumes length 9, fix that eventually. */
{
	int d, f, temp, sum;
	int hist[9] = {0,};
	/* cheat on the beginning, let it go unfiltered */
	for (d=0; d<18; d+=2) {
		hist[d/2] = data[d];
	}
	for (d=18; d<length; d+=2) {
		temp = data[d];
		sum = 0;
		sum += (hist[0] + hist[8]) * fir[1];
		sum += (hist[1] + hist[7]) * fir[2];
		sum += (hist[2] + hist[6]) * fir[3];
		sum += (hist[3] + hist[5]) * fir[4];
		sum +=            hist[4]  * fir[5];
		data[d] = (int16_t)(sum >> 15) ;
		hist[0] = hist[1];
		hist[1] = hist[2];
		hist[2] = hist[3];
		hist[3] = hist[4];
		hist[4] = hist[5];
		hist[5] = hist[6];
		hist[6] = hist[7];
		hist[7] = hist[8];
		hist[8] = temp;
	}
}

void downsample_iq(int16_t *data, int length)
{
	fifth_order(data, length);
	//remove_dc(data, length);
	fifth_order(data+1, length-1);
	//remove_dc(data+1, length-1);
}

long real_conj(int16_t real, int16_t imag)
/* real(n * conj(n)) */
{
	return ((long)real*(long)real + (long)imag*(long)imag);
}

void scanner(void)
{
	int i, j, j2, f, n_read, offset, bin_e, bin_len, buf_len, ds, ds_p;
	int32_t w;
	struct tuning_state *ts;
	bin_e = tunes[0].bin_e;
	bin_len = 1 << bin_e;
	buf_len = tunes[0].buf_len;
	for (i=0; i<tune_count; i++) {
		if (do_exit >= 2)
			{return;}
		ts = &tunes[i];
		f = (int)rtlsdr_get_center_freq(dev);
		if (f != ts->freq) {
			retune(dev, ts->freq);}
		rtlsdr_read_sync(dev, ts->buf8, buf_len, &n_read);
		if (n_read != buf_len) {
			fprintf(stderr, "Error: dropped samples.\n");}
		/* rms */
		if (bin_len == 1) {
			rms_power(ts);
			continue;
		}
		/* prep for fft */
		for (j=0; j<buf_len; j++) {
			fft_buf[j] = (int16_t)ts->buf8[j] - 127;
		}
		ds = ts->downsample;
		ds_p = ts->downsample_passes;
		if (boxcar && ds > 1) {
			j=2, j2=0;
			while (j < buf_len) {
				fft_buf[j2]   += fft_buf[j];
				fft_buf[j2+1] += fft_buf[j+1];
				fft_buf[j] = 0;
				fft_buf[j+1] = 0;
				j += 2;
				if (j % (ds*2) == 0) {
					j2 += 2;}
			}
		} else if (ds_p) {  /* recursive */
			for (j=0; j < ds_p; j++) {
				downsample_iq(fft_buf, buf_len >> j);
			}
			/* droop compensation */
			if (comp_fir_size == 9 && ds_p <= CIC_TABLE_MAX) {
				generic_fir(fft_buf, buf_len >> j, cic_9_tables[ds_p]);
				generic_fir(fft_buf+1, (buf_len >> j)-1, cic_9_tables[ds_p]);
			}
		}
		remove_dc(fft_buf, buf_len / ds);
		remove_dc(fft_buf+1, (buf_len / ds) - 1);
		/* window function and fft */
		for (offset=0; offset<(buf_len/ds); offset+=(2*bin_len)) {
			// todo, let rect skip this
			for (j=0; j<bin_len; j++) {
				w =  (int32_t)fft_buf[offset+j*2];
				w *= (int32_t)(window_coefs[j]);
				//w /= (int32_t)(ds);
				fft_buf[offset+j*2]   = (int16_t)w;
				w =  (int32_t)fft_buf[offset+j*2+1];
				w *= (int32_t)(window_coefs[j]);
				//w /= (int32_t)(ds);
				fft_buf[offset+j*2+1] = (int16_t)w;
			}
			//fix_fft(fft_buf+offset, bin_e);
			if (!peak_hold) {
				for (j=0; j<bin_len; j++) {
					ts->avg[j] += real_conj(fft_buf[offset+j*2], fft_buf[offset+j*2+1]);
				}
			} else {
				for (j=0; j<bin_len; j++) {
					ts->avg[j] = MAX(real_conj(fft_buf[offset+j*2], fft_buf[offset+j*2+1]), ts->avg[j]);
				}
			}
			ts->samples += ds;
		}
	}
}

void csv_dbm(struct tuning_state *ts)
{
	int i, len, ds, i1, i2, bw2, bin_count;
	long tmp;
	double dbm;
	len = 1 << ts->bin_e;
	ds = ts->downsample;
	/* fix FFT stuff quirks */
	if (ts->bin_e > 0) {
		/* nuke DC component (not effective for all windows) */
		ts->avg[0] = ts->avg[1];
		/* FFT is translated by 180 degrees */
		for (i=0; i<len/2; i++) {
			tmp = ts->avg[i];
			ts->avg[i] = ts->avg[i+len/2];
			ts->avg[i+len/2] = tmp;
		}
	}
	/* Hz low, Hz high, Hz step, samples, dbm, dbm, ... */
	bin_count = (int)((double)len * (1.0 - ts->crop));
	bw2 = (int)(((double)ts->rate * (double)bin_count) / (len * 2 * ds));
	fprintf(file, "%i, %i, %.2f, %i, ", ts->freq - bw2, ts->freq + bw2,
		(double)ts->rate / (double)(len*ds), ts->samples);
	// something seems off with the dbm math
	i1 = 0 + (int)((double)len * ts->crop * 0.5);
	i2 = (len-1) - (int)((double)len * ts->crop * 0.5);
	for (i=i1; i<=i2; i++) {
		dbm  = (double)ts->avg[i];
		dbm /= (double)ts->rate;
		dbm /= (double)ts->samples;
		dbm  = 10 * log10(dbm);
		fprintf(file, "%.2f, ", dbm);
	}
	dbm = (double)ts->avg[i2] / ((double)ts->rate * (double)ts->samples);
	if (ts->bin_e == 0) {
		dbm = ((double)ts->avg[0] / \
		((double)ts->rate * (double)ts->samples));}
	dbm  = 10 * log10(dbm);
	fprintf(file, "%.2f\n", dbm);
	for (i=0; i<len; i++) {
		ts->avg[i] = 0L;
	}
	ts->samples = 0;
}

int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	char *filename = NULL;
	int i, length, n_read, r, opt, wb_mode = 0;
	int f_set = 0;
	int gain = AUTO_GAIN; // tenths of a dB
	uint8_t *buffer;
	int dev_index = 0;
	int dev_given = 0;
	int ppm_error = 0;
	int custom_ppm = 0;
	int interval = 10;
	int fft_threads = 1;
	int smoothing = 0;
	int single = 0;
	int direct_sampling = 0;
	int offset_tuning = 0;
	double crop = 0.0;
	char *freq_optarg;
	time_t next_tick;
	time_t time_now;
	time_t exit_time = 0;
	char t_str[50];
	struct tm *cal_time;
	//double (*window_fn)(int, int) = rectangle;
	freq_optarg = "";

	frequency_range(freq_optarg);

	if (tune_count == 0) {
		usage();}

	if (argc <= optind) {
		filename = "-";
	} else {
		filename = argv[optind];
	}

	if (interval < 1) {
		interval = 1;}
	fprintf(stderr, "Reporting every %i seconds\n", interval);

	//Find device if not specified
	if (!dev_given) {
		dev_index = verbose_device_search("0");
	}

	if (dev_index < 0) {
		exit(1);
	}

	r = rtlsdr_open(&dev, (uint32_t)dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
		exit(1);
	}

	rtlsdr_close(dev);

	return r >= 0 ? r : -r;
}

// vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab