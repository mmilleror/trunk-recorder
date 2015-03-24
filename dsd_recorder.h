#ifndef DSD_RECORDER_H
#define DSD_RECORDER_H

#include <cstdio>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <fstream>


#include <boost/shared_ptr.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/log/trivial.hpp>

#include <gnuradio/io_signature.h>
#include <gnuradio/hier_block2.h>
#include <gnuradio/blocks/multiply_const_ff.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/filter/fir_filter_ccf.h>
#include <gnuradio/filter/fir_filter_fff.h>
#include <gnuradio/filter/freq_xlating_fir_filter_ccf.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/filter/rational_resampler_base_ccc.h>
#include <gnuradio/analog/quadrature_demod_cf.h>
#include <gnuradio/analog/quadrature_demod_cf.h>
#include "dsd_block_ff.h"
#include <gnuradio/analog/sig_source_f.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/blocks/multiply_cc.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/filter/rational_resampler_base_ccf.h>
#include <gnuradio/filter/rational_resampler_base_fff.h>
#include <gnuradio/block.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/copy.h>

#include <op25_repeater/gardner_costas_cc.h>
#include <gnuradio/filter/pfb_arb_resampler_ccf.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/analog/feedforward_agc_cc.h>

#include <gnuradio/digital/diff_phasor_cc.h>

#include <gnuradio/blocks/complex_to_arg.h>

//Valve
//#include <gnuradio/blocks/null_sink.h>
//#include <gnuradio/blocks/null_source.h>
#include <gnuradio/blocks/head.h>
//#include <gnuradio/kludge_copy.h>
//#include <smartnet_wavsink.h>
#include "nonstop_wavfile_sink.h"

//#include <gnuradio/blocks/wavfile_sink.h>
#include <gnuradio/blocks/file_sink.h>
//#include <blocks/wavfile_sink.h>
#include "recorder.h"
#include "smartnet.h"


class dsd_recorder;

typedef boost::shared_ptr<dsd_recorder> dsd_recorder_sptr;

dsd_recorder_sptr make_dsd_recorder(float f, float c, long s, long t, int n);

class dsd_recorder : public gr::hier_block2 , public Recorder
{
	friend dsd_recorder_sptr make_dsd_recorder(float f, float c, long s, long t, int n);
protected:
	dsd_recorder(double f, double c, long s, long t, int n);

public:
	~dsd_recorder();
	void tune_offset(double f);
	void activate( long t, double f, int n);

	void deactivate();
	double get_freq();
	long get_talkgroup();
	bool is_active();
	int lastupdate();
	long elapsed();
	void close();
	void mute();
	void unmute();
	char *get_filename();
	//void forecast(int noutput_items, gr_vector_int &ninput_items_required);
	static bool logging;
private:
	double center, freq;
	bool recording;
	long talkgroup;
	long samp_rate;
	time_t timestamp;
	time_t starttime;
	char filename[160];
	char status_filename[160];
	char raw_filename[160];
	int num;

	bool iam_logging;
	bool active;
	std::vector<float> lpf_coeffs;
	std::vector<float> arb_taps;
	std::vector<float> sym_taps;

	/* GR blocks */
	gr::filter::fir_filter_ccf::sptr lpf;
	gr::filter::fir_filter_fff::sptr lpf_second;
	gr::filter::fir_filter_fff::sptr sym_filter;
	gr::filter::freq_xlating_fir_filter_ccf::sptr prefilter;
	gr::analog::sig_source_c::sptr offset_sig;

	gr::blocks::multiply_cc::sptr mixer;
	gr::blocks::file_sink::sptr fs;
	gr::blocks::multiply_const_ff::sptr quiet;
	gr::blocks::multiply_const_ff::sptr levels;


	gr::filter::rational_resampler_base_ccf::sptr downsample_sig;
	gr::filter::rational_resampler_base_fff::sptr upsample_audio;
	//gr::analog::quadrature_demod_cf::sptr demod;
	gr::analog::quadrature_demod_cf::sptr demod;
	dsd_block_ff_sptr dsd;
	gr::blocks::nonstop_wavfile_sink::sptr wav_sink;
	gr::blocks::file_sink::sptr raw_sink;
	gr::blocks::null_sink::sptr null_sink;
	gr::blocks::head::sptr head_source;
	gr::blocks::copy::sptr valve;
	//gr_kludge_copy_sptr copier;


	gr::analog::sig_source_c::sptr lo;

gr::digital::diff_phasor_cc::sptr diffdec;


	gr::filter::pfb_arb_resampler_ccf::sptr arb_resampler;
	gr::analog::feedforward_agc_cc::sptr agc;
	gr::op25_repeater::gardner_costas_cc::sptr costas_clock;
	gr::blocks::multiply_const_ff::sptr multiplier;
	gr::blocks::multiply_const_ff::sptr rescale;
	gr::blocks::multiply_const_ff::sptr baseband_amp;
	gr::blocks::complex_to_arg::sptr to_float;

};


#endif
