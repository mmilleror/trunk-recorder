
#include "p25_recorder.h"
#include <boost/log/trivial.hpp>


p25_recorder_sptr make_p25_recorder(Source *src, bool qpsk)
{
	return gnuradio::get_initial_sptr(new p25_recorder(src, qpsk));
}

p25_recorder::p25_recorder(Source *src, bool qpsk)
	: gr::hier_block2 ("p25_recorder",
	                   gr::io_signature::make  (1, 1, sizeof(gr_complex)),
	                   gr::io_signature::make  (0, 0, sizeof(float)))
{
    source = src;
	freq = source->get_center();
	center = source->get_center();
	long samp_rate = source->get_rate();
    qpsk_mod = qpsk;
	talkgroup = 0;
	long capture_rate = samp_rate;

	num = 0;
	active = false;

	float offset = freq - center;



    float symbol_rate = 4800;
	double samples_per_symbol = 10;
	double system_channel_rate = symbol_rate * samples_per_symbol;
	float symbol_deviation = 600.0;


	std::vector<float> sym_taps;
	const double pi = M_PI; //boost::math::constants::pi<double>();

	timestamp = time(NULL);
	starttime = time(NULL);

        double input_rate = capture_rate;
        
        float if_rate = 48000; //24000;
        float gain_mu = 0.025;
        float costas_alpha = 0.04;
        float bb_gain = 1.0;

       	baseband_amp = gr::blocks::multiply_const_ff::make(bb_gain);



 	float xlate_bandwidth = 14000; //14000; //24260.0


  
        
	valve = gr::blocks::copy::make(sizeof(gr_complex));
	valve->set_enabled(false);
        
        
            lpf_coeffs = gr::filter::firdes::low_pass(1.0, input_rate, xlate_bandwidth/2, 1500, gr::filter::firdes::WIN_HANN);
        int decimation = int(input_rate / if_rate);
       
        prefilter = gr::filter::freq_xlating_fir_filter_ccf::make(decimation,
	            lpf_coeffs,
	            offset,
	            samp_rate);

        float resampled_rate = float(input_rate) / float(decimation); // rate at output of self.lpf
        float arb_rate = (float(if_rate) / resampled_rate);
        float arb_size = 32;
        float arb_atten=100;


            // Create a filter that covers the full bandwidth of the output signal

            // If rate >= 1, we need to prevent images in the output,
            // so we have to filter it to less than half the channel
            // width of 0.5.  If rate < 1, we need to filter to less
            // than half the output signal's bw to avoid aliasing, so
            // the half-band here is 0.5*rate.
            float percent = 0.80;
            if(arb_rate < 1) {
                float halfband = 0.5* arb_rate;
                float bw = percent*halfband;
                float tb = (percent/2.0)*halfband;


                // As we drop the bw factor, the optfir filter has a harder time converging;
                // using the firdes method here for better results.
                arb_taps = gr::filter::firdes::low_pass_2(arb_size, arb_size, bw, tb, arb_atten,
                                                      gr::filter::firdes::WIN_BLACKMAN_HARRIS);
            } else {
                BOOST_LOG_TRIVIAL(error) << "Something is probably wrong! Resampling rate too low";
                exit(0);
            	/*
                float halfband = 0.5;
                float bw = percent*halfband;
                float tb = (percent/2.0)*halfband;
                float ripple = 0.1;

                bool made = False;
                while not made:
                    try:
                        self._taps = optfir.low_pass(self._size, self._size, bw, bw+tb, ripple, atten)
                        made = True
                    except RuntimeError:
                        ripple += 0.01
                        made = False
                        print("Warning: set ripple to %.4f dB. If this is a problem, adjust the attenuation or create your own filter taps." % (ripple))

                        # Build in an exit strategy; if we've come this far, it ain't working.
                        if(ripple >= 1.0):
                            raise RuntimeError("optfir could not generate an appropriate filter.")*/
            }






        arb_resampler = gr::filter::pfb_arb_resampler_ccf::make(arb_rate, arb_taps );


        agc = gr::analog::feedforward_agc_cc::make(16, 1.0);


        float omega = float(if_rate) / float(symbol_rate);
        float gain_omega = 0.1  * gain_mu * gain_mu;

        float alpha = costas_alpha;
        float beta = 0.125 * alpha * alpha;
        float fmax = 2400;	// Hz
        fmax = 2*pi * fmax / float(if_rate);

        costas_clock = gr::op25_repeater::gardner_costas_cc::make(omega, gain_mu, gain_omega, alpha,  beta, fmax, -fmax);


        // Perform Differential decoding on the constellation
        diffdec = gr::digital::diff_phasor_cc::make();

        // take angle of the difference (in radians)
        to_float = gr::blocks::complex_to_arg::make();

        // convert from radians such that signal is in -3/-1/+1/+3
        rescale = gr::blocks::multiply_const_ff::make( (1 / (pi / 4)) );

        // fm demodulator (needed in fsk4 case)
        float fm_demod_gain = if_rate / (2.0 * pi * symbol_deviation);
        fm_demod = gr::analog::quadrature_demod_cf::make(fm_demod_gain);



	double symbol_decim = 1;

	valve = gr::blocks::copy::make(sizeof(gr_complex));
	valve->set_enabled(false);

	for (int i=0; i < samples_per_symbol; i++) {
		sym_taps.push_back(1.0 / samples_per_symbol);
	}

	sym_filter =  gr::filter::fir_filter_fff::make(symbol_decim, sym_taps);
	tune_queue = gr::msg_queue::make(2);
	traffic_queue = gr::msg_queue::make(2);
	rx_queue = gr::msg_queue::make(100);
	const float l[] = { -2.0, 0.0, 2.0, 4.0 };
	std::vector<float> levels( l,l + sizeof( l ) / sizeof( l[0] ) );
	fsk4_demod = gr::op25_repeater::fsk4_demod_ff::make(tune_queue, system_channel_rate, symbol_rate);
	slicer = gr::op25_repeater::fsk4_slicer_fb::make(levels);

	int udp_port = 0;
	int verbosity = 1; // 10 = lots of debug messages
	const char * wireshark_host="127.0.0.1";
	bool do_imbe = 1;
	bool do_output = 1;
	bool do_msgq = 0;
	bool do_audio_output = 1;
	bool do_tdma = 0;
	op25_frame_assembler = gr::op25_repeater::p25_frame_assembler::make(wireshark_host,udp_port,verbosity,do_imbe, do_output, do_msgq, rx_queue, do_audio_output, do_tdma);
	

        
	converter = gr::blocks::short_to_float::make(1, 2048.0); //8192.0);

	tm *ltm = localtime(&starttime);

	std::stringstream path_stream;
	path_stream << boost::filesystem::current_path().string() <<  "/" << 1900 + ltm->tm_year << "/" << 1 + ltm->tm_mon << "/" << ltm->tm_mday;

	boost::filesystem::create_directories(path_stream.str());
	sprintf(filename, "%s/%ld-%ld_%g.wav", path_stream.str().c_str(),talkgroup,timestamp,freq);
	wav_sink = gr::blocks::nonstop_wavfile_sink::make(filename,1,8000,16);



        valve->set_max_output_buffer(8192);
        to_float->set_max_output_buffer(8192);
        rescale->set_max_output_buffer(8192);
        slicer->set_max_output_buffer(8192);
        op25_frame_assembler->set_max_output_buffer(8192);
        converter->set_max_output_buffer(8192);
	if (!qpsk_mod) {
        connect(self(),0, valve,0);
		connect(valve,0, prefilter,0);
		connect(prefilter,0, arb_resampler, 0);
		connect(arb_resampler,0, fm_demod,0);
		connect(fm_demod, 0, baseband_amp, 0);
		connect(baseband_amp,0, sym_filter, 0);
		connect(sym_filter, 0, fsk4_demod, 0);
		connect(fsk4_demod, 0, slicer, 0);
		connect(slicer,0, op25_frame_assembler,0);
		connect(op25_frame_assembler, 0,  converter,0);
		connect(converter,  0, wav_sink,0);
	} else {
        connect(self(),0, valve,0);
		connect(valve,0, prefilter,0);
		connect(prefilter, 0, arb_resampler, 0);
		connect(arb_resampler,0, agc,0);
		connect(agc, 0, costas_clock, 0);
		connect(costas_clock,0, diffdec, 0);
		connect(diffdec, 0, to_float, 0);
		connect(to_float,0, rescale, 0);
		connect(rescale, 0, slicer, 0);
		connect(slicer,0, op25_frame_assembler,0);
		connect(op25_frame_assembler, 0,  converter,0);
		connect(converter, 0, wav_sink,0);
	}
}


p25_recorder::~p25_recorder() {

}

Source *p25_recorder::get_source() {
    return source;
}


bool p25_recorder::is_active() {
	return active;
}


double p25_recorder::get_freq() {
	return freq;
}

int p25_recorder::lastupdate() {
	return time(NULL) - timestamp;
}

long p25_recorder::elapsed() {
	return time(NULL) - starttime;
}


void p25_recorder::tune_offset(double f) {
	freq = f;
	int offset_amount = (f - center);
	prefilter->set_center_freq(offset_amount); // have to flip this for 3.7
	//BOOST_LOG_TRIVIAL(info) << "Offset set to: " << offset_amount << " Freq: "  << freq;
}

void p25_recorder::deactivate() {
	BOOST_LOG_TRIVIAL(info) << "p25_recorder.cc: Deactivating Logger \t[ " << num << " ] - freq[ " << freq << "] \t talkgroup[ " << talkgroup << " ]";

	active = false;
	valve->set_enabled(false);
    /*BOOST_LOG_TRIVIAL(info) << 
		  "Valve: \t" << valve->max_output_buffer(0) << "\n" <<
		  "Prefilter: \t" << prefilter->max_output_buffer(0) << "\n" <<
		  "arb_resampler: \t" << arb_resampler->max_output_buffer(0) << "\n" <<
		  "agc: \t\t" << agc->max_output_buffer(0) << "\n" <<
		  "costas_clock: \t" << costas_clock->max_output_buffer(0) << "\n" <<
		  "diffdec: \t" << diffdec->max_output_buffer(0) << "\n" <<
		"to_float: \t" << to_float->max_output_buffer(0) << "\n" <<
		  "rescale: \t" << rescale->max_output_buffer(0) << "\n" <<
		"slicer: \t" << slicer->max_output_buffer(0) << "\n" <<
		"op25: \t\t" << op25_frame_assembler->max_output_buffer(0) << "\n" <<
		"converter: \t" << converter->max_output_buffer(0) << "\n" <<
        "wav_sink: \t" <<  wav_sink->max_output_buffer(0);*/
	wav_sink->close();
}

void p25_recorder::activate(Call *call, int n) {

	timestamp = time(NULL);
	starttime = time(NULL);

	talkgroup = call->get_talkgroup();
	freq = call->get_freq();
    num = n;

	BOOST_LOG_TRIVIAL(info) << "p25_recorder.cc: Activating Logger   \t[ " << num << " ] - freq[ " << freq << "] \t talkgroup[ " << talkgroup << " ]";

	int offset_amount = (freq - center);
	prefilter->set_center_freq(offset_amount); 

	wav_sink->open(call->get_filename());
	active = true;
	valve->set_enabled(true);
}


