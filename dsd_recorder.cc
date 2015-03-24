
#include "dsd_recorder.h"
using namespace std;

bool dsd_recorder::logging = false;

dsd_recorder_sptr make_dsd_recorder(float freq, float center, long s, long t, int n)
{
	return gnuradio::get_initial_sptr(new dsd_recorder(freq, center, s, t, n));
}

dsd_recorder::dsd_recorder(double f, double c, long s, long t, int n)
	: gr::hier_block2 ("dsd_recorder",
	                   gr::io_signature::make  (1, 1, sizeof(gr_complex)),
	                   gr::io_signature::make  (0, 0, sizeof(float)))
{
	freq = f;
	center = c;
	samp_rate = s;
	talkgroup = t;
	num = n;
	active = false;

const double pi = M_PI;
	starttime = time(NULL);

	float symbol_rate = 4800;
		float symbol_deviation = 600.0;
		double samples_per_sym = 10;
       double input_rate = samp_rate;
        float if_rate = 48000;
        float gain_mu = 0.025;
        float costas_alpha = 0.04;
        double sps = 0.0;
        float bb_gain = 1.0;
        
       	baseband_amp = gr::blocks::multiply_const_ff::make(bb_gain);

        // local osc
        lo = gr::analog::sig_source_c::make(input_rate, gr::analog::GR_SIN_WAVE, 0, 1.0, 0);
        mixer = gr::blocks::multiply_cc::make();
        lpf_coeffs = gr::filter::firdes::low_pass(1.0, input_rate, 15000, 1500, gr::filter::firdes::WIN_HANN);
        int decimation = int(input_rate / if_rate);
        lpf = gr::filter::fir_filter_ccf::make(decimation, lpf_coeffs);

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
                float ripple = 0.1;

                // As we drop the bw factor, the optfir filter has a harder time converging;
                // using the firdes method here for better results.
                arb_taps = gr::filter::firdes::low_pass_2(arb_size, arb_size, bw, tb, arb_atten,
                                                      gr::filter::firdes::WIN_BLACKMAN_HARRIS);
            } else {
            	std::cout << " CRAP!! " << std::endl;
  
                    }






        arb_resampler = gr::filter::pfb_arb_resampler_ccf::make(arb_rate, arb_taps );

        



        float omega = float(if_rate) / float(symbol_rate);
        float gain_omega = 0.1  * gain_mu * gain_mu;

        float alpha = costas_alpha;
        float beta = 0.125 * alpha * alpha;
        float fmax = 2400;	// Hz
        fmax = 2*pi * fmax / float(if_rate);

        costas_clock = gr::op25_repeater::gardner_costas_cc::make(omega, gain_mu, gain_omega, alpha,  beta, fmax, -fmax);

        agc = gr::analog::feedforward_agc_cc::make(16, 1.0);

        // Perform Differential decoding on the constellation
        diffdec = gr::digital::diff_phasor_cc::make();

        // take angle of the difference (in radians)
        to_float = gr::blocks::complex_to_arg::make();

        // convert from radians such that signal is in -3/-1/+1/+3
        rescale = gr::blocks::multiply_const_ff::make( (1 / (pi / 4)) );








	valve = gr::blocks::copy::make(sizeof(gr_complex));
	valve->set_enabled(false);

	for (int i=0; i < samples_per_sym; i++) {
		sym_taps.push_back(1.0 / samples_per_sym);
	}
	sym_filter = gr::filter::fir_filter_fff::make(1, sym_taps);
	iam_logging = false;
	dsd = dsd_make_block_ff(dsd_FRAME_P25_PHASE_1,dsd_MOD_GFSK,4,1,1, false, num);

	tm *ltm = localtime(&starttime);

	std::stringstream path_stream;
	path_stream << boost::filesystem::current_path().string() <<  "/" << 1900 + ltm->tm_year << "/" << 1 + ltm->tm_mon << "/" << ltm->tm_mday;

	boost::filesystem::create_directories(path_stream.str());
	sprintf(filename, "%s/%ld-%ld_%g.wav", path_stream.str().c_str(),talkgroup,starttime,freq);
	sprintf(status_filename, "%s/%ld-%ld_%g.json", path_stream.str().c_str(),talkgroup,starttime,freq);
	wav_sink = gr::blocks::nonstop_wavfile_sink::make(filename,1,8000,16);
	null_sink = gr::blocks::null_sink::make(sizeof(gr_complex));

	
		connect(self(),0, mixer, 0);
		connect(lo,0, mixer, 1);
		connect(mixer,0, valve,0);
		connect(valve, 0, lpf, 0);
		connect(lpf, 0, arb_resampler, 0);
		connect(arb_resampler,0, agc,0);
		connect(agc, 0, costas_clock, 0);
		connect(costas_clock,0, diffdec, 0);
		connect(diffdec, 0, to_float, 0);
		connect(to_float,0, rescale, 0);
		connect(rescale, 0, dsd, 0);
		connect(dsd, 0, wav_sink,0);
}

dsd_recorder::~dsd_recorder() {

}


bool dsd_recorder::is_active() {
	return active;
}

long dsd_recorder::get_talkgroup() {
	return talkgroup;
}

double dsd_recorder::get_freq() {
	return freq;
}

char *dsd_recorder::get_filename() {
	return filename;
}

void dsd_recorder::tune_offset(double f) {
	freq = f;
	long offset_amount = (f - center);
	lo->set_frequency(-offset_amount);
	//prefilter->set_center_freq(offset_amount); // have to flip this for 3.7
}
void dsd_recorder::deactivate() {
	BOOST_LOG_TRIVIAL(info) << "dsd_recorder.cc: Deactivating Logger [ " << num << " ] - freq[ " << freq << "] \t talkgroup[ " << talkgroup << " ] " << std::endl;

	//lock();

	wav_sink->close();

/*	disconnect(self(), 0, prefilter, 0);
	connect(self(),0, null_sink,0);

	disconnect(prefilter, 0, downsample_sig, 0);
	disconnect(downsample_sig, 0, demod, 0);
	disconnect(demod, 0, sym_filter, 0);
	disconnect(sym_filter, 0, levels, 0);
	disconnect(levels, 0, dsd, 0);
	disconnect(dsd, 0, wav_sink,0);*/

	active = false;
	valve->set_enabled(false);


	//unlock();


	dsd_state *state = dsd->get_state();
	ofstream myfile (status_filename);
	if (myfile.is_open())
	{
		int level = (int) state->max / 164;
		int index=0;
		myfile << "{\n";
		myfile << "\"freq\": " << freq << ",\n";
		myfile << "\"num\": " << num << ",\n";
		myfile << "\"talkgroup\": " << talkgroup << ",\n";
		myfile << "\"center\": " << state->center << ",\n";
		myfile << "\"umid\": " << state->umid << ",\n";
		myfile << "\"lmid\": " << state->lmid << ",\n";
		myfile << "\"max\": " << state->max << ",\n";
		myfile << "\"inlvl\": " << level << ",\n";
		myfile << "\"nac\": " << state->nac << ",\n";
		myfile << "\"src\": " << state->lastsrc << ",\n";
		myfile << "\"dsdtg\": " << state->lasttg << ",\n";
		myfile << "\"headerCriticalErrors\": " << state->debug_header_critical_errors << ",\n";
		myfile << "\"headerErrors\": " << state->debug_header_errors << ",\n";
		myfile << "\"audioErrors\": " << state->debug_audio_errors << ",\n";
		myfile << "\"symbCount\": " << state->symbolcnt << ",\n";
		myfile << "\"mode\": \"digital\",\n";
		myfile << "\"srcList\": [ ";
		while(state->src_list[index]!=0) {
			if (index !=0) {
				myfile << ", " << state->src_list[index];
			} else {
				myfile << state->src_list[index];
			}
			index++;
		}
		myfile << " ]\n";
		myfile << "}\n";
		myfile.close();
	}
	else BOOST_LOG_TRIVIAL(error) << "Unable to open file";
	dsd->reset_state();
}

void dsd_recorder::activate( long t, double f, int n) {

	starttime = time(NULL);

	talkgroup = t;
	freq = f;

	tm *ltm = localtime(&starttime);
	BOOST_LOG_TRIVIAL(info) << "dsd_recorder.cc: Activating Logger [ " << num << " ] - freq[ " << freq << "] \t talkgroup[ " << talkgroup << " ]  "  <<std::endl;

	long offset_amount = (f - center);
	lo->set_frequency(-offset_amount);
	//prefilter->set_center_freq(f - center); // have to flip for 3.7


	std::stringstream path_stream;
	path_stream << boost::filesystem::current_path().string() <<  "/" << 1900 + ltm->tm_year << "/" << 1 + ltm->tm_mon << "/" << ltm->tm_mday;

	boost::filesystem::create_directories(path_stream.str());
	sprintf(filename, "%s/%ld-%ld_%g.wav", path_stream.str().c_str(),talkgroup,starttime,f);
	sprintf(status_filename, "%s/%ld-%ld_%g.json", path_stream.str().c_str(),talkgroup,starttime,freq);

	wav_sink->open(filename);

	active = true;
	valve->set_enabled(true);
}
