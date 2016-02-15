
#include "dsd_recorder.h"
using namespace std;

bool dsd_recorder::logging = false;

dsd_recorder_sptr make_dsd_recorder( Source *src, long t, int n)
{
	return gnuradio::get_initial_sptr(new dsd_recorder(src, t, n));
}

dsd_recorder::dsd_recorder(Source *src, long t, int n)
	: gr::hier_block2 ("dsd_recorder",
	                   gr::io_signature::make  (1, 1, sizeof(gr_complex)),
	                   gr::io_signature::make  (0, 0, sizeof(float)))
{
    source = src;
	freq = source->get_center();
	center = source->get_center();
	samp_rate = source->get_rate();
	talkgroup = t;
	num = n;
	active = false;
    bool fsk4 = true;


	starttime = time(NULL);

	float offset = 0; //have to flip for 3.7

	float symbol_rate = 4800;
            float gain_mu = 0.025;
        float costas_alpha = 0.04;
    const double pi = M_PI;
	int samp_per_sym = 10;
	double decim = floor(samp_rate / 100000);
	float xlate_bandwidth = 15000; //14000; //24260.0;
	float channel_rate = 4800 * samp_per_sym;
         double input_rate = samp_rate;
        float if_rate = 48000;
	double pre_channel_rate = samp_rate/decim;



	lpf_taps =  gr::filter::firdes::low_pass(1, samp_rate, 15000, 1500, gr::filter::firdes::WIN_HAMMING);

	prefilter = gr::filter::freq_xlating_fir_filter_ccf::make(decim,
	            lpf_taps,
	            offset,
	            samp_rate);
        
        
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
                BOOST_LOG_TRIVIAL(error) << "CRAP! Computer over!";
            }
        arb_resampler = gr::filter::pfb_arb_resampler_ccf::make(arb_rate, arb_taps );
        

	demod = gr::analog::quadrature_demod_cf::make(1.2); //1.6); //1.4);
	levels = gr::blocks::multiply_const_ff::make(1.0); //.40); //33);
	valve = gr::blocks::copy::make(sizeof(gr_complex));
	valve->set_enabled(false);

	for (int i=0; i < samp_per_sym; i++) {
		sym_taps.push_back(1.0 / samp_per_sym);
	}
	sym_filter = gr::filter::fir_filter_fff::make(1, sym_taps);
	lpf_second = gr::filter::fir_filter_fff::make(1,gr::filter::firdes::low_pass(1, 48000, 6000, 500));
	iam_logging = false;

	tm *ltm = localtime(&starttime);

	std::stringstream path_stream;
	path_stream << boost::filesystem::current_path().string() <<  "/" << 1900 + ltm->tm_year << "/" << 1 + ltm->tm_mon << "/" << ltm->tm_mday;

	boost::filesystem::create_directories(path_stream.str());
	sprintf(filename, "%s/%ld-%ld_%g.wav", path_stream.str().c_str(),talkgroup,starttime,freq);
	sprintf(status_filename, "%s/%ld-%ld_%g.json", path_stream.str().c_str(),talkgroup,starttime,freq);
	wav_sink = gr::blocks::nonstop_wavfile_sink::make(filename,1,8000,16);
	null_sink = gr::blocks::null_sink::make(sizeof(gr_complex));


        
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
        const float l[] = { -2.0, 0.0, 2.0, 4.0 };
	std::vector<float> leveler( l,l + sizeof( l ) / sizeof( l[0] ) );
        	slicer = gr::op25_repeater::fsk4_slicer_fb::make(leveler);

        
  	if (fsk4) {
        dsd = dsd_make_block_ff(dsd_FRAME_P25_PHASE_1,dsd_MOD_QPSK,4,1,1, false, num);

        connect(self(),0, valve,0);
		connect(valve,0, prefilter,0);
		connect(prefilter, 0, arb_resampler, 0);
		connect(arb_resampler, 0, demod, 0);
        connect(demod, 0, dsd, 0);
		//connect(demod, 0, sym_filter, 0);
		//connect(sym_filter, 0, levels, 0);
		//connect(levels, 0, dsd, 0);
		connect(dsd, 0, wav_sink,0);
        
        /*
		connect(self(),0, valve,0);
		connect(valve,0, prefilter,0);
		connect(prefilter, 0, arb_resampler, 0);
		connect(arb_resampler,0, fm_demod,0);
		connect(fm_demod, 0, baseband_amp, 0);
		connect(baseband_amp,0, sym_filter, 0);
		connect(sym_filter, 0, fsk4_demod, 0);
		connect(fsk4_demod, 0, slicer, 0);
		connect(slicer,0, op25_frame_assembler,0);
		connect(op25_frame_assembler, 0,  converter,0);
		connect(converter, 0, multiplier,0);
		connect(multiplier, 0, wav_sink,0);*/
	} else {
        dsd = dsd_make_block_ff(dsd_FRAME_P25_PHASE_1,dsd_MOD_QPSK,4,1,1, false, num);
		connect(self(),0, valve,0);
		connect(valve,0, prefilter,0);

		connect(prefilter, 0, arb_resampler, 0);
		connect(arb_resampler,0, agc,0);
        

		connect(agc, 0, costas_clock, 0);
		connect(costas_clock,0, diffdec, 0);

		connect(diffdec, 0, to_float, 0);


		connect(to_float,0,  rescale, 0);
		connect(rescale, 0, slicer, 0);

		connect(slicer,  0, dsd, 0);
		connect(dsd, 0, wav_sink,0);
        //connect(valve,0, slicer, 0);
		//connect(slicer,0, op25_frame_assembler,0);
        
   
	}
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

Source *dsd_recorder::get_source() {
    return source;
}

char *dsd_recorder::get_filename() {
	return filename;
}

void dsd_recorder::tune_offset(double f) {
	freq = f;
	long offset_amount = (f - center);
	prefilter->set_center_freq(offset_amount); // have to flip this for 3.7
}
void dsd_recorder::deactivate() {
	BOOST_LOG_TRIVIAL(info) << "dsd_recorder.cc: Deactivating Logger [ " << num << " ] - freq[ " << freq << "] \t talkgroup[ " << talkgroup << " ]";

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



void dsd_recorder::activate( long t, double f, int n,char *existing_filename) {

	starttime = time(NULL);

	talkgroup = t;
	freq = f;
    num = n;

	tm *ltm = localtime(&starttime);
	BOOST_LOG_TRIVIAL(info) << "dsd_recorder.cc: Activating Logger [ " << num << " ] - freq[ " << freq << "] \t talkgroup[ " << talkgroup << " ]";


	prefilter->set_center_freq(f - center); // have to flip for 3.7


	std::stringstream path_stream;
	path_stream << boost::filesystem::current_path().string() <<  "/" << 1900 + ltm->tm_year << "/" << 1 + ltm->tm_mon << "/" << ltm->tm_mday;

	boost::filesystem::create_directories(path_stream.str());
    if (existing_filename != NULL) {
    strcpy(filename,existing_filename);
    } else {
	sprintf(filename, "%s/%ld-%ld_%g.wav", path_stream.str().c_str(),talkgroup,starttime,f);
    }
	sprintf(status_filename, "%s/%ld-%ld_%g.json", path_stream.str().c_str(),talkgroup,starttime,freq);

	wav_sink->open(filename);

	active = true;
	valve->set_enabled(true);
}
