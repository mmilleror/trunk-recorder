#define DSD

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/tokenizer.hpp>
#include <boost/intrusive_ptr.hpp>
#include <boost/foreach.hpp>


#include <iostream>
#include <cassert>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <menu.h>
#include <time.h>
#include "recorder.h"

#include "analog_recorder.h"
#include "smartnet_trunking.h"
#include "smartnet_crc.h"
#include "smartnet_deinterleave.h"
#include "talkgroups.h"
#include "source.h"
#include "call.h"
#include "smartnet_parser.h"
#include "parser.h"

#include <osmosdr/source.h>

#include <gnuradio/msg_queue.h>
#include <gnuradio/message.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/top_block.h>


using namespace std;
namespace logging = boost::log;

std::vector<Source *> sources;
std::vector<double> control_channels;
std::map<long,long> unit_affiliations;
int current_control_channel = 0;
std::vector<Call *> calls;
Talkgroups *talkgroups;
std::string talkgroups_file;
string system_type;
gr::top_block_sptr tb;
smartnet_trunking_sptr smartnet_trunking;
gr::msg_queue::sptr queue;
Source *source;
Recorder *recorder1;
Recorder *recorder2;

volatile sig_atomic_t exit_flag = 0;
SmartnetParser *smartnet_parser;

Recorder *recorder1;

void exit_interupt(int sig) { // can be called asynchronously
    exit_flag = 1; // set flag
}

unsigned GCD(unsigned u, unsigned v) {
    while ( v != 0) {
        unsigned r = u % v;
        u = v;
        v = r;
    }
    return u;
}

std::vector<float> design_filter(double interpolation, double deci) {
    float beta = 5.0;
    float trans_width = 0.5 - 0.4;
    float mid_transition_band = 0.5 - trans_width/2;

    std::vector<float> result = gr::filter::firdes::low_pass(
                                                             interpolation,
                                                             1,
                                                             mid_transition_band/interpolation,
                                                             trans_width/interpolation,
                                                             gr::filter::firdes::WIN_KAISER,
                                                             beta
                                                             );

    return result;
}

/**
 * Method name: load_config()
 * Description: <#description#>
 * Parameters: <#parameters#>
 */

void load_config()
{

    try
    {

        const std::string json_filename = "config.json";

        boost::property_tree::ptree pt;
        boost::property_tree::read_json(json_filename, pt);

        BOOST_FOREACH( boost::property_tree::ptree::value_type  &node,pt.get_child("sources") )
        {
            double center = node.second.get<double>("center",0);
            double rate = node.second.get<double>("rate",0);
            double error = node.second.get<double>("error",0);
            int gain = node.second.get<int>("gain",0);
            int if_gain = node.second.get<int>("ifGain",0);
            int bb_gain = node.second.get<int>("bbGain",0);
            std::string antenna = node.second.get<string>("antenna","");
            int digital_recorders = node.second.get<int>("digitalRecorders",0);
            int debug_recorders = node.second.get<int>("debugRecorders",0);
            int analog_recorders = node.second.get<int>("analogRecorders",0);

            std::string driver = node.second.get<std::string>("driver","");
            std::string device = node.second.get<std::string>("device","");

            BOOST_LOG_TRIVIAL(info) << "Center: " << node.second.get<double>("center",0);
            BOOST_LOG_TRIVIAL(info) << "Rate: " << node.second.get<double>("rate",0);
            BOOST_LOG_TRIVIAL(info) << "Error: " << node.second.get<double>("error",0);
            BOOST_LOG_TRIVIAL(info) << "Gain: " << node.second.get<int>("gain",0);
            BOOST_LOG_TRIVIAL(info) << "IF Gain: " << node.second.get<int>("ifGain",0);
            BOOST_LOG_TRIVIAL(info) << "BB Gain: " << node.second.get<int>("bbGain",0);

            BOOST_LOG_TRIVIAL(info) << "Digital Recorders: " << node.second.get<int>("digitalRecorders",0);
            BOOST_LOG_TRIVIAL(info) << "Debug Recorders: " << node.second.get<int>("debugRecorders",0);
            BOOST_LOG_TRIVIAL(info) << "Analog Recorders: " << node.second.get<int>("analogRecorders",0);
            BOOST_LOG_TRIVIAL(info) << "driver: " << node.second.get<std::string>("driver","");


            source = new Source(center,rate,error,driver,device);
            BOOST_LOG_TRIVIAL(info) << "Max HZ: " << source->get_max_hz();
            BOOST_LOG_TRIVIAL(info) << "Min HZ: " << source->get_min_hz();
            source->set_if_gain(if_gain);
            source->set_bb_gain(bb_gain);
            source->set_gain(gain);
            source->create_digital_recorders(tb, digital_recorders);
            BOOST_LOG_TRIVIAL(info) << "All done here: "; 

        }

        BOOST_LOG_TRIVIAL(info) << "Control Channels: ";
        BOOST_FOREACH( boost::property_tree::ptree::value_type  &node,pt.get_child("system.control_channels") )
        {
            double control_channel = node.second.get<double>("",0);
            control_channels.push_back(control_channel);
            BOOST_LOG_TRIVIAL(info) << node.second.get<double>("",0) << " ";
        }
        BOOST_LOG_TRIVIAL(info);

        talkgroups_file = pt.get<std::string>("talkgroupsFile","");
        BOOST_LOG_TRIVIAL(info) << "Talkgroups File: " << talkgroups_file;
        system_type = pt.get<std::string>("system.type");

    }
    catch (std::exception const& e)
    {
        BOOST_LOG_TRIVIAL(error) << e.what();
    }

}

/**
 * Method name: start_recorder
 * Description: <#description#>
 * Parameters: TrunkMessage message
 */

void start_recorder() {

    

    recorder1 = source->get_digital_recorder(0);
    recorder1->activate( 1,856800000, calls.size());
    recorder2 = source->get_digital_recorder(0);
    recorder2->activate( 1,856800000, calls.size());

}
void stop_recorder() {
recorder1->deactivate();
    recorder2->deactivate();
  
}




void add_control_channel(double control_channel) {
    if (std::find(control_channels.begin(), control_channels.end(), control_channel) != control_channels.end()) {
        control_channels.push_back(control_channel);
    }
}



void unit_registration(long unit) {

}

void unit_deregistration(long unit) {
    std::map<long, long>::iterator it;

    it = unit_affiliations.find (unit);
    if (it != unit_affiliations.end()) {
        unit_affiliations.erase(it);
    }
}

void group_affiliation(long unit, long talkgroup) {
    unit_affiliations[unit] = talkgroup;
}

void update_recorder(TrunkMessage message) {

    for(vector<Call *>::iterator it = calls.begin(); it != calls.end(); ++it) {
        Call *call= *it;

        if (call->get_talkgroup() == message.talkgroup) {
            if (call->get_freq() != message.freq) {
                //BOOST_LOG_TRIVIAL(trace) << "\tUpdate Retune - Total calls: " << calls.size() << "\tTalkgroup: " << message.talkgroup << "\tOld Freq: " << call->get_freq() << "\tNew Freq: " << message.freq;
                // not sure what to do here; looks like we should retune

                if (call->get_recording() == true) {
                    BOOST_LOG_TRIVIAL(info) << "\tUpdate Retune - Elapsed: " << call->elapsed() << "s \tSince update: " << call->since_last_update() << "s \tTalkgroup: " << message.talkgroup << "\tOld Freq: " << call->get_freq() << "\tNew Freq: " << message.freq << std::endl;
                    call->get_recorder()->tune_offset(message.freq);

                }
                call->set_freq(message.freq);
                call->set_tdma(message.tdma);
            }
            call->update();
        }
    }
}

void unit_check() {
    std::map<long, long> talkgroup_totals;
    std::map<long, long>::iterator it;
    char shell_command[200];
    time_t starttime = time(NULL);
    tm *ltm = localtime(&starttime);
    char unit_filename[160];

    std::stringstream path_stream;
    path_stream << boost::filesystem::current_path().string() <<  "/" << 1900 + ltm->tm_year << "/" << 1 + ltm->tm_mon << "/" << ltm->tm_mday;

    boost::filesystem::create_directories(path_stream.str());



    for(it = unit_affiliations.begin(); it != unit_affiliations.end(); ++it) {
        talkgroup_totals[it->second]++;
    }

    sprintf(unit_filename, "%s/%ld-unit_check.json", path_stream.str().c_str(),starttime);

    ofstream myfile (unit_filename);
    if (myfile.is_open())
    {
        myfile << "{\n";
        myfile << "\"talkgroups\": {\n";
        for(it = talkgroup_totals.begin(); it != talkgroup_totals.end(); ++it) {
            if (it != talkgroup_totals.begin()) {
                myfile << ",\n";
            }
            myfile << "\"" << it->first << "\": " << it->second;

        }
        myfile << "\n}\n}\n";
        sprintf(shell_command,"./unit_check.sh %s > /dev/null 2>&1 &", unit_filename);
        system(shell_command);
        myfile.close();
    }
}



void monitor_messages() {
    gr::message::sptr msg;
    int messagesDecodedSinceLastReport = 0;
    float msgs_decoded_per_second = 0;

    time_t lastMsgCountTime = time(NULL);;
    time_t lastTalkgroupPurge = time(NULL);;
    time_t currentTime = time(NULL);
    time_t lastUnitCheckTime = time(NULL);
    std::vector<TrunkMessage> trunk_messages;

    while (1) {
        if(exit_flag) { // my action when signal set it 1
            printf("\n Signal caught!\n");
            return;
        }









    }
}




int main(void)
{
    signal(SIGINT, exit_interupt);
    logging::core::get()->set_filter
    (
     logging::trivial::severity >= logging::trivial::info
     );

    tb = gr::make_top_block("Trunking");



    load_config();







        tb->start();
        start_recorder();
        monitor_messages();
        stop_recorder();
        //------------------------------------------------------------------
        //-- stop flow graph execution
        //------------------------------------------------------------------
        BOOST_LOG_TRIVIAL(info) << "stopping flow graph";
        tb->stop();
        tb->wait();




    return 0;

}
