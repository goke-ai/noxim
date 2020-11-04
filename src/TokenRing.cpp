/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the implementation of the processing element
 */
#include "Hub.h"
#include "TokenRing.h"

void TokenRing::updateTokenPacket(int channel)
{
    int token_pos = token_position[channel];
    int token_holder = rings_mapping[channel][token_pos];
    // TEST HOLD BUG
    //if (flag[channel][token_pos]->read() == RELEASE_CHANNEL)

    if (flag[channel][token_holder]->read() == RELEASE_CHANNEL)
    {
        // number of hubs of the ring
        int num_hubs = rings_mapping[channel].size();

        cout << "*GK* Channel " << channel << " number of Hubs " << num_hubs << endl;
        //
        Hub *h = rings_mapping_hubs[channel][token_pos];
        cout << "*GK* Current hub " << token_holder << " as Packet: " << h->wireless_communications_counter << endl;
        printWirelessPackets(channel);

        // +gk
        if (GlobalParams::token_topology == "C_MAC")       
        {
            tuple<int, unsigned int> hMax = assignNextHub(channel);
            token_position[channel] = std::get<0>(hMax);

            if ((num_hubs - hubs_cycle.size()) == 1)
            {
                hubs_cycle.clear();
                cout << "\t*GK* Clear cycle ====\n";
            }
            hubs_cycle.
            insert(token_position[channel]);
        }
        else
        {
            token_position[channel] = (token_position[channel] + 1) % num_hubs;
        }
        
        // -gk

        int new_token_holder = rings_mapping[channel][token_position[channel]];

        Hub *h2 = rings_mapping_hubs[channel][token_position[channel]];
        cout << "*GK* Next hub " << new_token_holder << " as Packet: " << h2->wireless_communications_counter << endl;
        cout << endl;

        LOG << "*** Token of channel " << channel << " has been assigned to Hub_" << new_token_holder << endl;
        current_token_holder[channel]->write(new_token_holder);
        // TEST HOLD BUG
        //flag[channel][token_position[channel]]->write(HOLD_CHANNEL);
        flag[channel][new_token_holder]->write(HOLD_CHANNEL);
    }
}

void TokenRing::updateTokenMaxHold(int channel)
{
    if (--token_hold_count[channel] == 0 ||
        flag[channel][token_position[channel]]->read() == RELEASE_CHANNEL)
    {
        token_hold_count[channel] = atoi(GlobalParams::channel_configuration[channel].macPolicy[1].c_str());
        // number of hubs of the ring
        int num_hubs = rings_mapping[channel].size();

        token_position[channel] = (token_position[channel] + 1) % num_hubs;
        LOG << "*** Token of channel " << channel << " has been assigned to Hub_" << rings_mapping[channel][token_position[channel]] << endl;

        current_token_holder[channel]->write(rings_mapping[channel][token_position[channel]]);
    }

    current_token_expiration[channel]->write(token_hold_count[channel]);
}

void TokenRing::updateTokenHold(int channel)
{
    if (--token_hold_count[channel] == 0)
    {
        token_hold_count[channel] = atoi(GlobalParams::channel_configuration[channel].macPolicy[1].c_str());
        // number of hubs of the ring
        int num_hubs = rings_mapping[channel].size();

        token_position[channel] = (token_position[channel] + 1) % num_hubs;
        LOG << "*** Token of channel " << channel << " has been assigned to Hub_" << rings_mapping[channel][token_position[channel]] << endl;

        current_token_holder[channel]->write(rings_mapping[channel][token_position[channel]]);
    }

    current_token_expiration[channel]->write(token_hold_count[channel]);
}

void TokenRing::updateTokens()
{
    if (reset.read())
    {
        for (map<int, ChannelConfig>::iterator i = GlobalParams::channel_configuration.begin();
             i != GlobalParams::channel_configuration.end();
             i++)
            current_token_holder[i->first]->write(rings_mapping[i->first][0]);
    }
    else
    {

        for (map<int, ChannelConfig>::iterator i = GlobalParams::channel_configuration.begin(); i != GlobalParams::channel_configuration.end(); i++)
        {
            int channel = i->first;
            //int channel_holder;
            //channel_holder = current_token_holder[channel]->read();

            string macPolicy = getPolicy(channel).first;

            if (macPolicy == TOKEN_PACKET)
                updateTokenPacket(channel);
            else if (macPolicy == TOKEN_HOLD)
                updateTokenHold(channel);
            else if (macPolicy == TOKEN_MAX_HOLD)
                updateTokenMaxHold(channel);
            else
                assert(false);
        }
    }
}

// +gk
// void TokenRing::attachHub(int channel, int hub, sc_in<int>* hub_token_holder_port, sc_in<int>* hub_token_expiration_port, sc_inout<int>* hub_flag_port)
// -gk
void TokenRing::attachHub(int channel, int hub, sc_in<int> *hub_token_holder_port, sc_in<int> *hub_token_expiration_port, sc_inout<int> *hub_flag_port, Hub *pHub)
{
    // If port for requested channel is not present, create the
    // port and connect a signal
    if (!current_token_holder[channel])
    {
        token_position[channel] = 0;

        // +gk
        hubs_cycle.insert(token_position[channel]);
        // -gk

        // TEST HOLDBUG
        //token_position[channel] = hub;
        current_token_holder[channel] = new sc_out<int>();
        current_token_expiration[channel] = new sc_out<int>();

        token_holder_signals[channel] = new sc_signal<int>();
        token_expiration_signals[channel] = new sc_signal<int>();

        current_token_holder[channel]->bind(*(token_holder_signals[channel]));
        current_token_expiration[channel]->bind(*(token_expiration_signals[channel]));

        // initial value that will be overwritten if mac policy != TOKEN_PACKET
        token_hold_count[channel] = 0;

        if (GlobalParams::channel_configuration[channel].macPolicy[0] != TOKEN_PACKET)
        {
            // checking max hold cycles vs wireless transmission latency
            // consistency
            //TODO move this check: max_hold_cycles depends on the Channel not on the Hub
            double delay_ps = 1000 * GlobalParams::flit_size / GlobalParams::channel_configuration[channel].dataRate;
            int cycles = ceil(delay_ps / GlobalParams::clock_period_ps);
            int max_hold_cycles = atoi(GlobalParams::channel_configuration[channel].macPolicy[1].c_str());
            assert(cycles < max_hold_cycles);

            token_hold_count[channel] = atoi(GlobalParams::channel_configuration[channel].macPolicy[1].c_str());
        }
    }

    flag[channel][hub] = new sc_inout<int>();
    flag_signals[channel][hub] = new sc_signal<int>();
    flag[channel][hub]->bind(*(flag_signals[channel][hub]));
    hub_flag_port->bind(*(flag_signals[channel][hub]));

    // Connect tokenring to hub
    hub_token_holder_port->bind(*(token_holder_signals[channel]));
    hub_token_expiration_port->bind(*(token_expiration_signals[channel]));

    //LOG << "Attaching Hub " << hub << " to the token ring for channel " << channel << endl;
    rings_mapping[channel].push_back(hub);
    rings_mapping_hubs[channel].push_back(pHub);

    // TEST HOLD BUG
    int starting_hub = rings_mapping[channel][0];
    current_token_holder[channel]->write(starting_hub);
}

void TokenRing::printWirelessPackets(int channel)
{
    unsigned int packets = 0;
    int count = 0;
    for (auto &&h : rings_mapping_hubs[channel])
    {
        unsigned int packet = h->wireless_communications_counter;
        cout << "*GK* Hub_" << count << " => Packet: " << h->wireless_communications_counter << endl;
        ++count;

        packets += packet;
    }
    cout << "*GK* Hubs => Packets: " << packets << endl;
}

tuple<int, unsigned int> TokenRing::assignNextHub(int channel)
{
    vector<tuple<int, unsigned int>> hub_packets;
    unsigned int packets = 0;
    int count = 0;
    // get hub packet
    for (auto &&h : rings_mapping_hubs[channel])
    {
        unsigned int packet = h->wireless_communications_counter;
        // cout << "*GK* Hub_" << count << " => Packet: " << h->wireless_communications_counter << endl;
        hub_packets.push_back(make_tuple(count, packet));
        ++count;

        packets += packet;
    }
    // cout << "*GK* Hubs => Packets: " << packets << endl;

    vector<tuple<int, unsigned int>> w_hub_packets(hub_packets.size());
    copy_if(hub_packets.begin(), hub_packets.end(), w_hub_packets.begin(), [&](const tuple<int, unsigned int> &hub_p) {
        bool isNot = true;
        for (auto &&i : hubs_cycle)
        {

            int k;
            unsigned int p;
            std::tie(k, p) = hub_p;
            // auto [k, p ] = hub_p;
            if (i == k)
            {
                isNot = false;
                break;
            }
        }
        return isNot;
    });

    // for (auto &&h : w_hub_packets)
    // {
    //     cout << "\t*GK* Hub_" << std::get<0>(h) << " => Packet: " << std::get<1>(h) << endl;
    // }

    // find maximum
    auto hMax = std::max_element(w_hub_packets.begin(), w_hub_packets.end(),
                                 [](const tuple<int, unsigned int> &h1, const tuple<int, unsigned int> &h2) {
                                     int k1, k2;
                                     unsigned int p1, p2;
                                     std::tie(k1, p1) = h1;
                                     std::tie(k2, p2) = h2;
                                     return p1 < p2;
                                 });

    cout << "\t*GK* Max Hub_" << std::get<0>(*hMax) << " => Packet: " << std::get<1>(*hMax) << endl;

    return *hMax;
}
