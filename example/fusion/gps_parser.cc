#pragma once

#include "gps_parser.h"

#include "SFML/Network.hpp"

// Continuously parses the gps text and updated with respect to timestamp. Run in separate thead!
void gps_parser::start_reading(openvslam::util::time_sync* time_s) {
    this->terminate = false;

    ifstream fin;
    fin.open(this->file_path, fstream::in);

    if (!fin.fail()) {
        this->is_valid = true;
        spdlog::info("Parsing gps file @ " + file_path);
        string msg;
        string txt[10];
        int ind;

        chrono::milliseconds sleep_time(0);
        long long last_stamp = 0;

        chrono::milliseconds wait_for_vid_thread(0);

        while (getline(fin, msg)) {
            auto tp_1 = std::chrono::steady_clock::now();

            if (this->terminate)
                break;

            //spdlog::info("parser: " + msg);
            if (msg.find("#") != string::npos)
                continue;

            std::stringstream msg_stream(msg);
            this->is_valid = false;
            ind = 0;

            while (msg_stream.good()) {
                getline(msg_stream, txt[ind], ',');
                ind++;
                if (ind > 9) {
                    this->is_valid = true;
                    break;
                }
            }

            if (this->is_valid) {
                this->timestamp = stoll(txt[0]);
                if (last_stamp == 0) {
                    last_stamp = this->timestamp;
                }
                gps_data data(stod(txt[1]), stod(txt[2]), stod(txt[3]), stod(txt[4]),
                              stod(txt[5]), stoi(txt[6]), stoi(txt[7]), stod(txt[8]), txt[9]);
                gps_records.insert(std::pair<long,gps_data>((long)(this->timestamp - last_stamp), data));

				curr_gps_data = data;
            }
            else {
                spdlog::warn(msg);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }


            const auto tp_2 = std::chrono::steady_clock::now();
            // double (s)
            //const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            long long track_time = duration_cast<milliseconds>(tp_2 - tp_1).count();

            sleep_time = chrono::milliseconds(this->timestamp - (last_stamp + track_time));
            //spdlog::info("dt: "+to_string(this->timestamp - last_stamp) +" track: "+ to_string(track_time));
            //spdlog::info("time gap (in ms) between process: {} and gps time: {} = {}", time_s->get_dt_start(), time_s->gps_timestamp.count(), time_s->get_dt_start() - time_s->gps_timestamp.count());

            time_s->gps_timestamp += chrono::milliseconds(this->timestamp - last_stamp);

            wait_for_vid_thread = time_s->is_gps_caught_up_video();
            if (wait_for_vid_thread + sleep_time > chrono::milliseconds(10)) {
                //spdlog::info("sleeping gps parser for: vid_w:" + to_string(wait_for_vid_thread.count()) + ", s_t: " + to_string(sleep_time.count()));
                if (wait_for_vid_thread > sleep_time)
                    std::this_thread::sleep_for(wait_for_vid_thread);
                else
                    std::this_thread::sleep_for(sleep_time);
            }

            last_stamp = this->timestamp;
        }
        spdlog::info("reached end of gps input txt file");
    }
    else {
        this->is_valid = false;
        spdlog::critical("Failed to read gps file @ " + file_path);
    }
}

void gps_parser::connect_and_read(openvslam::util::time_sync* time_s) {
    
    this->terminate = false;

	string msg;
    string txt[10];
    int ind;
    sf::TcpSocket socket;
    sf::Socket::Status status;

    //initialize connection with GPS server
    status = socket.connect(this->ip_addr, this->port_num);
    if (status != sf::Socket::Done) {
        spdlog::critical("Failed to initialize TCP connection. server: {}:{}", this->ip_addr,this->port_num);
        return;
    }

    char data[300];
    std::size_t received;
    long long last_stamp = 0;

	// TCP socket:
    while (true) {
        auto tp_1 = std::chrono::steady_clock::now();
        if (this->terminate) {
            socket.disconnect();
            break;
		}

        if (socket.receive(data, 300, received) != sf::Socket::Done) {
            this->terminate = true;
            spdlog::critical("TCP: error receiving data. socket.error_code: {}",socket.Error);
            //?break;
        }
        else {
            msg = string(data);
            //spdlog::info("TCP: " + msg);
            msg = msg.substr(msg.find_first_of('(') + 1, msg.find_first_of(')') - msg.find_first_of('(') - 1);
            

            if (this->terminate)
                break;

            //spdlog::info("parser: " + msg);
            if (msg.find("#") != string::npos)
                continue;

            std::stringstream msg_stream(msg);
            this->is_valid = false;
            ind = 0;

			while (msg_stream.good())
			{
                getline(msg_stream, txt[ind], ',');
                ind++;
                if (ind > 9) {
                    this->is_valid = true;
                    break;
                }
			}

            if (this->is_valid) {
                this->timestamp = stoll(txt[0]);
                gps_data gdata(stod(txt[1]), stod(txt[2]), stod(txt[3]), stod(txt[4]),
                              stod(txt[5]), stoi(txt[6]), stoi(txt[7]), stod(txt[8]), txt[9]);
                gps_records.insert(std::pair<long, gps_data>((long)(this->timestamp - last_stamp), gdata));

                curr_gps_data = gdata;
            }
            else {
                spdlog::warn(msg);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            if (last_stamp == 0) {
                last_stamp = this->timestamp;
            }

            const auto tp_2 = std::chrono::steady_clock::now();
            long long track_time = duration_cast<milliseconds>(tp_2 - tp_1).count();
            time_s->gps_timestamp += chrono::milliseconds(this->timestamp - last_stamp);
            last_stamp = this->timestamp;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }//end of while loop
}

void gps_parser::terminate_process() {
    this->terminate = true;
    spdlog::info("Terminating gps parser");
}

//update and convert new wgs84 to utm with latest value
void gps_parser::update_gps_value(geo_utm* gps) {
    //gps->zone = LatLonToUTMXY(gps->ref_ellipsoid_id, this->lat, this->lon, gps->x, gps->y);
    //gps->altitude = this->alt;
    gps->zone = this->curr_gps_data.zone;
    gps->x = this->curr_gps_data.utmx;
    gps->y = this->curr_gps_data.utmy;
    gps->uncertainity = this->curr_gps_data.accumulated_delta_range;
    (this->curr_gps_data.lat > 0.0) ? gps->southhemi = false : gps->southhemi = true;
    //spdlog::info("parser: gps {},{},{}\nutm: {}", this->lat, this->lon, this->alt, gps->value());
}

void gps_parser::get_gps_value(geo_utm* interp_gps, long t_stamp)
{
	if (t_stamp >= this->timestamp || this->gps_records.size()<2) {
        //video thread ahead of gps
        update_gps_value(interp_gps);
        return;
	}

	for (auto it = this->gps_records.rbegin();it!=this->gps_records.rend();++it)
	{
        if (it->first > t_stamp)
            continue;

		long dt = prev(it)->first - it->first;
        double dx = prev(it)->second.utmx - it->second.utmx;
        double dy = prev(it)->second.utmy - it->second.utmy;

		//lerp between utm
        interp_gps->x = it->second.utmx + (t_stamp - it->first) / dt * dx;
        interp_gps->y = it->second.utmy + (t_stamp - it->first) / dt * dy;

        interp_gps->uncertainity = it->second.accumulated_delta_range;
		break;
	}
}

long gps_parser::get_last_timestamp() {
    return this->timestamp;
}

//update gps to new value
void gps_parser::update_gps_value(geo_location* gps) {
    gps->latitude = this->curr_gps_data.lat;
    gps->longitude = this->curr_gps_data.lon;
    gps->altitude = this->curr_gps_data.alt;
}

// Return direction vector3d for (p2->x - p1->x, altitude, p2->y - p1->y)
Eigen::Vector3d gps_parser::get_direction_vector(geo_utm& p1, geo_utm& p2, double altitude) {
    return Eigen::Vector3d(p2.x - p1.x, altitude, p2.y - p1.y);
}

/*
 __  _ _____   ____  _ _____  
|  \| | __\ `v' /  \| |_   _| 
| | ' | _| `. .'| | ' | | |   
|_|\__|_|   !_! |_|\__| |_|
 
*/