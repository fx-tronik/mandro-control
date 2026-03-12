#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <iostream>

#include <errno.h>

#include <ros/ros.h>
#include <soem/ethercat.h>

class Controller
{
public:
    Controller( ros::NodeHandle &nh, const std::string &ifname);

    ~Controller();

    void write(int slave_no, uint8_t channel, uint8_t value);
    uint8_t readInput(int slave_no, uint8_t channel) const;
    uint8_t readOutput(int slave_no, uint8_t channel) const;

    template <typename T>
    uint8_t writeSDO(int slave_no, uint16_t index, uint8_t subidx, T value) const;

    template <typename T>
    T readSDO(int slave_no, uint16_t index, uint8_t subidx) const;

    int getNumClients() const;
    void getStatus(int slave_no, std::string &name, int &eep_man, int &eep_id, int &eep_rev, int &obits, int &ibits, int &state, int &pdelay, int &hasdc, int &activeports, int &configadr) const;
    void getMan(int slave_no, int &eep_man, int &eep_id, int &eep_rev) const;

private:
    ros::NodeHandle &nh_;
    bool initSoem();
    const std::string ifname_;
    uint8_t iomap_[8192];
    int num_clients_ = 0;
    std::thread cycle_thread_;
    mutable std::mutex iomap_mutex_;
    bool stop_flag_ = false;
    int64 integral = 0;

    static void timespecInc(struct timespec &tick, int nsec, int64 adj_time);
    static void handleErrors();
    int64 ec_sync(int64 reftime, int64 cycletime);
    void cycleWorker(std::mutex &mutex, bool &stop_flag);
    static int slave_conf_common(uint16 cnt);
    static int slave_conf_homing(uint16 cnt, uint8_t homing_method, uint32_t homing_speed, uint32_t acceleration, uint32_t home_offset);
    static int slave_conf(uint16 cnt);
};
