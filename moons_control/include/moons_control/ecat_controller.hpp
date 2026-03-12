#ifndef ECAT_CONTROLLER_H
#define ECAT_CONTROLLER_H

#include <cstring>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <scoped_allocator>
#include <stdint.h>
#include <sys/time.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <atomic>
#include <pthread.h>
#include <linux/sched.h>

#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatfoe.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatprint.h>

class EtherCatError : public std::runtime_error
{
public:
  explicit EtherCatError(const std::string &what)
      : std::runtime_error(what)
  {
  }
};

namespace moons_control
{
namespace ethercat_controller
{

class Controller
{
public:
  Controller(const std::string &ifname);

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

private:
  bool initSoem();
  const std::string ifname_;
  uint8_t iomap_[8192];
  int num_clients_;
  std::thread cycle_thread_;
  mutable std::mutex iomap_mutex_;
  bool stop_flag_;
};

} // namespace ethercat_controller
} // namespace moons_control

#endif