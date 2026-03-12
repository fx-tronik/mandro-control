#include <ethercat_controller/controller.hpp>

static constexpr unsigned THREAD_SLEEP_TIME = 2000;
static constexpr unsigned EC_TIMEOUTMON = 500;
static constexpr int NSEC_PER_SECOND = 1e+9;

Controller::Controller(ros::NodeHandle &nh, const std::string &ifname)
    : ifname_(ifname),
      nh_(nh)
{
    ROS_INFO_NAMED("ethercat_controller", "Initializing SOEM driver on %s", ifname_.data());
    if (!initSoem())
    {
        ROS_FATAL_NAMED("ethercat_controller", "Did not manage to initialize soem driver, shutting down");
        throw std::exception{}; //TODO: proper exception
    }
    else
    {
        // Send/receive process data once before creating new thread - thread overhead causes connection to timeout sometimes
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);

        // Create cycle thread for PDO communication
        cycle_thread_ = std::thread(&Controller::cycleWorker, this, std::ref(iomap_mutex_), std::ref(stop_flag_));
        ROS_INFO_NAMED("ethercat_controller", "Created cycleWorker thread for PDO communication");

        // Set FIFO scheduler and max priority for ethercat communication
        sched_param thread_param;
        thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO);

        ROS_INFO_NAMED("ethercat_controller", "Setting scheduler and priority of cycleWorker thread");
        if (pthread_setschedparam(cycle_thread_.native_handle(), SCHED_FIFO, &thread_param))
        {
            int errsv = errno;
            ROS_ERROR_NAMED("ethercat_controller", "Couldnt set scheduler and priority for SOEM cycleWorker: %s", std::strerror(errsv));
        }
    }
}

Controller::~Controller()
{
    ROS_INFO_NAMED("ethercat_controller", "EtherCat Controller shutting down");
    stop_flag_ = true;

    // Request init operational state for all slaves
    ROS_INFO_NAMED("ethercat_controller", "Requesting init state for all slaves");
    ec_slave[0].state = EC_STATE_INIT;
    // Change syncmode back to free run
    for (int i = 1; i <= num_clients_; i++)
    {
        ec_dcsync0(i, 0, 4000000, 5000);
    }
    /* request init state for all slaves */
    ec_writestate(0);
    //stop SOEM, close socket
    ec_close();
    cycle_thread_.join();
    ROS_INFO_NAMED("ethercat_controller", "EtherCat Controller deleted");
}

bool Controller::initSoem()
{
    if (!ec_init(ifname_.c_str()))
    {
        ROS_FATAL_NAMED("ethercat_controller", "Could not initialize ethercat driver on %s", ifname_.c_str());
        return false;
    }

    /* find and auto-config slaves */
    if (ec_config_init(FALSE) <= 0)
    {
        ROS_ERROR_NAMED("ethercat_controller", "No slaves found on %s", ifname_.c_str());
        return false;
    }

    ROS_INFO_NAMED("ethercat_controller", "SOEM found %d slaves", ec_slavecount);
    for (int cnt = 1; cnt <= ec_slavecount; cnt++)
    {
        // MINAS-A5B Serial Man = 066Fh, ID = [5/D]****[0/4/8][0-F]*
        ROS_DEBUG_NAMED("ethercat_controller", "Man: %8.8x ID: %8.8x Rev: %8.8x", (int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id, (int)ec_slave[cnt].eep_rev);
        num_clients_++;
    }

    if (ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4) != EC_STATE_PRE_OP)
    {
        ROS_ERROR_NAMED("ethercat_controller", "Could not set EC_STATE_PRE_OP");
        return false;
    }

    for (int cnt = 1; cnt <= ec_slavecount; cnt++)
    {
        ec_slave[cnt].PO2SOconfig = &slave_conf;
    }

    ec_configdc();

    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    // configure IOMap
    int iomap_size = ec_config_map(iomap_);
    ROS_DEBUG_NAMED("ethercat_controller", "SOEM IOMap size: %d", iomap_size);

    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) != EC_STATE_SAFE_OP)
    {
        ROS_ERROR_NAMED("ethercat_controller", "Could not set EC_STATE_SAFE_OP");
        return false;
    }

    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    for (int cnt = 1; cnt <= ec_slavecount; cnt++)
    {

        int ret = 0, l;
        uint16_t sync_mode;
        uint32_t cycle_time;
        uint32_t minimum_cycle_time;
        uint32_t sync0_cycle_time;
        l = sizeof(sync_mode);
        ret += ec_SDOread(cnt, 0x1c32, 0x01, FALSE, &l, &sync_mode, EC_TIMEOUTRXM);
        l = sizeof(cycle_time);
        ret += ec_SDOread(cnt, 0x1c32, 0x02, FALSE, &l, &cycle_time, EC_TIMEOUTRXM);
        l = sizeof(minimum_cycle_time);
        ret += ec_SDOread(cnt, 0x1c32, 0x05, FALSE, &l, &minimum_cycle_time, EC_TIMEOUTRXM);
        l = sizeof(sync0_cycle_time);
        ret += ec_SDOread(cnt, 0x1c32, 0x0a, FALSE, &l, &sync0_cycle_time, EC_TIMEOUTRXM);
        ROS_DEBUG_NAMED("ethercat_controller", "PDO syncmode %02x, cycle time %d ns (min %d), sync0 cycle time %d ns, ret = %d", sync_mode, cycle_time, minimum_cycle_time, sync0_cycle_time, ret);
    }

    ec_writestate(0);
    int chk = 40;
    do
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000); // 50 ms wait for state check
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    if (ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL)
    {
        ROS_ERROR_NAMED("ethercat_controller", "OPERATIONAL state not set, exiting");
        for (int cnt = 1; cnt <= ec_slavecount; cnt++)
        {
            uint16_t code = ec_slave[cnt].ALstatuscode;
            char *res = ec_ALstatuscode2string(code);
            ROS_ERROR_NAMED("ethercat_controller", "Slave %d status: %s", cnt, res);
        }
        return false;
    }

    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    ec_readstate();
    for (int cnt = 1; cnt <= ec_slavecount; cnt++)
    {

       ROS_DEBUG_NAMED("ethercat_controller", "Slave:%d Name:%s Output size: %dbits Input size: %dbits State: %d Delay: %d[ns] Has DC: %d",
               cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
               ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
        if (ec_slave[cnt].hasdc)
        {
            ROS_DEBUG_NAMED("ethercat_controller", " DCParentport:%d", ec_slave[cnt].parentport);
        }
        ROS_DEBUG_NAMED("ethercat_controller", " Activeports:%d.%d.%d.%d", (ec_slave[cnt].activeports & 0x01) > 0,
               (ec_slave[cnt].activeports & 0x02) > 0,
               (ec_slave[cnt].activeports & 0x04) > 0,
               (ec_slave[cnt].activeports & 0x08) > 0);
        ROS_DEBUG_NAMED("ethercat_controller", " Configured address: %4.4x", ec_slave[cnt].configadr);
    }

    ROS_INFO_NAMED("ethercat_controller", "Finished configuration successfully");

    return true;
}

void Controller::cycleWorker(std::mutex &mutex, bool &stop_flag)
{
    // 4ms in nanoseconds
    double period = THREAD_SLEEP_TIME * 1000;
    uint64_t adj_time = 0;
    // get curren ttime
    struct timespec tick;
    clock_gettime(CLOCK_MONOTONIC, &tick);
    timespecInc(tick, period, 0);
    while (!stop_flag)
    {
        int expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
        int sent, wkc;
        {
            std::scoped_lock lock(mutex);
            sent = ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
        }

        if (wkc < expected_wkc)
        {
            ROS_WARN_NAMED("ethercat_controller", "Wrong expected WKC = %d", wkc);
            handleErrors();
        }

        adj_time = ec_sync(ec_DCtime, THREAD_SLEEP_TIME);

        // check overrun
        struct timespec before;
        clock_gettime(CLOCK_MONOTONIC, &before);
        double overrun_time = (before.tv_sec + double(before.tv_nsec) / NSEC_PER_SECOND) - (tick.tv_sec + double(tick.tv_nsec) / NSEC_PER_SECOND);
        if (overrun_time > 0.0)
        {
            ROS_WARN_THROTTLE_NAMED(2, "ethercat_controller", "overrun: %f", overrun_time);
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tick, NULL);
        timespecInc(tick, period, adj_time);
    }
    ROS_INFO_NAMED("ethercat_controller", "cycleWorker finished");
}

int Controller::getNumClients() const
{
    return num_clients_;
}

void Controller::write(int slave_no, uint8_t channel, uint8_t value)
{
    std::scoped_lock lock(iomap_mutex_);
    ec_slave[slave_no].outputs[channel] = value;
}

uint8_t Controller::readInput(int slave_no, uint8_t channel) const
{
    std::scoped_lock lock(iomap_mutex_);
    if (slave_no > ec_slavecount)
    {
        ROS_FATAL_NAMED("ethercat_controller", "ERROR : slave_no(%d) is larger than ec_slavecount(%d)", slave_no, ec_slavecount);
        exit(1);
    }
    if (channel * 8 > ec_slave[slave_no].Ibits)
    {
        //printf("slave number : %d", slave_no);
        ROS_FATAL_NAMED("ethercat_controller", "ERROR : channel(%d) is larget thatn Input bits (%d), channel = %d", channel * 8, ec_slave[slave_no].Ibits, channel);
        exit(1);
    }
    return ec_slave[slave_no].inputs[channel];
}

uint8_t Controller::readOutput(int slave_no, uint8_t channel) const
{
    std::scoped_lock lock(iomap_mutex_);
    if (slave_no > ec_slavecount)
    {
        ROS_FATAL_NAMED("ethercat_controller", "ERROR : slave_no(%d) is larger than ec_slavecount(%d)", slave_no, ec_slavecount);
        exit(1);
    }
    if (channel * 8 > ec_slave[slave_no].Obits)
    {
        ROS_FATAL_NAMED("ethercat_controller", "ERROR : channel(%d) is larget thatn Output bits (%d)", channel * 8, ec_slave[slave_no].Obits);
        exit(1);
    }
    return ec_slave[slave_no].outputs[channel];
}

template <typename T>
uint8_t Controller::writeSDO(int slave_no, uint16_t index, uint8_t subidx, T value) const
{
    int ret;
    ret = ec_SDOwrite(slave_no, index, subidx, FALSE, sizeof(value), &value, EC_TIMEOUTSAFE);
    return ret;
}

template <typename T>
T Controller::readSDO(int slave_no, uint16_t index, uint8_t subidx) const
{
    int ret, l;
    T val;
    l = sizeof(val);
    ret = ec_SDOread(slave_no, index, subidx, FALSE, &l, &val, EC_TIMEOUTRXM);
    if (ret <= 0)
    { // ret = Workcounter from last slave response
        ROS_FATAL_NAMED("ethercat_controller", "Failed to read from ret:%d, slave_no:%d, index:0x%04x, subidx:0x%02x", ret, slave_no, index, subidx);
    }
    return val;
}

void Controller::timespecInc(struct timespec &tick, int nsec, int64 adj_time)
{
    tick.tv_nsec += nsec;
    tick.tv_nsec += adj_time;
    while (tick.tv_nsec >= NSEC_PER_SECOND)
    {
        tick.tv_nsec -= NSEC_PER_SECOND;
        tick.tv_sec++;
    }
}

void Controller::handleErrors()
{
    /* one ore more slaves are not responding */
    ec_group[0].docheckstate = FALSE;
    ec_readstate();
    for (int slave = 1; slave <= ec_slavecount; slave++)
    {

        uint16 code = ec_slave[slave].ALstatuscode;
        char *res = ec_ALstatuscode2string(code);
        if ((ec_slave[slave].group == 0) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
        {
            ec_group[0].docheckstate = TRUE;
            if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
            {
                ROS_WARN_NAMED("ethercat_controller", "ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.", slave);
                ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                ec_writestate(slave);
            }
            else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
            {
                ROS_WARN_NAMED("ethercat_controller", "WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.", slave);
                ec_slave[slave].state = EC_STATE_OPERATIONAL;
                ec_writestate(slave);
            }
            else if (ec_slave[slave].state > 0)
            {
                if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                {
                    ec_slave[slave].islost = FALSE;
                    ROS_INFO_NAMED("ethercat_controller", "MESSAGE : slave %d reconfigured", slave);
                }
            }
            else if (!ec_slave[slave].islost)
            {
                /* re-check state */
                ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                if (!ec_slave[slave].state)
                {
                    ec_slave[slave].islost = TRUE;
                    ROS_ERROR_NAMED("ethercat_controller", "ERROR : slave %d lost", slave);
                }
            }
        }
        if (ec_slave[slave].islost)
        {
            if (!ec_slave[slave].state)
            {
                if (ec_recover_slave(slave, EC_TIMEOUTMON))
                {
                    ec_slave[slave].islost = FALSE;
                    ROS_INFO_NAMED("ethercat_controller", "MESSAGE : slave %d recovered", slave);
                }
            }
            else
            {
                ec_slave[slave].islost = FALSE;
                ROS_INFO_NAMED("ethercat_controller", "MESSAGE : slave %d found", slave);
            }
        }
    }
}

int64 Controller::ec_sync(int64 reftime, int64 cycletime)
{
    int64 adj_time = 0;
    int64 sync_error;
    /* set linux sync point 50us later than DC sync, just as example */
    sync_error = (reftime - 50000) % cycletime;
    if (sync_error > (cycletime / 2))
    {
        sync_error = sync_error - cycletime;
    }
    if (sync_error > 0)
    {
        integral++;
    }
    if (sync_error < 0)
    {
        integral--;
    }
    adj_time = -(sync_error / 100) - (integral / 20);
    return adj_time;
}

// Slave configuration function - PDO, Sync Mode
int Controller::slave_conf_common(uint16 cnt)
{
    {
        // RXPDO
        int ret = 0, l;
        uint8_t num_entries;
        l = sizeof(num_entries);
        ret += ec_SDOread(cnt, 0x1601, 0x00, FALSE, &l, &num_entries, EC_TIMEOUTRXM);
        num_entries = 0;
        ret += ec_SDOwrite(cnt, 0x1601, 0x00, FALSE, sizeof(num_entries), &num_entries, EC_TIMEOUTRXM);
        uint32_t mapping;
        mapping = 0x60400010; // Controlword 6040
        ret += ec_SDOwrite(cnt, 0x1601, 0x01, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
        mapping = 0x60600008; // Mode of operation 6060
        ret += ec_SDOwrite(cnt, 0x1601, 0x02, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
        mapping = 0x607A0020; // Target Position 607A
        ret += ec_SDOwrite(cnt, 0x1601, 0x03, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
        mapping = 0x60FF0020; // Target Velocity 60FF
        ret += ec_SDOwrite(cnt, 0x1601, 0x04, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);

        num_entries = 4;
        ret += ec_SDOwrite(cnt, 0x1601, 0x00, FALSE, sizeof(num_entries), &num_entries, EC_TIMEOUTRXM);
        ret += ec_SDOread(cnt, 0x1601, 0x00, FALSE, &l, &num_entries, EC_TIMEOUTRXM);
    }

    {
        // TXPDO
        int ret = 0, l;
        uint8_t num_entries;
        l = sizeof(num_entries);
        ret += ec_SDOread(cnt, 0x1a01, 0x00, FALSE, &l, &num_entries, EC_TIMEOUTRXM);
        num_entries = 0;
        ret += ec_SDOwrite(cnt, 0x1a01, 0x00, FALSE, sizeof(num_entries), &num_entries, EC_TIMEOUTRXM);

        uint32_t mapping;
        mapping = 0x603F0010; // Error code 603F
        ret += ec_SDOwrite(cnt, 0x1a01, 0x01, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
        mapping = 0x60410010; // Statusword 6041
        ret += ec_SDOwrite(cnt, 0x1a01, 0x02, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
        mapping = 0x60610008; // Modes of operation display 6061
        ret += ec_SDOwrite(cnt, 0x1a01, 0x03, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
        mapping = 0x60640020; // Position actual value 6064
        ret += ec_SDOwrite(cnt, 0x1a01, 0x04, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
        mapping = 0x60F40020; // Follow Error Actual Value 60F4
        ret += ec_SDOwrite(cnt, 0x1a01, 0x05, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
        mapping = 0x606c0020; // Velocity Actual Value 606C
        ret += ec_SDOwrite(cnt, 0x1a01, 0x06, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
        mapping = 0x60770010; // Torque Actual Value 6077
        ret += ec_SDOwrite(cnt, 0x1a01, 0x07, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
        mapping = 0x200F0020; // Alarm Code 200F
        ret += ec_SDOwrite(cnt, 0x1a01, 0x08, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
        mapping = 0x60780010; // Current actual value 6078
        ret += ec_SDOwrite(cnt, 0x1a01, 0x09, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);

        num_entries = 9;
        ret += ec_SDOwrite(cnt, 0x1a01, 0x00, FALSE, sizeof(num_entries), &num_entries, EC_TIMEOUTRXM);
        ret += ec_SDOread(cnt, 0x1a01, 0x00, FALSE, &l, &num_entries, EC_TIMEOUTRXM);
    }
    {
        // SM2 & SM3
        int ret = 0, l;
        uint8_t num_pdo;
        // set 0 change PDO mapping index
        num_pdo = 0;
        ret += ec_SDOwrite(cnt, 0x1c12, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
        // set to default PDO mapping 4
        uint16_t idx_rxpdo = 0x1601;
        ret += ec_SDOwrite(cnt, 0x1c12, 0x01, FALSE, sizeof(idx_rxpdo), &idx_rxpdo, EC_TIMEOUTRXM);
        // set number of assigned PDOs
        num_pdo = 1;
        ret += ec_SDOwrite(cnt, 0x1c12, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
        ROS_DEBUG_NAMED("ethercat_controller", "RxPDO mapping object index %d = %04x ret=%d", cnt, idx_rxpdo, ret);

        // set 0 change PDO mapping index
        num_pdo = 0;
        ret += ec_SDOwrite(cnt, 0x1c13, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
        // set to default PDO mapping 4
        uint16_t idx_txpdo = 0x1a01;
        ret += ec_SDOwrite(cnt, 0x1c13, 0x01, FALSE, sizeof(idx_txpdo), &idx_txpdo, EC_TIMEOUTRXM);
        // set number of assigned PDOs
        num_pdo = 1;
        ret += ec_SDOwrite(cnt, 0x1c13, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
        ROS_DEBUG_NAMED("ethercat_controller", "TxPDO mapping object index %d = %04x ret=%d", cnt, idx_txpdo, ret);
    }
    // Sync mode - DC mode (synchronous with SYNC0/1 events)
    // https://infosys.beckhoff.com/content/1033/ethercatsystem/2469122443.html
    //ec_dcsync01(cnt, 1, 1000000, 1000000, 5000);
    ec_dcsync0(cnt, 1, 2000000, 5000);
    return 0;
}

int Controller::slave_conf_homing(uint16 cnt, uint8_t homing_method, uint32_t homing_speed, uint32_t acceleration, uint32_t home_offset)
{
    // Homing params
    // https://www.moons.com.cn/medias/EtherCAT-User-Manual-for-M2-EtherCAT.pdf, pages 24-40
    //TODO: Separate different methods and params for different drives? We could identify them with an alias in eeprom.

    int ret = 0;
    ret += ec_SDOwrite(cnt, 0x6099, 0x01, FALSE, sizeof(homing_speed), &homing_speed, EC_TIMEOUTRXM);
    ret += ec_SDOwrite(cnt, 0x6099, 0x02, FALSE, sizeof(homing_speed), &homing_speed, EC_TIMEOUTRXM);
    ret += ec_SDOwrite(cnt, 0x609A, 0x00, FALSE, sizeof(acceleration), &acceleration, EC_TIMEOUTRXM);
    ret += ec_SDOwrite(cnt, 0x607C, 0x00, FALSE, sizeof(home_offset), &home_offset, EC_TIMEOUTRXM);
    ret += ec_SDOwrite(cnt, 0x6098, 0x00, FALSE, sizeof(homing_method), &homing_method, EC_TIMEOUTRXM);

    return 0;
}

int slave_conf_profile_position(uint16 cnt, uint32_t position, uint32_t velocity, uint32_t acceleration)
{
    int ret = 0;
    ret += ec_SDOwrite(cnt, 0x607A, 0x00, FALSE, sizeof(position), &position, EC_TIMEOUTRXM);
    ret += ec_SDOwrite(cnt, 0x6081, 0x00, FALSE, sizeof(velocity), &velocity, EC_TIMEOUTRXM);
    ret += ec_SDOwrite(cnt, 0x6083, 0x00, FALSE, sizeof(acceleration), &acceleration, EC_TIMEOUTRXM);
    ret += ec_SDOwrite(cnt, 0x6084, 0x00, FALSE, sizeof(acceleration), &acceleration, EC_TIMEOUTRXM);

    return 0;
}

int Controller::slave_conf(uint16 cnt)
{
    // Apply drive specific settings
    switch (cnt)
    {
    case 1:
        slave_conf_homing(cnt, 0x12, 5000, 100000, 364851); //oś X, bazowanie w prawo patrząc od strony korytarza, tj. z tyłu robota
        slave_conf_profile_position(cnt, 0, 5000, 100000);
        break;
    case 2:
        slave_conf_homing(cnt, 18, 10000, 100000, 0); //oś Z, bazowanie w górę
        slave_conf_profile_position(cnt, 0, 10000, 200000);
        break;
    case 3:
        slave_conf_homing(cnt, 18, 5000, 100000, 204200); //oś C1, duża obrotowa, bazowanie przeciwnie do obrotu ws00kazówek zegara patrząc od góry, tj. w lewo
        slave_conf_profile_position(cnt, 0, 5000, 100000);
        break;
    case 4:
        slave_conf_homing(cnt, 18, 5000, 100000, 12500); //oś C2, mała obrotowa, bazowanie zgodnie z obrotem wskazówek zegera patrząc od góry, tj. w prawo
        slave_conf_profile_position(cnt, 0, 5000, 100000);
        break;
    case 5:
        slave_conf_profile_position(cnt, 0, 2500, 50000);
        slave_conf_homing(cnt, 17, 2500, 100000, -22000); //oś A, pochylenie chwytaka, obraca się w stronę jedynej krańcówki
        break;
    case 6:
        slave_conf_homing(cnt, 18, 2500, 100000, 0); //oś Z, łapki, bazowanie w stronę jedynej dostępnej krańcówki
        slave_conf_profile_position(cnt, 0, 2500, 50000);
        break;
    }

    // Apply common settings (PDO, sync mode)
    slave_conf_common(cnt);

    return 0;
}

void Controller::getStatus(int slave_no, std::string &name, int &eep_man, int &eep_id, int &eep_rev, int &obits, int &ibits, int &state, int &pdelay, int &hasdc, int &activeports, int &configadr) const
{
    if (slave_no > ec_slavecount)
    {
        ROS_FATAL_NAMED("ethercat_controller", "ERROR : slave_no(%d) is larger than ec_slavecount(%d)", slave_no, ec_slavecount);
        exit(1);
    }
    name = static_cast<std::string>(ec_slave[slave_no].name);
    eep_man = static_cast<int>(ec_slave[slave_no].eep_man);
    eep_id = static_cast<int>(ec_slave[slave_no].eep_id);
    eep_rev = static_cast<int>(ec_slave[slave_no].eep_rev);
    obits = ec_slave[slave_no].Obits;
    ibits = ec_slave[slave_no].Ibits;
    state = ec_slave[slave_no].state;
    pdelay = ec_slave[slave_no].pdelay;
    hasdc = ec_slave[slave_no].hasdc;
    activeports = ec_slave[slave_no].activeports;
    configadr = ec_slave[slave_no].configadr;
}

void Controller::getMan(int slave_no, int &eep_man, int &eep_id, int &eep_rev) const
{
    if (slave_no > ec_slavecount)
    {
        ROS_FATAL_NAMED("ethercat_controller", "ERROR : slave_no(%d) is larger than ec_slavecount(%d)", slave_no, ec_slavecount);
        exit(1);
    }
    eep_man = static_cast<int>(ec_slave[slave_no].eep_man);
    eep_id = static_cast<int>(ec_slave[slave_no].eep_id);
    eep_rev = static_cast<int>(ec_slave[slave_no].eep_rev);
}

template uint8_t Controller::writeSDO<char>(int slave_no, uint16_t index, uint8_t subidx, char value) const;
template uint8_t Controller::writeSDO<int>(int slave_no, uint16_t index, uint8_t subidx, int value) const;
template uint8_t Controller::writeSDO<short>(int slave_no, uint16_t index, uint8_t subidx, short value) const;
template uint8_t Controller::writeSDO<long>(int slave_no, uint16_t index, uint8_t subidx, long value) const;
template uint8_t Controller::writeSDO<unsigned char>(int slave_no, uint16_t index, uint8_t subidx, unsigned char value) const;
template uint8_t Controller::writeSDO<unsigned int>(int slave_no, uint16_t index, uint8_t subidx, unsigned int value) const;
template uint8_t Controller::writeSDO<unsigned short>(int slave_no, uint16_t index, uint8_t subidx, unsigned short value) const;
template uint8_t Controller::writeSDO<unsigned long>(int slave_no, uint16_t index, uint8_t subidx, unsigned long value) const;

template char Controller::readSDO<char>(int slave_no, uint16_t index, uint8_t subidx) const;
template int Controller::readSDO<int>(int slave_no, uint16_t index, uint8_t subidx) const;
template short Controller::readSDO<short>(int slave_no, uint16_t index, uint8_t subidx) const;
template long Controller::readSDO<long>(int slave_no, uint16_t index, uint8_t subidx) const;
template unsigned char Controller::readSDO<unsigned char>(int slave_no, uint16_t index, uint8_t subidx) const;
template unsigned int Controller::readSDO<unsigned int>(int slave_no, uint16_t index, uint8_t subidx) const;
template unsigned short Controller::readSDO<unsigned short>(int slave_no, uint16_t index, uint8_t subidx) const;
template unsigned long Controller::readSDO<unsigned long>(int slave_no, uint16_t index, uint8_t subidx) const;