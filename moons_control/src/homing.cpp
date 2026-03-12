#include <moons_control/ecat_controller.hpp>
#include <moons_control/servo_client.hpp>
#include <vector>
#include <future>

using moons_control::ethercat_controller::Controller;
using moons_control::servo::ServoClient;
using moons_control::servo::ServoInput;
using moons_control::servo::ServoOutput;

typedef std::vector<ServoClient *> ServoClientContainer;

int main(int argc, char **argv)
{
    Controller *manager = nullptr;

    try
    {
         manager = new Controller(argv[1]);
    }
    catch (const std::exception &e)
    {

        std::cerr << e.what() << '\n';
        return 0;
    }

    ServoClientContainer servos;
    
    auto clients = manager->getNumClients();

    for (int i = 1; i <= clients; i++)
    {
        ServoOutput output;
        memset(&output, 0x00, sizeof(ServoOutput));
        ServoClient *servo = new ServoClient(*manager, i, output);
        servo->reset();
        if (!i==5)
            servo->servoOn();
        servos.push_back(servo);
    }

    std::cout << "Press ENTER to start homing axis A" << std::endl;
    std::cin.ignore();
    servos[4]->home();
    std::cout << "Press ENTER to return axis A to position 0" << std::endl;
    std::cin.ignore();
    servos[4]->PPGoTo(0);
    std::cout << "Press ENTER to start homing axis Z" << std::endl;
    std::cin.ignore();
    servos[1]->home();
    std::cout << "Press ENTER to start homing axis C1" << std::endl;
    std::cin.ignore();
    servos[2]->home();
    std::cout << "Press ENTER to start homing axis X" << std::endl;
    std::cin.ignore();
    servos[0]->home();
    std::cout << "Press ENTER to start homing axis C2" << std::endl;
    std::cin.ignore();
    servos[3]->home();
    std::cout << "Press ENTER to return axis C2 to position 0" << std::endl;
    std::cin.ignore();
    servos[3]->PPGoTo(0);
    std::cout << "Press ENTER to start homing axis V" << std::endl;
    std::cin.ignore();
    servos[5]->servoOn();
    servos[4]->servoOff();
    servos[5]->home();

    std::cout << "Press ENTER to lower axis Z" << std::endl;
    std::cin.ignore();
    servos[1]->PPGoTo(-3000);
    std::cout << "Press ENTER to move axis X towards position 0" << std::endl;
    std::cin.ignore();
    servos[0]->PPGoTo(320000);
    std::cout << "Press ENTER to move axis C1 towards position 0" << std::endl;
    std::cin.ignore();
    servos[2]->PPGoTo(180000);
    std::cout << "Press ENTER to move axis V towards middle" << std::endl;
    std::cin.ignore();
    servos[5]->PPGoTo(30000);

    std::cout << "Homing done, shutting down servos" << std::endl;

    for (ServoClient * servo : servos)
    {
        servo->servoOff();
        delete(servo);
    }

    delete (manager);

    while (true)
    {
        osal_usleep(1e3);
    }
}