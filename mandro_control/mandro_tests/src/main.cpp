#include <ethercat_controller/controller.hpp>
#include <moons_servo/client.hpp>
#include <vector>

typedef std::vector<MoonsServo *> ServoClientContainer;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "main_loop");
    ros::NodeHandle node;

    Controller *manager = new Controller(node, "enp0s31f6");
    MoonsServo *servo = new MoonsServo(node, *manager, 1);

    ros::ServiceClient client = node.serviceClient<moons_servo::reset>("reset_");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    //moons_servo::reset srv;
    servo->Reset();
    servo->ServoOn();
    //srv.request.reset = true;
    //std::cout <<"calling service \n";
    //if(client.call(srv))
    //    std::cout << "service result: " << (bool(srv.response.success) ? "success" : "failure") <<" response: " << srv.response.message << std::endl;
    while(ros::ok()){};

    return 0;

}
