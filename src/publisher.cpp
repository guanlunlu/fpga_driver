#include "brlos/NodeHandler.hpp"
// #include "msg/Control_msg.hpp"
// #include "msg/Geometry_msg.hpp"
#include <brlos/parameters_parser.hpp>
#include <msg.hpp>
#include <unistd.h>
#include "boost/bind.hpp"
#include "boost/thread.hpp"

std::string local_ip = "169.254.254.89";
std::string master_ip = "169.254.105.68";
uint32_t master_port = 8888;

int main(int argc, char* argv[])
{
    NodeHandler nh;
    // Subscriber motor_sub = nh.subscriber("/motor_feedback");
    Publisher cmd_pub = nh.publisher("/fpga_cmd");
    boost::thread node_service(boost::bind(&boost::asio::io_service::run, &node_ios));

    while (1)
    {
        FpgaCmdMsg msg;
        msg.digital_on_ = true;
        msg.power_on_ = true;
        msg.signal_on_ = true;
        msg.stop_ = true;

        cmd_pub.publish(msg);
        usleep(1000 * 1000);
        std::cout << "pub\n";
    }
    return 0;
}
