#include "ros/ros.h"
#include "std_msgs/String.h"
#include "styx_msgs/Lane.h"
#include <thread>
#include <uWS/uWS.h>
#include "json.hpp"
#include <vector>

using namespace std;

// for convenience
using json = nlohmann::json;

// global variables
uWS::Hub gh;
thread gh_thread;
vector<double> x;
vector<double> y;

void StartGraphThread() {
    gh.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Graph Hub Connected!!!" << std::endl;

        json msgJson;

        msgJson["type"] = "worldmap";
        msgJson["worldmap_x"] = x;
        msgJson["worldmap_y"] = y;

        auto msg = msgJson.dump();
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        cout << "Worldmap sent" << endl;
    });


    gh.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        std::cout << "Graph Hub Disconnected" << std::endl;
    });

    if (gh.listen("0.0.0.0", 4568)) {
        cout << "Listening on port 4568" << endl;

        gh_thread = thread([](){
            gh.run();
        });
    } else {
        cerr << "Could not start graph listener" << endl;
    }
}


void wayback_callback(const styx_msgs::Lane::ConstPtr &msg) {
    ROS_INFO("Callback has been called\n");

    for (auto wp : msg->waypoints) {
        auto pos = wp.pose.pose.position;
        x.push_back(pos.x);
        y.push_back(pos.y);
    }

    cout << "/base_waypoints received. Total: " << msg->waypoints.size() << endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wp_server");

    ros::NodeHandle n;

    ros::Subscriber s = n.subscribe("/base_waypoints", 1000, wayback_callback);

    ROS_INFO("Starting ws_server\n");
    
    StartGraphThread();
    ros::spin();

    return 0;
}