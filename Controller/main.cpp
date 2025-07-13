#include <iostream>
#include <thread>
#include <csignal>

#include "Controller.h"
#include "Robot.h"
#include "ServoTest.h"
#include "Standup.h"
#include "hexapodmath/conversion_2d.h"
#include "hexapodmath/forward_kinematics.h"
#include "hexapodmath/inverse_kinematics.h"
#include "hexapodmath/matrix_3d.h"

std::mutex terminate;
std::condition_variable terminate_lock;
std::unique_lock<std::mutex> lk(terminate);

int createModel();
int removeModel();
int pauze(bool pause);
int follow();

void SignalHandler(int nSignalNumber)
{
    std::cout << "received signal " << nSignalNumber << std::endl;
    terminate_lock.notify_one();
}

int main() {
    std::cout << "Starting Hexapod Controller" << std::endl;

    struct sigaction SignalAction{};
    memset(&SignalAction, 0, sizeof(SignalAction));
    SignalAction.sa_handler = SignalHandler;
    sigaction(SIGTERM, &SignalAction, nullptr);
    sigaction(SIGINT, &SignalAction, nullptr);

    if (!pauze(false)) {
        std::cerr << "Failed to start the simulation" << std::endl;
        return 1;
    }

    if (!createModel()) {
        std::cerr << "Model create failed" << std::endl;
        return 1;
    }

    sleep(1);

    if (!follow()) {
        std::cerr << "Follow failed" << std::endl;
        return 1;
    }

    auto *instance = new Controller(r);
    //auto *instance = new ServoTest();
    if (!instance->init()) {
        std::cerr << "Init failed" << std::endl;
        return 1;
    }

    std::thread t1(&Controller::run, instance);

    terminate_lock.wait(lk);
    instance->shutdown();
    t1.join();

    if (!removeModel()) {
        std::cerr << "Model create failed" << std::endl;
        return 1;
    }

    if (!pauze(true)) {
        std::cerr << "Failed to pauze the simulation" << std::endl;
        return 1;
    }

    return 0;
}

int createModel() {
    gz::transport::Node node;
    gz::msgs::EntityFactory req{};
    gz::msgs::Boolean res;
    bool result;

    req.set_sdf_filename("/models/hexspider/hexspider.sdf");
    req.set_name("hexspider"); // New name for the entity, overrides the name on the SDF.
    req.set_allow_renaming(false); // allowed to rename the entity in case of overlap with existing entities


    bool executed = node.Request("/world/hexspider_world/create", req, 1000, res, result);
    if (!executed) {
        std::cerr << "Timeout while calling create service" << std::endl;
        return 0;
    }

    if (!result) {
        std::cerr << "Service call failed" << std::endl;
        return 0;
    }

    std::cout << "Model created" << std::endl;
    return 1;
}

int removeModel() {
    gz::transport::Node node;
    gz::msgs::Entity req{};
    gz::msgs::Boolean res;
    bool result;

    req.set_name("hexspider");
    req.set_type(gz::msgs::Entity_Type_MODEL);

    bool executed = node.Request("/world/hexspider_world/remove", req, 1000, res, result);
    if (!executed) {
        std::cerr << "Timeout while calling create service" << std::endl;
        return 0;
    }

    if (!result) {
        std::cerr << "Service call failed" << std::endl;
        return 0;
    }

    std::cout << "Model removed" << std::endl;
    return 1;
}

int pauze(bool pause) {
    gz::transport::Node node;
    gz::msgs::WorldControl req{};
    gz::msgs::Boolean res;
    bool result;

    req.set_pause(pause);

    bool executed = node.Request("/world/hexspider_world/control", req, 1000, res, result);
    if (!executed) {
        std::cerr << "Timeout while calling create service" << std::endl;
        return 0;
    }

    if (!result) {
        std::cerr << "Service call failed" << std::endl;
        return 0;
    }

    std::cout << "Pause set to " << pause << "." << std::endl;
    return 1;
}

int follow() {
    gz::transport::Node node;
    gz::msgs::Vector3d offset;
    gz::msgs::StringMsg target;
    gz::msgs::Boolean res;

    bool result;

    target.set_data("hexspider");
    offset.set_x(0.0f);
    offset.set_y(-0.50f);
    offset.set_z(0.30f);

    bool executed = node.Request("/gui/follow/offset", offset, 1000, res, result);
    if (!executed) {
        std::cerr << "Timeout while calling follow service" << std::endl;
        return 0;
    }

    if (!result) {
        std::cerr << "Service call failed" << std::endl;
        return 0;
    }

    executed = node.Request("/gui/follow", target, 1000, res, result);
    if (!executed) {
        std::cerr << "Timeout while calling follow service" << std::endl;
        return 0;
    }

    if (!result) {
        std::cerr << "Service call failed" << std::endl;
        return 0;
    }


    return 1;
}

