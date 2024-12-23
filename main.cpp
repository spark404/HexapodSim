#include <iostream>
#include <thread>
#include <csignal>

#include "Controller.h"
#include "Robot.h"

std::mutex terminate;
std::condition_variable terminate_lock;
std::unique_lock<std::mutex> lk(terminate);

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

    auto *instance = new Controller(r);
    if (!instance->init()) {
        std::cerr << "Init failed" << std::endl;
        return 1;
    }

    std::thread t1(&Controller::run, instance);

    terminate_lock.wait(lk);
    instance->shutdown();
    t1.join();

    return 0;
}
