//
// Created by Hugo Trippaers on 16/04/2025.
//

#ifndef SERVOTEST_H
#define SERVOTEST_H

#include <mutex>
#include <gz/transport.hh>
#include <gz/msgs.hh>

class ServoTest {
  public:
    ServoTest() = default;
    ~ServoTest() = default;

    int init();
    int run();
    void shutdown();

  private:
    std::mutex _tick_mutex;
    std::condition_variable _tick;

    gz::transport::Node _node;

    volatile bool terminate = false;
    uint64_t _time_us{};

    void clockCallback(const gz::msgs::Clock &clock);
    void jointStateCallback(const gz::msgs::Model &model);
};

#endif //SERVOTEST_H
