#include "motor.h"
#include <cassert>
#include <iostream>
#include <thread>
#include <chrono>

using namespace obot;

void test_mode_color() {
    assert(mode_color(ModeDesired::OPEN) == "azure");
    assert(mode_color((ModeDesired) 200) == "red");
    assert(mode_color(ModeDesired::DRIVER_DISABLE) == "white");
}

int main() {
    test_mode_color();
    return 0;
}