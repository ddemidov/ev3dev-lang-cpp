#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <vector>
#include <sstream>
#include <cstdlib>
#include <ev3dev.h>

namespace ev3 = ev3dev;

void populate_arena(const std::vector<const char*> &devices) {
    std::ostringstream command;
    command << FAKE_SYS "/populate_arena.py";
    for (auto d : devices) command << " " << d;

    system(FAKE_SYS "/clean_arena.py");
    system(command.str().c_str());
}

TEST_CASE( "Device" ) {
    populate_arena({"medium_motor:0@outA", "infrared_sensor:0@in1"});

    ev3::device d;

    SECTION("connect any motor") {
        d.connect(SYS_ROOT "/tacho-motor/", "motor", {});
        REQUIRE(d.connected());
    }

    SECTION("connect specific motor") {
        d.connect(SYS_ROOT "/tacho-motor/", "motor0", {});
        REQUIRE(d.connected());
    }

    SECTION("connect a motor by driver name") {
        d.connect(SYS_ROOT "/tacho-motor/", "motor",
                {{std::string("driver_name"), {std::string("lego-ev3-m-motor")}}});
        REQUIRE(d.connected());
    }

    SECTION("connect a motor by address") {
        d.connect(SYS_ROOT "/tacho-motor/", "motor",
                {{std::string("address"), {ev3::OUTPUT_A}}});
        REQUIRE(d.connected());
    }

    SECTION("invalid driver name") {
        d.connect(SYS_ROOT "/tacho-motor/", "motor",
                {{std::string("driver_name"), {std::string("not-valid")}}});
        REQUIRE(!d.connected());
    }

    SECTION("connect a sensor") {
        d.connect(SYS_ROOT "/lego-sensor/", "sensor", {});
        REQUIRE(d.connected());
    }
}

TEST_CASE("Medium Motor") {
    populate_arena({"medium_motor:0@outA"});

    ev3::medium_motor m;

    REQUIRE(m.connected());
    REQUIRE(m.device_index() == 0);

    SECTION("reading same attribute twice") {
        REQUIRE(m.driver_name() == "lego-ev3-m-motor");
        REQUIRE(m.driver_name() == "lego-ev3-m-motor");
    }

    SECTION("check attribute values") {
        std::set<std::string> commands = {
            "run-forever", "run-to-abs-pos", "run-to-rel-pos", "run-timed",
            "run-direct", "stop", "reset"
            };

        std::set<std::string> state = {"running"};

        REQUIRE(m.count_per_rot() == 360);
        REQUIRE(m.commands()      == commands);
        REQUIRE(m.duty_cycle()    == 0);
        REQUIRE(m.duty_cycle_sp() == 42);
        REQUIRE(m.polarity()      == "normal");
        REQUIRE(m.address()       == "outA");
        REQUIRE(m.position()      == 42);
        REQUIRE(m.position_sp()   == 42);
        REQUIRE(m.ramp_down_sp()  == 0);
        REQUIRE(m.ramp_up_sp()    == 0);
        REQUIRE(m.speed()         == 0);
        REQUIRE(m.speed_sp()      == 0);
        REQUIRE(m.state()         == state);
        REQUIRE(m.stop_action()   == "coast");
        REQUIRE(m.time_sp()       == 1000);
    }
}

TEST_CASE("Infrared Sensor") {
    populate_arena({"infrared_sensor:0@in1"});
    ev3::infrared_sensor s;

    REQUIRE(s.connected());

    REQUIRE(s.device_index()    == 0);
    REQUIRE(s.bin_data_format() == "s8");
    REQUIRE(s.num_values()      == 1);
    REQUIRE(s.address()         == "in1");
    REQUIRE(s.value(0)          == 16);

    std::vector<char> v(1);
    s.bin_data(v.data());

    REQUIRE(v[0] == 16);
    REQUIRE(s.bin_data() == v);
}
