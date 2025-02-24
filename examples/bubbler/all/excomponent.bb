package api.excomponent;

struct RovExcompPeripheralData[6] {
    bool lazer_en[1];
    bool grip_en[1];
    bool manip_en[1];
    bool sonar_en[1];
    bool propeller_en[1];
    bool ptz_en[1];
}

struct RovExcompWorkloadData[6] {
    bool worker[1];
    bool servo_a[1];
    bool servo_b[1];
    bool servo_c[1];
    bool servo_d[1];
    bool mode[1];
}
