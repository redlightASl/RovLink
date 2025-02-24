package api.control;

struct RovControlPropellerAData[6] {
    uint16 p1[2] [order = "big"];
    uint16 p2[2] [order = "big"];
    uint16 p3[2] [order = "big"];
}

struct RovControlLightAData[6] {
    uint16 l1[2] [order = "big"];
    uint16 l2[2] [order = "big"];
    uint16 l3[2] [order = "big"];
}

struct RovControlPtzData[6] {
    uint16 roll[2] [order = "big"];
    uint16 tilt[2] [order = "big"];
    uint16 pan[2] [order = "big"];
}

struct RovControlServoAData[6] {
    uint16 s1[2] [order = "big"];
    uint16 s2[2] [order = "big"];
    uint16 s3[2] [order = "big"];
}

struct RovControlPostureData[6] {
    uint16 forward[2] [order = "big"];
    uint16 rotate[2] [order = "big"];
    uint16 vertical[2] [order = "big"];
}

struct RovControlMovementData[6] {
    uint16 speed_x[2] [order = "big"];
    uint16 speed_y[2] [order = "big"];
    uint16 speed_z[2] [order = "big"];
}

struct RovControlHeadingData[6] {
    uint16 pitch[2] [order = "big"];
    uint16 roll[2] [order = "big"];
    uint16 yaw[2] [order = "big"];
}
