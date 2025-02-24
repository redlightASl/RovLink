package api.monitor;

struct RovMonitorPropellerAData[6] {
    uint16 p1[2] [order = "big"];
    uint16 p2[2] [order = "big"];
    uint16 p3[2] [order = "big"];
}

struct RovMonitorLightAData[6] {
    uint16 l1[2] [order = "big"];
    uint16 l2[2] [order = "big"];
    uint16 l3[2] [order = "big"];
}

struct RovMonitorPtzData[6] {
    uint16 roll[2] [order = "big"];
    uint16 tilt[2] [order = "big"];
    uint16 pan[2] [order = "big"];
}

struct RovMonitorServoAData[6] {
    uint16 s1[2] [order = "big"];
    uint16 s2[2] [order = "big"];
    uint16 s3[2] [order = "big"];
}