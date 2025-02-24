package api.state;

struct RovStateEnvData[6] {
    uint16 longitude[2] [order = "big"];
    uint16 latitude[2] [order = "big"];
    uint16 altitude[2] [order = "big"];
}

struct RovStateSpeedData[6] {
    uint16 x[2] [order = "big"];
    uint16 y[2] [order = "big"];
    uint16 z[2] [order = "big"];
}

struct RovStatePositionData[6] {
    uint16 x[2] [order = "big"];
    uint16 y[2] [order = "big"];
    uint16 z[2] [order = "big"];
}

struct RovStateManipulatorAData[6] {
    float32 param[4] [order = "big"];
    uint16 id[2] [order = "big"];
}

struct RovStateManipulatorBData[6] {
    float32 param[4] [order = "big"];
    uint16 id[2] [order = "big"];
}
