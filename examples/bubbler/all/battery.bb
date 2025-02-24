package api.battery;

struct RovBatVoltageData[6] {
    uint16 voltage[2] [order = "big"];
    uint16 power[2] [order = "big"];
    uint16 discharge[2] [order = "big"];
}

struct RovBatVoltageCalibData[6] {
    float32 param[4] [order = "big"];
    uint16 group[2] [order = "big"];
}

struct RovBatCurrentCalibData[6] {
    float32 param[4] [order = "big"];
    uint16 group[2] [order = "big"];
}

struct RovBatCurrentData[6] {
    uint16 sample1[2] [order = "big"];
    uint16 sample2[2] [order = "big"];
    uint16 sample3[2] [order = "big"];
}

struct RovBatSOCData[6] {
    uint16 soc[2] [order = "big"];
    uint16 pu_time[2] [order = "big"];
    uint16 pd_time[2] [order = "big"];
}

struct RovBatPackSOCAData[6] {
    bool a1[1];
    bool a2[1];
    bool a3[1];
    bool a4[1];
    bool a5[1];
    bool a6[1];
}

struct RovBatPackSOCBData[6] {
    bool b1[1];
    bool b2[1];
    bool b3[1];
    bool b4[1];
    bool b5[1];
    bool b6[1];
}
