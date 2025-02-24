package api.excontrol;

struct RovExcontrolManipData[6] {
    int16 grip[2] [order = "big"];
    int16 wrist[2] [order = "big"] {
        get percent(float64): value / 3000.0 * 100.0;
        get percent_int(int32): (int32)(value / 3000.0 * 100.0);
        set percent(float64): value > -0.5 && value < 0.5 ? (float64)0 : value;
    };
    int16 hold[2] [order = "big"];
}

struct RovExcontrolManipulatorAData[6] {
    uint16 m1[2] [order = "big"];
    uint16 m2[2] [order = "big"];
    uint16 m3[2] [order = "big"];
}
