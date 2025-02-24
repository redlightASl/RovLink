package api.sensor;

struct RovSensorCabTempHumidPressData[6] {
    uint16 temperature[2] [order = "big"] {
        get temperature_quat(float32): value / 100.0;
        set temperature_quat(float32): (uint16)(value * 100.0);
    };
    uint16 humidity[2] [order = "big"] {
        get humidity_quat(float32): value / 100.0;
        set humidity_quat(float32): (uint16)(value * 100.0);
    };
    uint16 pressure[2] [order = "big"] {
        get pressure_quat(float32): value / 100.0;
        set pressure_quat(float32): (uint16)(value * 100.0);
    };
}

struct RovSensorCabinTempHumiPressData[6] {
    uint16 temperature[2] [order = "big"] {
        get temperature_quat(float64): value / 100.0;
        set temperature_quat(float64): (uint16)(value * 100.0);
    };
    uint16 humidity[2] [order = "big"] {
        get trustable(bool): (value / 100.0) < 1.0 ? false : true;
        get confidence(int32): (value / 100.0) < 1.0 ? 0 : 100;
        get humidity_quat(float64): value / 100.0;
        set humidity_quat(float64): (uint16)(value * 100.0);
    };
    uint16 pressure[2] [order = "big"] {
        get pressure_quat(float64): value / 100.0;
        set pressure_quat(float64): (uint16)(value * 100.0);
    };
}

struct RovSensorWaterTempDepthPressData[6] {
    uint16 temperature[2] [order = "big"] {
        get temperature_quat(float32): value / 100.0;
        set temperature_quat(float32): (uint16)(value * 100.0);
    };
    uint16 depth[2] [order = "big"] {
        get trustable(bool): (value / 100.0) < 1.0 ? false : true;
        get confidence(int32): (value / 100.0) < 1.0 ? (int32)0 : (int32)100;
        get depth_quat(float32): value / 100.0;
        set depth_quat(float32): (uint16)(value * 100.0);
    };
    uint16 pressure[2] [order = "big"] {
        get pressure_quat(float32): value / 100.0;
        set pressure_quat(float32): (uint16)(value * 100.0);
    };
}

struct RovSensorAccelerationData[6] {
    uint16 x[2] [order = "big"] {
        get x_quat(float32): value / 32768.0 * 16.0 * 9.8;
        set x_quat(float32): (uint16)(value / 16.0 / 9.8 * 32768.0);
    };
    uint16 y[2] [order = "big"] {
        get y_quat(float32): value / 32768.0 * 16.0 * 9.8;
        set y_quat(float32): (uint16)(value / 16.0 / 9.8 * 32768.0);
    };
    uint16 z[2] [order = "big"] {
        get z_quat(float32): value / 32768.0 * 16.0 * 9.8;
        set z_quat(float32): (uint16)(value / 16.0 / 9.8 * 32768.0);
    };
}

struct RovSensorAngularVelocityData[6] {
    uint16 x[2] [order = "big"] {
        get x_quat(float32): value / 32768.0 * 2000.0;
        set x_quat(float32): (uint16)(value / 2000.0 * 32768.0);
    };
    uint16 y[2] [order = "big"] {
        get y_quat(float32): value / 32768.0 * 2000.0;
        set y_quat(float32): (uint16)(value / 2000.0 * 32768.0);
    };
    uint16 z[2] [order = "big"] {
        get z_quat(float32): value / 32768.0 * 2000.0;
        set z_quat(float32): (uint16)(value / 2000.0 * 32768.0);
    };
}

struct RovSensorMagneticData[6] {
    uint16 x[2] [order = "big"] {
        get x_quat(float32): value / 32768.0 * 4912.0;
        set x_quat(float32): (uint16)(value / 4912.0 * 32768.0);
    };
    uint16 y[2] [order = "big"] {
        get y_quat(float32): value / 32768.0 * 4912.0;
        set y_quat(float32): (uint16)(value / 4912.0 * 32768.0);
    };
    uint16 z[2] [order = "big"] {
        get z_quat(float32): value / 32768.0 * 4912.0;
        set z_quat(float32): (uint16)(value / 4912.0 * 32768.0);
    };
}

struct RovSensorEulerAngleData[6] {
    int16 pitch[2] [order = "big"] {
        get pitch_quat(float32): (float32)value;
        set pitch_quat(float32): (int16)value;
    };
    int16 roll[2] [order = "big"] {
        get roll_quat(float32): (float32)value;
        set roll_quat(float32): (int16)value;
    };
    uint16 yaw[2] [order = "big"] {
        get yaw_quat(float32): (float32)value;
        set yaw_quat(float32): (uint16)value;
    };
}

struct RovSensorHeightSonarData[6] {
    uint16 height[2] [order = "big"] {
        get height_quat(float32): value / 100.0;
        set height_quat(float32): (uint16)(value * 100.0);
    };
    uint16 confidence[2] [order = "big"] {
        get trustable(bool): (value / 100.0) <= 95 ? false : true;
        get confidence_quat(float32): value / 100.0;
        set confidence_quat(float32): (uint16)(value * 100.0);
    };
    void [2];
}

struct RovSensorDistanceSonarData[6] {
    uint16 distance[2] [order = "big"] {
        get distance_quat(float32): value / 100.0;
        set distance_quat(float32): (uint16)(value * 100.0);
    };
    uint16 confidence[2] [order = "big"] {
        get confidence_quat(float32): value / 100.0;
        set confidence_quat(float32): (uint16)(value * 100.0);
    };
    void [2];
}

struct RovSensorCircularSonarData[6] {
    uint16 angle[2] [order = "big"] {
        get angle_quat(float32): value / 100.0;
        set angle_quat(float32): (uint16)(value * 100.0);
    };
    uint16 distance[2] [order = "big"] {
        get distance_quat(float32): value / 100.0;
        set distance_quat(float32): (uint16)(value * 100.0);
    };
    uint16 confidence[2] [order = "big"] {
        get confidence_quat(float32): value / 100.0;
        set confidence_quat(float32): (uint16)(value * 100.0);
    };
}

struct RovSensorScanSonarData[6] {
    uint16 x[2] [order = "big"] {
        get x_quat(float32): value / 100.0;
        set x_quat(float32): (uint16)(value * 100.0);
    };
    uint16 distance[2] [order = "big"] {
        get distance_quat(float32): value / 100.0;
        set distance_quat(float32): (uint16)(value * 100.0);
    };
    uint16 confidence[2] [order = "big"] {
        get confidence_quat(float32): value / 100.0;
        set confidence_quat(float32): (uint16)(value * 100.0);
    };
}

struct RovSensorQuaternionData[6] {
    uint16 q[2] [order = "big"] {
        get q_quat(float32): value / 100.0;
        set q_quat(float32): (uint16)(value * 100.0);
    };
    uint16 group[2] [order = "big"];
    void [2];
}

struct RovSensorWaterMiscData[6] {
    uint16 salinity[2] [order = "big"] {
        get salinity_quat(float32): value / 100.0;
        set salinity_quat(float32): (uint16)(value * 100.0);
    };
    uint16 turbidity[2] [order = "big"] {
        get turbidity_quat(float32): value / 100.0;
        set turbidity_quat(float32): (uint16)(value * 100.0);
    };
    uint16 illuminance[2] [order = "big"] {
        get illuminance_quat(float32): value / 100.0;
        set illuminance_quat(float32): (uint16)(value * 100.0);
    };
}
