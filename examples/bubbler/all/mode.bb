package api.mode;

struct RovModeModeAData[6] {
    bool side_push[1];
    bool tilt[1];
    bool roll[1];
    bool auxiliary[1];
    bool rescue[1];
    bool submodule[1];
}

struct RovModeModeBData[6] {
    bool orbit_en[1];
    bool hover_en[1];
    bool stabilize_en[1];
    bool planner_en[1];
    bool execute_en[1];
    bool autopilot_en[1]; 
}

struct RovModeModeCData[6] {
    bool navi_mode[1];
    bool env_monitor[1];
    bool auto_dive[1];
    bool auto_float[1];
    bool fixed_dive[1];
    bool fixed_float[1];
}
