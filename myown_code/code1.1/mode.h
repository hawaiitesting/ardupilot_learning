/*
    这个模式是解锁后直接满油门，速度一定后抬头，然后航向锁定，直接算航向上的误差，直接控制舵面。
    但是这个只有简单的控制，最后会不可避免的走向发散。绕过了ardupilot的所有PID控制。
    直接控制舵面，可以很好的感受到，不同pid值对飞机的影响。
*/
class ModeNewmode : public Mode
{
public:
    float takeoff_heading;
    bool heading_set;
    static bool heading_set;
  
    Number mode_number() const override { return Number::NEWMODE; }
    const char *name() const override { return "NEWMODE"; }
    const char *name4() const override { return "NEWM"; }

    enum TakeoffState {
        GROUND_RUN = 0,
        ROTATE     =1,
    };
    TakeoffState state;


    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    // true if throttle min/max limits should be applied
    bool use_throttle_limits() const override;

    // true if voltage correction should be applied to throttle
    bool use_battery_compensation() const override { return false; }

};
