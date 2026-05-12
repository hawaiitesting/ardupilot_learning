/*
    这个是去掉了自己去做PID，改用飞控自己的PID，去设置自己想要的值。
    让飞控自己去算该调多少值。但是这个同样最后走向了发散。
*/

class ModeNewmode : public Mode
{
public:
    float takeoff_heading;
    bool heading_set;
  
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