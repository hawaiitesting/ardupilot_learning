///*
 *       Example of PID library.
 *       2012 Code by Jason Short, Randy Mackay. DIYDrones.com
 *       读取遥控器通道值 → 计算误差 → 运行 PID 算法 → 输出控制结果
 *       代码整体功能

            初始化 ArduPilot 硬件抽象层（HAL）、遥控器通道
            定义 PID 控制器参数（P/I/D/ 滤波 / 积分限幅）
            循环读取遥控器通道 0 的输入值
            以遥控器中位为基准计算误差值
            调用 AC_PID 库计算 P/I/D 分项输出
            串口打印遥控器值、误差、PID 总输出

 */

#include <AP_HAL/AP_HAL.h>         // ArduPilot 硬件抽象层（跨平台驱动）
#include <AC_PID/AC_PID.h>         // 核心PID控制库
#include <AC_PID/AC_HELI_PID.h>    // 直升机专用PID（本代码未启用）
#include <RC_Channel/RC_Channel.h> // 遥控器通道驱动

// we need a board config created so that the io processor is available
#if HAL_WITH_IO_MCU
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
AP_BoardConfig BoardConfig;
#endif

void setup();
void loop();

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

class RC_Channel_PIDTest : public RC_Channel // 这个不带s的是单个通道管理器
{
}; // RC_Channel不允许使用原始的RC_Channel类，需要用户自己继承一个自己的类使用。

class RC_Channels_PIDTest : public RC_Channels // 这个是所有通道的管理器
{
public:
    RC_Channel *channel(uint8_t chan) override
    {
        return &obj_channels[chan];
    } // 这里返回了一个指向数组的指针，&这个是取地址。

    RC_Channel_PIDTest obj_channels[NUM_RC_CHANNELS];

private:
    int8_t flight_mode_channel_number() const override { return -1; };
};

#define RC_CHANNELS_SUBCLASS RC_Channels_PIDTest // 宏不会递归替换，只换
#define RC_CHANNEL_SUBCLASS RC_Channel_PIDTest

#include <RC_Channel/RC_Channels_VarInfo.h> //变量信息

RC_Channels_PIDTest _rc; // 实例

// default PID values
#define TEST_P 1.0f
#define TEST_I 0.01f
#define TEST_D 0.2f
#define TEST_IMAX 10
#define TEST_FILTER 5.0f
#define TEST_DT 0.01f
#define TEST_INITIAL_FF 0.0f

// setup function
void setup()
{
    hal.console->printf("ArduPilot AC_PID library test\n"); // 打印信息

#if HAL_WITH_IO_MCU
    BoardConfig.init();
#endif

    rc().init();

    hal.scheduler->delay(1000); // 延时只有在特殊情况才需要延时，这里是等待对频，防止接收到错误的数据。
}

// main loop
void loop()
{
    // setup (unfortunately must be done here as we cannot create a global AC_PID object)
    AC_PID *pid = NEW_NOTHROW AC_PID(TEST_P, TEST_I, TEST_D, 0.0f, TEST_IMAX * 100.0f, 0.0f, 0.0f, TEST_FILTER); // 这个一个通过new关键字分配内存
    // AC_HELI_PID *heli_pid= NEW_NOTHROW AC_HELI_PID(TEST_P, TEST_I, TEST_D, TEST_INITIAL_FF, TEST_IMAX * 100, 0.0f, 0.0f, TEST_FILTER);

    // display PID gains
    hal.console->printf("P %f  I %f  D %f  imax %f\n", (double)pid->kP(), (double)pid->kI(), (double)pid->kD(), (double)pid->imax());

    RC_Channel *c = rc().channel(0);
    if (c == nullptr)
    {
        while (true)
        {
            hal.console->printf("No channel 0?");
            hal.scheduler->delay(1000);
        }
    }

    // capture radio trim
    const uint16_t radio_trim = c->get_radio_in(); // trim校准值，get_radio_in这个是得到最原始的数据

    while (true)
    {
        rc().read_input(); // poll the radio for new values
        const uint16_t radio_in = c->get_radio_in();
        const int16_t error = radio_in - radio_trim;
        pid->update_error(error, TEST_DT);    // TEST_DT时间增量， 循环时间
        const float control_P = pid->get_p(); // 得到计算结果
        const float control_I = pid->get_i();
        const float control_D = pid->get_d();

        // display pid results
        hal.console->printf("radio: %d\t err: %d\t pid:%4.2f (p:%4.2f i:%4.2f d:%4.2f)\n",
                            (int)radio_in, (int)error,
                            (double)(control_P + control_I + control_D),
                            (double)control_P, (double)control_I, (double)control_D);
        hal.scheduler->delay(50);
    }
}

AP_HAL_MAIN();
/*
 *       Example of PID library.
 *       2012 Code by Jason Short, Randy Mackay. DIYDrones.com
 *       读取遥控器通道值 → 计算误差 → 运行 PID 算法 → 输出控制结果
 *       代码整体功能

            初始化 ArduPilot 硬件抽象层（HAL）、遥控器通道
            定义 PID 控制器参数（P/I/D/ 滤波 / 积分限幅）
            循环读取遥控器通道 0 的输入值
            以遥控器中位为基准计算误差值
            调用 AC_PID 库计算 P/I/D 分项输出
            串口打印遥控器值、误差、PID 总输出

 */

#include <AP_HAL/AP_HAL.h>         // ArduPilot 硬件抽象层（跨平台驱动）
#include <AC_PID/AC_PID.h>         // 核心PID控制库
#include <AC_PID/AC_HELI_PID.h>    // 直升机专用PID（本代码未启用）
#include <RC_Channel/RC_Channel.h> // 遥控器通道驱动

// we need a board config created so that the io processor is available
#if HAL_WITH_IO_MCU
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
AP_BoardConfig BoardConfig;
#endif

void setup();
void loop();

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

class RC_Channel_PIDTest : public RC_Channel // 这个不带s的是单个通道管理器
{
}; // RC_Channel不允许使用原始的RC_Channel类，需要用户自己继承一个自己的类使用。

class RC_Channels_PIDTest : public RC_Channels // 这个是所有通道的管理器
{
public:
    RC_Channel *channel(uint8_t chan) override
    {
        return &obj_channels[chan];
    } // 这里返回了一个指向数组的指针，&这个是取地址。

    RC_Channel_PIDTest obj_channels[NUM_RC_CHANNELS];

private:
    int8_t flight_mode_channel_number() const override { return -1; };
};

#define RC_CHANNELS_SUBCLASS RC_Channels_PIDTest // 宏不会递归替换，只换
#define RC_CHANNEL_SUBCLASS RC_Channel_PIDTest

#include <RC_Channel/RC_Channels_VarInfo.h> //变量信息

RC_Channels_PIDTest _rc; // 实例

// default PID values
#define TEST_P 1.0f
#define TEST_I 0.01f
#define TEST_D 0.2f
#define TEST_IMAX 10
#define TEST_FILTER 5.0f
#define TEST_DT 0.01f
#define TEST_INITIAL_FF 0.0f

// setup function
void setup()
{
    hal.console->printf("ArduPilot AC_PID library test\n"); // 打印信息

#if HAL_WITH_IO_MCU
    BoardConfig.init();
#endif

    rc().init();

    hal.scheduler->delay(1000); // 延时只有在特殊情况才需要延时，这里是等待对频，防止接收到错误的数据。
}

// main loop
void loop()
{
    // setup (unfortunately must be done here as we cannot create a global AC_PID object)
    AC_PID *pid = NEW_NOTHROW AC_PID(TEST_P, TEST_I, TEST_D, 0.0f, TEST_IMAX * 100.0f, 0.0f, 0.0f, TEST_FILTER); // 这个一个通过new关键字分配内存
    // AC_HELI_PID *heli_pid= NEW_NOTHROW AC_HELI_PID(TEST_P, TEST_I, TEST_D, TEST_INITIAL_FF, TEST_IMAX * 100, 0.0f, 0.0f, TEST_FILTER);

    // display PID gains
    hal.console->printf("P %f  I %f  D %f  imax %f\n", (double)pid->kP(), (double)pid->kI(), (double)pid->kD(), (double)pid->imax());

    RC_Channel *c = rc().channel(0);
    if (c == nullptr)
    {
        while (true)
        {
            hal.console->printf("No channel 0?");
            hal.scheduler->delay(1000);
        }
    }

    // capture radio trim
    const uint16_t radio_trim = c->get_radio_in(); // trim校准值，get_radio_in这个是得到最原始的数据

    while (true)
    {
        rc().read_input(); // poll the radio for new values
        const uint16_t radio_in = c->get_radio_in();
        const int16_t error = radio_in - radio_trim;
        pid->update_error(error, TEST_DT);    // TEST_DT时间增量， 循环时间
        const float control_P = pid->get_p(); // 得到计算结果
        const float control_I = pid->get_i();
        const float control_D = pid->get_d();

        // display pid results
        hal.console->printf("radio: %d\t err: %d\t pid:%4.2f (p:%4.2f i:%4.2f d:%4.2f)\n",
                            (int)radio_in, (int)error,
                            (double)(control_P + control_I + control_D),
                            (double)control_P, (double)control_I, (double)control_D);
        hal.scheduler->delay(50);
    }
}

AP_HAL_MAIN();