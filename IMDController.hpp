
#ifndef _IMD_CONTROLLER_HPP_
#define _IMD_CONTROLLER_HPP_

#include "SerialBridge/src/SerialBridge.hpp"
#include "SerialBridge/src/Message.hpp"
#include "SerialBridge/src/MCP2210Linux.hpp"

#include "msgs/IMDConfigMsgs.hpp"
#include "msgs/CtrlMsgs.hpp"
#include "msgs/ParamMsgs.hpp"

class IMDController
{
public:
    /**
     * @brief どのモータをしているかを示す列挙型
     */
    typedef enum MotorType
    {
        M1 = 0,
        M2,
        MOTOR_NUM,
    }motor_t;

    /**
     * @brief PID制御パラメータ構造体型
     */
    typedef struct PIDParameterType
    {
        /* 比例制御ゲイン */
	    float kp;
	    /* 積分制御ゲイン */
	    float ki;
	    /* 微分制御ゲイン */
	    float kd;
    }pid_param_t;

    /**
     * @brief モータの制御設定構造体
     */
    typedef struct MotorParameterType
    {
        /* エンコーダのCPR(count par revolution)単相パルス数(PPR)の４倍 */
	    uint16_t encoder_cpr;
	    /* ギア比(分子は1で固定 例 50:1->gear_ratio=50)(エンコーダがモータ出力軸側にある場合gear_ratio=1) */
	    float gear_ratio;
	    /* モータの(出力軸の)定格回転速度[rps] */
	    float max_rps;
	    /* 回転方向を反転させる。(エンコーダの向きに合わせて設定する) */
	    bool dir_reverse;
        /* 速度制御パラメータ */
        pid_param_t vel;
        /* 電流制御パラメータ */
        pid_param_t cur;
    }motor_param_t;

    /**
     * @brief IMDのSPI通信の設定
     */
    enum{
        IMD_BUFFER_SIZE = 28,
        IMD_SPI_SPEED = 5000000,
        IMD_SPI_MODE = 3,
    };

    static void display_serial_number()
    {
        MCP2210Linux::display_serial_number();
    }

    IMDController(const char *serial_number, MCP2210Linux::cs_pin_t active_cs);

    ~IMDController();
    
    void ctrl_begin(motor_param_t param[2]);

    int update();

    int update(uint32_t count);

    void set_speed(motor_t m, float rps);

    ctrl_feed_msg_t &get_state();

    bool state_updated();

private:

    /**
     * @brief 各メッセージのフレームIDの先頭を示す。
     */
    enum {
        VEL_MSG_HEAD     = 3,
        CUR_MSG_HEAD     = 5,
        INIT_MSG_HEAD    = 7,
    };

    IMD::CtrlrMsg ctrlr_msg_;
    CtrlCmdMsg cmd_;
    CtrlFeedMsg feed_;

    long count_;
    MCP2210Linux dev_;
    SerialBridge serial_;

    CtrlParamMsg vel_param_[2];
    CtrlParamMsg cur_param_[2];
    CtrlInitMsg init_[2];
};

#endif  /* #ifndef _IMD_CONTROLLER_HPP_ */
