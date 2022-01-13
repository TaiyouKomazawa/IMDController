
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

    typedef enum MotorType
    {
        M1 = 0,
        M2,
        MOTOR_NUM,
    }motor_t;

    enum {
        VEL_MSG_HEAD     = 3,
        CUR_MSG_HEAD     = 5,
        INIT_MSG_HEAD    = 7,
    };

    typedef struct PIDParameterType
    {
        /* 比例制御ゲイン */
	    float kp;
	    /* 積分制御ゲイン */
	    float ki;
	    /* 微分制御ゲイン */
	    float kd;
    }pid_param_t;

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

    CtrlCmdMsg cmd;
    CtrlFeedMsg feed;

    IMD::CtrlrMsg ctrlr_msg;

    IMDController(const char *serial_number, MCP2210Linux::cs_pin_t active_cs, 
                        uint8_t buffer_size = MCP2210Linux::DEFAULRT_BUFFER_SIZE,
                        uint32_t spi_speed = MCP2210Linux::SPI_SPEED,
                        uint8_t spi_mode = MCP2210Linux::SPI_MODE);
    
    void ctrl_begin(motor_param_t param[2]);

    int update();

    int update(long count);

private:
    long count_;
    MCP2210Linux dev_;
    SerialBridge serial_;

    CtrlParamMsg vel_param_[2];
    CtrlParamMsg cur_param_[2];
    CtrlInitMsg init_[2];
};

#endif  /* #ifndef _IMD_CONTROLLER_HPP_ */
