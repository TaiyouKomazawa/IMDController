
#ifndef _IMD_CONTROLLER_HPP_
#define _IMD_CONTROLLER_HPP_

#include "SerialBridge/src/SerialBridge.hpp"
#include "SerialBridge/src/Message.hpp"
#include "SerialBridge/src/MCP2210Linux.hpp"

#include "IMDConfigMsgs.hpp"
#include "CtrlMsgs.hpp"
#include "ParamMsgs.hpp"

class IMDController
{
public:
    typedef ctrl_param_msg_t ctrl_param_t;
    typedef ctrl_init_msg_t ctrl_init_t;

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

    CtrlCmdMsg cmd;
    CtrlFeedMsg feed;

    IMD::CtrlrMsg ctrlr_msg;

    IMDController(const char *serial_number, MCP2210Linux::cs_pin_t active_cs, 
                        uint8_t buffer_size = MCP2210Linux::DEFAULRT_BUFFER_SIZE,
                        uint32_t spi_speed = MCP2210Linux::SPI_SPEED,
                        uint8_t spi_mode = MCP2210Linux::SPI_MODE);
    
    void ctrl_begin(ctrl_init_t init[2], ctrl_param_t param[2]);

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
