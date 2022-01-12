
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

    IMDController(char *serial_number, MCP2210Linux::cs_pin_t active_cs, 
                        uint8_t buffer_size = MCP2210Linux::DEFAULRT_BUFFER_SIZE,
                        uint32_t spi_speed = MCP2210Linux::SPI_SPEED,
                        uint8_t spi_mode = MCP2210Linux::SPI_MODE)
    :   dev_((wchar_t*)serial_number, active_cs, buffer_size, spi_speed, spi_speed),
        serial_(&dev_)
    {
        serial_.add_frame(0, &ctrlr_msg);
        serial_.add_frame(1, &cmd);
        serial_.add_frame(2, &feed);
        serial_.add_frame(3, &vel_param_[M1]);
        serial_.add_frame(4, &vel_param_[M2]);
        serial_.add_frame(5, &cur_param_[M1]);
        serial_.add_frame(6, &cur_param_[M2]);
        serial_.add_frame(7, &init_[M1]);
        serial_.add_frame(8, &init_[M2]);
    }

    void ctrl_begin(ctrl_init_t init[2], ctrl_param_t param[2])
    {
        init_[M1].tx = (ctrl_init_msg_t)init[M1];
        init_[M2].tx = (ctrl_init_msg_t)init[M2];
        vel_param_[M1].tx = (ctrl_param_msg_t)param[M1];
        vel_param_[M2].tx = (ctrl_param_msg_t)param[M2];
        
        ctrlr_msg.tx.command = IMD::CMD_RESET;
        serial_.write(0);
        serial_.update();

        for(int i = 0; i < MOTOR_NUM; i++){
            bool init_ok = false;
            while(!init_ok){
                serial_.update();
                if(init_[i].was_updated()){
                    init_ok = true;
                    serial_.write(INIT_MSG_HEAD + i);
                    serial_.update();
                }
            }
        }

        for(int i = 0; i < MOTOR_NUM; i++){
            while(!vel_param_[i].rx.is_received){
                serial_.write(VEL_MSG_HEAD + i);
                serial_.update();
            }
            while(!cur_param_[i].rx.is_received){
                serial_.write(CUR_MSG_HEAD + i);
                serial_.update();
            }
        }

        count_ = 0;
    }

    int update()
    {
        cmd.data.frame_id = count_;
        count_++;
        serial_.write(1);
        return serial_.update();
    }

    int update(long count)
    {
        count_ = 0;
        return update();
    }

private:
    long count_;
    MCP2210Linux dev_;
    SerialBridge serial_;

    CtrlParamMsg vel_param_[2];
    CtrlParamMsg cur_param_[2];
    CtrlInitMsg init_[2];
};

#endif  /* #ifndef _IMD_CONTROLLER_HPP_ */
