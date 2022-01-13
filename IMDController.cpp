#include "IMDController.hpp"

IMDController::IMDController(const char *serial_number, MCP2210Linux::cs_pin_t active_cs, uint8_t buffer_size,uint32_t spi_speed, uint8_t spi_mode)
:   dev_((wchar_t*)serial_number, active_cs, buffer_size, spi_speed, spi_speed),
    serial_(&dev_)
{
    serial_.add_frame(0, &ctrlr_msg_);
    serial_.add_frame(1, &cmd_);
    serial_.add_frame(2, &feed_);
    serial_.add_frame(3, &vel_param_[M1]);
    serial_.add_frame(4, &vel_param_[M2]);
    serial_.add_frame(5, &cur_param_[M1]);
    serial_.add_frame(6, &cur_param_[M2]);
    serial_.add_frame(7, &init_[M1]);
    serial_.add_frame(8, &init_[M2]);
}

void IMDController::ctrl_begin(IMDController::motor_param_t param[2])
{
    for(int i = 0; i < MOTOR_NUM; i++){
        init_[i].tx.is_received = true;
        init_[i].tx.dir_reverse = param[i].dir_reverse;
        init_[i].tx.encoder_cpr = param[i].encoder_cpr;
        init_[i].tx.gear_ratio = param[i].gear_ratio;
        init_[i].tx.max_rps = param[i].max_rps;

        vel_param_[i].tx.kp = param[i].vel.kp;
        vel_param_[i].tx.ki = param[i].vel.ki;
        vel_param_[i].tx.kd = param[i].vel.kd;

        cur_param_[i].tx.kp = param[i].cur.kp;
        cur_param_[i].tx.ki = param[i].cur.ki;
        cur_param_[i].tx.kd = param[i].cur.kd;
    }
    
    ctrlr_msg_.tx.command = IMD::CMD_RESET;
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

int IMDController::update()
{
    cmd_.data.frame_id = count_;
    count_++;
    serial_.write(1);
    return serial_.update();
}

int IMDController::update(uint32_t count)
{
    count_ = 0;
    return update();
}


void IMDController::set_speed(IMDController::motor_t m, float rps)
{
    if(m < MOTOR_NUM)
        cmd_.data.command[m] = rps;
}

ctrl_feed_msg_t &IMDController::get_state()
{
    return feed_.data;
}

bool IMDController::state_updated()
{
    return feed_.was_updated();
}