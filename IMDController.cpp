/**
 * @file IMDController.cpp
 * @author Taiyou Komazawa (taiyou24690@gmail.com)
 * @brief InteractiveMDを制御するためのクラス
 * @version 1.0.0
 * @date 2022-01-13
 * 
 */

#include "IMDController.hpp"
/**
 * @brief Construct a new IMDController::IMDController object
 * 
 * @param[in] serial_number MCP2210各デバイス固有のシリアルナンバー
 * @param[in] active_cs     MCP2210Linux::cs_pin_t列挙型(使用するチップセレクタピン名)
 */
IMDController::IMDController(wchar_t *serial_number, MCP2210Linux::cs_pin_t active_cs)
:   dev_(serial_number, active_cs, IMD_BUFFER_SIZE, IMD_SPI_SPEED, IMD_SPI_MODE),
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

/**
 * @brief Destroy the IMDController::IMDController object
 */
IMDController::~IMDController()
{
}

/**
 * @brief 必要なパラメータを送信し制御をスタートします
 * 
 * @param[in] param IMDController::motor_param_t 2要素配列(M1,M2それぞれのコンフィグ)
 * @retval None
 */
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
            sleep(0.01);
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

/**
 * @brief データの更新を行います(必ずループ内で呼び出してください)
 * 
 * @retval  0 : メッセージを更新
 * @retval -1 : 受信失敗
 * @retval -2 : メッセージの値が不正
 * @retval -3 : 設定されていないIDのメッセージ
 */
int IMDController::update()
{
    cmd_.data.frame_id = count_;
    count_++;
    serial_.write(1);
    return serial_.update();
}

/**
 * @brief データの更新を行います(必ずループ内で呼び出してください)
 * 
 * @param[in] count 現在のフレームカウントをcountの値に変更 
 * @retval  0 : メッセージを更新
 * @retval -1 : 受信失敗
 * @retval -2 : メッセージの値が不正
 * @retval -3 : 設定されていないIDのメッセージ
 */
int IMDController::update(uint32_t count)
{
    count_ = count;
    return update();
}

/**
 * @brief 指定されたモータの回転速度を変更します
 * 
 * @param[in] m     対象のモータ
 * @param[in] rps   回転速度[rps]
 * @retval None
 */
void IMDController::set_speed(IMDController::motor_t m, float rps)
{
    if(m < MOTOR_NUM)
        cmd_.data.command[m] = rps;
}

/**
 * @brief 現在のモータの状態を示す構造体を返します
 * 
 * @return ctrl_feed_msg_t& 現在のモータの状態を示す構造体
 */
ctrl_feed_msg_t &IMDController::get_state()
{
    return feed_.data;
}

/**
 * @brief モータの状態の更新状況を返します
 * 
 * @retval true     データは更新された
 * @retval false    データは未更新
 */
bool IMDController::state_updated()
{
    return feed_.was_updated();
}