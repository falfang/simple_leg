/**
 * @brief 自作ros_controller．脚先位置を指令値とする．
 * 
 */

#include <simple_leg_controllers/leg_position_controller.h>

namespace simple_leg_controllers{

LegPositionController::LegPositionController(void){}

LegPositionController::~LegPositionController(void){
    sub_command.shutdown();
}

// コントローラを生成したときに実行される関数？
bool LegPositionController::init(hardware_interface::LegPositionInterface* hw, ros::NodeHandle& nh){
    // get all leg from the hardware interface
    // const std::vector<std::string>& leg_names = hw->getNames();
    leg_ = hw->getHandle("FL"); // これを入れ忘れていた！！
    // 受け取った脚先位置指令値を格納
    sub_command = nh.subscribe<simple_leg_msgs::leg_command>("command", 1, &LegPositionController::commandCB, this);

    return true;
}

// コントローラを読み込んだときに実行される関数？
void LegPositionController::starting(const ros::Time&){
    cmd.x = 0.0;
    cmd.z = -0.35;
}

// シミュレーションの周期毎に呼び出される関数
void LegPositionController::update(const ros::Time&, const ros::Duration&){
    enforceWorkspaceLimit(cmd);     // 指令位置に対して制限等をかける
    leg_.setCommand(cmd.x, cmd.z);  // 調節した指令値をhandleに代入(エラー発生)
}

// コントローラを終了したときの処理？
void LegPositionController::stopping(const ros::Time&){

}

// Subscriberのコールバック関数
void LegPositionController::commandCB(const simple_leg_msgs::leg_commandConstPtr& msg){
    cmd.x = msg->x;
    cmd.z = msg->z;
}


std::string LegPositionController::getLegName(void){
    return leg_.getName();
}

}

PLUGINLIB_EXPORT_CLASS(simple_leg_controllers::LegPositionController, controller_interface::ControllerBase)