/**
 * @brief 自作ros_controller
 * 
 */

#include <ros_control_tutorial/leg_position_controller.h>

namespace leg_position_controller{

LegPositionController::LegPositionController(void){}

LegPositionController::~LegPositionController(void){
    sub_command.shutdown();
}


bool LegPositionController::init(hardware_interface::LegPositionInterface* hw, ros::NodeHandle& nh){
    // get all leg from the hardware interface
    // const std::vector<std::string>& leg_names = hw->getNames();

    // 受け取った脚先位置指令値を格納
    sub_command = nh.subscribe<ros_control_tutorial::leg_command>("command", 1, &LegPositionController::setCommandCB, this);

    return true;
}

// コントローラを読み込んだときに実行される関数？
void LegPositionController::starting(const ros::Time&){
    cmd.x = 0.0;
    cmd.z = 0.35;
}

// シミュレーションの周期毎に呼び出される関数
void LegPositionController::update(const ros::Time&, const ros::Duration&){
    enforceWorkspaceLimit(cmd);     // 指令位置に対して制限等をかける
    leg_.setCommand(cmd.x, cmd.z);  // handleに値を代入
}

// コントローラを終了したときの処理？
void LegPositionController::stopping(const ros::Time&){

}

void LegPositionController::setCommandCB(const ros_control_tutorial::leg_commandConstPtr& msg){
    leg_.setCommand(msg->x, msg->z);
}

std::string LegPositionController::getLegName(void){
    return leg_.getName();
}

}

PLUGINLIB_EXPORT_CLASS(leg_position_controller::LegPositionController, controller_interface::ControllerBase)