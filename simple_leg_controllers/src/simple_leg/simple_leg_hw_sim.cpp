#include <simple_leg_controllers/simple_leg_hw_sim.h>

namespace simple_leg_hw_sim{

using namespace hardware_interface;

SimpleLegHWSim::SimpleLegHWSim()
 : gazebo_ros_control::RobotHWSim(){}

/**
 * @brief 
 * 
 * @param robot_namespace 
 * @param model_nh 
 * @param model 
 * @param urdf_model 
 * @param transmissions 
 * @return true 
 * @return false 
 */
bool SimpleLegHWSim::initSim(
        const std::string& robot_namespace,
        ros::NodeHandle model_nh,
        gazebo::physics::ModelPtr parent_model,
        const urdf::Model* const urdf_model,
        std::vector<transmission_interface::TransmissionInfo> transmissions)
{
    const ros::NodeHandle joint_limit_nh(model_nh);

    n_dof_ = transmissions.size();
    joint_names_.resize(n_dof_);
    joint_types_.resize(n_dof_);
    joint_position_.resize(n_dof_);
    joint_velocity_.resize(n_dof_);
    joint_effort_.resize(n_dof_);
    joint_position_command_.resize(n_dof_);
    
    for(unsigned int i=0; i<n_dof_; i++){
        if(transmissions[i].joints_.size() == 0){
            ROS_ERROR_STREAM_NAMED("simple_leg_hw_sim", "Transmission" << transmissions[i].name_ << "has no associated joints.");
            continue;
        }
        else if(transmissions[i].joints_.size() > 1){
            continue;
        }

        std::vector<std::string> joint_interfaces = transmissions[i].joints_[0].hardware_interfaces_;

        // Add data for transmission
        joint_names_[i] = transmissions[i].joints_[0].name_;
        joint_position_[i] = 0.0;
        joint_velocity_[i] = 0.0;
        joint_effort_[i] = 0.0;
        joint_position_command_[i] = 0.0;

        const std::string& hardware_interface = joint_interfaces.front();

        // Create joint_state_interface for all joints 
        joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
            joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));

        // Decide what kind of command interface this actuator/joint has
        leg_position_interface_.registerHandle(hardware_interface::LegPositionHandle("FL", &cmd_x, &cmd_z));

        // Get the gazebo joint that corresponds to the robot joint.
        gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[i]);
        if(!joint){
            ROS_ERROR_STREAM_NAMED("default_robot_hw", "This robot has a joint named \"" << joint_names_[i] << "\" which is not in the gazebo model.");
            return false;
        }
        sim_joints_.push_back(joint);

        // get physics engine type
    #if GAZEBO_MAJOR_VERSION >= 8
        gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();
    #else
        gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->GetPhysicsEngine();
    #endif

    physics_type_ = physics->GetType();

    }

    // Register Interface
    registerInterface(&joint_state_interface_);
    registerInterface(&leg_position_interface_);

    return true;
}


void SimpleLegHWSim::readSim(ros::Time time, ros::Duration period){
    for(size_t i=0; i<n_dof_; i++){
        #if GAZEBO_MAJOR_VERSION >= 8
            double position = sim_joints_[i]->Position(0);
        #else
            double position = sim_joints_[i]->GetAngle(0).Radian();
        #endif
        joint_position_[i] += angles::shortest_angular_distance(joint_position_[i], position); 
        joint_velocity_[i] = sim_joints_[i]->GetVelocity(0);
        joint_effort_[i] = sim_joints_[i]->GetForce((unsigned int)(0));
    }
}


void SimpleLegHWSim::writeSim(ros::Time time, ros::Duration period){
    // Enforce joint limits
    jnt_limits_interface_.enforceLimits(period);
    solve_IK();
    
    for(size_t i=0; i<n_dof_; i++){
        #if GAZEBO_MAJOR_VERSION >= 9
            sim_joints_[i]->SetPosition(0, joint_position_command_[i]);
            // This command makes the model destructed
            // sim_joints_[i]->SetPosition(0, joint_position_command_[i], true);
        #else
            sim_joints_[i]->SetPosition(0, joint_position_command_[i]);
        #endif
    }
    // for debug
    ROS_INFO_STREAM("cmd_x: " << cmd_x << ", cmd_z: " << cmd_z);
    // ROS_INFO_STREAM("hip: " << joint_position_command_[0] << ", knee: " << joint_position_command_[1]);
}


void SimpleLegHWSim::solve_IK(void){
    using namespace simple_leg;
    double L = std::hypot(cmd_x, cmd_z);

    joint_position_command_[0] = -atan2(cmd_x, -cmd_z) + acos((pow(L, 2) + pow(l1, 2) - pow(l2, 2)) / (2 * L * l1));
    joint_position_command_[1] = -M_PI + acos((pow(l1, 2.0) + pow(l2, 2.0) - pow(L, 2)) / (2 * l1 * l2));
}

}   // namespace simple_leg_hw_sim

PLUGINLIB_EXPORT_CLASS(simple_leg_hw_sim::SimpleLegHWSim, gazebo_ros_control::RobotHWSim)
