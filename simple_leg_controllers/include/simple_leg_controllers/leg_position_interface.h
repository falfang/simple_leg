/**
 * @file leg_position_interface.h
 * @author your name (you@domain.com)
 * @brief 自作ros_controller用のinterface
 * @version 0.1
 * @date 2021-10-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace hardware_interface{

class LegPositionHandle{
private:
    double *x   = {nullptr};
    double *z   = {nullptr};
    std::string leg_name;

public:
    LegPositionHandle(std::string name_, double* x_, double* z_)
     : leg_name(name_), x(x_), z(z_){
        if(!x){   throw HardwareInterfaceException("Cannot create handle. command x pointer is null"); *x = 0.0; }
        if(!z){   throw HardwareInterfaceException("Cannot create handle. command z pointer is null"); *z = 0.4; }
    }
    ~LegPositionHandle(){}

    void setCommand(double x_, double z_){
        assert(x); *x = x_;
        assert(z); *z = z_;
    }

    std::string getName() const { return leg_name; }
    double getX() const { assert(x); return *x; }
    double getZ() const { assert(z); return *z; }

    const double* getXPtr() const { assert(x); return x; }
    const double* getZPtr() const { assert(z); return z; }
};

class LegPositionInterface : public HardwareResourceManager<LegPositionHandle>{};

}