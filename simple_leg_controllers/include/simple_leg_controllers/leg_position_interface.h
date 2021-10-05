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
#include <cassert>

namespace hardware_interface{

class LegPositionHandle{
private:
    double *x_   = {nullptr};
    double *z_   = {nullptr};
    std::string leg_name;

public:
    LegPositionHandle() = default;
    LegPositionHandle(const std::string name_, double* x, double* z)
     : leg_name(name_), x_(x), z_(z){
        if(!x_){ throw HardwareInterfaceException("Cannot create handle. command x pointer is null"); }
        if(!z_){ throw HardwareInterfaceException("Cannot create handle. command z pointer is null"); }
    }
    ~LegPositionHandle(){}

    void setCommand(double x, double z){
        assert(x_);
        *x_ = x;
        *z_ = z;
    }

    std::string getName() const { return leg_name; }
    double getX() const { assert(x_); return *x_; }
    double getZ() const { assert(z_); return *z_; }

    const double* getXPtr() const { assert(x_); return x_; }
    const double* getZPtr() const { assert(z_); return z_; }
};

class LegPositionInterface : public HardwareResourceManager<LegPositionHandle>{};

}