/**
 * @file pi3hat_hardware_interface.hpp
 * @author John Bush (johncbus@usc.edu)
 * @brief This file defines the Pi3HatHardwareInterface class, which is a 
 *          ROS2 Control Hardware System Type Interface.
 * @details implementation follows the ROS2 documentation found at: 
 *              <https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html>
 * @version 0.1
 * @date 2024-03-21
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef PI3HAT_HARDWARE_INTERFACE__PI3HAT_HARDWARE_INTERFACE_HPP_
#define PI3HAT_HARDWARE_INTERFACE__PI3HAT_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <optional>
#include <ctime>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// header for the base class definition
#include "base_interface/base_interface.hpp"

#include "pi3hat/pi3hat.h"
#include "actuators/actuator_base.h"
#include "actuators/odrive_actuator.h"
#include "actuators/moteus_actuator.h"

#define MAX_NUM_CAN_FRAMES 12*8

namespace pi3hat_hardware_interface
{
    class Pi3HatHardwareInterface : public base_interface::BaseInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Pi3HatHardwareInterface)

        ~Pi3HatHardwareInterface() override;

        /**
         * @details on_init is expected to vary depending on the URDF file. 
         *          The SystemInterface::on_init(info) call fills out the info 
         *          object with specifics from the URDF. For example, 
         *          the info object has fields for joints, sensors, gpios, and more. 
         *          Suppose the sensor field has a name value of tcp_force_torque_sensor. 
         *          Then the on_init must try to establish communication with that sensor. 
         *          If it fails, then an error value is returned.
         * 
         * @param info 
         * @return hardware_interface::CallbackReturn 
         */
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        /**
         * @brief Called after the on_init function. Setup the communication to the hardware and set everything up so that the hardware can be activated.
         * 
         * @param previous_state 
         * @return hardware_interface::CallbackReturn 
         */
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Does opposite of on_configure()
         * 
         * @param previous_state
        */
        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Exposes state interfaces for proper initialization of ROS2 control topics
         * 
         * @return std::vector<hardware_interface::StateInterface> 
         */
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        /**
         * @brief Exports the command interfaces for proper initialization of ROS2 control topics
         * 
         * 
         * @return std::vector<hardware_interface::CommandInterface> 
         */
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        /**
         * @brief Called after on_configure. This is where the hardware is activated -- "powered up"
         * 
         * @param previous_state 
         * @return hardware_interface::CallbackReturn 
         */
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Opposite of on_activate. This is where the hardware is deactivated -- "powered down"
         * 
         * @param previous_state 
         * @return hardware_interface::CallbackReturn 
         */
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Core function of the ROS2 control loop. In the case of the Pi3Hat, this function does nothing.
         * 
         * @param time 
         * @param period 
         * @return hardware_interface::return_type 
         */
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        /**
         * @brief Carries out CAN frame writing and reading to control and read feedback from actuators.
         * 
         * @param time 
         * @param period 
         * @return hardware_interface::return_type 
         */
        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        /**
         * @brief Allocates a number of frames for an actuator within the Span object of the MJBots Pi3Hat
         * 
         * @param size -- number of frames to allocate
         * @return std::shared_ptr<mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>> 
         */
        std::shared_ptr<mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>> allocateTxSpan(size_t size);

        /**
         * @brief Utility function to distribute incoming CAN frames to actuators
         * 
         * @param result the status object from a Pi3Hat cycle command
         * @return true indicates successful distribution
         * @return false indicates no frames were distributed successfully
         */
        bool distribute_rx_input(mjbots::pi3hat::Pi3Hat::Output result);
        
    private:
        // Utility functions for converting between double and uint
        int double_to_uint(double x, double x_min, double x_max,
                          int bits);
        double uint_to_double(int x_int, double x_min, double x_max,
                            int bits);

        // Utility functions for dealing with angle wrapping
        double wrap_angle(double angle, double angle_min, double angle_max);
        double unwrap_angle(double angle, double prev_angle, double angle_min,
                           double angle_max);

        void assign_frame(mjbots::pi3hat::CanFrame frame);

        void update_actuator_state_interfaces();

        void busy_wait_us(unsigned long microsec) {
            // Convert microseconds to clock ticks. std::CLOCKS_PER_SEC is the number of clock ticks per second.
            std::clock_t end = std::clock() + (microsec * CLOCKS_PER_SEC / 1000000);
            
            while (std::clock() < end) {
                // Busy wait
            }
        }

        int findIndex(const std::vector<int>& vec, int value) {
            for (size_t i = 0; i < vec.size(); ++i) {
                if (vec[i] == value) {
                    return i; // Found the element, return its index
                }
            }
            return -1; // Element not found, return -1
        };

        // Enum for CAN protocol types: cheetah, myactuator, or moteus
        enum class CanProtocol
        {
            CHEETAH,    // MIT Mini Cheetah actuators, SteadyWin, CubeMars
            MYACTUATOR, // MyActuator, LKMTech
            MOTEUS,      // Moteus (non-FD CAN)
            ODRIVE,      // ODrive
        };

        std::unordered_map<std::string, CanProtocol> protocolMap = {
            {"cheetah", CanProtocol::CHEETAH},
            {"myactuator", CanProtocol::MYACTUATOR},
            {"moteus", CanProtocol::MOTEUS},
            {"odrive", CanProtocol::ODRIVE}
        };

        enum TxAllocation
        {
            CHEETAH_TX     = 1,
            MYACTUATOR_TX  = 1,
            MOTEUS_TX      = 1,
            ODRIVE_TX      = 6,
        };

        enum RxAllocation
        {
            CHEETAH_RX     = 1,
            MYACTUATOR_RX  = 1,
            MOTEUS_RX      = 1,
            ODRIVE_RX      = 12,
        };

        const unsigned char cheetahEnableMsg[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                                   0xFF, 0xFF, 0xFF, 0xFC};

        const unsigned char cheetahDisableMsg[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                                    0xFF, 0xFF, 0xFF, 0xFD};

        const unsigned char cheetahSetZeroPositionMsg[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                                            0xFF, 0xFF, 0xFF, 0xFE};

        const unsigned char cheetahSetIdleCmdMsg[8] = {0x7F, 0xFF, 0x7F, 0xF0,
                                                       0x00, 0x00, 0x07, 0xFF};

        mjbots::pi3hat::Pi3Hat *pi3hat_;
        mjbots::pi3hat::Pi3Hat::Input pi3hat_input_;
        mjbots::pi3hat::Attitude attitude_;
        // mjbots::pi3hat::CanFrame tx_can_frames_[MAX_NUM_CAN_FRAMES];
        // mjbots::pi3hat::CanFrame rx_can_frames_[MAX_NUM_CAN_FRAMES];

        // Pi3Hat input handling
        std::vector<mjbots::pi3hat::CanFrame> tx_can_frames_;
        std::vector<mjbots::pi3hat::CanFrame> rx_can_frames_;
        size_t nextTxStart = 0; // Tracks the next start position for Tx allocations
        size_t nextRxStart = 0; // Tracks the next start position for Rx allocations

        // ! DO THESE NEED TO BE STATIC??
        size_t tx_capacity_ = 36; // Default initial capacity
        size_t rx_capacity_ = 36; // Default initial capacity

        // Actuator List
        std::vector<std::shared_ptr<ActuatorBase>> hw_actuators_;
    };

} // namespace pi3hat_hardware_interface

#endif // PI3HAT_HARDWARE_INTERFACE_PUBLIC__PI3HAT_HARDWARE_INTERFACE_HPP_