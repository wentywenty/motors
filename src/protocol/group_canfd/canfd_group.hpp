/**
 * @file canfd_group_manager.hpp
 * @brief Responsible for merging independent motor control commands into a single 64-byte CAN-FD frame using a "jigsaw" approach.
 */

#pragma once

#include <vector>
#include <memory>
#include <stdint.h>

#include "motor_driver.hpp"
#include "protocol/canfd/socket_canfd.hpp"

/**
 * @class CanfdGroupManager
 * @brief Management class for CAN-FD Multi-Drop motor control.
 * * Core Logic:
 * 1. Dynamically manages a group of motors (no fixed count, as long as total bytes <= 64).
 * 2. Queries each motor for its command length and allocates a "Slot" within the 64-byte packet.
 * 3. Instructs each motor to write its byte data into the assigned "Slot".
 */
class CanfdGroupManager {
public:
    /**
     * @param canfd Pointer to the underlying SocketCANFD communication interface.
     * @param group_can_id The broadcast control ID for this group (e.g., 0x20 for EVO-FD).
     */
    CanfdGroupManager(std::shared_ptr<MotorsSocketCANFD> canfd, uint32_t group_can_id);
    ~CanfdGroupManager() = default;

    /**
     * @brief Adds a motor to the management group.
     * The layout order within the 64-byte packet depends on the order in which motors are added.
     */
    void add_motor(std::shared_ptr<MotorDriver> motor);

    /**
     * @brief [Synchronous Transmit] Executes the jigsaw merging logic:
     * 1. Prepares a blank 64-byte CAN-FD frame.
     * 2. Iterates through all motors, allowing each to fill its specific data into the frame.
     * 3. Sends the final aggregated packet via the bus.
     */
    void sync_transmit();

private:
    std::shared_ptr<MotorsSocketCANFD> canfd_interface_; // Underlying hardware interface
    uint32_t broadcast_can_id_;                          // Group broadcast ID
    std::vector<std::shared_ptr<MotorDriver>> motor_list_; // List of managed motors
};