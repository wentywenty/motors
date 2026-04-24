#include "canfd_group.hpp"
#include <cstring>
#include <spdlog/spdlog.h>

CanfdGroupManager::CanfdGroupManager(std::shared_ptr<MotorsSocketCANFD> canfd, uint32_t group_can_id)
    : canfd_interface_(canfd), broadcast_can_id_(group_can_id) {}

void CanfdGroupManager::add_motor(std::shared_ptr<MotorDriver> motor) {
    if (motor) {
        motor_list_.push_back(motor);
    }
}

void CanfdGroupManager::sync_transmit() {
    // 1. Check if the physical interface is ready
    if (!canfd_interface_) {
        return;
    }

    // 2. Prepare the base: construct a blank 64-byte CAN-FD frame
    canfd_frame tx_frame{};
    tx_frame.can_id = broadcast_can_id_;
    tx_frame.len = 64;           // Set CAN-FD payload length to maximum
    tx_frame.flags = CANFD_BRS;  // Enable Bit Rate Switching (5Mbps mode)

    // 3. Start merging: fill each motor's slot sequentially
    uint8_t write_pointer = 0; // Starting index for the current slot (0-63)
    
    for (auto& motor : motor_list_) {
        // Query: How much space does this motor need?
        uint8_t slot_size = motor->get_command_size();
        
        // Check: Prevent exceeding the 64-byte physical limit
        if (write_pointer + slot_size > 64) {
            spdlog::error("CAN-FD Frame Overflow! Cannot fit more motors.");
            break; 
        }

        // Fill: Pass the starting address of the slot to the motor's specialized packing method
        motor->pack_cmd_data(&tx_frame.data[write_pointer]);
        
        // Move: Update the starting position for the next slot
        write_pointer += slot_size;
    }

    // 4. Transmit: Send the single aggregated frame via the physical layer
    canfd_interface_->transmit(tx_frame);
}