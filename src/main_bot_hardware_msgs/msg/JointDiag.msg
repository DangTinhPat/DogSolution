# Low-rate MCU/CAN diagnostics. All arrays use JointIndex_t order.
uint16 ready_mask
uint16 fresh_mask
uint16 runtime_fault_mask
uint16[12] status_raw_le
uint16[12] telemetry_age_ms
uint8 can_bus_off_mask
uint32[2] can_tx_fail_count
uint32 command_rx_count
uint32 command_seq_gap_count
uint32 command_seq_duplicate_count
uint32 command_seq_out_of_order_count
uint32 joint_fb_publish_fail_count
uint32 imu_publish_fail_count
uint32 joint_diag_publish_fail_count
