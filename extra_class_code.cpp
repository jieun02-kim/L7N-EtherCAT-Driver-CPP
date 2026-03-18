
// driver.h
// 모터 하나의 EtherCAT 데이터 주소(오프셋)들을 저장하는 구조체
struct Motor_Offsets {
    unsigned int control_word;
    unsigned int target_position;
    unsigned int target_velocity;
    unsigned int modes_of_operation;
    unsigned int status_word;
    unsigned int position_actual_value;
    unsigned int velocity_actual_value;
    unsigned int modes_of_operation_display;
};

class EthercatDriver {
    // ... 기존 코드 ...
private:
    // 모터별 오프셋을 배열로 관리 (기존 개별 배열들을 대체)
    Motor_Offsets motor_offsets[DEFAULT_MOTOR_NUM];
    
    // ...
};



// driver.cpp

ec_pdo_entry_reg_t* EthercatDriver::create_domain1_regs(int motor_num)
{
    // 각 모터당 8개의 항목 + 마지막 NULL 종료 항목
    int entries_per_motor = 8;
    int total_entries = (motor_num * entries_per_motor) + 1;
    ec_pdo_entry_reg_t* regs = new ec_pdo_entry_reg_t[total_entries];

    for (int i = 0; i < motor_num; i++) {
        int base = i * entries_per_motor;

        // 0x6040: Control Word
        regs[base + 0] = {0, i, SERVO_MOTOR, 0x6040, 0x00, &motor_offsets[i].control_word};
        // 0x607A: Target Position
        regs[base + 1] = {0, i, SERVO_MOTOR, 0x607A, 0x00, &motor_offsets[i].target_position};
        // 0x60FF: Target Velocity
        regs[base + 2] = {0, i, SERVO_MOTOR, 0x60FF, 0x00, &motor_offsets[i].target_velocity};
        // 0x6060: Modes of Operation
        regs[base + 3] = {0, i, SERVO_MOTOR, 0x6060, 0x00, &motor_offsets[i].modes_of_operation};
        // 0x6041: Status Word
        regs[base + 4] = {0, i, SERVO_MOTOR, 0x6041, 0x00, &motor_offsets[i].status_word};
        // 0x6064: Position Actual Value
        regs[base + 5] = {0, i, SERVO_MOTOR, 0x6064, 0x00, &motor_offsets[i].position_actual_value};
        // 0x606C: Velocity Actual Value
        regs[base + 6] = {0, i, SERVO_MOTOR, 0x606C, 0x00, &motor_offsets[i].velocity_actual_value};
        // 0x6061: Modes of Operation Display
        regs[base + 7] = {0, i, SERVO_MOTOR, 0x6061, 0x00, &motor_offsets[i].modes_of_operation_display};
    }

    // 마지막에 빈 항목 추가 (EtherLab 라이브러리 규칙)
    regs[total_entries - 1] = {}; 
    
    return regs;
}

// 지금 당장 구조체로 바꾸면 모터 개수 등 조정하는데 편리하긴 한데
// 일단은 4축 모터 구동을 먼저 목표로 하자





void EthercatDriver::Display_modes(void)
{
    cout << "--- Motor Operation Modes ---" << endl;
    int8_t modes_of_operation = EC_READ_U8(domain1_pd + offset_modes_of_operation_display[i]);

    cout << "Motor [" << i << "]: ";
    switch (modes_of_operation) {
            case 0:  cout << "No mode change/assigned"; break;
            case 1:  cout << "Pp (Profile Position mode)"; break;
            case 2:  cout << "Reserved (keep last mode)"; break;
            case 3:  cout << "Pv (Profile Velocity mode)"; break;
            case 4:  cout << "Tq (Profile Torque mode)"; break;
            case 6:  cout << "Hm (Homing mode)"; break;
            case 7:  cout << "Ip (Interpolated Position mode)"; break;
            case 8:  cout << "Csp (Cyclic Sync Position mode)"; break;
            case 9:  cout << "Csv (Cyclic Sync Velocity mode)"; break;
            case 10: cout << "Cst (Cyclic Sync Torque mode)"; break;
            default: cout << "Reserved/Other (keep last mode) [" << (int)mode << "]"; break;
        }
};

void EthercatDriver::Mode_Switching(int new_mode)
{
    
    cout << "operation modes switching : "
    switch (new_mode) {
            case 0:  cout << "No mode change/assigned"; break;
            case 1:  cout << "Pp (Profile Position mode)"; break;
            case 2:  cout << "Reserved (keep last mode)"; break;
            case 3:  cout << "Pv (Profile Velocity mode)"; break;
            case 4:  cout << "Tq (Profile Torque mode)"; break;
            case 6:  cout << "Hm (Homing mode)"; break;
            case 7:  cout << "Ip (Interpolated Position mode)"; break;
            case 8:  cout << "Csp (Cyclic Sync Position mode)"; break;
            case 9:  cout << "Csv (Cyclic Sync Velocity mode)"; break;
            case 10: cout << "Cst (Cyclic Sync Torque mode)"; break;
            default: cout << "Reserved/Other (keep last mode) [" << (int)mode << "]"; break;
        }
    EC_WRITE_U8(domain1_pd +offset_modes_of_operation[i], new_mode);
    


}
