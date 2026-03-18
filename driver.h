//#pragma once ??
#ifndef __DRIVER_H_
#define __DRIVER_H_

#include <cstdint>
#include "ecrt.h"
#include "cia402_basic.h"
#include <vector>
#include <atomic>
#include <iostream>
#include <thread>
using namespace std;

//========================================================================
// Macro... ect
//========================================================================




//========================================================================
// 추후 추가할 config 등 자잘 한 기능들 (현재는 cia402 베이직 코드에서 extern하여 사용중)
//========================================================================


// class MotorConfig
// {
//     public:
//         uint32_t vendor_id;
//         uint32_t product_code;
//         uint16_t alias;
//         uint16_t position;
        
//         // 모터마다 다를 수 있는 제어 파라미터
//         uint32_t drive_mode; // 예: Profile Position, CSP 등
//         int32_t max_pos;
//         int32_t min_pos;
        
//         // 기본 생성자 (초기값 설정)
//         MotorConfig() : vendor_id(0x000000E0), product_code(0x00000000), 
//                         alias(0), position(0), drive_mode(8) {}
// };


// // 1. 통신 전용 클래스 (기종별로 상속 가능)
// class ServoMotor_PDO {
//     public:
//         struct Offsets {
//             // Master -> Slave (RxPDO)
//             unsigned int ctrl_word;   // 0x6040
//             unsigned int target_pos;  // 0x607A
//             unsigned int target_vel;  // 0x60FF
//             unsigned int mode_op;     // 0x6060
//             unsigned int target_tor;  // 0x6071

//             // Slave -> Master (TxPDO)
//             unsigned int status_word; // 0x6041
//             unsigned int act_pos;     // 0x6064
//             unsigned int act_vel;     // 0x606C
//             unsigned int mode_disp;   // 0x6061
//             unsigned int act_tor;     // 0x6077
//         } offset;

//         // 통신으로 직접 오고가는 Raw 데이터
//         uint16_t status;
//         int32_t  actual_pos_raw;
//         int8_t   active_mode;

//         virtual void register_pdo(...) = 0; // 자식 클래스에서 구현
// };


//========================================================================
// servo motor config... and  actual value for changing mode
//========================================================================


// 2. 실제 제어 클래스 (사용자가 직접 다루는 객체)
class ServoMotor {
    private:
        
        // 추후 슬레이브 변경에 대한 기능 구현 시 사용. 
        // 지금은 cia402 basic 코드에서 사용하는 extern된 객체 사용 
        //ServoMotor_PDO* pdo; // 통신 객체를 품고 있음

        

        // // [제어 계산용 변수]      
        // double dt;
        // int32_t first_pos;
        // int32_t first_vel;
        // int32_t first_tor;



    public:
        // [물리 파라미터]
        int encoder;      
        int gear_ratio;   
        float joint_limit;   // 아직 안 씀
        double amplification; // 엔코더 * 기어비 (증폭비)


        //ServoMotor(ServoMotor_PDO* pdo_ptr) : pdo(pdo_ptr), encoder(524288), gear_ratio(1) {}
        ServoMotor() :  encoder(524288), gear_ratio(1) {update_amp();}

        void Set_parameter()
        {
            cout<<"encoder -> 524288 : "; cin>>encoder;
            cout<<"gear_ratio -> : "; cin>>gear_ratio;
            update_amp();
        }

        void update_amp() {
            amplification = (double)encoder * gear_ratio;
        }

        double Get_amplification() const { return amplification; }

        // 각도로 명령을 내림
        void SetTargetDegree(float degree)
        {
            // 내부에서 pdo->target_pos_raw = (계산값) 처리를 수행
        }
};







//========================================================================
// create new Ethercat object and init
//========================================================================
class EthercatDriver
{
    public:
        EthercatDriver();
        ~EthercatDriver();

        //init
        int Init_Master(void);
        //int Config_MS(ec_domain_t *d, ec_master_t *m, uint16_t alias, uint16_t position, uint32_t vendor_id, uint32_t product_code);
        int Config_MS(void);
        int Activate_Master_Domain(void);
        int Set_Priority(void);
        int Lock_Memory(void);
        void Init_ec_loop(void);
        void cyclic_task(void);
        
        // mode check and switching
        void Display_modes(int motor_num);
        void Mode_Switching(int new_mode, int motor_num);
 

        ec_pdo_entry_reg_t* create_domain1_regs(int motor_num, uint32_t v_id, uint32_t p_id);
        void release_domain1_regs(ec_pdo_entry_reg_t*& regs);
        void initial_control(int i);
        void control_motor(int j);
        void check_master_state(void);
        void check_slave_config_states(void);
        void check_domain1_state(void);
        unsigned short GetStatusWordN(int iNode);
        void reset_default_motor_num(void);
        bool is_stopped(uint16_t statusWord, int i);
        void RequestModeChange(int8_t new_mode);
        void RunMenu(void);

        std::vector<int8_t> next_mode;
        std::vector<int8_t> current_mode;

        std::vector<uint16_t> StatusWord;
        std::vector<uint16_t> PrevStatusWord;
        std::vector<double> dt;
        std::vector<int32_t> nFirstPos, nTargetPos;
        std::vector<int32_t> nFirstVel, nTargetVel;
        std::vector<int16_t> nFirstTor, nTargetTor;
        int nRun = 0; // 클래스 내부에서 관리할 카운터
        
        // servo motor count
        int default_motor_num = 4;


    private:
        // EtherCAT object
        ec_master_t *master = nullptr;
        ec_master_state_t master_state = {};

        ec_domain_t *domain1 = nullptr;
        ec_domain_state_t domain1_state = {};

        ec_slave_config_t *slave_config = nullptr;
        ec_slave_config_state_t slave_config_state = {};
        unsigned int sync_ref_counter = 0;
        struct timespec cycletime = {0, PERIOD_NS};

        // Time object
        struct timespec wakeupTime;
        struct timespec m_time;

        // process data
        uint8_t *domain1_pd = NULL;

        

        // servo motor object
        std::vector<ServoMotor*> motors;


    };





#endif