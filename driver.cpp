#include "driver.h"
#include "cia402_basic.h"
#include <iostream>
#include <sys/mman.h> 
#include <signal.h>
using namespace std;



//========================================================================
// logger and macro
//========================================================================
#define print_error(format,...) \
do { \
    fprintf(stderr, "[EtherCAT] : " format ".\n", ##__VA_ARGS__); \
    return -1; \
} while(0)

// This macro is for pointer when 0 means fail
#define check_error(name, format,...) \
do { \
    if (!(name)) { \
        print_error(format, ##__VA_ARGS__); \
    } \
} while(0)

// This macro is for functions where 0 means success
#define check_is_success(name, format,...) \
do { \
    if (name != 0) { \
        print_error(format, ##__VA_ARGS__); \
    } \
} while(0)



//========================================================================
// logger and macro
//========================================================================

bool keep_running = true;

// Ctrl+C를 누르면 실행될 함수
void signal_handler(int sig) {
    keep_running = false;
}

atomic<bool> mode_change_req{false};
//bool mode_change_req = {false}; // 모드 전환 요청


//========================================================================
// Class Constructor and Destructor
//========================================================================

EthercatDriver::EthercatDriver() {
    cout << "[EtherCAT] : Creating EthercatDriver object..." << endl;
}

EthercatDriver::~EthercatDriver() {
    cout << "[EtherCAT] : Ethercat Destructor..." << endl;
    
    for (auto m : motors) {
        delete m;
    }
    motors.clear();
    cout << "[EtherCAT] : Disappearing servo motor object..." << endl;
    

    // release master object. (extern으로 가져온 master 변수 사용)
    if (master) {
        ecrt_release_master(master);
        master = nullptr;
    }
    cout << "[EtherCAT] : Disappearing EthercatDriver object..." << endl;
    
}




//========================================================================
// function
//========================================================================

int EthercatDriver::Init_Master(void)
{
    int ret = 0;
    cout << "Starting EtherCAT Master..." << endl;

    master = ecrt_request_master(0);
    check_error(master, "Failed to create domain.\n");

    domain1 = ecrt_master_create_domain(master);
    check_error(domain1, "Failed to create domain.\n");

    return ret;
}


int EthercatDriver::Config_MS(void)
{
    int ret = 0;
    printf("Configuring PDOs...\n");
    check_error(master && domain1, "Master initial error!\nCall Init_Master() first.");

    for (int i=0; i<default_motor_num ; i++)
    {
        // 1. 설정 객체 생성
        printf("SLAVE %d ecrt_master_slave_config \n", i);
        slave_config = ecrt_master_slave_config(master, 0, i, SERVO_MOTOR);
        check_error(slave_config, "Failed to get slave configuration.\n");

        // 3. sdo 설정 예약        
        // 0x605B(Shutdown Option)를 1로 설정하도록 예약
        ecrt_slave_config_sdo16(slave_config, 0x605B, 0, 1);
        // 0x605A(Quick Stop Option)를 2로 설정하도록 예약
        ecrt_slave_config_sdo16(slave_config, 0x605A, 0, 2);
        ecrt_slave_config_sdo8(slave_config, 0x6060, 0, 8); // 시작할 때 무조건 CSP(8)       

        // 3. pdo 설정
        printf("SLAVE %d ecrt_slave_config_pdos \n", i);
        check_is_success(ecrt_slave_config_pdos(slave_config, EC_END, servo_motor_syncs), "Failed to configure PDOs.\n");
        
        ecrt_slave_config_dc(slave_config, 0x0300, PERIOD_NS, 12500, 0, 0);
    }
    check_is_success(ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs), "PDO entry registration failed!\n");
    


    
    
    return ret;
}

/*
int EthercatDriverbool InitMember(int enc, int gear, float jnt, float pos);
{
    int ret = 0;
    printf("Configuring PDOs...\n");
    check_error(m && d, "Master initial error!\nCall Init_Master() first.");

    for (int i = 0; i < motor_num; i++)
    {
        printf("SLAVE %d ecrt_master_slave_config \n", i);
        slave_config = ecrt_master_slave_config(m, ali, i, v_id, p_code);
        check_error(slave_config, "Failed to get slave configuration.\n");

        printf("SLAVE %d ecrt_slave_config_pdos \n", i);
        check_is_success(ecrt_slave_config_pdos(slave_config, EC_END, servo_motor_syncs), "Failed to configure PDOs.\n");

        ecrt_slave_config_dc(slave_config, 0x0300, PERIOD_NS, 12500, 0, 0);
    }
    domain1_regs = create_domain1_regs(motor_num, v_id, p_code);
    check_is_success(ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs), "PDO entry registration failed!\n");
    return ret;
}
*/







int EthercatDriver::Activate_Master_Domain(void)
{
    int ret = 0;
    printf("Activating master...\n");
    check_is_success(ecrt_master_activate(master), "Failed to activate master!!!");
    printf("Activating domain...\n");
    domain1_pd = ecrt_domain_data(domain1);
    check_error(domain1_pd, "Failed to activate domain!!!");
    return ret;
}


// Set priority 
int EthercatDriver::Set_Priority(void)
{
    int ret = 0;
    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
 

    printf("[EtherCAT] : Using priority %i.", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }
    return ret;
}

// lock memory
int EthercatDriver::Lock_Memory(void)
{
    int ret = 0;
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        fprintf(stderr, "[EtherCAT] : Warning: Failed to lock memory: %s\n",
                strerror(errno));
    }
    stack_prefault();

    printf("Starting RT task with dt=%u ns.\n", PERIOD_NS);
    return ret;
}

// Loop
void EthercatDriver::Init_ec_loop(){

    printf("Starting cyclic function.\n");
    // get current time
    clock_gettime(CLOCK_MONOTONIC, &this->wakeupTime);

    while(keep_running){
        cyclic_task();
    }
}

void EthercatDriver::cyclic_task(void)
{
    this->wakeupTime = timespec_add(this->wakeupTime, cycletime);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &this->wakeupTime, NULL);

    // Write application time to master
    //
    // It is a good idea to use the target time (not the measured time) as
    // application time, because it is more stable.
    //
    ecrt_master_application_time(master, TIMESPEC2NS(this->wakeupTime));
    

    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);


    // check master state
    check_master_state();

    // check slave config states
    check_slave_config_states();

    // check process data state
    check_domain1_state();

    if (domain1_state.wc_state == 2)
    {
        nRun = 0;
        bool all_modes_synced = true; // 모든 모터가 목표 모드에 도달했는지 체크

        for(int i=0; i < default_motor_num; ++i)
        {
            StatusWord[i] = GetStatusWordN(i);
            uint16_t masked_status = StatusWord[i] & 0x006F; // 0000 0000 0110 1111
            int8_t actual_mode = EC_READ_S8(domain1_pd + offset_modes_of_operation_display[i]);


            // 상태가 0x27이면 nRun을 올림
            if (masked_status == 0x0027) { //0000 0000 0010 0111 : Operation enabled
                nRun++;
            }

            // 실제 모드가 목표 모드와 다르면 아직 동기화 안 됨
            if (actual_mode != next_mode[i]) {
                all_modes_synced = false;
            }

            // 모드 전환 버튼이나 플래그가 눌렸을 때 mode change
            if (mode_change_req.load())
            {
                this->initial_control(i);
                //EC_WRITE_U16(domain1_pd + offset_control_word[i], QUICK_STOP); // Quick Stop
                EC_WRITE_S8(domain1_pd + offset_modes_of_operation[i], next_mode[i]);


                if (i == default_motor_num - 1)
                {
                    mode_change_req.store(false);
                    printf("[System] Mode Change Success: -> %d\n", next_mode[i]);
                }

                // if (is_stopped(StatusWord[i], i))
                // {
                //     current_mode[i] = next_mode[i];
                //     EC_WRITE_S8(domain1_pd + offset_modes_of_operation[i], current_mode[i]);
                //     this->initial_control(i); // 여기서 nFirstPos 등을 현재 값으로 동기화!
                //     EC_WRITE_U16(domain1_pd + offset_control_word[i], ENABLE_OPERATION); // 다시 시작

                //     if (i == default_motor_num - 1)
                //     {
                //         mode_change_req.store(false);
                //     }
                // }
                all_modes_synced = false;
                
            }

            
            

            // if(PrevStatusWord[i] == StatusWord[i])
            // {
            //     continue;
            // }

            // 이전 마스킹된 상태와 같으면 스킵 (불필요한 로그 방지)
            if (PrevStatusWord[i] != masked_status)
            {
                switch(masked_status)
                {	
                    case SWITCH_ON_DISABLED:
                        EC_WRITE_U16(domain1_pd + offset_control_word[i],READY);
                        break;
                    case READY_TO_SWITCH_ON:
                        EC_WRITE_U16(domain1_pd + offset_control_word[i],SWITCH_ON);
                        break;
                    case SWITCH_ON_ENABLED:
                        EC_WRITE_U16(domain1_pd + offset_control_word[i], ENABLE_OPERATION);
                        break;
                    case OPERATION_ENABLED:
                        this->initial_control(i);
                        //this->initial_control(i);
                        
                        // nFirstPos[i] = EC_READ_S32(domain1_pd + offset_position_actual_value[i]);
                        // EC_WRITE_S32(domain1_pd + offset_target_position[i], nFirstPos[i]);
                        //nRun += 1;
                        break;
                    case FAULT:
                        //nRun -= 1;
                        EC_WRITE_U16(domain1_pd + offset_control_word[i],FAULT_RESET);
                        break;
                    //default:
                    //    break;
                }
                PrevStatusWord[i] = masked_status; 
            }
        }



        if (nRun >= default_motor_num && all_modes_synced)
    {
        for (int j=0; j< default_motor_num; j++)
        {
            this->control_motor(j);
        }
    }



    }
    
    if (sync_ref_counter) 
    {
        sync_ref_counter--;
    } else 
    {
        sync_ref_counter = 1; // sync every cycle
        clock_gettime(CLOCK_MONOTONIC, &this->m_time);
        ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(this->m_time));
    }
    ecrt_master_sync_slave_clocks(master);

    // send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);

}

//========================================================================
// mode check and switching
//========================================================================

// for monitoring, don't use it at loop
void EthercatDriver::Display_modes(int motor_num)
{

    for(int i=0; i<motor_num; i++)
    {
        int8_t modes_of_operation = EC_READ_U8(domain1_pd + offset_modes_of_operation_display[i]);
        if(StatusWord[i]!=OPERATION_ENABLED)
        {
            cout << "Motor Operation Modes [" << i << "]: ";
            switch (modes_of_operation)
            {
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
                default: cout << "Reserved/Other (keep last mode) [" << (int)modes_of_operation << "]"; break;
            }
            cout << endl;
        }
    }
    
};



void EthercatDriver::Mode_Switching(int new_mode, int motor_num)
{
    
    for(int i=0; i<motor_num; i++)
    {
        EC_WRITE_U8(domain1_pd +offset_modes_of_operation[i], new_mode);
        StatusWord[i] = GetStatusWordN(i);
        uint16_t masked_status = StatusWord[i] & 0x006F;
        
        if(masked_status!=OPERATION_ENABLED)
        {
            cout << "operation modes switching [" << i <<"]: ";
            switch (new_mode)
            {
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
                default: cout << "Reserved/Other (keep last mode) [" << (int)new_mode << "]"; break;
            }
            cout << endl;
        } 
    }
};

void EthercatDriver::initial_control(int i)
{
    dt[i] = 0;

    nFirstPos[i] = EC_READ_S32(domain1_pd + offset_position_actual_value[i]);
    EC_WRITE_S32(domain1_pd + offset_target_position[i], nFirstPos[i]);
    nFirstVel[i] = 0;
    EC_WRITE_S32(domain1_pd + offset_target_velocity[i], nFirstVel[i]);
    nFirstTor[i] = 0;
    EC_WRITE_S16(domain1_pd + offset_target_torque[i], nFirstTor[i]); 
}
// void EthercatDriver::initial_control(int i)
// {
//     dt[i] = 0;

//     switch (modes_of_operation)
//     {
//         case 0x08:
//             nFirstPos[i] = EC_READ_S32(domain1_pd + offset_position_actual_value[i]);
//             EC_WRITE_S32(domain1_pd + offset_target_position[i], nFirstPos[i]); break;

//         case 0x09:
//             nFirstVel[i] = 0;
//             EC_WRITE_S32(domain1_pd + offset_target_velocity[i], nFirstVel[i]); break;

//         case 0x0A:
//             nFirstTor[i] = 0;
//             EC_WRITE_S16(domain1_pd + offset_target_torque[i], nFirstTor[i]); break;
//     }
// }



void EthercatDriver::control_motor(int j)
{
    int8_t modes_of_operation = EC_READ_U8(domain1_pd + offset_modes_of_operation_display[j]);
    double amp = motors[j]->Get_amplification(); // encoder*gear_ratio;

    switch (modes_of_operation)
    {
        case 0x08:
            nTargetPos[j] = nFirstPos[j] + (amp/2 * sin(TWO_M_PI * 0.1 *  dt[j]));
            EC_WRITE_S8(domain1_pd + offset_modes_of_operation[j], 0x08);
            EC_WRITE_S32(domain1_pd + offset_target_position[j], nTargetPos[j]);
            dt[j] += 1/1000.; break;
            

        case 0x09:    
            nTargetVel[j] = nFirstVel[j] + amp*2* 1 * sin(TWO_M_PI * 0.1 * dt[j]); 
            EC_WRITE_S8(domain1_pd + offset_modes_of_operation[j], 0x09);
            EC_WRITE_S32(domain1_pd + offset_target_velocity[j], nTargetVel[j]);
            dt[j] += 1/1000.; break;

        case 0x0A:
            // 현재 모터에 부하 없음.
            // 정격의 2% 힘 내에서 밀고 당기기 반복. 정격토크의 0.1%단위
            nTargetTor[j] = nFirstTor[j] + 20* sin(TWO_M_PI * 0.1 * dt[j]);
            EC_WRITE_S8(domain1_pd + offset_modes_of_operation[j], 0x0A);
            EC_WRITE_S16(domain1_pd + offset_target_torque[j], nTargetTor[j]);
            dt[j] += 1/1000.; break;
        }
    
}

bool EthercatDriver::is_stopped(uint16_t statusWord, int i) 
{
    int32_t actual_vel = EC_READ_S32(domain1_pd + offset_velocity_actual_value[i]);
    int8_t mode_feedback = EC_READ_S8(domain1_pd + offset_modes_of_operation_display[i]);
    bool stop_condition = false;
    
    if (mode_feedback == 0x09) { // CSV 모드
        // CSV에서는 비트 10이 '목표 속도 도달'이므로 속도 0 명령 시 1이 됨
        stop_condition = (statusWord & 0x0400); 
    } 
    else if (mode_feedback == 0x08 || mode_feedback == 0x0A) { // CSP 또는 CST
        // CSP/CST는 비트 10이 불안정할 수 있으므로, 실제 속도에 더 비중을 둠
        // 또는 정지 관련 다른 비트(Bit 5: Quick Stop Active 여부)를 참고
        stop_condition = true; // 실제 속도 체크로 갈음
    }

    // 최종 판단: 물리적 속도가 0에 가깝고 + 모드별 정지 조건 만족
    if (abs(actual_vel) < 5 && stop_condition) {
        return true;
    }
    
    return false;
}






void EthercatDriver::check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states) {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up) {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }
    master_state = ms;
}

void EthercatDriver::check_slave_config_states(void)
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(slave_config, &s);

    if (s.al_state != slave_config_state.al_state) {
        printf("SERVO_DRIVE: State 0x%02X.\n", s.al_state);
    }
    if (s.online != slave_config_state.online) {
        printf("SERVO_DRIVE: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != slave_config_state.operational) {
        printf("SERVO_DRIVE: %soperational.\n", s.operational ? "" : "Not ");
    }

    slave_config_state = s;
}

void EthercatDriver::check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state) {
        printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

unsigned short EthercatDriver::GetStatusWordN(int iNode)
{
	unsigned short statusWord;

	statusWord = GetStatusVal(EC_READ_U16(domain1_pd + offset_status_word[iNode]));

	return statusWord;
}

void EthercatDriver::reset_default_motor_num()
{
    // 1. 혹시 이미 생성된 모터가 있다면 메모리 해제 (중복 생성 방지)
    for (auto m : motors) {
        delete m;
    }
    motors.clear();

    // 2. 새로운 개수 세팅
    int num = this->default_motor_num;
    this->nRun = 0; // 카운터 리셋
    ServoMotor* prototype = new ServoMotor();
    prototype->Set_parameter();
    double common_amp = prototype->Get_amplification();

    // 3. 개수만큼 모터 객체 생성 및 벡터에 삽입
    for (int i = 0; i < num; i++) {
        motors.push_back(new ServoMotor(*prototype));
        printf("[Motor %d] Initialized with Prototype Amp: %f\n", i, common_amp);
    }
    delete prototype; // 원본은 역할 끝났으니 삭제

    next_mode.assign(num, 8);    // 기본 CSP(8) 모드로 초기화
    current_mode.assign(num, 8);
    StatusWord.assign(num, 0);
    PrevStatusWord.assign(num, 0);
    
    dt.assign(num, 0.0);
    nFirstPos.assign(num, 0); nTargetPos.assign(num, 0);
    nFirstVel.assign(num, 0); nTargetVel.assign(num, 0);
    nFirstTor.assign(num, 0); nTargetTor.assign(num, 0);



    printf("[EthercatDriver] %d motors initialized.\n", num);
}

void EthercatDriver::RequestModeChange(int8_t new_mode) {
    
    for(int i=0; i<default_motor_num; i++) {
        next_mode[i] = new_mode; // 어떤 모드로 바꿀지 저장
    }
    mode_change_req.store(true);
}

//========================================================================
// mode switching non rt function
//========================================================================
void EthercatDriver::RunMenu()
{
    int menu;
        while (keep_running) {
            cout << "\n-----------------------------------" << endl;
            cout << " [8] CSP | [9] CSV | [10] CST | [0] EXIT" << endl;
            cout << "-----------------------------------" << endl;
            cout << "Mode Selection: ";
            cin >> menu;
            
            if (menu == 8 || menu == 9 || menu == 10) {
                RequestModeChange(menu);
            } 
            else if (menu == 0){
                cout << "프로그램을 종료합니다..." << endl;
                keep_running = false; // 루프 탈출
                break;
            }
            else{
                cout << "[Warning] Invalid Mode! Select 8, 9, or 10." << endl;
            }

        }
}


//========================================================================
// Private Helper
// Not needed yet. Will be updated later to create a domain object.
//========================================================================
// ec_pdo_entry_reg_t* EthercatDriver::create_domain1_regs(int motor_num, uint32_t v_id, uint32_t p_id)
// {
//     // number of ec_pdo_entry_reg_t entry is 8
//     int total_size = (motor_num * 8) + 1;

//     ec_pdo_entry_reg_t* regs = new ec_pdo_entry_reg_t[total_size];

//     for (int i = 0; i < motor_num; i++) {
//         ec_pdo_entry_reg_t temp[] = { MOTOR_REGS(i, v_id, p_id) };
//         memcpy(&regs[i * 8], temp, sizeof(temp));
//     }
//     regs[motor_num * 8] = {};
//     return regs;
// }

// void EthercatDriver::release_domain1_regs(ec_pdo_entry_reg_t*& regs)
// {
//     delete[] regs;
//     regs = nullptr;
// }



//========================================================================
// test application
//========================================================================

int main(int argc, char **argv)
{
    int ret = 0;
    signal(SIGINT, signal_handler); // 시그널 등록
 
    EthercatDriver ec_driver;
    ec_driver.reset_default_motor_num();

    ec_driver.Init_Master();
    ec_driver.Config_MS();
    ec_driver.Activate_Master_Domain();
    ec_driver.Set_Priority();
    ec_driver.Lock_Memory(); 

    thread rt_thread([&]() {
        ec_driver.Init_ec_loop();
    });

    ec_driver.RunMenu();  

    cout << "\n[Main] Terminating RT thread..." << endl;
    if (rt_thread.joinable()) {
        rt_thread.join();
    }
    cout << "[Main] Program finished safely." << endl;

    return ret;
}