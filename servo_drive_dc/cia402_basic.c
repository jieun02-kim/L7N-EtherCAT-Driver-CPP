#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h> /* clock_gettime() */
#include <sys/mman.h> /* mlockall() */
#include <sched.h> /* sched_setscheduler() */
#include <pthread.h>
#include <math.h>

/****************************************************************************/

#include "ecrt.h"
#include "cia402_basic.h"

/****************************************************************************/

#ifndef M_PI
#define M_PI 3.14159265358979323846  
#endif

#ifndef TWO_M_PI
#define TWO_M_PI 6.28318530718
#endif

/** Task period in ns. */
#define PERIOD_NS   (1000000)

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is
                                     guranteed safe to access without
                                     faulting */

/****************************************************************************/

/* Constants */
#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)
#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
/****************************************************************************/

// EtherCAT
ec_master_t *master = NULL;
ec_master_state_t master_state = {};

ec_domain_t *domain1 = NULL;
ec_domain_state_t domain1_state = {};

ec_slave_config_t *slave_config = NULL;
ec_slave_config_state_t slave_config_state = {};
unsigned int sync_ref_counter = 0;
struct timespec cycletime = {0, PERIOD_NS};
/****************************************************************************/

// process data
uint8_t *domain1_pd = NULL;

#define SERVO_MOTOR 0x00007595, 0x00000000 // Product Number 확인 필요
#define SERVO_MOTOR_NUM 4
#define ALIAS_POSITION(x) 	0,x
#define _ALLNODE 		(-5)
#define ENCODER_RESOLUTION			524288

// offsets for PDO entries - MDP module for PPM RxPDO   master > slave
unsigned int offset_control_word[SERVO_MOTOR_NUM];
unsigned int offset_target_position[SERVO_MOTOR_NUM];
unsigned int offset_target_velocity[SERVO_MOTOR_NUM];
unsigned int offset_modes_of_operation[SERVO_MOTOR_NUM];
unsigned int offset_target_torque[SERVO_MOTOR_NUM];

// offsets for PDO entries - MDP module for PPM TxPDO   slave > master
unsigned int offset_status_word[SERVO_MOTOR_NUM];
unsigned int offset_position_actual_value[SERVO_MOTOR_NUM];
unsigned int offset_velocity_actual_value[SERVO_MOTOR_NUM];
unsigned int offset_modes_of_operation_display[SERVO_MOTOR_NUM];
unsigned int offset_torque_actual_value[SERVO_MOTOR_NUM];

#define SERVO_MOTOR_REGS(x) \
	{ALIAS_POSITION(x), SERVO_MOTOR, 0x6040, 0x00, &offset_control_word[x]}, \
	{ALIAS_POSITION(x), SERVO_MOTOR, 0x607A, 0x00, &offset_target_position[x]}, \
    {ALIAS_POSITION(x), SERVO_MOTOR, 0x60FF, 0x00, &offset_target_velocity[x]}, \
	{ALIAS_POSITION(x), SERVO_MOTOR, 0x6060, 0x00, &offset_modes_of_operation[x]}, \
    {ALIAS_POSITION(x), SERVO_MOTOR, 0x6071, 0x00, &offset_target_torque[x]}, \
    {ALIAS_POSITION(x), SERVO_MOTOR, 0x6041, 0x00, &offset_status_word[x]}, \
	{ALIAS_POSITION(x), SERVO_MOTOR, 0x6064, 0x00, &offset_position_actual_value[x]}, \
	{ALIAS_POSITION(x), SERVO_MOTOR, 0x606C, 0x00, &offset_velocity_actual_value[x]}, \
	{ALIAS_POSITION(x), SERVO_MOTOR, 0x6061, 0x00, &offset_modes_of_operation_display[x]}, \
	{ALIAS_POSITION(x), SERVO_MOTOR, 0x6077, 0x00, &offset_torque_actual_value[x]}



const ec_pdo_entry_reg_t domain1_regs[] = 
{
    SERVO_MOTOR_REGS(0),
	SERVO_MOTOR_REGS(1),
    SERVO_MOTOR_REGS(2),
    SERVO_MOTOR_REGS(3),
    {}
};


/*****************************************************************************/
ec_pdo_entry_info_t servo_motor_pdo_entries[] = 
{
    //TxPDO
    {0x6040, 0x00, 16},	// control word
    {0x607A, 0x00, 32},	// target position
    {0x60FF, 0x00, 32},	// target velocity
    {0x6060, 0x00, 8},  // modes_of_operation
    {0x6071, 0x00, 16}, // target torque



    //RxPDO
    {0x6041, 0x00, 16},	// status word
    {0x6064, 0x00, 32},	// position actual value
    {0x606C, 0x00, 32},	// velocity actual value
    {0x6061, 0x00, 8}, // modes_of_operation display
    {0x6077, 0x00, 16}, // torque actual value
};

ec_pdo_info_t servo_motor_pdos[] = 
{
    //
    {0x1600,	5,	servo_motor_pdo_entries + 0},

    {0x1a00,	5,	servo_motor_pdo_entries + 5}
    
};

//slave sync manager
ec_sync_info_t servo_motor_syncs[] = {
	{ 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
	{ 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
	{ 2, EC_DIR_OUTPUT, 1, servo_motor_pdos + 0, EC_WD_DISABLE },
	{ 3, EC_DIR_INPUT,  1, servo_motor_pdos + 1, EC_WD_DISABLE },
	{ 0xff }
};

/*****************************************************************************/

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    } else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

/*****************************************************************************/

void check_domain1_state(void)
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

/*****************************************************************************/


void check_master_state(void)
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

/*****************************************************************************/

void check_slave_config_states(void)
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

/*****************************************************************************/

uint16_t status_word;
uint32_t error;

#define	NOT_READY_TO_SWITCH_ON      0x0000
#define	SWITCH_ON_DISABLED		    0x0040
#define	READY_TO_SWITCH_ON		    0x0021
#define	SWITCH_ON_ENABLED           0x0023
#define	OPERATION_ENABLED		    0x0027
#define	QUICK_STOP_ACTIVE		    0x0007
#define	FAULT_REACTION_ACTIVE		0x000F
#define	FAULT				        0x0008


#define SHUTDOWN				0x0006
#define READY				    0x0006
#define	SWITCH_ON				0x0007
#define	SWITCH_OFF			    0x0007
#define	DISABLE_VOLTAGE		    0x0000
#define	QUICK_STOP			    0x0002
#define	DISABLE_OPERATION		0x0007
#define	ENABLE_OPERATION		0x000F
#define	FAULT_RESET			    0x0080


unsigned short GetStatusVal(unsigned short statusWord)
{
	if((statusWord & 0x4F) == 0x00) {
		statusWord = NOT_READY_TO_SWITCH_ON;
	}
	else if((statusWord & 0x4F) == 0x08) {
		statusWord =  FAULT;
	}
	else if((statusWord & 0x4F) == 0x40) {
		statusWord =  SWITCH_ON_DISABLED;
	}
	else if((statusWord & 0x6F) == 0x27) {
		statusWord =  OPERATION_ENABLED;
	}
	else if((statusWord & 0x6F) == 0x23) {
		statusWord =  SWITCH_ON_ENABLED;
	}
	else if((statusWord & 0x6F) == 0x21) {
		statusWord =  READY_TO_SWITCH_ON;
	}
	else if((statusWord & 0x6F) == 0x07) {
		statusWord =  QUICK_STOP_ACTIVE;
	}
	else if((statusWord & 0x4F) == 0x0F) {
		statusWord =  FAULT_REACTION_ACTIVE;
	}
	else {
		return 0xFFFF;
	}
	return statusWord;
}

unsigned short GetStatusWordN(int iNode)
{
	unsigned short statusWord;

	statusWord = GetStatusVal(EC_READ_U16(domain1_pd + offset_status_word[iNode]));

	return statusWord;
}


int StatusWord[SERVO_MOTOR_NUM] = {0,};
int PrevStatusWord[SERVO_MOTOR_NUM] = {0,};

int nRun = 0;

double dt[SERVO_MOTOR_NUM]= {0., };
int nFirstPos[SERVO_MOTOR_NUM] = {0,};
int nFirstVel[SERVO_MOTOR_NUM] = {0,};
int nFirstTor[SERVO_MOTOR_NUM] = {0,};

int nTargetPos[SERVO_MOTOR_NUM] = {0,};
int nTargetVel[SERVO_MOTOR_NUM] = {0,};
int nTargetTor[SERVO_MOTOR_NUM] = {0,};



void cyclic_task()
{
    struct timespec wakeupTime, cur_time;

    // get current time
    clock_gettime(CLOCK_MONOTONIC, &wakeupTime);


    while(1)
    {
        wakeupTime = timespec_add(wakeupTime, cycletime);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeupTime, NULL);

        // Write application time to master
        //
        // It is a good idea to use the target time (not the measured time) as
        // application time, because it is more stable.
        //
        ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));

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
            for(int i=0; i < SERVO_MOTOR_NUM; ++i)
            {
                
                StatusWord[i] = GetStatusWordN(i);
                // sanyoActualVelR[i] = sanyoGetActualVelocityN(i);

                if(PrevStatusWord[i] == StatusWord[i])
                {
                    continue;
                }
                switch(StatusWord[i])
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
                        nFirstPos[i] = EC_READ_S32(domain1_pd + offset_position_actual_value[i]);
                        EC_WRITE_S32(domain1_pd + offset_target_position[i], nFirstPos[i]);
                        nRun += 1;
                        break;
                    case FAULT:
                        nRun -= 1;
                        EC_WRITE_U16(domain1_pd + offset_control_word[i],FAULT_RESET);
                        break;
                    default:
                        break;
                }
                PrevStatusWord[i] = StatusWord[i]; 
            }

            if (nRun >= SERVO_MOTOR_NUM)
            {
                for (int j=0; j< SERVO_MOTOR_NUM; j++)
                {
                    nTargetPos[j] = nFirstPos[j] + (ENCODER_RESOLUTION/2 * sin(TWO_M_PI * 0.1 *  dt[j]));
                    EC_WRITE_S8(domain1_pd + offset_modes_of_operation[j], 0x08);
                    EC_WRITE_S32(domain1_pd + offset_target_position[j], nTargetPos[j]);
                    dt[j] += 1/1000.;
                }
            }
        }
        
        if (sync_ref_counter) 
        {
            sync_ref_counter--;
        } else 
        {
            sync_ref_counter = 1; // sync every cycle
            clock_gettime(CLOCK_MONOTONIC, &cur_time);
            ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(cur_time));
        }
        ecrt_master_sync_slave_clocks(master);

        // send process data
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
    }
}

/****************************************************************************/

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
}

/*********************************************************************/
// functions for wrapping to c++ driver
// CAUTION!!!!!!!! : This section may be removed at any time.


// // 1. initial : request master, create domain, PDO...
// int ec_setup_test(void)
// {
//     int ret = 0;

//     master = ecrt_request_master(0);
//     if (!master)
//     {
//         fprintf(stderr, "Failed to create master.\n");
//         return -1;
//     }

//     domain1 = ecrt_master_create_domain(master);
//     if (!domain1) 
//     {
//         fprintf(stderr, "Failed to create domain.\n");
//         return -1;
//     }
    
//     printf("Configuring PDOs...\n"); 
//     for (int i=0; i<SERVO_MOTOR_NUM; i++)
//     {
//         printf("SLAVE %d ecrt_master_slave_config \n", i);
//         if (!(slave_config = ecrt_master_slave_config(master, 0, i, SERVO_MOTOR))) 
//         {
//             fprintf(stderr, "Failed to get slave configuration.\n");
//             return -1;
//         }

//         printf("SLAVE %d ecrt_slave_config_pdos \n", i);
//         if (ecrt_slave_config_pdos(slave_config, EC_END, servo_motor_syncs)) 
//         {
//             fprintf(stderr, "Failed to configure PDOs.\n");
//             return -1;
//         }
//         ecrt_slave_config_dc(slave_config, 0x0300, PERIOD_NS, 12500, 0, 0);
//     }
 
//     if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) 
//     {
//         fprintf(stderr, "PDO entry registration failed!\n");
//         return -1;
//     }

//     printf("Activating master...\n");
//     if (ecrt_master_activate(master)) {
//         return -1;
//     }

//     printf("Activating domain...\n");
//     if (!(domain1_pd = ecrt_domain_data(domain1))) {
//         return -1;
//     }

//     // Set priority 
//     struct sched_param param = {};
//     param.sched_priority = sched_get_priority_max(SCHED_FIFO);

//     printf("Using priority %i.", param.sched_priority);
//     if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
//         perror("sched_setscheduler failed");
//     }

//     // Lock memory 
//     if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
//         fprintf(stderr, "Warning: Failed to lock memory: %s\n",
//                 strerror(errno));
//     }

//     stack_prefault();

//     printf("Starting RT task with dt=%u ns.\n", PERIOD_NS);
//     return ret;
// }

// // 2. run : Periodic communication task
// void ec_run_cycle_test(struct timespec *wakeupTime)
// {

//     *wakeupTime = timespec_add(*wakeupTime, cycletime);
//     clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, wakeupTime, NULL);

//     // Write application time to master
//     //
//     // It is a good idea to use the target time (not the measured time) as
//     // application time, because it is more stable.
//     //
//     ecrt_master_application_time(master, TIMESPEC2NS(*wakeupTime));

//     // receive process data
//     ecrt_master_receive(master);
//     ecrt_domain_process(domain1);

    
//     // check master state
//     check_master_state();

//     // check slave config states
//     check_slave_config_states();
    
//     // check process data state
//     check_domain1_state();

//     if (domain1_state.wc_state == 2)
//     {
//         for(int i=0; i < SERVO_MOTOR_NUM; ++i)
//         {
            
//             StatusWord[i] = GetStatusWordN(i);
//             // sanyoActualVelR[i] = sanyoGetActualVelocityN(i);

//             if(PrevStatusWord[i] == StatusWord[i])
//             {
//                 continue;
//             }
//             switch(StatusWord[i])
//             {	
//                 case SWITCH_ON_DISABLED:
//                     EC_WRITE_U16(domain1_pd + offset_control_word[i],READY);
//                     break;
//                 case READY_TO_SWITCH_ON:
//                     EC_WRITE_U16(domain1_pd + offset_control_word[i],SWITCH_ON);
//                     break;
//                 case SWITCH_ON_ENABLED:
//                     EC_WRITE_U16(domain1_pd + offset_control_word[i], ENABLE_OPERATION);
//                     break;
//                 case OPERATION_ENABLED:
//                     nFirstPos[i] = EC_READ_S32(domain1_pd + offset_position_actual_value[i]);
//                     EC_WRITE_S32(domain1_pd + offset_target_position[i], nFirstPos[i]);
//                     nRun += 1;
//                     break;
//                 case FAULT:
//                     nRun -= 1;
//                     EC_WRITE_U16(domain1_pd + offset_control_word[i],FAULT_RESET);
//                     break;
//                 default:
//                     break;
//             }
//             PrevStatusWord[i] = StatusWord[i]; 
//         }

//         if (nRun >= SERVO_MOTOR_NUM)
//         {
//             for (int j=0; j< SERVO_MOTOR_NUM; j++)
//             {
//                 nTargetPos[j] = nFirstPos[j] + (ENCODER_RESOLUTION/2 * sin(TWO_M_PI * 0.1 *  dt[j]));
//                 EC_WRITE_S8(domain1_pd + offset_modes_of_operation[j], 0x08);
//                 EC_WRITE_S32(domain1_pd + offset_target_position[j], nTargetPos[j]);
//                 dt[j] += 1/1000.;
//             }
//         }
//     }
    
//     if (sync_ref_counter) 
//     {
//         sync_ref_counter--;
//     } else 
//     {
//         sync_ref_counter = 1; // sync every cycle
//         clock_gettime(CLOCK_MONOTONIC, &cur_time);
//         ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(cur_time));
//     }
//     ecrt_master_sync_slave_clocks(master);

//     // send process data
//     ecrt_domain_queue(domain1);
//     ecrt_master_send(master);

// }

// 3.Data exchange


/****************************************************************************/
/*
int main(int argc, char **argv)
{
    // struct timespec wakeup_time;
    int ret = 0;

    master = ecrt_request_master(0);
    if (!master) 
    {
        fprintf(stderr, "Failed to create master.\n");
        return -1;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) 
    {
        fprintf(stderr, "Failed to create domain.\n");
        return -1;
    }
    
    printf("Configuring PDOs...\n"); 
    for (int i=0; i<SERVO_MOTOR_NUM; i++)
    {
        printf("SLAVE %d ecrt_master_slave_config \n", i);
        if (!(slave_config = ecrt_master_slave_config(master, 0, i, SERVO_MOTOR))) 
        {
            fprintf(stderr, "Failed to get slave configuration.\n");
            return -1;
        }

        printf("SLAVE %d ecrt_slave_config_pdos \n", i);
        if (ecrt_slave_config_pdos(slave_config, EC_END, servo_motor_syncs)) 
        {
            fprintf(stderr, "Failed to configure PDOs.\n");
            return -1;
        }
        ecrt_slave_config_dc(slave_config, 0x0300, PERIOD_NS, 12500, 0, 0);
    }
 
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) 
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        return -1;
    }

    printf("Activating domain...\n");
    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }

    // Set priority 
    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i.", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

    // Lock memory 
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n",
                strerror(errno));
    }

    stack_prefault();

    printf("Starting RT task with dt=%u ns.\n", PERIOD_NS);

    // clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    // wakeup_time.tv_sec += 1; // start in future
    // wakeup_time.tv_nsec = 0;

    // while (1) 
    // {
    //     ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
    //             &wakeup_time, NULL);
    //     if (ret) 
    //     {
    //         fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
    //         break;
    //     }
        
    //     cyclic_task();

    //     wakeup_time.tv_nsec += PERIOD_NS;
    //     while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
    //         wakeup_time.tv_nsec -= NSEC_PER_SEC;
    //         wakeup_time.tv_sec++;
    //     }
    // }
    printf("Starting cyclic function.\n");
    cyclic_task();
    return ret;
}
*/

/****************************************************************************/

