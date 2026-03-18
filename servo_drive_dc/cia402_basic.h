#ifndef __CIA402_BASIC_H_
#define __CIA402_BASIC_H_


#include <errno.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h> /* clock_gettime() */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <time.h>
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif
#ifndef TIMER_ABSTIME
#define TIMER_ABSTIME 1
#endif
#include <sys/mman.h> /* mlockall() */
#include <sched.h> /* sched_setscheduler() */
#include <pthread.h>
#include <math.h>
#include <stdatomic.h>
/****************************************************************************/

#include "ecrt.h"



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

// // EtherCAT
// extern ec_master_t *master;
// extern ec_master_state_t master_state;

// extern ec_domain_t *domain1;
// extern ec_domain_state_t domain1_state;

// extern ec_slave_config_t *slave_config;
// extern ec_slave_config_state_t slave_config_state;
// extern unsigned int sync_ref_counter;
// extern struct timespec cycletime;
// /****************************************************************************/

// // process data
// extern uint8_t *domain1_pd;

#define SERVO_MOTOR 0x00007595, 0x00000000 // Product Number 확인 필요
#define SERVO_MOTOR_NUM 4
#define ALIAS_POSITION(x) 	0,x
#define _ALLNODE 		(-5)
#define ENCODER_RESOLUTION			524288

// offsets for PDO entries - MDP module for PPM RxPDO   master > slave
extern unsigned int offset_control_word[SERVO_MOTOR_NUM];
extern unsigned int offset_target_position[SERVO_MOTOR_NUM];
extern unsigned int offset_target_velocity[SERVO_MOTOR_NUM];
extern unsigned int offset_modes_of_operation[SERVO_MOTOR_NUM];
extern unsigned int offset_target_torque[SERVO_MOTOR_NUM];

// offsets for PDO entries - MDP module for PPM TxPDO   slave > master
extern unsigned int offset_status_word[SERVO_MOTOR_NUM];
extern unsigned int offset_position_actual_value[SERVO_MOTOR_NUM];
extern unsigned int offset_velocity_actual_value[SERVO_MOTOR_NUM];
extern unsigned int offset_modes_of_operation_display[SERVO_MOTOR_NUM];
extern unsigned int offset_torque_actual_value[SERVO_MOTOR_NUM];

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

extern const ec_pdo_entry_reg_t domain1_regs[];

/*****************************************************************************/
extern ec_pdo_entry_info_t servo_motor_pdo_entries[];
extern ec_pdo_info_t servo_motor_pdos[];

//slave sync manager
extern ec_sync_info_t servo_motor_syncs[];
/*****************************************************************************/



/*****************************************************************************/

extern uint16_t status_word;
extern uint32_t error;

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




extern int StatusWord[SERVO_MOTOR_NUM];
extern int PrevStatusWord[SERVO_MOTOR_NUM];

extern int nRun;

extern double dt[SERVO_MOTOR_NUM];
extern int nFirstPos[SERVO_MOTOR_NUM];
extern int nFirstVel[SERVO_MOTOR_NUM];
extern int nFirstTor[SERVO_MOTOR_NUM];

extern int nTargetPos[SERVO_MOTOR_NUM];
extern int nTargetVel[SERVO_MOTOR_NUM];
extern int nTargetTor[SERVO_MOTOR_NUM];

//====================================================================


//======================================================================
//======================================================================

// struct for data exchange
typedef struct{
    uint32_t priod_ns;
    int motor_num;

} EC_CONIFG;



// wrapped? c func
#ifdef __cplusplus
extern "C" {
    #endif

    // Test wrapper for the RAIMLAB 4-axis servo system
    int ec_setup_test(void);
    void ec_run_cycle_test(struct timespec *wakeupTime);

    // Temporary wrapper for refactoring
    void check_domain1_state(void);
    void check_master_state(void);
    unsigned short GetStatusVal(unsigned short statusWord);

    unsigned short GetStatusWordN(int iNode);
    void stack_prefault(void);

/*****************************************************************************/

    void check_slave_config_states(void);
    extern struct timespec timespec_add(struct timespec time1, struct timespec time2);
/*****************************************************************************/


    // PDO data exchange interface (process data access functions)




#ifdef __cplusplus
}
#endif

#endif