#include <stdbool.h>

enum CANSTATE { INIT, PREOP, OP };
enum CHARGESTAT { IDLE, CC, CV };

typedef struct ChargingStruct {
    bool enable_command;
    uint8_t can_state, charge_state;
    uint16_t voltage_feedback, current_feedback;
    uint16_t VRef, IRef, Imin;
    uint32_t last_rcv_ms, last_value;
} ChargingStruct;

typedef struct {
    uint8_t Data[8];
    uint16_t Length;
    uint32_t ID;
} CAN_msg_typedef;

ChargingStruct myChargeStruct;
CAN_msg_typedef Can_tx;
CAN_msg_typedef Can_rx;

uint32_t time_ms;

void CAN_write(CAN_msg_typedef *msg);
bool CAN_read(CAN_msg_typedef *msg);  // return true if there is received msg
void increase_value(void);  // dummy driver for increasing voltage / current
void decrease_value(void);  // dummy driver for decreasing voltage / current

/**
 * @brief Init values for struct
 *
 */
void Initialization(void) {
    // reset struct
    memset(&myChargeStruct, 0, sizeof(myChargeStruct));
    memset(&Can_tx, 0, sizeof(Can_tx));
    memset(&Can_rx, 0, sizeof(Can_rx));
    time_ms = 0;

    // Reference values with dummy values
    myChargeStruct.Imin = 1;
}

void stop_charging_handler(void) {
    memset(&myChargeStruct, 0, sizeof(myChargeStruct));
    memset(&Can_tx, 0, sizeof(Can_tx));
    memset(&Can_rx, 0, sizeof(Can_rx));
    myChargeStruct.can_state = PREOP;
}

/**
 * @brief main control algorithm for CC and CV
 *
 */
void control_routine(void) {
    switch (myChargeStruct.charge_state) {
        case CC:  // constant current
            if (myChargeStruct.voltage_feedback < myChargeStruct.VRef) {
                if (myChargeStruct.current_feedback < myChargeStruct.IRef) {
                    increase_value();
                } else if (myChargeStruct.current_feedback >
                           myChargeStruct.IRef) {
                    decrease_value();
                }
            } else {
                myChargeStruct.charge_state = CV;
            }
            break;
        case CV:  // constant voltage
            if (myChargeStruct.current_feedback > myChargeStruct.Imin) {
                if (myChargeStruct.voltage_feedback < myChargeStruct.VRef) {
                    increase_value();
                } else if (myChargeStruct.voltage_feedback >
                           myChargeStruct.VRef) {
                    decrease_value();
                }
            } else {
                stop_charging_handler();
            }
            break;
        default:
            break;
    }
    time_ms++;
}

/**
 * @brief state machine for charging
 *
 */
void main_state_machine(void) {
    if (myChargeStruct.enable_command) {
        control_routine();
    }
}

/**
 * @brief handling values to be write to CAN
 *
 */
void CAN_write_handler(void) {
    if (time_ms % 1000 == 0) {
        Can_tx.Data[0] = myChargeStruct.can_state;
        Can_tx.ID = 0x701;
        Can_tx.Length = 1;
        CAN_write(Can_tx);
        memset(&Can_tx, 0, sizeof(Can_tx));  // clear struct after write
    }
    if (myChargeStruct.charge_state != IDLE) {
        if (time_ms % 200 == 0) {
            myChargeStruct.voltage_feedback =
                myChargeStruct.voltage_feedback * 10;
            Can_rx.Data[0] = (uint8_t)(myChargeStruct.voltage_feedback >> 8);
            Can_rx.Data[1] = (uint8_t)myChargeStruct.voltage_feedback;
            myChargeStruct.current_feeback =
                myChargeStruct.current_feeback * 10;
            Can_rx.Data[2] = (uint8_t)(myChargeStruct.current_feeback >> 8);
            Can_rx.Data[3] = (uint8_t)myChargeStruct.current_feeback;
            Can_rx.Data[4] = 1;  // is charging
            CAN_write(Can_tx);
            memset(&Can_tx, 0, sizeof(Can_tx));  // clear struct after write
        }
    }
}

/**
 * @brief handling values read in via CAN
 *
 */
void CAN_read_handler(void) {
    memset(&Can_rx, 0, sizeof(Can_rx));  // clear struct before read
    if (CAN_read(Can_rx)) {
        switch (Can_rx.ID) {
            case 0x201:
                myChargeStruct.VRef =
                    ((uint16_t)Can_tx.Data[0] << 8) | Can_tx.Data[1];
                myChargeStruct.VRef = myChargeStruct.VRef / 10;
                myChargeStruct.IRef =
                    ((uint16_t)Can_tx.Data[3] << 8) | Can_tx.Data[2];
                myChargeStruct.IRef = myChargeStruct.IRef / 10;
                if (Can_tx.Data[4] == 1) {  // start charging
                    if (myChargeStruct.charge_state == IDLE) {
                        myChargeStruct.charge_state = CC;
                    }
                    myChargeStruct.can_state = OP;
                    myChargeStruct.enable_command = true;
                } else {  // stop charging
                    stop_charging_handler();
                }
                break;
            default:
                break;
        }
        myChargeStruct.last_rcv_ms = time_ms;
    }
    myChargeStruct.last_rcv_ms =
        time_ms - myChargeStruct.last_rcv_ms + myChargeStruct.last_value;
    myChargeStruct.last_value = 0;
    if (myChargeStruct.last_rcv_ms >= 5000) {
        stop_charging_handler();
    }
}

/**
 * @brief network management of CAN
 *
 */
void network_management(void) {
    CAN_write_handler();
    CAN_read_handler();
}

/**
 * @brief housekeeping for time_ms overflow
 *
 */
void housekeeping_timems(void) {
    if (time_ms == 0xFFFFFFFF) {
        myChargeStruct.last_value = time_ms - myChargeStruct.last_rcv_ms;
        time_ms = 0;
        myChargeStruct.last_rcv_ms = 0;
    }
}

void main(void) {
    Initialization();
    PieVectTable.EPWM1_INT = &control_routine;
    myChargeStruct.can_state = PREOP;
    while (true) {
        main_state_machine();
        network_management();
        housekeeping_timems();
    }
}
