#include "mbed.h"
#include "RotaryEncoder.h"
#include "DriveMotor.h"

#define NECK_RY_MAX 2038
#define NECK_RY_MIN 1022
#define NECK_RY_ACCELERATION 0.2
#define CHIN_MAX 2100
#define CHIN_MIN 1338
#define CHIN_ACCELERATION 0.2
#define WHEELS_ACCELERATION 0.3
#define LIFT_ACCELERATION 0.5

RawSerial PC(USBTX, USBRX);
RawSerial XBee(PC_10, PC_11, 230400);

DriveMotor M_leftwheel(PC_6, PC_8);
DriveMotor M_rightwheel(PB_8, PC_9);

DriveMotor M_necklift(PA_7, PA_6, 0.5);

PwmOut S_neckry(PA_9);
PwmOut S_chin(PA_8);

void initRobot(void);
void driveLeftWheel(void);
void driveRightWheel(void);
void driveNeckLift(void);
void driveNeckRY(void);
void driveChin(void);
inline void receiveSignal(void);
inline void decordSignal(void);

int extended_sign_pool = 0x00;
int chin_sign_pool = 0x00;
int neck_rx_sign_pool = 0x00;
int neck_ry_sign_pool = 0x00;
int neck_rz_sign_pool = 0x00;

int extended_sign = 0x00;
int chin_sign = 0x00;
int neck_rx_sign = 0x80;
int neck_ry_sign = 0x00;
int neck_rz_sign = 0x80;
int check_sum_sign = 0x00;

int move_direction = 0;
int lift_direction = 0;
int back_flag = 0;

int left_target_speed = 0;
int right_target_speed = 0;

int check_sum_correct = 0;

// main() runs in its own thread in the OS
int main()
{
    initRobot();
    while (true) {
        // PC.printf("\033[Hneck-ry%5d\r\nneck-rz%5d\r\n", RE_neckry.Get_Count(), RE_neckrz.Get_Count());
        // PC.printf("\033[Hextended_sign_pool: %#04X\r\nchin_sign_pool    : %#04X\r\nneck_rx_sign_pool : %#04X\r\nneck_ry_sign_pool : %#04X\r\nneck_rz_sign_pool : %#04X", extended_sign, chin_sign_pool, neck_rx_sign_pool, neck_ry_sign_pool, neck_rz_sign_pool);
        driveLeftWheel();
        driveRightWheel();
        driveNeckLift();
        driveNeckRY();
        driveChin();
    }
}

void initRobot(void){    
    XBee.attach(&receiveSignal, SerialBase::RxIrq);
    M_leftwheel.setPeriod_us(500);
    M_rightwheel.setPeriod_us(500);
    M_necklift.setPeriod_us(500);
    S_chin.period_ms(20);
    S_neckry.period_ms(20);
}

void driveNeckRY(void) {
    static int neck_ry_pulse_us_now = 1530;
    int neck_ry_pulse_us;

    neck_ry_pulse_us = (int)(NECK_RY_MIN + neck_rx_sign * 4.0);
    if(neck_ry_pulse_us > NECK_RY_MIN) neck_ry_pulse_us = NECK_RY_MAX;

    neck_ry_pulse_us_now += (int)((neck_ry_pulse_us - neck_ry_pulse_us_now) * NECK_RY_ACCELERATION);
    S_neckry.pulsewidth_us(neck_ry_pulse_us_now);

    // PC.printf("%d\r\n", RE_neckry.Get_Count());
}

void driveChin(void) {
    static int chin_pulse_us_now = 1338;
    int chin_pulse_us;
    
    chin_pulse_us = (int)(CHIN_MIN + chin_sign * 3.0);
    if(chin_pulse_us > CHIN_MAX) chin_pulse_us = CHIN_MAX;

    chin_pulse_us_now += (int)((chin_pulse_us - chin_pulse_us_now) * CHIN_ACCELERATION);
    
    S_chin.pulsewidth_us(chin_pulse_us_now);
    // PC.printf("neck-ry:%4d\r\n",chin_pulse_us_now);
}

void driveWheelLeft(void) {
    double current_speed = M_leftwheel.read();
    current_speed += (left_target_speed - current_speed) * WHEELS_ACCELERATION;
    M_leftwheel.drive(current_speed);
}

void driveWheelRight(void) {
    double current_speed = M_rightwheel.read();
    current_speed += (right_target_speed - current_speed) * WHEELS_ACCELERATION;
    M_rightwheel.drive(current_speed);
}

void driveNeckLift(void) {
    double current_speed = M_necklift.read();
    current_speed += (lift_direction - current_speed) * LIFT_ACCELERATION;
    M_necklift.drive(current_speed);
}

inline void receiveSignal(void) {
    int read_sign = XBee.getc();
    static int octets = 0;
    static int check_sum = 0x00;

    if(read_sign == 0xFF) {
        octets = 0;
    }
    if (read_sign >= 0){
        switch(octets) {
            case 0: check_sum = 0x00; ++octets; break;
            case 1: extended_sign_pool = read_sign; check_sum += read_sign; ++octets; break;
            case 2: chin_sign_pool = read_sign; check_sum += read_sign; ++octets; break;
            case 3: neck_ry_sign_pool = read_sign; check_sum += read_sign; ++octets; break;
            case 4: neck_rx_sign_pool = read_sign; check_sum += read_sign; ++octets; break;
            case 5: neck_rz_sign_pool = read_sign; check_sum += read_sign; ++octets; break;
            case 6: 
                check_sum_sign = read_sign;
                if(((check_sum % 0x100) == read_sign) || ((check_sum == 0xFF) && (read_sign == 0xFE))) {
                    check_sum_correct = 1;
                    extended_sign = extended_sign_pool;
                    int encoder_lock = (extended_sign >> 1) % 0b10;
                    if(!encoder_lock) {
                        chin_sign = chin_sign_pool;
                        neck_rx_sign = neck_rx_sign_pool;
                        neck_ry_sign = neck_ry_sign_pool;
                        neck_rz_sign = neck_rz_sign_pool;
                    }
                }
                else {
                    check_sum_correct = 0;
                }
                octets = 0; 
                break;
        }
    }
    decordSignal();
}

inline void decordSignal(void) {
    back_flag = (extended_sign >> 6) % 0b10;
    move_direction = (extended_sign >> 4) % 0b100;
    lift_direction = (extended_sign >> 2) % 0b100;
    
    left_target_speed = back_flag ? -1 : ((move_direction % 0b10) ? 1 : 0);
    right_target_speed = back_flag ? 1 : (((move_direction >> 1) % 0b10) ? -1 : 0);

    if(lift_direction == 2) lift_direction = 1;
    else if(lift_direction == 1) lift_direction = -1;
    else lift_direction = 0;
}