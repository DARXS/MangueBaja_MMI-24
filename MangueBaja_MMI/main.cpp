#include "mbed.h"
#include "stats_report.h"
/* Instances Libraries */
#include "CANMsg.h"
#include "LSM6DS3.h"
#include "Kalman.h"
/* User Libraries */
#include "front_defs.h"

/* Communication Protocols */
CAN can(PB_8, PB_9, CAN_BPS_1000K); // RD, TD, Frequency
Serial serial(PA_2, PA_3, 115200);  // TX, RX, Baudrate
LSM6DS3 LSM6DS3(PB_7, PB_6);        // SDA, SCL
//RFM69 radio(PB_15/*mosi*/, PB_14/*miso*/, PB_13/*sclk*/, PB_12/*CS*/, PA_8/*Interrupt*/); 

/* I/O pins */
//DigitalIn acopl_4x4(PA_0, PullNone);
InterruptIn acopl_4x4(PA_0, PullDown);
InterruptIn freq_sensor(PB_4, PullNone);
InterruptIn choke_switch(PA_7, PullUp);     // servomotor CHOKE mode
InterruptIn run_switch(PA_5, PullUp);       // servomotor RUN mode
DigitalOut led(PC_13);
/* Debug pins */
PwmOut signal(PA_6);
DigitalOut db(PB_11);

/* Mbed OS tools */
Thread eventThread;
EventQueue queue(1024);
CircularBuffer<state_t, BUFFER_SIZE> state_buffer;
Ticker ticker1Hz;
Ticker ticker5Hz;
Ticker ticker20Hz;
Ticker tickerTrottle;
Timeout debounce_throttle;
Timeout debounce_button;

/* Debug variables */
Timer t;
bool buffer_full = false;
/* Global variables */
FIR filter(0.55, 0.57); // FIR filter coefficients
Txtmng strc_data;
packet_t data;
state_t current_state = IDLE_ST;
bool switch_clicked = false;
bool button_clicked = false;
uint8_t array_data[sizeof(Txtmng)];
uint8_t imu_failed = 0;                      // number of times before a new connection attempt with imu 
uint8_t pulse_counter = 0, sot = 0x00;
uint16_t dt = 0;
uint32_t imu_last_acq = 0;
uint64_t last_acq = 0;
uint64_t current_period = 0, last_count = 0;
float rpm_hz;

/* Interrupt handlers */
void canHandler();
void throttleDebounceHandler();
void ButtonDebouceHandler();
/* Interrupt services routine */
void canISR();
void frequencyCounterISR();
void servoSwitchISR();
void Button4x4ISR();
void ticker1HzISR();
void ticker5HzISR();
void ticker20HzISR();
/* General functions*/
void setupInterrupts();
void filterMessage(CANMsg msg);
void calcAngles(int16_t accx, int16_t accy, int16_t accz, int16_t grx, int16_t gry, int16_t grz, int16_t dt);
void Servo_flag(uint8_t state);
void displayData(uint16_t vel, uint16_t Hz, uint8_t temp, /*uint16_t comb,*/ uint8_t tempCVT, uint8_t SOC, uint8_t SOT);

/* CAN Variables */
uint16_t RPM = 0;            // 2by
uint8_t flags = 0x00;        // 1by
// flag = SoC | CVT | MOTOR | FUEL | MQTT | SERVO-ERROR | CHOKE | RUN |
uint8_t switch_state = 0x00; // 1by
/* 
    imu messages:
        ACC = x(2by) + y(2by) + z(2by)    // 6by
        DPS = x(2by) + y(2by) + z(2by)    // 6by
*/
// Pitch(2by) + Roll(2by) =  4by
int16_t angle_roll = 0, angle_pitch = 0; 

int main ()
{
    /* Main variables */
    CANMsg txMsg; 
    /* Initialization */
    t.start();
    //horn = horn_button.read();                               // horn OFF
    //headlight = headlight_switch.read();                     // headlight OFF
    eventThread.start(callback(&queue, &EventQueue::dispatch_forever));

    uint16_t lsm_addr = LSM6DS3.begin(LSM6DS3.G_SCALE_245DPS, LSM6DS3.A_SCALE_2G, \
                                      LSM6DS3.G_ODR_26_BW_2, LSM6DS3.A_ODR_26); 
    
    setupInterrupts();

    // acopl_4x4.read()==0 => 4x4(button pressed) || acopl_4x4.read()==1 => 4x2(button raised)  
    /* Check the 4x4 */
    sot |= ((acopl_4x4.read() << 1) & 0x02);

    while(true)
    {
        if(state_buffer.full())
        {
            buffer_full = true;
            //led = 0;
            state_buffer.pop(current_state);
        } else {
            //led = 1;
            buffer_full = false;
            if(!state_buffer.empty())
                state_buffer.pop(current_state);
            else
                current_state = IDLE_ST;
        }

        switch(current_state) 
        {
            case IDLE_ST:
                //serial.printf("idle\r\n");
                //Thread::wait(2);
                break;

            case IMU_ST:
                //serial.printf("imu\r\n");
                //t0 = t.read_us();

                if(lsm_addr)
                {
                    bool nack = LSM6DS3.readAccel();     // read accelerometer data into LSM6DS3.aN_raw
                    if(!nack)
                        nack = LSM6DS3.readGyro();       //  "   gyroscope data into LSM6DS3.gN_raw

                    if(nack)
                    {
                        lsm_addr = 0;
                        LSM6DS3.ax_raw = 0;
                        LSM6DS3.ay_raw = 0;
                        LSM6DS3.az_raw = 0;
                        LSM6DS3.gx_raw = 0;
                        LSM6DS3.gy_raw = 0;
                        LSM6DS3.gz_raw = 0;
                    }
                }

                else if(imu_failed == IMU_TRIES)
                {
                    lsm_addr = LSM6DS3.begin(LSM6DS3.G_SCALE_245DPS, LSM6DS3.A_SCALE_2G, LSM6DS3.G_ODR_26_BW_2, LSM6DS3.A_ODR_26);                                    
                    //t1 = t.read_us();
                    imu_failed = 0;
                    //serial.printf("%d\r\n", (t1 - t0));
                } 
                
                else 
                {
                    imu_failed++;
                }

                last_acq = t.read_ms();
                calcAngles(LSM6DS3.ax_raw, LSM6DS3.ay_raw, LSM6DS3.az_raw, LSM6DS3.gx_raw, LSM6DS3.gy_raw, LSM6DS3.gz_raw, dt);


                /* Send accelerometer data */
                txMsg.clear(IMU_ACC_ID);
                txMsg << LSM6DS3.ax_raw << LSM6DS3.ay_raw << LSM6DS3.az_raw;
                if(can.write(txMsg))
                {
                    /* Send gyroscope data only if accelerometer data succeeds */
                    imu_last_acq = t.read_ms();

                    txMsg.clear(IMU_DPS_ID);
                    txMsg << LSM6DS3.gx_raw << LSM6DS3.gx_raw << LSM6DS3.gx_raw /*<< dt*/;
                    if(can.write(txMsg))
                    {
                        /* Send the Angle of Pitch and Roll if gyroscope data succeeds */
                        txMsg.clear(ANGLE_ID);
                        txMsg << angle_roll << angle_pitch;
                        can.write(txMsg);
                    }
                }

                break;

            case RPM_ST:
                //  serial.printf("rpm\r\n");
                freq_sensor.fall(NULL);         // disable interrupt

                if(current_period!=0) 
                {
                    //rpm_hz = ((float)pulse_counter/(current_period/1000000.0));    //calculates frequency in Hz
                    //if (switch_state != CHOKE_MODE)
                    rpm_hz = pulse_counter*5*60;
                    
                    /* Send CHOKE MODE message */
                    txMsg.clear(THROTTLE_ID);
                    txMsg << CHOKE_MODE;
                    can.write(txMsg);

                    Servo_flag(CHOKE_MODE);
                    //engine_counter.start();
                } 
                
                else
                {
                    rpm_hz = 0;
                    //writeServo(switch_state);
                    //engine_counter.stop();
                }
                rpm_hz /= 1.55;
                RPM = (uint16_t)(filter.filt(rpm_hz));

                /* Send RPM data */
                txMsg.clear(RPM_ID);
                txMsg << RPM;
                can.write(txMsg);

                /* prepare to re-init RPM counter */
                pulse_counter = 0;
                current_period = 0;                                   // reset pulses related variables
                last_count = t.read_us();
                freq_sensor.fall(&frequencyCounterISR);               // enable interrupt

                break;

            case VERIFY_4x4_ST:
                //serial.printf("verify 4x4");
                //sot <<= acopl_4x4.read();

                if(button_clicked)
                {   
                    if(acopl_4x4.read())
                    {
                        sot |= 0x02;
                        db = 1;
                    }

                    else
                    {
                        sot &= ~0x02;
                        db = 0;
                    }

                    button_clicked = false;
                }

                break;

            case THROTTLE_ST:    
                //serial.printf("throttle ok\r\n");

                if(switch_clicked)
                {                    
                    switch_state = (!choke_switch.read() << 1) | (!run_switch.read() << 0);
                    //serial.printf("switch_state = %d\t", switch_state);
                    
                    /* Send CAN message */
                    txMsg.clear(THROTTLE_ID);
                    txMsg << (RPM!=0 ? CHOKE_MODE : switch_state);
                    can.write(txMsg);

                    Servo_flag(RPM!=0 ? CHOKE_MODE : switch_state);

                    switch_clicked = false;
                }

                break;

            case FLAGS_ST:
                //serial.printf("flags\r\n");

                /* Send flags message */
                txMsg.clear(FLAGS_ID);
                txMsg << flags;
                can.write(txMsg);
                //if(can.write(txMsg))
                //    led = !led;
            
                break;

            case DISPLAY_ST:
                //serial.printf("display\r\n"); 
                displayData(data.speed, RPM, data.tempMOTOR, /*data.fuel,*/ data.tempCVT, data.soc, sot);
                break;

            case DEBUG_ST:
                //serial.printf("Debug state\r\n");
                //serial.printf("Accx = %f\r\n", (float)(LSM6DS3.ax_raw*0.061 / 1000));
                //serial.printf("Accy = %f\r\n", (float)(LSM6DS3.ay_raw*0.061 / 1000));
                //serial.printf("Accz = %f\r\n", (float)(LSM6DS3.az_raw*0.061 / 1000));
                //serial.printf("DPSx = %d\r\n", LSM6DS3.gx_raw);
                //serial.printf("DPSy = %d\r\n", LSM6DS3.gy_raw);
                //serial.printf("DPSz = %d\r\n", LSM6DS3.gz_raw);
                //serial.printf("Angle Roll = %d\r\n", angle_roll);
                //serial.printf("Angle Pitch = %d\r\n", angle_pitch);
                //serial.printf("RPM = %d\r\n", RPM);
                //serial.printf("4x4 = %s\r\n", acopl_4x4.read() ? "Yes" : "No");
                //serial.printf("switch state = %s\r\n", switch_state==0 ? "MID" : switch_state==1 ? "RUN" : "CHOKE" );
                //serial.printf("flags = %d\r\n", flags);
                //serial.printf("\n\n\n");
                break;
        }
    }
}

/* Global Functions */ 
void setupInterrupts()
{
    /* General Interrupts */
    can.attach(&canISR, CAN::RxIrq);
    freq_sensor.fall(&frequencyCounterISR);

    /* Servo interrupts */
    choke_switch.rise(&servoSwitchISR);     // trigger throttle interrupt in both edges
    choke_switch.fall(&servoSwitchISR);     // trigger throttle interrupt in both edges
    run_switch.rise(&servoSwitchISR);       // trigger throttle interrupt in both edges
    run_switch.fall(&servoSwitchISR);       // trigger throttle interrupt in both edges

    /* 4x4 Interrupts */
    acopl_4x4.rise(&Button4x4ISR);
    acopl_4x4.fall(&Button4x4ISR);

    /* Tickers */
    ticker1Hz.attach(&ticker1HzISR, 1.0);
    ticker5Hz.attach(&ticker5HzISR, 0.2); 
    ticker20Hz.attach(&ticker20HzISR, 0.05); 
    
    /* PWM Signal */
    signal.period_ms(100);
    signal.write(0.5f);
}

void filterMessage(CANMsg msg) 
{
    led = !led;

    if(msg.id==SPEED_ID)
    {
        msg >> data.speed;
    }

    if(msg.id==TEMPERATURE_ID)
    {
        msg >> data.tempMOTOR;
        ((data.tempMOTOR > 110) ? flags |= (0x80 >> 2) : flags &= ~(0x80 >> 2));
    }

    //if(msg.id==FUEL_ID)
    //{
    //    msg >> data.fuel;
    //    ((data.fuel==20) ? flags |= (0x08 >> 3) : flags &= ~(0x08 >> 3));
    //}

    if(msg.id==SOC_ID)
    {
        msg >> data.soc;
        ((data.soc <= 20) ? flags |= 0x80 : flags &= ~0x80);
        //(data.soc < 20) ? data.flags |= (0x80) : 0;
    }

    if(msg.id==CVT_ID)
    {
        msg >> data.tempCVT;
        ((data.tempCVT > 110) ? flags |= (0x80 >> 1) : flags &= ~(0x80 >> 1)); 
    }

    if(msg.id==SOT_ID)
    {
        uint8_t s;
        msg >> s;
        
        //sot |= s;
        if(s==0) { sot &= ~0x01; sot &= ~0x04; }

        else if(s==1) { sot |= 0x01; sot &= ~0x04; }

        else if(s==4) { sot &= ~0x01; sot |= 0x04; }

        ((sot==1 || sot==3) ? flags |= 0x08 : flags &= ~0x08);
    }
}

/* Function adapted from Kristian Lauszus library example, source: https://github.com/TKJElectronics/KalmanFilter */
void calcAngles(int16_t accx, int16_t accy, int16_t accz, int16_t grx, int16_t gry, int16_t grz, int16_t dt)
{
    static Kalman kalmanX, kalmanY;
    float kalAngleX, kalAngleY;
    float pitch, roll;
    float gyroXrate, gyroYrate;
    float ax, ay, az;
    static bool first_execution = true;
    
    ax = (float) accx * TO_G;
    ay = (float) accy * TO_G;
    az = (float) accz * TO_G;
    pitch = atan2(ay, az) * RAD_TO_DEGREE;
    roll = atan(-ax / sqrt(ay*ay + az*az)) * RAD_TO_DEGREE;
    gyroXrate = grx / TO_DPS;                            // Convert to deg/s
    gyroYrate = gry / TO_DPS;                            // Convert to deg/s

    if(first_execution)
    {
        // set starting angle if first execution
        first_execution = false;
        kalmanX.setAngle(roll);
        kalmanY.setAngle(pitch);
    }

    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
    {
        kalmanX.setAngle(roll);
        kalAngleX = roll;
    } else {
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    }

    if(abs(kalAngleX) > 90)
    {
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    }
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

    angle_roll = roll;
    angle_pitch = pitch;
}

void Servo_flag(uint8_t state)
{
    flags &= ~(0x07); // reset servo-related flags

    switch(state) 
    {
        case MID_MODE:
            //dbg3 = !dbg3;
            //servo.pulsewidth_us(SERVO_MID);
            flags &= ~(SERVO_RESET); // reset run and choke flags
            break;

        case RUN_MODE:
            //dbg3 = !dbg3;
            //servo.pulsewidth_us(SERVO_RUN);
            flags |= RUN_MODE; // set run flag
            break;

        case CHOKE_MODE:
            //dbg3 = !dbg3;
            //servo.pulsewidth_us(SERVO_CHOKE);
            flags |= CHOKE_MODE;    // set choke flag
            break;

        default:
            //serial.printf("Choke/run error\r\n");
            flags |= SERVOR_ERROR;    // set servo error flag
            break;
    }
}

void displayData(uint16_t vel, uint16_t Hz, uint8_t temp, /*uint16_t comb,*/ uint8_t tempCVT, uint8_t SOC, uint8_t SOT)
{
    strc_data.speed = vel;
    strc_data.rpm = Hz;
    strc_data.temp_motor = temp;
    //strc_data.level = comb;
    strc_data.level = 0;
    strc_data.battery = SOC;
    strc_data.temp_cvt = tempCVT;
    strc_data.sot = sot;

    memcpy(&array_data, (uint8_t *)&strc_data, sizeof(strc_data));

    for(uint8_t CountBytes = 0; CountBytes < sizeof(Txtmng); CountBytes++) 
    {
        serial.putc(array_data[CountBytes]);
    }
}

/* Interrupts routine */
void canISR()
{
    CAN_IER &= ~CAN_IER_FMPIE0;                 // disable RX interrupt
    queue.call(&canHandler);                    // add canHandler() to events queue
}

void frequencyCounterISR()
{
    //db = !db;
    pulse_counter++;
    current_period += t.read_us() - last_count;
    last_count = t.read_us();
}

void servoSwitchISR()
{
    choke_switch.rise(NULL);     //  throttle interrupt in both edges dettach
    choke_switch.fall(NULL);     //  throttle interrupt in both edges dettach
    run_switch.rise(NULL);       //  throttle interrupt in both edges dettach
    run_switch.fall(NULL);       //  throttle interrupt in both edges dettach
    switch_clicked = true;
    debounce_throttle.attach(&throttleDebounceHandler, 0.1);
}

void Button4x4ISR()
{
    acopl_4x4.rise(NULL);
    acopl_4x4.fall(NULL);
    button_clicked = true;
    debounce_button.attach(&ButtonDebouceHandler, 0.1);
}

void ticker1HzISR()
{
    state_buffer.push(FLAGS_ST);
    //state_buffer.push(VERIFY_4x4_ST);
}

void ticker5HzISR()
{
    state_buffer.push(RPM_ST);
    state_buffer.push(DISPLAY_ST);
    //state_buffer.push(DEBUG_ST);
    //state_buffer.push(RADIO_ST);
}

void ticker20HzISR()
{
    state_buffer.push(IMU_ST);
}

/* Interrupts handlers */
void canHandler()
{
    CANMsg rxMsg;

    can.read(rxMsg);
    filterMessage(rxMsg);
    CAN_IER |= CAN_IER_FMPIE0;                  // enable RX interrupt
}

void throttleDebounceHandler()
{
    state_buffer.push(THROTTLE_ST);
    choke_switch.rise(&servoSwitchISR);     // trigger throttle interrupt in both edges
    choke_switch.fall(&servoSwitchISR);     // trigger throttle interrupt in both edges
    run_switch.rise(&servoSwitchISR);       // trigger throttle interrupt in both edges
    run_switch.fall(&servoSwitchISR);       // trigger throttle interrupt in both edges
}

void ButtonDebouceHandler()
{
    state_buffer.push(VERIFY_4x4_ST);
    acopl_4x4.rise(&Button4x4ISR);
    acopl_4x4.fall(&Button4x4ISR);
}
