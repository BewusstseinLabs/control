#include "../source/dynamixel_motor/dynamixel_motor.h"

#include <unistd.h>

int main()
{
    // Create a motor instance
    bewusstsein::ctrl::core::dynamixel_motor motor(2, "/dev/ttyUSB0", 1000000, 2.0);

    // EEPROM Area
    //..................................................
    motor.set_ADDRESS_MODEL_NUMBER(0); // 2 bytes, Read, Initial value: 1,090
    motor.set_ADDRESS_MODEL_INFORMATION(2); // 4 bytes, Read
    motor.set_ADDRESS_FIRMWARE_VERSION(6); // 1 byte, Read
    motor.set_ADDRESS_ID(7); // 1 byte, Read/Write, Initial value: 1, Range: 0 ~ 252
    motor.set_ADDRESS_BAUD_RATE(8); // 1 byte, Read/Write, Initial value: 1, Range: 0 ~ 7
    motor.set_ADDRESS_RETURN_DELAY_TIME(9); // 1 byte, Read/Write, Initial value: 250, Range: 0 ~ 254, Unit: 2 [μsec]
    motor.set_ADDRESS_DRIVE_MODE(10); // 1 byte, Read/Write, Initial value: 0, Range: 0 ~ 5
    motor.set_ADDRESS_OPERATING_MODE(11); // 1 byte, Read/Write, Initial value: 3, Range: 0 ~ 16
    motor.set_ADDRESS_SECONDARY_ID(12); // 1 byte, Read/Write, Initial value: 255, Range: 0 ~ 252
    motor.set_ADDRESS_PROTOCOL_TYPE(13); // 1 byte, Read/Write, Initial value: 2, Range: 1 ~ 2
    motor.set_ADDRESS_HOMING_OFFSET(20); // 4 bytes, Read/Write, Initial value: 0, Range: -1,044,479 ~ 1,044,479, Unit: 1 [pulse]
    motor.set_ADDRESS_MOVING_THRESHOLD(24); // 4 bytes, Read/Write, Initial value: 10, Range: 0 ~ 1,023, Unit: 0.229 [rev/min]
    motor.set_ADDRESS_TEMPERATURE_LIMIT(31); // 1 byte, Read/Write, Initial value: 72, Range: 0 ~ 100, Unit: 1 [°C]
    motor.set_ADDRESS_MAX_VOLTAGE_LIMIT(32); // 2 bytes, Read/Write, Initial value: 140, Range: 60 ~ 140, Unit: 0.1 [V]
    motor.set_ADDRESS_MIN_VOLTAGE_LIMIT(34); // 2 bytes, Read/Write, Initial value: 60, Range: 60 ~ 140, Unit: 0.1 [V]
    motor.set_ADDRESS_PWM_LIMIT(36); // 2 bytes, Read/Write, Initial value: 885, Range: 0 ~ 885, Unit: 0.113 [%]
    motor.set_ADDRESS_VELOCITY_LIMIT(44); // 4 bytes, Read/Write, Initial value: 250, Range: 0 ~ 1,023, Unit: 0.229 [rev/min]
    motor.set_ADDRESS_MAX_POSITION_LIMIT(48); // 4 bytes, Read/Write, Initial value: 4,095, Range: 0 ~ 4,095, Unit: 1 [pulse]
    motor.set_ADDRESS_MIN_POSITION_LIMIT(52); // 4 bytes, Read/Write, Initial value: 0, Range: 0 ~ 4,095, Unit: 1 [pulse]
    motor.set_ADDRESS_STARTUP_CONFIGURATION(60); // 1 byte, Read/Write, Initial value: 0, Range: 3
    motor.set_ADDRESS_SHUTDOWN(63); // 1 byte, Read/Write, Initial value: 52
    //..................................................

    // RAM Area
    //..................................................
    motor.set_ADDRESS_TORQUE_ENABLE(64); // 1 byte, Read/Write, Initial value: 0, Range: 0 ~ 1
    motor.set_ADDRESS_LED(65); // 1 byte, Read/Write, Initial value: 0, Range: 0 ~ 1
    motor.set_ADDRESS_STATUS_RETURN_LEVEL(68); // 1 byte, Read/Write, Initial value: 2, Range: 0 ~ 2
    motor.set_ADDRESS_REGISTERED_INSTRUCTION(69); // 1 byte, Read, Initial value: 0, Range: 0 ~ 1
    motor.set_ADDRESS_HARDWARE_ERROR_STATUS(70); // 1 byte, Read, Initial value: 0
    motor.set_ADDRESS_VELOCITY_I_GAIN(76); // 2 bytes, Read/Write, Initial value: 1,800, Range: 0 ~ 16,383
    motor.set_ADDRESS_VELOCITY_P_GAIN(78); // 2 bytes, Read/Write, Initial value: 100, Range: 0 ~ 16,383
    motor.set_ADDRESS_POSITION_D_GAIN(80); // 2 bytes, Read/Write, Initial value: 2,000, Range: 0 ~ 16,383
    motor.set_ADDRESS_POSITION_I_GAIN(82); // 2 bytes, Read/Write, Initial value: 0, Range: 0 ~ 16,383
    motor.set_ADDRESS_POSITION_P_GAIN(84); // 2 bytes, Read/Write, Initial value: 640, Range: 0 ~ 16,383
    motor.set_ADDRESS_FEEDFORWARD_2ND_GAIN(88); // 2 bytes, Read/Write, Initial value: 0, Range: 0 ~ 16,383
    motor.set_ADDRESS_FEEDFORWARD_1ST_GAIN(90); // 2 bytes, Read/Write, Initial value: 0, Range: 0 ~ 16,383
    motor.set_ADDRESS_BUS_WATCHDOG(98); // 1 byte, Read/Write, Initial value: 0, Range: 1 ~ 127, Unit: 20 [msec]
    motor.set_ADDRESS_GOAL_PWM(100); // 2 bytes, Read/Write, Range: -PWM Limit(36) ~ PWM Limit(36), Unit: 0.113 [%]
    motor.set_ADDRESS_GOAL_VELOCITY(104); // 4 bytes, Read/Write, Range: -Velocity Limit(44) ~ Velocity Limit(44), Unit: 0.229 [rev/min]
    motor.set_ADDRESS_PROFILE_ACCELERATION(108); // 4 bytes, Read/Write, Initial value: 0, Range: 0 ~ 32,767, Unit: 214.577 [rev/min2]
    motor.set_ADDRESS_PROFILE_VELOCITY(112); // 4 bytes, Read/Write, Initial value: 0, Range: 0 ~ 32,767, Unit: 0.229 [rev/min]
    motor.set_ADDRESS_GOAL_POSITION(116); // 4 bytes, Read/Write, Range: Min Position Limit(52) ~ Max Position Limit(48), Unit: 1 [pulse]
    motor.set_ADDRESS_REALTIME_TICK(120); // 2 bytes, Read, Range: 0 ~ 32,767, Unit: 1 [msec]
    motor.set_ADDRESS_MOVING(122); // 1 byte, Read, Initial value: 0, Range: 0 ~ 1
    motor.set_ADDRESS_MOVING_STATUS(123); // 1 byte, Read, Initial value: 0
    motor.set_ADDRESS_PRESENT_PWM(124); // 2 bytes, Read
    motor.set_ADDRESS_PRESENT_LOAD(126); // 2 bytes, Read, Range: -1,000 ~ 1,000, Unit: 0.1 [%]
    motor.set_ADDRESS_PRESENT_VELOCITY(128); // 4 bytes, Read, Unit: 0.229 [rev/min]
    motor.set_ADDRESS_PRESENT_POSITION(132); // 4 bytes, Read, Unit: 1 [pulse]
    motor.set_ADDRESS_VELOCITY_TRAJECTORY(136); // 4 bytes, Read, Unit: 0.229 [rev/min]
    motor.set_ADDRESS_POSITION_TRAJECTORY(140); // 4 bytes, Read, Unit: 1 [pulse]
    motor.set_ADDRESS_PRESENT_INPUT_VOLTAGE(144); // 2 bytes, Read, Unit: 0.1 [V]
    motor.set_ADDRESS_PRESENT_TEMPERATURE(146); // 1 byte, Read, Unit: 1 [°C]
    motor.set_ADDRESS_BACKUP_READY(147); // 1 byte, Read, Range: 0 ~ 1
    motor.set_ADDRESS_INDIRECT_ADDRESS(168); // 2 bytes, Read/Write, Initial value: 224, Range: 64 ~ 661
    motor.set_ADDRESS_INDIRECT_DATA(224); // 1 byte, Read/Write, Initial value: 0, Range: 0 ~ 255
    //..................................................

    // Connect to the motor
    if (!motor.connect())
    {
        printf("Failed to connect to the motor!\n");
        return -1;
    }

    // Set the motor to velocity control mode
    if (!motor.set_mode_velocity())
    {
        printf("Failed to set the motor to velocity control mode!\n");
        return -1;
    }

    // Enable the motor torque
    if (!motor.enable_torque())
    {
        printf("Failed to enable the motor torque!\n");
        return -1;
    }

    // Set the motor speed
    int speed = 200; // Set your desired speed here
    if (!motor.set_speed(speed))
    {
        printf("Failed to set the motor speed!\n");
        return -1;
    }

    usleep(10000 * 1000);

    // Disable the motor torque
    if (!motor.disable_torque())
    {
        printf("Failed to disable the motor torque!\n");
        return -1;
    }

    return 0;
}