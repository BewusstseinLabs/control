// Copyright 2024 Shane W. Mulcahy

//: C++ Headers
#include <string>

//: Project Headers
#include "bewusstsein_control/c++/include/dynamixel_motor.hpp"

//: This Header
#include "bewusstsein_control/c++/include/dynamixel_2xl430-w250-t.hpp"

namespace ctrl
{
    // :UNITS
        #define REV_PER_MIN 0.229
        #define REV_PER_SEC 0.0038166666666666666

    //: EEPROM
        #define ADDRESS_MODEL_NUMBER 0              // 2 bytes, Read, Initial value: 1,090
        #define ADDRESS_MODEL_INFORMATION 2         // 4 bytes, Read
        #define ADDRESS_FIRMWARE_VERSION 6          // 1 byte, Read
        #define ADDRESS_ID 7                        // 1 byte, Read/Write, Initial value: 1, Range: 0 ~ 252
        #define ADDRESS_BAUD_RATE 8                 // 1 byte, Read/Write, Initial value: 1, Range: 0 ~ 7
        #define ADDRESS_RETURN_DELAY_TIME 9         // 1 byte, Read/Write, Initial value: 250, Range: 0 ~ 254, Unit: 2 [μsec]
        #define ADDRESS_DRIVE_MODE 10               // 1 byte, Read/Write, Initial value: 0, Range: 0 ~ 5
        #define ADDRESS_OPERATING_MODE 11           // 1 byte, Read/Write, Initial value: 3, Range: 0 ~ 16
        #define ADDRESS_SECONDARY_ID 12             // 1 byte, Read/Write, Initial value: 255, Range: 0 ~ 252
        #define ADDRESS_PROTOCOL_TYPE 13            // 1 byte, Read/Write, Initial value: 2, Range: 1 ~ 2
        #define ADDRESS_HOMING_OFFSET 20            // 4 bytes, Read/Write, Initial value: 0, Range: -1,044,479 ~ 1,044,479, Unit: 1 [pulse]
        #define ADDRESS_MOVING_THRESHOLD 24         // 4 bytes, Read/Write, Initial value: 10, Range: 0 ~ 1,023, Unit: 0.229 [rev/min]
        #define ADDRESS_TEMPERATURE_LIMIT 31        // 1 byte, Read/Write, Initial value: 72, Range: 0 ~ 100, Unit: 1 [°C]
        #define ADDRESS_MAX_VOLTAGE_LIMIT 32        // 2 bytes, Read/Write, Initial value: 140, Range: 60 ~ 140, Unit: 0.1 [V]
        #define ADDRESS_MIN_VOLTAGE_LIMIT 34        // 2 bytes, Read/Write, Initial value: 60, Range: 60 ~ 140, Unit: 0.1 [V]
        #define ADDRESS_PWM_LIMIT 36                // 2 bytes, Read/Write, Initial value: 885, Range: 0 ~ 885, Unit: 0.113 [%]
        #define ADDRESS_VELOCITY_LIMIT 44           // 4 bytes, Read/Write, Initial value: 250, Range: 0 ~ 1,023, Unit: 0.229 [rev/min]
        #define ADDRESS_MAX_POSITION_LIMIT 48       // 4 bytes, Read/Write, Initial value: 4,095, Range: 0 ~ 4,095, Unit: 1 [pulse]
        #define ADDRESS_MIN_POSITION_LIMIT 52       // 4 bytes, Read/Write, Initial value: 0, Range: 0 ~ 4,095, Unit: 1 [pulse]
        #define ADDRESS_STARTUP_CONFIGURATION 60    // 1 byte, Read/Write, Initial value: 0, Range: 3
        #define ADDRESS_SHUTDOWN 63                 // 1 byte, Read/Write, Initial value: 52

    //: RAM
        #define ADDRESS_TORQUE_ENABLE 64            // 1 byte, Read/Write, Initial value: 0, Range: 0 ~ 1
        #define ADDRESS_LED 65                      // 1 byte, Read/Write, Initial value: 0, Range: 0 ~ 1
        #define ADDRESS_STATUS_RETURN_LEVEL 68      // 1 byte, Read/Write, Initial value: 2, Range: 0 ~ 2
        #define ADDRESS_REGISTERED_INSTRUCTION 69   // 1 byte, Read, Initial value: 0, Range: 0 ~ 1
        #define ADDRESS_HARDWARE_ERROR_STATUS 70    // 1 byte, Read, Initial value: 0
        #define ADDRESS_VELOCITY_I_GAIN 76          // 2 bytes, Read/Write, Initial value: 1,800, Range: 0 ~ 16,383
        #define ADDRESS_VELOCITY_P_GAIN 78          // 2 bytes, Read/Write, Initial value: 100, Range: 0 ~ 16,383
        #define ADDRESS_POSITION_D_GAIN 80          // 2 bytes, Read/Write, Initial value: 2,000, Range: 0 ~ 16,383
        #define ADDRESS_POSITION_I_GAIN 82          // 2 bytes, Read/Write, Initial value: 0, Range: 0 ~ 16,383
        #define ADDRESS_POSITION_P_GAIN 84          // 2 bytes, Read/Write, Initial value: 640, Range: 0 ~ 16,383
        #define ADDRESS_FEEDFORWARD_2ND_GAIN 88     // 2 bytes, Read/Write, Initial value: 0, Range: 0 ~ 16,383
        #define ADDRESS_FEEDFORWARD_1ST_GAIN 90     // 2 bytes, Read/Write, Initial value: 0, Range: 0 ~ 16,383
        #define ADDRESS_BUS_WATCHDOG 98             // 1 byte, Read/Write, Initial value: 0, Range: 1 ~ 127, Unit: 20 [msec]
        #define ADDRESS_GOAL_PWM 100                // 2 bytes, Read/Write, Range: -PWM Limit(36) ~ PWM Limit(36), Unit: 0.113 [%]
        #define ADDRESS_GOAL_VELOCITY 104           // 4 bytes, Read/Write, Range: -Velocity Limit(44) ~ Velocity Limit(44), Unit: 0.229 [rev/min]
        #define ADDRESS_PROFILE_ACCELERATION 108    // 4 bytes, Read/Write, Initial value: 0, Range: 0 ~ 32,767, Unit: 214.577 [rev/min2]
        #define ADDRESS_PROFILE_VELOCITY 112        // 4 bytes, Read/Write, Initial value: 0, Range: 0 ~ 32,767, Unit: 0.229 [rev/min]
        #define ADDRESS_GOAL_POSITION 116           // 4 bytes, Read/Write, Range: Min Position Limit(52) ~ Max Position Limit(48), Unit: 1 [pulse]
        #define ADDRESS_REALTIME_TICK 120           // 2 bytes, Read, Range: 0 ~ 32,767, Unit: 1 [msec]
        #define ADDRESS_MOVING 122                  // 1 byte, Read, Initial value: 0, Range: 0 ~ 1
        #define ADDRESS_MOVING_STATUS 123           // 1 byte, Read, Initial value: 0
        #define ADDRESS_PRESENT_PWM 124             // 2 bytes, Read
        #define ADDRESS_PRESENT_LOAD 126            // 2 bytes, Read, Range: -1,000 ~ 1,000, Unit: 0.1 [%]
        #define ADDRESS_PRESENT_VELOCITY 128        // 4 bytes, Read, Unit: 0.229 [rev/min]
        #define ADDRESS_PRESENT_POSITION 132        // 4 bytes, Read, Unit: 1 [pulse]
        #define ADDRESS_VELOCITY_TRAJECTORY 136     // 4 bytes, Read, Unit: 0.229 [rev/min]
        #define ADDRESS_POSITION_TRAJECTORY 140     // 4 bytes, Read, Unit: 1 [pulse]
        #define ADDRESS_PRESENT_INPUT_VOLTAGE 144   // 2 bytes, Read, Unit: 0.1 [V]
        #define ADDRESS_PRESENT_TEMPERATURE 146     // 1 byte, Read, Unit: 1 [°C]
        #define ADDRESS_BACKUP_READY 147            // 1 byte, Read, Range: 0 ~ 1
        #define ADDRESS_INDIRECT_ADDRESS 168        // 2 bytes, Read/Write, Initial value: 224, Range: 64 ~ 661
        #define ADDRESS_INDIRECT_DATA 224           // 1 byte, Read/Write, Initial value: 0, Range: 0 ~ 255

    //: Constructors
        Dynamixel_2XL430_W250_T::Dynamixel_2XL430_W250_T(const uint id, const std::string& device_name, const uint baudrate, const float protocol_version) : DynamixelMotor(id, device_name, baudrate, protocol_version) {}

    //: Methods
        uint16_t    Dynamixel_2XL430_W250_T::EEPROM_get_model_number                () const                                { return this->get_2byte(ADDRESS_MODEL_NUMBER); }
        uint32_t    Dynamixel_2XL430_W250_T::EEPROM_get_model_information           () const                                { return this->get_4byte(ADDRESS_MODEL_INFORMATION); }

        uint8_t     Dynamixel_2XL430_W250_T::EEPROM_get_firmware_version            () const                                { return this->get_1byte(ADDRESS_FIRMWARE_VERSION); }

        uint8_t     Dynamixel_2XL430_W250_T::EEPROM_get_id                          () const                                { return this->get_1byte(ADDRESS_ID); }
        void        Dynamixel_2XL430_W250_T::EEPROM_set_id                          ( const uint8_t id )                    { this->set_1byte(ADDRESS_ID, id); }
        void        Dynamixel_2XL430_W250_T::EEPROM_update_id                       ( uint8_t id )                          { uint8_t current_id = this->EEPROM_get_id();
                                                                                                                                if (id == current_id) { return; }
                                                                                                                                this->EEPROM_set_id(id); }

        uint8_t     Dynamixel_2XL430_W250_T::EEPROM_get_baud_rate                   () const                                { return this->get_1byte(ADDRESS_BAUD_RATE); }
        void        Dynamixel_2XL430_W250_T::EEPROM_set_baud_rate                   ( const uint8_t baud_rate )             { this->set_1byte(ADDRESS_BAUD_RATE, baud_rate); }
        void        Dynamixel_2XL430_W250_T::EEPROM_update_baud_rate                ( const uint8_t baud_rate )             { uint8_t current_baud_rate = this->EEPROM_get_baud_rate();
                                                                                                                                if (baud_rate == current_baud_rate) { return; }
                                                                                                                                this->EEPROM_set_baud_rate(baud_rate); }

        uint8_t     Dynamixel_2XL430_W250_T::EEPROM_get_return_delay_time           () const                                { return this->get_1byte(ADDRESS_RETURN_DELAY_TIME); }
        void        Dynamixel_2XL430_W250_T::EEPROM_set_return_delay_time           ( const uint8_t return_delay_time )     { this->set_1byte(ADDRESS_RETURN_DELAY_TIME, return_delay_time); }
        void        Dynamixel_2XL430_W250_T::EEPROM_update_return_delay_time        ( const uint8_t return_delay_time )     { uint8_t current_return_delay_time = this->EEPROM_get_return_delay_time();
                                                                                                                                if (return_delay_time == current_return_delay_time) { return; }
                                                                                                                                this->EEPROM_set_return_delay_time(return_delay_time); }

        uint8_t     Dynamixel_2XL430_W250_T::EEPROM_get_drive_mode                  () const                                { return this->get_1byte(ADDRESS_DRIVE_MODE); }
        void        Dynamixel_2XL430_W250_T::EEPROM_set_drive_mode                  ( const uint8_t drive_mode )            { this->set_1byte(ADDRESS_DRIVE_MODE, drive_mode); }
        void        Dynamixel_2XL430_W250_T::EEPROM_update_drive_mode               ( const uint8_t drive_mode )            { uint8_t current_drive_mode = this->EEPROM_get_drive_mode();
                                                                                                                                if (drive_mode == current_drive_mode) { return; }
                                                                                                                                this->EEPROM_set_drive_mode(drive_mode); }

        uint8_t     Dynamixel_2XL430_W250_T::EEPROM_get_operating_mode              () const                                { return this->get_1byte(ADDRESS_OPERATING_MODE); }
        void        Dynamixel_2XL430_W250_T::EEPROM_set_operating_mode              ( const uint8_t operating_mode )        { this->set_1byte(ADDRESS_OPERATING_MODE, operating_mode); }
        void        Dynamixel_2XL430_W250_T::EEPROM_update_operating_mode           ( const uint8_t operating_mode )        { uint8_t current_operating_mode = this->EEPROM_get_operating_mode();
                                                                                                                                if (operating_mode == current_operating_mode) { return; }
                                                                                                                                this->EEPROM_set_operating_mode(operating_mode); }

        uint8_t     Dynamixel_2XL430_W250_T::EEPROM_get_secondary_id                () const                                { return this->get_1byte(ADDRESS_SECONDARY_ID); }
        void        Dynamixel_2XL430_W250_T::EEPROM_set_secondary_id                ( const uint8_t secondary_id )          { this->set_1byte(ADDRESS_SECONDARY_ID, secondary_id); }
        void        Dynamixel_2XL430_W250_T::EEPROM_update_secondary_id             ( const uint8_t secondary_id )          { uint8_t current_secondary_id = this->EEPROM_get_secondary_id();
                                                                                                                                if (secondary_id == current_secondary_id) { return; }
                                                                                                                                this->EEPROM_set_secondary_id(secondary_id); }

        uint8_t     Dynamixel_2XL430_W250_T::EEPROM_get_protocol_type               () const                                { return this->get_1byte(ADDRESS_PROTOCOL_TYPE); }
        void        Dynamixel_2XL430_W250_T::EEPROM_set_protocol_type               ( const uint8_t protocol_type )         { this->set_1byte(ADDRESS_PROTOCOL_TYPE, protocol_type); }
        void        Dynamixel_2XL430_W250_T::EEPROM_update_protocol_type            ( const uint8_t protocol_type )         { uint8_t current_protocol_type = this->EEPROM_get_protocol_type();
                                                                                                                                if (protocol_type == current_protocol_type) { return; }
                                                                                                                                this->EEPROM_set_protocol_type(protocol_type); }

        uint32_t    Dynamixel_2XL430_W250_T::EEPROM_get_homing_offset               () const                                { return this->get_4byte(ADDRESS_HOMING_OFFSET); }
        void        Dynamixel_2XL430_W250_T::EEPROM_set_homing_offset               ( const int32_t homing_offset )         { this->set_4byte(ADDRESS_HOMING_OFFSET, homing_offset); }
        void        Dynamixel_2XL430_W250_T::EEPROM_update_homing_offset            ( const int32_t homing_offset )         { int32_t current_homing_offset = this->EEPROM_get_homing_offset();
                                                                                                                                if (homing_offset == current_homing_offset) { return; }
                                                                                                                                this->EEPROM_set_homing_offset(homing_offset); }

        uint32_t    Dynamixel_2XL430_W250_T::EEPROM_get_moving_threshold            () const                                { return this->get_4byte(ADDRESS_MOVING_THRESHOLD); }
        void        Dynamixel_2XL430_W250_T::EEPROM_set_moving_threshold            ( const uint32_t moving_threshold )     { this->set_4byte(ADDRESS_MOVING_THRESHOLD, moving_threshold); }
        void        Dynamixel_2XL430_W250_T::EEPROM_update_moving_threshold         ( const uint32_t moving_threshold )     { uint32_t current_moving_threshold = this->EEPROM_get_moving_threshold();
                                                                                                                                if (moving_threshold == current_moving_threshold) { return; }
                                                                                                                                this->EEPROM_set_moving_threshold(moving_threshold); }

        uint8_t     Dynamixel_2XL430_W250_T::EEPROM_get_temperature_limit           () const                                { return this->get_1byte(ADDRESS_TEMPERATURE_LIMIT); }
        void        Dynamixel_2XL430_W250_T::EEPROM_set_temperature_limit           ( const uint8_t temperature_limit )     { this->set_1byte(ADDRESS_TEMPERATURE_LIMIT, temperature_limit); }
        void        Dynamixel_2XL430_W250_T::EEPROM_update_temperature_limit        ( const uint8_t temperature_limit )     { uint8_t current_temperature_limit = this->EEPROM_get_temperature_limit();
                                                                                                                                if (temperature_limit == current_temperature_limit) { return; }
                                                                                                                                this->EEPROM_set_temperature_limit(temperature_limit); }

        uint16_t    Dynamixel_2XL430_W250_T::EEPROM_get_max_voltage_limit           () const                                { return this->get_2byte(ADDRESS_MAX_VOLTAGE_LIMIT); }
        void        Dynamixel_2XL430_W250_T::EEPROM_set_max_voltage_limit           ( const uint16_t max_voltage_limit )    { this->set_2byte(ADDRESS_MAX_VOLTAGE_LIMIT, max_voltage_limit); }
        void        Dynamixel_2XL430_W250_T::EEPROM_update_max_voltage_limit        ( const uint16_t max_voltage_limit )    { uint16_t current_max_voltage_limit = this->EEPROM_get_max_voltage_limit();
                                                                                                                                if (max_voltage_limit == current_max_voltage_limit) { return; }
                                                                                                                                this->EEPROM_set_max_voltage_limit(max_voltage_limit); }

        uint16_t    Dynamixel_2XL430_W250_T::EEPROM_get_min_voltage_limit           () const                                { return this->get_2byte(ADDRESS_MIN_VOLTAGE_LIMIT); }
        void        Dynamixel_2XL430_W250_T::EEPROM_set_min_voltage_limit           ( const uint16_t min_voltage_limit )    { this->set_2byte(ADDRESS_MIN_VOLTAGE_LIMIT, min_voltage_limit); }
        void        Dynamixel_2XL430_W250_T::EEPROM_update_min_voltage_limit        ( const uint16_t min_voltage_limit )    { uint16_t current_min_voltage_limit = this->EEPROM_get_min_voltage_limit();
                                                                                                                                if (min_voltage_limit == current_min_voltage_limit) { return; }
                                                                                                                                this->EEPROM_set_min_voltage_limit(min_voltage_limit); }

        uint16_t    Dynamixel_2XL430_W250_T::EEPROM_get_pwm_limit                   () const                                { return this->get_2byte(ADDRESS_PWM_LIMIT); }
        void        Dynamixel_2XL430_W250_T::EEPROM_set_pwm_limit                   ( const uint16_t pwm_limit )            { this->set_2byte(ADDRESS_PWM_LIMIT, pwm_limit); }
        void        Dynamixel_2XL430_W250_T::EEPROM_update_pwm_limit                ( const uint16_t pwm_limit )            { uint16_t current_pwm_limit = this->EEPROM_get_pwm_limit();
                                                                                                                                if (pwm_limit == current_pwm_limit) { return; }
                                                                                                                                this->EEPROM_set_pwm_limit(pwm_limit); }

        uint32_t    Dynamixel_2XL430_W250_T::EEPROM_get_velocity_limit              () const                                { return this->get_4byte(ADDRESS_VELOCITY_LIMIT); }
        void        Dynamixel_2XL430_W250_T::EEPROM_set_velocity_limit              ( const uint32_t velocity_limit )       { this->set_4byte(ADDRESS_VELOCITY_LIMIT, velocity_limit); }
        void        Dynamixel_2XL430_W250_T::EEPROM_update_velocity_limit           ( const uint32_t velocity_limit )       { uint32_t current_velocity_limit = this->EEPROM_get_velocity_limit();
                                                                                                                                if (velocity_limit == current_velocity_limit) { return; }
                                                                                                                                this->EEPROM_set_velocity_limit(velocity_limit); }

        uint32_t    Dynamixel_2XL430_W250_T::EEPROM_get_max_position_limit          () const                                { return this->get_4byte(ADDRESS_MAX_POSITION_LIMIT); }
        void        Dynamixel_2XL430_W250_T::EEPROM_set_max_position_limit          ( const uint32_t max_position_limit )   { this->set_4byte(ADDRESS_MAX_POSITION_LIMIT, max_position_limit); }
        void        Dynamixel_2XL430_W250_T::EEPROM_update_max_position_limit       ( const uint32_t max_position_limit )   { uint32_t current_max_position_limit = this->EEPROM_get_max_position_limit();
                                                                                                                                if (max_position_limit == current_max_position_limit) { return; }
                                                                                                                                this->EEPROM_set_max_position_limit(max_position_limit); }

        uint32_t    Dynamixel_2XL430_W250_T::EEPROM_get_min_position_limit          () const                                { return this->get_4byte(ADDRESS_MIN_POSITION_LIMIT); }
        void        Dynamixel_2XL430_W250_T::EEPROM_set_min_position_limit          ( const uint32_t min_position_limit )   { this->set_4byte(ADDRESS_MIN_POSITION_LIMIT, min_position_limit); }
        void        Dynamixel_2XL430_W250_T::EEPROM_update_min_position_limit       ( const uint32_t min_position_limit )   { uint32_t current_min_position_limit = this->EEPROM_get_min_position_limit();
                                                                                                                                if (min_position_limit == current_min_position_limit) { return; }
                                                                                                                                this->EEPROM_set_min_position_limit(min_position_limit); }

        uint8_t     Dynamixel_2XL430_W250_T::EEPROM_get_startup_configuration       () const                                { return this->get_1byte(ADDRESS_STARTUP_CONFIGURATION); }
        void        Dynamixel_2XL430_W250_T::EEPROM_set_startup_configuration       ( const uint8_t startup_configuration ) { this->set_1byte(ADDRESS_STARTUP_CONFIGURATION, startup_configuration); }
        void        Dynamixel_2XL430_W250_T::EEPROM_update_startup_configuration    ( const uint8_t startup_configuration ) { uint8_t current_startup_configuration = this->EEPROM_get_startup_configuration();
                                                                                                                                if (startup_configuration == current_startup_configuration) { return; }
                                                                                                                                this->EEPROM_set_startup_configuration(startup_configuration); }

        uint8_t     Dynamixel_2XL430_W250_T::EEPROM_get_shutdown                    () const                                { return this->get_1byte(ADDRESS_SHUTDOWN); }
        void        Dynamixel_2XL430_W250_T::EEPROM_set_shutdown                    ( const uint8_t shutdown )              { this->set_1byte(ADDRESS_SHUTDOWN, shutdown); }
        void        Dynamixel_2XL430_W250_T::EEPROM_update_shutdown                 ( const uint8_t shutdown )              { uint8_t current_shutdown = this->EEPROM_get_shutdown();
                                                                                                                                if (shutdown == current_shutdown) { return; }
                                                                                                                                this->EEPROM_set_shutdown(shutdown); }

        uint8_t     Dynamixel_2XL430_W250_T::RAM_get_torque_enable                  () const                                { return this->get_1byte(ADDRESS_TORQUE_ENABLE); }
        void        Dynamixel_2XL430_W250_T::RAM_set_torque_enable                  ( const uint8_t torque_enable )         { this->set_1byte(ADDRESS_TORQUE_ENABLE, torque_enable); }

        uint8_t     Dynamixel_2XL430_W250_T::RAM_get_led                            () const                                { return this->get_1byte(ADDRESS_LED); }
        void        Dynamixel_2XL430_W250_T::RAM_set_led                            ( const uint8_t led )                   { this->set_1byte(ADDRESS_LED, led); }

        uint8_t     Dynamixel_2XL430_W250_T::RAM_get_status_return_level            () const                                { return this->get_1byte(ADDRESS_REGISTERED_INSTRUCTION); }
        void        Dynamixel_2XL430_W250_T::RAM_set_status_return_level            ( const uint8_t status_return_level )   { this->set_1byte(ADDRESS_REGISTERED_INSTRUCTION, status_return_level); }

        uint8_t     Dynamixel_2XL430_W250_T::RAM_get_registered_instruction         () const                                { return this->get_1byte(ADDRESS_REGISTERED_INSTRUCTION); }
        void        Dynamixel_2XL430_W250_T::RAM_set_registered_instruction         ( const uint8_t registered_instruction ){ this->set_1byte(ADDRESS_REGISTERED_INSTRUCTION, registered_instruction); }

        uint8_t     Dynamixel_2XL430_W250_T::RAM_get_hardware_error_status          () const                                { return this->get_1byte(ADDRESS_HARDWARE_ERROR_STATUS); }
        void        Dynamixel_2XL430_W250_T::RAM_set_hardware_error_status          ( const uint8_t hardware_error_status ) { this->set_1byte(ADDRESS_HARDWARE_ERROR_STATUS, hardware_error_status); }

        uint16_t    Dynamixel_2XL430_W250_T::RAM_get_velocity_i_gain                () const                                { return this->get_2byte(ADDRESS_VELOCITY_I_GAIN); }
        void        Dynamixel_2XL430_W250_T::RAM_set_velocity_i_gain                ( const uint16_t velocity_i_gain )      { this->set_2byte(ADDRESS_VELOCITY_I_GAIN, velocity_i_gain); }

        uint16_t    Dynamixel_2XL430_W250_T::RAM_get_velocity_p_gain                () const                                { return this->get_2byte(ADDRESS_VELOCITY_P_GAIN); }
        void        Dynamixel_2XL430_W250_T::RAM_set_velocity_p_gain                ( const uint16_t velocity_p_gain )      { this->set_2byte(ADDRESS_VELOCITY_P_GAIN, velocity_p_gain); }

        uint16_t    Dynamixel_2XL430_W250_T::RAM_get_position_d_gain                () const                                { return this->get_2byte(ADDRESS_POSITION_D_GAIN); }
        void        Dynamixel_2XL430_W250_T::RAM_set_velocity_d_gain                ( const uint16_t position_d_gain )      { this->set_2byte(ADDRESS_POSITION_D_GAIN, position_d_gain); }

        uint16_t    Dynamixel_2XL430_W250_T::RAM_get_position_i_gain                () const                                { return this->get_2byte(ADDRESS_POSITION_I_GAIN); }
        void        Dynamixel_2XL430_W250_T::RAM_set_position_i_gain                ( const uint16_t position_i_gain )      { this->set_2byte(ADDRESS_POSITION_I_GAIN, position_i_gain); }

        uint16_t    Dynamixel_2XL430_W250_T::RAM_get_position_p_gain                () const                                { return this->get_2byte(ADDRESS_POSITION_P_GAIN); }
        void        Dynamixel_2XL430_W250_T::RAM_set_position_p_gain                ( const uint16_t position_p_gain )      { this->set_2byte(ADDRESS_POSITION_P_GAIN, position_p_gain); }

        uint16_t    Dynamixel_2XL430_W250_T::RAM_get_feed_forward_2nd_gain          () const                                { return this->get_2byte(ADDRESS_FEEDFORWARD_2ND_GAIN); }
        void        Dynamixel_2XL430_W250_T::RAM_set_feed_forward_2nd_gain          ( const uint16_t feed_forward_2nd_gain ){ this->set_2byte(ADDRESS_FEEDFORWARD_2ND_GAIN, feed_forward_2nd_gain); }

        uint16_t    Dynamixel_2XL430_W250_T::RAM_get_feed_forward_1st_gain          () const                                { return this->get_2byte(ADDRESS_FEEDFORWARD_1ST_GAIN); }
        void        Dynamixel_2XL430_W250_T::RAM_set_feed_forward_1st_gain          ( const uint16_t feed_forward_1st_gain ){ this->set_2byte(ADDRESS_FEEDFORWARD_1ST_GAIN, feed_forward_1st_gain); }

        uint8_t     Dynamixel_2XL430_W250_T::RAM_get_bus_watchdog                   () const                                { return this->get_1byte(ADDRESS_BUS_WATCHDOG); }
        void        Dynamixel_2XL430_W250_T::RAM_set_bus_watchdog                   ( const uint8_t bus_watchdog )          { this->set_1byte(ADDRESS_BUS_WATCHDOG, bus_watchdog); }

        uint16_t    Dynamixel_2XL430_W250_T::RAM_get_goal_pwm                       () const                                { return this->get_2byte(ADDRESS_GOAL_PWM); }
        void        Dynamixel_2XL430_W250_T::RAM_set_goal_pwm                       ( const int16_t goal_pwm )              { this->set_2byte(ADDRESS_GOAL_PWM, goal_pwm); }

        int32_t     Dynamixel_2XL430_W250_T::RAM_get_goal_velocity                  () const                                { return this->get_4byte(ADDRESS_GOAL_VELOCITY); }
        void        Dynamixel_2XL430_W250_T::RAM_set_goal_velocity                  ( const int32_t goal_velocity )         { this->set_4byte(ADDRESS_GOAL_VELOCITY, goal_velocity); }

        uint32_t    Dynamixel_2XL430_W250_T::RAM_get_profile_acceleration           () const                                { return this->get_4byte(ADDRESS_PROFILE_ACCELERATION); }
        void        Dynamixel_2XL430_W250_T::RAM_set_profile_acceleration           ( const uint32_t profile_acceleration ) { this->set_4byte(ADDRESS_PROFILE_ACCELERATION, profile_acceleration); }

        uint32_t    Dynamixel_2XL430_W250_T::RAM_get_profile_velocity               () const                                { return this->get_4byte(ADDRESS_PROFILE_VELOCITY); }
        void        Dynamixel_2XL430_W250_T::RAM_set_profile_velocity               ( const uint32_t profile_velocity )     { this->set_4byte(ADDRESS_PROFILE_VELOCITY, profile_velocity); }

        uint32_t    Dynamixel_2XL430_W250_T::RAM_get_goal_position                  () const                                { return this->get_4byte(ADDRESS_GOAL_POSITION); }
        void        Dynamixel_2XL430_W250_T::RAM_set_goal_position                  ( const uint32_t goal_position )        { this->set_4byte(ADDRESS_GOAL_POSITION, goal_position); }

        uint16_t    Dynamixel_2XL430_W250_T::RAM_get_realtime_tick                  () const                                { return this->get_2byte(ADDRESS_REALTIME_TICK); }

        uint8_t     Dynamixel_2XL430_W250_T::RAM_get_moving                         () const                                { return this->get_1byte(ADDRESS_MOVING); }

        uint8_t     Dynamixel_2XL430_W250_T::RAM_get_moving_status                  () const                                { return this->get_1byte(ADDRESS_MOVING_STATUS); }

        uint16_t    Dynamixel_2XL430_W250_T::RAM_get_present_pwm                    () const                                { return this->get_2byte(ADDRESS_PRESENT_PWM); }

        int16_t     Dynamixel_2XL430_W250_T::RAM_get_present_load                   () const                                { return this->get_2byte(ADDRESS_PRESENT_LOAD); }

        int32_t     Dynamixel_2XL430_W250_T::RAM_get_present_velocity               () const                                { return this->get_4byte(ADDRESS_PRESENT_VELOCITY); }

        uint32_t    Dynamixel_2XL430_W250_T::RAM_get_present_position               () const                                { return this->get_4byte(ADDRESS_PRESENT_POSITION); }

        int32_t     Dynamixel_2XL430_W250_T::RAM_get_velocity_trajectory            () const                                { return this->get_4byte(ADDRESS_VELOCITY_TRAJECTORY); }

        uint32_t    Dynamixel_2XL430_W250_T::RAM_get_position_trajectory            () const                                { return this->get_4byte(ADDRESS_POSITION_TRAJECTORY); }

        uint16_t    Dynamixel_2XL430_W250_T::RAM_get_present_input_voltage          () const                                { return this->get_2byte(ADDRESS_PRESENT_INPUT_VOLTAGE); }

        int16_t     Dynamixel_2XL430_W250_T::RAM_get_present_temperature            () const                                { return this->get_2byte(ADDRESS_PRESENT_TEMPERATURE); }

        uint8_t     Dynamixel_2XL430_W250_T::RAM_get_backup_ready                   () const                                { return this->get_1byte(ADDRESS_BACKUP_READY); }

        uint16_t    Dynamixel_2XL430_W250_T::RAM_get_indirect_address               () const                                { return this->get_2byte(ADDRESS_INDIRECT_ADDRESS); }
        void        Dynamixel_2XL430_W250_T::RAM_set_indirect_address               ( const uint16_t value )                { this->set_2byte(ADDRESS_INDIRECT_ADDRESS, value); }

        uint8_t     Dynamixel_2XL430_W250_T::RAM_get_indirect_data                  () const                                { return this->get_1byte(ADDRESS_INDIRECT_DATA); }
        void        Dynamixel_2XL430_W250_T::RAM_set_indirect_data                  ( const uint8_t value )                 { this->set_1byte(ADDRESS_INDIRECT_DATA, value); }

        uint16_t    Dynamixel_2XL430_W250_T::get_model_number           () const    { return this->EEPROM_get_model_number(); }
        uint32_t    Dynamixel_2XL430_W250_T::get_model_information      () const    { return this->EEPROM_get_model_information(); }
        uint8_t     Dynamixel_2XL430_W250_T::get_firmware_version       () const    { return this->EEPROM_get_firmware_version(); }
        uint8_t     Dynamixel_2XL430_W250_T::get_id                     () const    { return this->EEPROM_get_id(); }
        uint8_t     Dynamixel_2XL430_W250_T::get_baud_rate              () const    { return this->EEPROM_get_baud_rate(); }
        uint8_t     Dynamixel_2XL430_W250_T::get_return_delay_time      () const    { return this->EEPROM_get_return_delay_time(); }
        uint8_t     Dynamixel_2XL430_W250_T::get_drive_mode             () const    { return this->EEPROM_get_drive_mode(); }
        uint8_t     Dynamixel_2XL430_W250_T::get_operating_mode         () const    { return this->EEPROM_get_operating_mode(); }
        uint8_t     Dynamixel_2XL430_W250_T::get_secondary_id           () const    { return this->EEPROM_get_secondary_id(); }
        uint8_t     Dynamixel_2XL430_W250_T::get_protocol_type          () const    { return this->EEPROM_get_protocol_type(); }
        uint32_t    Dynamixel_2XL430_W250_T::get_homing_offset          () const    { return this->EEPROM_get_homing_offset(); }
        uint32_t    Dynamixel_2XL430_W250_T::get_moving_threshold       () const    { return this->EEPROM_get_moving_threshold(); }
        uint8_t     Dynamixel_2XL430_W250_T::get_temperature_limit      () const    { return this->EEPROM_get_temperature_limit(); }
        uint16_t    Dynamixel_2XL430_W250_T::get_max_voltage_limit      () const    { return this->EEPROM_get_max_voltage_limit(); }
        uint16_t    Dynamixel_2XL430_W250_T::get_min_voltage_limit      () const    { return this->EEPROM_get_min_voltage_limit(); }
        uint16_t    Dynamixel_2XL430_W250_T::get_pwm_limit              () const    { return this->EEPROM_get_pwm_limit(); }
        uint32_t    Dynamixel_2XL430_W250_T::get_velocity_limit         () const    { return this->EEPROM_get_velocity_limit(); }
        uint32_t    Dynamixel_2XL430_W250_T::get_max_position_limit     () const    { return this->EEPROM_get_max_position_limit(); }
        uint32_t    Dynamixel_2XL430_W250_T::get_min_position_limit     () const    { return this->EEPROM_get_min_position_limit(); }
        uint8_t     Dynamixel_2XL430_W250_T::get_startup_configuration  () const    { return this->EEPROM_get_startup_configuration(); }
        uint8_t     Dynamixel_2XL430_W250_T::get_shutdown               () const    { return this->EEPROM_get_shutdown(); }
        void        Dynamixel_2XL430_W250_T::enable_torque              ()          { this->RAM_set_torque_enable( 1 ); }
        void        Dynamixel_2XL430_W250_T::disable_torque             ()          { this->RAM_set_torque_enable( 0 ); }

        bool        Dynamixel_2XL430_W250_T::is_torque_enabled          () const    { uint8_t value = this->RAM_get_torque_enable();
                                                                                        if (value == 1) { return true; }
                                                                                        return false; }

        double      Dynamixel_2XL430_W250_T::get_current_velocity       () const                    { return this->RAM_get_present_velocity ( ) * 1.374; }
        double      Dynamixel_2XL430_W250_T::get_target_velocity        () const                    { return this->RAM_get_goal_velocity ( ) * 1.374; }
        void        Dynamixel_2XL430_W250_T::set_target_velocity        ( const double velocity )   { this->RAM_set_goal_velocity( (int32_t)(velocity / 1.374) ); }
        double      Dynamixel_2XL430_W250_T::get_current_angle          () const                    { return this->RAM_get_present_position ( ) * 0.087890625; }
        double      Dynamixel_2XL430_W250_T::get_target_angle           () const                    { return this->RAM_get_goal_position ( ) * 0.087890625; }
        void        Dynamixel_2XL430_W250_T::set_target_angle           ( const double angle )      { this->RAM_set_goal_position( (int32_t)(angle / 0.087890625) ); }
} // namespace ctrl
