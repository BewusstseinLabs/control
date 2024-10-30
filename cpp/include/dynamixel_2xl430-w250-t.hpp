// Copyright 2024 Shane W. Mulcahy

#ifndef BEWUSSTSEIN_CONTROL_CPP_INCLUDE_DYNAMIXEL_2XL430_W250_T_HPP_
#define BEWUSSTSEIN_CONTROL_CPP_INCLUDE_DYNAMIXEL_2XL430_W250_T_HPP_

//: C++ Headers
#include <string>

//: Project Headers
#include "bewusstsein_control/c++/include/dynamixel_motor.hpp"

namespace ctrl
{
    class Dynamixel_2XL430_W250_T : public DynamixelMotor
    {
        //: Constructors
        public:
            Dynamixel_2XL430_W250_T ( const uint id, const std::string& device_name, const uint baudrate, const float protocol_version );

        // :Destructors
        public:
            virtual ~Dynamixel_2XL430_W250_T ();

        //: Methods
        public:
            uint16_t    EEPROM_get_model_number             () const;
            uint32_t    EEPROM_get_model_information        () const;
            uint8_t     EEPROM_get_firmware_version         () const;
            uint8_t     EEPROM_get_id                       () const;
            void        EEPROM_set_id                       ( const uint8_t id );
            void        EEPROM_update_id                    ( const uint8_t id );
            uint8_t     EEPROM_get_baud_rate                () const;
            void        EEPROM_set_baud_rate                ( const uint8_t baud_rate );
            void        EEPROM_update_baud_rate             ( const uint8_t baud_rate );
            uint8_t     EEPROM_get_return_delay_time        () const;
            void        EEPROM_set_return_delay_time        ( const uint8_t return_delay_time );
            void        EEPROM_update_return_delay_time     ( const uint8_t return_delay_time );
            uint8_t     EEPROM_get_drive_mode               () const;
            void        EEPROM_set_drive_mode               ( const uint8_t drive_mode );
            void        EEPROM_update_drive_mode            ( const uint8_t drive_mode );
            uint8_t     EEPROM_get_operating_mode           () const;
            void        EEPROM_set_operating_mode           ( const uint8_t operating_mode );
            void        EEPROM_update_operating_mode        ( const uint8_t operating_mode );
            uint8_t     EEPROM_get_secondary_id             () const;
            void        EEPROM_set_secondary_id             ( const uint8_t secondary_id );
            void        EEPROM_update_secondary_id          ( const uint8_t secondary_id );
            uint8_t     EEPROM_get_protocol_type            () const;
            void        EEPROM_set_protocol_type            ( const uint8_t protocol_type );
            void        EEPROM_update_protocol_type         ( const uint8_t protocol_type );
            uint32_t    EEPROM_get_homing_offset            () const;
            void        EEPROM_set_homing_offset            ( const int32_t homing_offset );
            void        EEPROM_update_homing_offset         ( const int32_t homing_offset );
            uint32_t    EEPROM_get_moving_threshold         () const;
            void        EEPROM_set_moving_threshold         ( const uint32_t moving_threshold );
            void        EEPROM_update_moving_threshold      ( const uint32_t moving_threshold );
            uint8_t     EEPROM_get_temperature_limit        () const;
            void        EEPROM_set_temperature_limit        ( const uint8_t temperature_limit );
            void        EEPROM_update_temperature_limit     ( const uint8_t temperature_limit );
            uint16_t    EEPROM_get_max_voltage_limit        () const;
            void        EEPROM_set_max_voltage_limit        ( const uint16_t max_voltage_limit );
            void        EEPROM_update_max_voltage_limit     ( const uint16_t max_voltage_limit );
            uint16_t    EEPROM_get_min_voltage_limit        () const;
            void        EEPROM_set_min_voltage_limit        ( const uint16_t min_voltage_limit );
            void        EEPROM_update_min_voltage_limit     ( const uint16_t min_voltage_limit );
            uint16_t    EEPROM_get_pwm_limit                () const;
            void        EEPROM_set_pwm_limit                ( const uint16_t pwm_limit );
            void        EEPROM_update_pwm_limit             ( const uint16_t pwm_limit );
            uint32_t    EEPROM_get_velocity_limit           () const;
            void        EEPROM_set_velocity_limit           ( const uint32_t velocity_limit );
            void        EEPROM_update_velocity_limit        ( const uint32_t velocity_limit );
            uint32_t    EEPROM_get_max_position_limit       () const;
            void        EEPROM_set_max_position_limit       ( const uint32_t max_position_limit );
            void        EEPROM_update_max_position_limit    ( const uint32_t max_position_limit );
            uint32_t    EEPROM_get_min_position_limit       () const;
            void        EEPROM_set_min_position_limit       ( const uint32_t min_position_limit );
            void        EEPROM_update_min_position_limit    ( const uint32_t min_position_limit );
            uint8_t     EEPROM_get_startup_configuration    () const;
            void        EEPROM_set_startup_configuration    ( const uint8_t startup_configuration );
            void        EEPROM_update_startup_configuration ( const uint8_t startup_configuration );
            uint8_t     EEPROM_get_shutdown                 () const;
            void        EEPROM_set_shutdown                 ( const uint8_t shutdown );
            void        EEPROM_update_shutdown              ( const uint8_t shutdown );
            uint8_t     RAM_get_torque_enable               () const;
            void        RAM_set_torque_enable               ( const uint8_t torque_enable );
            uint8_t     RAM_get_led                         () const;
            void        RAM_set_led                         ( const uint8_t led );
            uint8_t     RAM_get_status_return_level         () const;
            void        RAM_set_status_return_level         ( const uint8_t status_return_level );
            uint8_t     RAM_get_registered_instruction      () const;
            void        RAM_set_registered_instruction      ( const uint8_t registered_instruction );
            uint8_t     RAM_get_hardware_error_status       () const;
            void        RAM_set_hardware_error_status       ( const uint8_t hardware_error_status );
            uint16_t    RAM_get_velocity_i_gain             () const;
            void        RAM_set_velocity_i_gain             ( const uint16_t velocity_i_gain );
            uint16_t    RAM_get_velocity_p_gain             () const;
            void        RAM_set_velocity_p_gain             ( const uint16_t velocity_p_gain );
            uint16_t    RAM_get_position_d_gain             () const;
            void        RAM_set_velocity_d_gain             ( const uint16_t position_d_gain );
            uint16_t    RAM_get_position_i_gain             () const;
            void        RAM_set_position_i_gain             ( const uint16_t position_i_gain );
            uint16_t    RAM_get_position_p_gain             () const;
            void        RAM_set_position_p_gain             ( const uint16_t position_p_gain );
            uint16_t    RAM_get_feed_forward_2nd_gain       () const;
            void        RAM_set_feed_forward_2nd_gain       ( const uint16_t feed_forward_2nd_gain );
            uint16_t    RAM_get_feed_forward_1st_gain       () const;
            void        RAM_set_feed_forward_1st_gain       ( const uint16_t feed_forward_1st_gain );
            uint8_t     RAM_get_bus_watchdog                () const;
            void        RAM_set_bus_watchdog                ( const uint8_t bus_watchdog );
            uint16_t    RAM_get_goal_pwm                    () const;
            void        RAM_set_goal_pwm                    ( const int16_t goal_pwm );
            int32_t     RAM_get_goal_velocity               () const;
            void        RAM_set_goal_velocity               ( const int32_t goal_velocity );
            uint32_t    RAM_get_profile_acceleration        () const;
            void        RAM_set_profile_acceleration        ( const uint32_t profile_acceleration );
            uint32_t    RAM_get_profile_velocity            () const;
            void        RAM_set_profile_velocity            ( const uint32_t profile_velocity );
            uint32_t    RAM_get_goal_position               () const;
            void        RAM_set_goal_position               ( const uint32_t goal_position );
            uint16_t    RAM_get_realtime_tick               () const;
            uint8_t     RAM_get_moving                      () const;
            uint8_t     RAM_get_moving_status               () const;
            uint16_t    RAM_get_present_pwm                 () const;
            int16_t     RAM_get_present_load                () const;
            int32_t     RAM_get_present_velocity            () const;
            uint32_t    RAM_get_present_position            () const;
            int32_t     RAM_get_velocity_trajectory         () const;
            uint32_t    RAM_get_position_trajectory         () const;
            uint16_t    RAM_get_present_input_voltage       () const;
            int16_t     RAM_get_present_temperature         () const;
            uint8_t     RAM_get_backup_ready                () const;
            uint16_t    RAM_get_indirect_address            () const;
            void        RAM_set_indirect_address            ( const uint16_t value );
            uint8_t     RAM_get_indirect_data               () const;
            void        RAM_set_indirect_data               ( const uint8_t value );

        //: Methods
        public:
            uint16_t    get_model_number            () const;
            uint32_t    get_model_information       () const;
            uint8_t     get_firmware_version        () const;
            uint8_t     get_id                      () const;
            uint8_t     get_baud_rate               () const;
            uint8_t     get_return_delay_time       () const;
            uint8_t     get_drive_mode              () const;
            uint8_t     get_operating_mode          () const;
            uint8_t     get_secondary_id            () const;
            uint8_t     get_protocol_type           () const;
            uint32_t    get_homing_offset           () const;
            uint32_t    get_moving_threshold        () const;
            uint8_t     get_temperature_limit       () const;
            uint16_t    get_max_voltage_limit       () const;
            uint16_t    get_min_voltage_limit       () const;
            uint16_t    get_pwm_limit               () const;
            uint32_t    get_velocity_limit          () const;
            uint32_t    get_max_position_limit      () const;
            uint32_t    get_min_position_limit      () const;
            uint8_t     get_startup_configuration   () const;
            uint8_t     get_shutdown                () const;
            void        enable_torque               ();
            void        disable_torque              ();
            bool        is_torque_enabled           () const;
            double      get_current_velocity        () const;
            double      get_target_velocity         () const;
            void        set_target_velocity         ( const double velocity );
            double      get_current_angle           () const;
            double      get_target_angle            () const;
            void        set_target_angle            ( const double angle );
    };
} // namespace ctrl

#endif // BEWUSSTSEIN_CONTROL_CPP_INCLUDE_DYNAMIXEL_2XL430_W250_T_HPP_
