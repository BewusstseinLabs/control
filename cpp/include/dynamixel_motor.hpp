// Copyright 2024 Shane W. Mulcahy

#ifndef BEWUSSTSEIN_CONTROL_CPP_INCLUDE_DYNAMIXEL_MOTOR_HPP_
#define BEWUSSTSEIN_CONTROL_CPP_INCLUDE_DYNAMIXEL_MOTOR_HPP_

//: C++ Headers
#include <string>
#include <iostream>

//: C++ Library Headers
#include <dynamixel_sdk/dynamixel_sdk.h>

namespace ctrl
{
    typedef uint8_t dxl_error;
    typedef uint dxl_comm_result;

    class DynamixelMotor
    {
        //: Members
        protected:
            dynamixel::PortHandler      *portHandler;
            dynamixel::PacketHandler    *packetHandler;
            uint                        id;
            uint                        baudrate;

        //: Constructors
        public:
            DynamixelMotor ( const uint id, const std::string& device_name, const uint baudrate, const float protocol_version );


        //: Destructors
        public:
            virtual ~DynamixelMotor () { portHandler->closePort(); }

        //: Methods
        public:
            template <typename T> bool dxl_range_handler    ( const T min, const T max, const T value ) const;
            template <typename T> T dxl_range_clip          ( const T min, const T max, const T value ) const;

            void        dxl_error_handler           ( const dxl_error error ) const;
            void        dxl_comm_result_handler     ( const dxl_comm_result result ) const;

            bool        connect     ();
            void        disconnect  ();

            void        set_1byte ( const uint16_t address, const uint8_t value );
            uint8_t     get_1byte ( const uint16_t address ) const;

            void        set_2byte ( const uint16_t address, const uint16_t value );
            uint16_t    get_2byte ( const uint16_t address ) const;

            void        set_4byte ( const uint16_t address, const uint32_t value );
            uint32_t    get_4byte ( const uint16_t address ) const;
    };
} // namespace ctrl

#endif // BEWUSSTSEIN_CONTROL_CPP_INCLUDE_DYNAMIXEL_MOTOR_HPP_
