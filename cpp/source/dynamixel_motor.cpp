// Copyright 2024 Shane W. Mulcahy

//: C++ Headers
#include <string>
#include <iostream>

//: C++ Library Headers
#include <dynamixel_sdk/dynamixel_sdk.h>

//: This Header
#include "bewusstsein_control/c++/include/dynamixel_motor.hpp"

namespace ctrl
{
    //: Constructors
        DynamixelMotor::DynamixelMotor( const uint id, const std::string& device_name, const uint baudrate, const float protocol_version ) :
            id( id ), baudrate( baudrate )
        {
            portHandler     = dynamixel::PortHandler::getPortHandler( device_name.c_str() );
            packetHandler   = dynamixel::PacketHandler::getPacketHandler( protocol_version );
        }

    //: Methods
        template <typename T>
        bool DynamixelMotor::dxl_range_handler( const T min, const T max, const T value ) const
        {
            if ( ( value < min ) || ( value > max ) )
            {
                return false;
            }
            return true;
        }

        template <typename T>
        T DynamixelMotor::dxl_range_clip( const T min, const T max, const T value ) const
        {
            if ( value < min )
            {
                std::cout << "Value is less than minimum. Value has been clipped to minimum" << std::endl;
                return min;
            }
            else if ( value > max )
            {
                std::cout << "Value is greater than maximum. Value has been clipped to maximum" << std::endl;
                return max;
            }
            else
            {
                return value;
            }
        }

        void DynamixelMotor::dxl_error_handler( const dxl_error error ) const
        {
            if ( error != 0 )
            {
                std::cout << packetHandler->getRxPacketError( error ) << std::endl;
            }
        }

        void DynamixelMotor::dxl_comm_result_handler( const dxl_comm_result result ) const
        {
            if ( result != COMM_SUCCESS )
            {
                std::cout << packetHandler->getTxRxResult( result ) << std::endl;
            }
        }

        bool DynamixelMotor::connect( )
        {
            if ( portHandler->openPort() )
            {
                if ( portHandler->setBaudRate(this->baudrate) )
                {
                    return true;
                }
                else
                {
                    std::cout << "Failed to change the baudrate!" << std::endl;
                    return false;
                }
            }
            else
            {
                std::cout << "Failed to open the port!" << std::endl;
                return false;
            }
        }

        void DynamixelMotor::disconnect()
        {
            portHandler->closePort();
        }

        void DynamixelMotor::set_1byte( const uint16_t address, const uint8_t value )
        {
            dxl_error error = 0;
            dxl_comm_result result = packetHandler->write1ByteTxRx( portHandler, id, address, value, &error );
            this->dxl_error_handler( error );
            this->dxl_comm_result_handler( result );
        }

        uint8_t DynamixelMotor::get_1byte( const uint16_t address ) const
        {
            dxl_error error = 0;
            uint8_t value = 0;
            dxl_comm_result result = packetHandler->read1ByteTxRx( portHandler, id, address, &value, &error );
            this->dxl_error_handler( error );
            this->dxl_comm_result_handler( result );
            return value;
        }

        void DynamixelMotor::set_2byte( const uint16_t address, const uint16_t value)
        {
            dxl_error error = 0;
            dxl_comm_result result = packetHandler->write2ByteTxRx( portHandler, id, address, value, &error );
            this->dxl_error_handler( error );
            this->dxl_comm_result_handler( result );
        }

        uint16_t DynamixelMotor::get_2byte( const uint16_t address ) const
        {
            dxl_error error = 0;
            uint16_t value = 0;
            dxl_comm_result result = packetHandler->read2ByteTxRx( portHandler, id, address, &value, &error );
            this->dxl_error_handler( error );
            this->dxl_comm_result_handler( result );
            return value;
        }

        void DynamixelMotor::set_4byte( const uint16_t address, const uint32_t value )
        {
            dxl_error error = 0;
            dxl_comm_result result = packetHandler->write4ByteTxRx( portHandler, id, address, value, &error );
            this->dxl_error_handler( error );
            this->dxl_comm_result_handler( result );
        }

        uint32_t DynamixelMotor::get_4byte( const uint16_t address ) const
        {
            dxl_error error = 0;
            uint32_t value = 0;
            dxl_comm_result result = packetHandler->read4ByteTxRx( portHandler, id, address, &value, &error );
            this->dxl_error_handler( error );
            this->dxl_comm_result_handler( result );
            return value;
        }
} // namespace ctrl
