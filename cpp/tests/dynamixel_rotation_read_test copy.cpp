#include "../source/bewusstsein_ctrl.h"

int main()
{
    //..................................................
    // Create a motor instance
    bewusstsein::ctrl::core::dynamixel_2xl430_w250_t motor(2, "/dev/ttyUSB1", 1000000, 2.0);
    //..................................................

    //..................................................
    motor.connect();

    motor.enable_torque();

    srand(clock());

    for(int i = 0; i < 10; i++)
    {
        //if(system("cls")){throw("Terminal clear failure");}
        if(system("clear")){throw("Terminal clear failure");}

        std::cout << "Torque Enabed: " << motor.is_torque_enabled() << std::endl;
        motor.set_target_angle(rand() / ((RAND_MAX/(360))));
        std::cout << "Velocity: " << motor.get_current_velocity() << " degrees/second" << std::endl;
        std::cout << "Position: " << motor.get_current_angle() << " degrees" << std::endl;

        fflush(stdout);

        bewusstsein::util::core::sleep(400);
    }

    motor.disable_torque();

    std::cout << "Torque Enabed: " << motor.is_torque_enabled() << std::endl;

    motor.disconnect();
    //..................................................

    //..................................................
    return 0;
    //..................................................
}