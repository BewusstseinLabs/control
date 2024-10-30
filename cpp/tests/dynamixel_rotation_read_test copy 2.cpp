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

    double angle = 0;
    double step = 180;
    double steps = 360/step;

    for(int i = 0; i < steps; i++)
    {
        motor.set_target_angle(angle);

        //if(system("cls")){throw("Terminal clear failure");}
        if(system("clear")){throw("Terminal clear failure");}

        std::cout << "Torque Enabed: " << motor.is_torque_enabled() << std::endl;
        std::cout << "Velocity: " << motor.get_current_velocity() << " degrees/second" << std::endl;
        std::cout << "Position: " << motor.get_current_angle() << " degrees" << std::endl;

        fflush(stdout);

        bewusstsein::util::core::sleep(0);

        angle += step;
    }

    motor.disable_torque();

    std::cout << "Torque Enabed: " << motor.is_torque_enabled() << std::endl;

    motor.disconnect();
    //..................................................

    //..................................................
    return 0;
    //..................................................
}