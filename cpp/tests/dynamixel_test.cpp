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

    motor.set_velocity(180);

    bewusstsein::util::core::sleep(2000);

    motor.set_velocity(-180);

    bewusstsein::util::core::sleep(2000);

    motor.disable_torque();

    motor.disconnect();
    //..................................................

    //..................................................
    return 0;
    //..................................................
}