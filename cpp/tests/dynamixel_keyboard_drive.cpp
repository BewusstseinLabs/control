#include "../source/bewusstsein_ctrl.h"

int getch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main()
{
    bewusstsein::ctrl::core::dynamixel_2xl430_w250_t motor(2, "/dev/ttyUSB1", 1000000, 2.0);

    motor.connect();

    motor.enable_torque();

    while (true)
    {
        int key = getch();
        if (key == 27) // Escape key or arrow key
        {
            if (getch() == 91) // Check for arrow key
            {
                key = getch();
                if (key == 65) // Up arrow key
                {
                    double current_angle = motor.get_current_angle();
                    motor.set_target_angle(current_angle + 1);
                }
                else if (key == 66) // Down arrow key
                {
                    double current_angle = motor.get_current_angle();
                    motor.set_target_angle(current_angle - 1);
                }
            }
            else // Escape key
            {
                break;
            }
        }
    }

    motor.disable_torque();

    motor.disconnect();

    return 0;
}