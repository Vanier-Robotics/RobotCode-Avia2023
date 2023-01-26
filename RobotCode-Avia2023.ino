#include <CrcLib.h>

using namespace Crc;

#define FRONT_RIGHT_MOTOR CRC_PWM_6
#define FRONT_LEFT_MOTOR CRC_PWM_7
#define BACK_RIGHT_MOTOR CRC_PWM_11
#define BACK_LEFT_MOTOR CRC_PWM_12

void setup() {
  CrcLib::Initialize();

  CrcLib::InitializePwmOutput(FRONT_RIGHT_MOTOR);
  CrcLib::InitializePwmOutput(FRONT_LEFT_MOTOR);
  CrcLib::InitializePwmOutput(BACK_RIGHT_MOTOR);
  CrcLib::InitializePwmOutput(BACK_LEFT_MOTOR);
}

void loop() {
  CrcLib::Update();

  if (CrcLib::IsCommValid())
  {
    CrcLib::MoveHolonomic(
      -max(CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK1_Y), -127), // forward
      CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK2_X), // yaw
      CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK1_X), // strafe
      FRONT_LEFT_MOTOR, BACK_LEFT_MOTOR, FRONT_RIGHT_MOTOR, BACK_RIGHT_MOTOR);
  }
  //CrcLib::MoveHolonomic(127, 0, 0, FRONT_LEFT_MOTOR, BACK_LEFT_MOTOR, FRONT_RIGHT_MOTOR, BACK_RIGHT_MOTOR);
}
