#include <ArduinoExtra.h>
#include <RobotUtils.h>
#include <CrcLib.h>

using namespace Crc;
using namespace rou;

PwmHandle frontLeftMotor(CRC_PWM_5);
PwmHandle frontRightMotor(CRC_PWM_6);

PwmHandle frontLeftMotorLow(CRC_PWM_5,0,1000);
PwmHandle frontRightMotorLow(CRC_PWM_6,0,1000);

PwmHandle backRightMotor(CRC_PWM_7);
PwmHandle backLeftMotor(CRC_PWM_8);

PwmHandle liftMotor(CRC_PWM_4);
PwmHandle clawMotor(CRC_PWM_9);

PwmHandle clawServoRight(CRC_PWM_11);
PwmHandle clawServoLeft(CRC_PWM_12);

EncoderHandle clawEncoder(CRC_ENCO_A, CRC_ENCO_B);

class IdleMode : public Mode
{
public:
  static Mode* StartingMode;

  IdleMode()
  {
    m_controller.digitalBind(BUTTON::START, start); 
  }

  static void start(bool isPressed)
  {
    if (isPressed)
    {
      Mode::ModeManager.changeMode(StartingMode);
    }
  }

  void load() override
  {
  }

  void unload() override
  {
  }

  void update(float dt) override
  {
    if (CrcLib::IsCommValid())
    {
      m_controller.update();
    }
  }
};

class MainMode : public Mode
{
public:
  static Mode* StoppedMode;
  static Mode* SlowDriveMode;

  MainMode(PwmHandle* frontLeftMotor, PwmHandle* frontRightMotor, PwmHandle* backRightMotor, PwmHandle* backLeftMotor,
    PwmHandle* liftMotor)
  : m_arcadeDriveModule(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor)
  , m_liftModule(liftMotor)
  {
    m_controller.digitalBind(BUTTON::START, nextMode); 
    m_controller.analogBind(ANALOG::JOYSTICK1_Y, aex::Function<void(int8_t)>::bind<MainMode>(*this, &MainMode::setForwardChannel));
    m_controller.analogBind(ANALOG::JOYSTICK2_X, aex::Function<void(int8_t)>::bind<MainMode>(*this, &MainMode::setYawChannel));

    m_controller.analogBind(ANALOG::GACHETTE_R, aex::Function<void(int8_t)>::bind<MainMode>(*this, &MainMode::moveLiftUp));
    m_controller.analogBind(ANALOG::GACHETTE_L, aex::Function<void(int8_t)>::bind<MainMode>(*this, &MainMode::moveLiftDown));
  }

  static void nextMode(bool isPressed)
  {
    if (isPressed)
    {
      Mode::ModeManager.changeMode(SlowDriveMode);
    }
  }

  void load() override
  {
  }

  void unload() override
  {
    m_arcadeDriveModule.move(0, 0);
  }

  void update(float dt) override
  {
    if (CrcLib::IsCommValid())
    {
      m_controller.update();
    }
    else
    {
      Mode::ModeManager.changeMode(StoppedMode);
    }

    brakeStatus(60); // (Detect point to 60)

    if(!m_isBraking) 
    {
      m_arcadeDriveModule.move(m_forwardChannel, m_yawChannel);
    }
    else
    {
      if(m_forwardChannel > 0)
      { 
        m_arcadeDriveModule.move(m_forwardChannel, m_yawChannel);
      }
      else
      {
        // not super smooth, but better than nothing...
        m_arcadeDriveModule.move(20, m_yawChannel); 
      }

      if ((breakTime -= dt) <= 0.f)
      {
        m_isBraking = false;
      }
    }

    m_liftModule.setSpeed(m_liftSpeed);

    m_liftSpeed = 0;
    m_forwardChannel = 0;
    m_yawChannel = 0;
  }

  void setForwardChannel(int8_t value)
  {
    m_forwardChannel = static_cast<int8_t>(min(max(-static_cast<int16_t>(value), -128), 127));
  }

  void setYawChannel(int8_t value)
  {
    m_yawChannel = static_cast<int8_t>(min(max(-static_cast<int16_t>(value), -128), 127));
  }

  void moveLiftUp(int8_t value)
  {
    if (value > -120)
    {
      m_liftSpeed += 120;
    }
  }

  void moveLiftDown(int8_t value)
  {
    if (value > -120)
    {
      m_liftSpeed -= 120;
    }
  }

  void brakeStatus(int8_t switchPoint)
  {
    if (m_forwardChannel <= switchPoint && m_isGoingForward)
    {
      breakTime = 0.3f;
      m_isBraking = true;
    }

    m_isGoingForward = (m_forwardChannel > switchPoint);
  }

private:
  ArcadeDriveModule m_arcadeDriveModule;
  MotorModule m_liftModule;

  int8_t m_forwardChannel;
  int8_t m_yawChannel;
  int8_t m_liftSpeed;
  bool m_isGoingForward = false;
  bool m_isBraking = false;
  float breakTime = 0.f;
};

class DriftMode : public Mode
{
public:
  static Mode* StoppedMode;
  static Mode* DriveMode;

  DriftMode(PwmHandle* frontLeftMotorLow, PwmHandle* frontRightMotorLow, PwmHandle* backRightMotor, PwmHandle* backLeftMotor,
    PwmHandle* liftMotor)
  : m_holonomicDriveModule(frontLeftMotorLow, backLeftMotor, frontRightMotorLow, backRightMotor)
  , m_liftModule(liftMotor)
  {
    m_controller.digitalBind(BUTTON::START, nextMode); 
    m_controller.analogBind(ANALOG::JOYSTICK1_Y, aex::Function<void(int8_t)>::bind<DriftMode>(*this, &DriftMode::setForwardChannel));
    m_controller.analogBind(ANALOG::JOYSTICK2_X, aex::Function<void(int8_t)>::bind<DriftMode>(*this, &DriftMode::setYawChannel));
    m_controller.analogBind(ANALOG::JOYSTICK1_X, aex::Function<void(int8_t)>::bind<DriftMode>(*this, &DriftMode::setStrafeChannel));

    m_controller.analogBind(ANALOG::GACHETTE_R, aex::Function<void(int8_t)>::bind<DriftMode>(*this, &DriftMode::moveLiftUp));
    m_controller.analogBind(ANALOG::GACHETTE_L, aex::Function<void(int8_t)>::bind<DriftMode>(*this, &DriftMode::moveLiftDown));
  }

  static void nextMode(bool isPressed)
  {
    if (isPressed)
    {
      Mode::ModeManager.changeMode(DriveMode);
    }
  }

  void load() override
  {
  }

  void unload() override
  {
    m_holonomicDriveModule.move(0, 0, 0);
  }

  void update(float dt) override
  {
    if (CrcLib::IsCommValid())
    {
      m_controller.update();
    }
    else
    {
      Mode::ModeManager.changeMode(StoppedMode);
    }

    m_liftModule.setSpeed(m_liftSpeed);

    m_liftSpeed = 0;
    m_forwardChannel = 0;
    m_yawChannel = 0;
  }

  void setForwardChannel(int8_t value)
  {
    m_forwardChannel = static_cast<int8_t>(min(max(-static_cast<int16_t>(value), -128), 127));
  }

  void setYawChannel(int8_t value)
  {
    m_yawChannel = static_cast<int8_t>(min(max(-static_cast<int16_t>(value), -128), 127));
  }

  void setStrafeChannel(int8_t value)
  {
    m_strafeChannel = value;
  }

  void moveLiftUp(int8_t value)
  {
    if (value > -120)
    {
      m_liftSpeed += 120;
    }
  }

  void moveLiftDown(int8_t value)
  {
    if (value > -120)
    {
      m_liftSpeed -= 120;
    }
  }

private:
  HolonomicDriveModule m_holonomicDriveModule;
  MotorModule m_liftModule;

  int8_t m_forwardChannel;
  int8_t m_yawChannel;
  int8_t m_liftSpeed;
  int8_t m_strafeChannel;
};

MainMode mainMode(&frontLeftMotor, &frontRightMotor, &backRightMotor, &backLeftMotor, &liftMotor);
DriftMode driftMode(&frontLeftMotorLow, &frontRightMotorLow, &backRightMotor, &backLeftMotor, &liftMotor);
IdleMode idleMode;
ModeManager modeManager;
HandleManager handleManager;

Mode* IdleMode::StartingMode = &mainMode;
Mode* MainMode::StoppedMode = &idleMode;
Mode* MainMode::SlowDriveMode = &driftMode;
Mode* DriftMode::StoppedMode = &idleMode;
Mode* DriftMode::DriveMode = &mainMode;
ModeManager& Mode::ModeManager = modeManager;

unsigned long lastUpdateTime = 0;

void setup() {
  CrcLib::Initialize();
  handleManager.addHandle(&backLeftMotor);
  handleManager.addHandle(&backRightMotor);
  handleManager.addHandle(&frontLeftMotorLow);
  handleManager.addHandle(&frontRightMotorLow);

  handleManager.addHandle(&liftMotor);

  modeManager.changeMode(&idleMode);

  lastUpdateTime = millis();
}

void loop() {
  CrcLib::Update();

  unsigned long currentTime = millis();
  float dt = static_cast<float>(currentTime - lastUpdateTime) / 1000.f;
  lastUpdateTime = currentTime;

  handleManager.releaseAll();

  modeManager.update(dt);
}
