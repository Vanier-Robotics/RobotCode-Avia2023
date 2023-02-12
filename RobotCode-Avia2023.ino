#include <ArduinoExtra.h>
#include <RobotUtils.h>
#include <CrcLib.h>

using namespace Crc;
using namespace rou;

PwmHandle frontLeftMotor(CRC_PWM_7);
PwmHandle frontRightMotor(CRC_PWM_6);
PwmHandle backRightMotor(CRC_PWM_11);
PwmHandle backLeftMotor(CRC_PWM_12);

PwmHandle liftMotor(CRC_PWM_5);

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

  MainMode(PwmHandle* frontLeftMotor, PwmHandle* frontRightMotor, PwmHandle* backRightMotor, PwmHandle* backLeftMotor,
    PwmHandle* liftMotor)
  : m_arcadeDriveModule(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor)
  , m_liftModule(liftMotor)
  {
    m_controller.analogBind(ANALOG::JOYSTICK1_Y, aex::Function<void(int8_t)>::bind<MainMode>(*this, &MainMode::setForwardChannel));
    m_controller.analogBind(ANALOG::JOYSTICK1_X, aex::Function<void(int8_t)>::bind<MainMode>(*this, &MainMode::setYawChannel));

    m_controller.digitalBind(BUTTON::R1, aex::Function<void(bool)>::bind<MainMode>(*this, &MainMode::moveLiftUp));
    m_controller.digitalBind(BUTTON::L1, aex::Function<void(bool)>::bind<MainMode>(*this, &MainMode::moveLiftDown));
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

      if ((breakTime -= dt) < 0)
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

  void moveLiftUp(bool isPressed)
  {
    if (isPressed)
    {
      m_liftSpeed += 120;
    }
  }

  void moveLiftDown(bool isPressed)
  {
    if (isPressed)
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
  float breakTime;
};

MainMode mainMode(&frontLeftMotor, &frontRightMotor, &backRightMotor, &backLeftMotor, &liftMotor);
IdleMode idleMode;
ModeManager modeManager;
HandleManager handleManager;

Mode* IdleMode::StartingMode = &mainMode;
Mode* MainMode::StoppedMode = &idleMode;
ModeManager& Mode::ModeManager = modeManager;

unsigned long lastUpdateTime = 0;

void setup() {
  CrcLib::Initialize();

  handleManager.addHandle(&frontLeftMotor);
  handleManager.addHandle(&frontRightMotor);
  handleManager.addHandle(&backLeftMotor);
  handleManager.addHandle(&backRightMotor);

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
