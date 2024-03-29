#include <ArduinoExtra.h>
#include <RobotUtils.h>
#include <CrcLib.h>

using namespace Crc;
using namespace rou;

PwmHandle frontLeftMotor(CRC_PWM_5);
PwmHandle frontRightMotor(CRC_PWM_6);
PwmHandle backRightMotor(CRC_PWM_7);
PwmHandle backLeftMotor(CRC_PWM_8);

PwmHandle liftMotor(CRC_PWM_4);
PwmHandle clawMotor(CRC_PWM_10);

PwmHandle clawServoLeft(CRC_PWM_12, 500, 2500);
PwmHandle clawServoRight(CRC_PWM_11, 500, 2500);

//EncoderHandle clawEncoder(CRC_ENCO_A, CRC_ENCO_B);

constexpr float CLAW_SPEED = 2.0f;

constexpr float CLAW_SPEED = 2.0f;

class IdleMode : public Mode
{
public:
  static Mode* StartingMode;

  IdleMode(PwmHandle* clawServoLeft, PwmHandle* clawServoRight)
  : m_leftClawModule(clawServoLeft)
  , m_rightClawModule(clawServoRight)
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


    //m_leftClawModule.setSpeed(0);
    //m_rightClawModule.setSpeed(20);
    
  }

private:
  MotorModule m_leftClawModule;
  MotorModule m_rightClawModule;
};

class MainMode : public Mode
{
public:
  static Mode* StoppedMode;

  MainMode(PwmHandle* frontLeftMotor, PwmHandle* frontRightMotor, PwmHandle* backRightMotor, PwmHandle* backLeftMotor,
    PwmHandle* liftMotor, PwmHandle* leftClaw, PwmHandle* rightClaw, PwmHandle* claw)
  : m_holonomicDriveModule(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor)
  , m_liftModule(liftMotor)
  , m_clawModule(claw)
  , m_leftClawModule(leftClaw)
  , m_rightClawModule(rightClaw)
  {
    m_controller.analogBind(ANALOG::JOYSTICK1_Y, aex::Function<void(int8_t)>::bind<MainMode>(*this, &MainMode::setForwardChannel));
    m_controller.analogBind(ANALOG::JOYSTICK1_X, aex::Function<void(int8_t)>::bind<MainMode>(*this, &MainMode::setYawChannel));
    //m_controller.analogBind(ANALOG::JOYSTICK2_X, aex::Function<void(int8_t)>::bind<MainMode>(*this, &MainMode::setStrafeChannel));

    m_controller.analogBind(ANALOG::GACHETTE_R, aex::Function<void(int8_t)>::bind<MainMode>(*this, &MainMode::moveLiftUp));
    m_controller.analogBind(ANALOG::GACHETTE_L, aex::Function<void(int8_t)>::bind<MainMode>(*this, &MainMode::moveLiftDown));

    m_controller.digitalBind(BUTTON::COLORS_UP, aex::Function<void(bool)>::bind<MainMode>(*this, &MainMode::openClaw));
    m_controller.digitalBind(BUTTON::COLORS_DOWN, aex::Function<void(bool)>::bind<MainMode>(*this, &MainMode::closeClaw));
    m_controller.digitalBind(BUTTON::COLORS_RIGHT, aex::Function<void(bool)>::bind<MainMode>(*this, &MainMode::setClawCCW));
    m_controller.digitalBind(BUTTON::COLORS_LEFT, aex::Function<void(bool)>::bind<MainMode>(*this, &MainMode::setClawCW));
  }

  void load() override
  {
    m_openClaw = false;
    m_closeClaw = false;

    m_clawPositionRight = 0.0f;
    m_clawPositionLeft = 0.0f;
  }

  void unload() override
  {
    m_holonomicDriveModule.move(0, 0, 0);
    m_clawModule.setSpeed(0);
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
      m_holonomicDriveModule.move(m_forwardChannel, m_yawChannel, m_strafeChannel);
    }
    else
    {
      if(m_forwardChannel > 0)
      { 
        m_holonomicDriveModule.move(m_forwardChannel, m_yawChannel, m_strafeChannel);
      }
      else
      {
        // not super smooth, but better than nothing...
        m_holonomicDriveModule.move(20, m_yawChannel, m_strafeChannel); 
      }

      if ((breakTime -= dt) <= 0.f)
      {
        m_isBraking = false;
      }
    }

    m_liftModule.setSpeed(m_liftSpeed);
    m_clawModule.setSpeed(m_clawRotation);

    if (m_openClaw)
    {
      m_clawPositionRight -= dt * CLAW_SPEED;
      m_clawPositionLeft -= dt * CLAW_SPEED;
    }
    if (m_closeClaw)
    {
      m_clawPositionRight += dt * CLAW_SPEED;
      m_clawPositionLeft += dt * CLAW_SPEED;
    }

    if (m_clawPositionRight < 0.0f) m_clawPositionRight = 0.0f;
    else if (m_clawPositionRight > 1.0f) m_clawPositionRight = 1.0f;

    if (m_clawPositionLeft < 0.0f) m_clawPositionLeft = 0.0f;
    else if (m_clawPositionLeft > 1.0f) m_clawPositionLeft = 1.0f;


    m_leftClawModule.setSpeed(-20 + static_cast<int8_t>(147.f * m_clawPositionLeft));
    m_rightClawModule.setSpeed(20 - static_cast<int8_t>(148.f * m_clawPositionRight));


    m_liftSpeed = 0;
    m_forwardChannel = 0;
    m_yawChannel = 0;
    m_clawRotation = 0;
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

  void openClaw(bool value)
  {
    m_openClaw = value;
  }
  
  void closeClaw(bool value)
  {
    m_closeClaw = value;
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

  void setClawCCW(bool value)
  {
    if (value)
    {

      //if (clawEncoder.getPosition() > -100)
      {
        m_clawRotation = -60;
      }
    }
  }

  void setClawCW(bool value)
  {
    if (value)
    {
      //if (clawEncoder.getPosition() < 400)
      {
        m_clawRotation = 60;
      }
    }
  }

private:
  HolonomicDriveModule m_holonomicDriveModule;
  MotorModule m_liftModule;
  MotorModule m_clawModule;
  MotorModule m_leftClawModule;
  MotorModule m_rightClawModule;

  int8_t m_forwardChannel;
  int8_t m_yawChannel;
  int8_t m_liftSpeed;
  int8_t m_strafeChannel;
  bool m_isGoingForward = false;
  bool m_isBraking = false;
  float breakTime = 0.f;
  bool m_openClaw;
  bool m_closeClaw;
  float m_clawPositionRight;
  float m_clawPositionLeft;
  int8_t m_clawRotation;
};

MainMode mainMode(&frontLeftMotor, &frontRightMotor, &backRightMotor, &backLeftMotor, &liftMotor, &clawServoLeft, &clawServoRight, &clawMotor);
IdleMode idleMode(&clawServoLeft, &clawServoRight);
ModeManager modeManager;
HandleManager handleManager;

Mode* IdleMode::StartingMode = &mainMode;
Mode* MainMode::StoppedMode = &idleMode;
ModeManager& Mode::ModeManager = modeManager;

unsigned long lastUpdateTime = 0;

void setup() {
  CrcLib::Initialize();

  //Serial.begin(9600);

  handleManager.addHandle(&frontLeftMotor);
  handleManager.addHandle(&frontRightMotor);
  handleManager.addHandle(&backLeftMotor);
  handleManager.addHandle(&backRightMotor);

  handleManager.addHandle(&liftMotor);
  handleManager.addHandle(&clawMotor);
  handleManager.addHandle(&clawServoRight);
  handleManager.addHandle(&clawServoLeft);
  //handleManager.addHandle(&clawEncoder);

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
