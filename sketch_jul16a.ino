#include "MatrixMiniR4.h"

// -------------------Biến Global------------------

// Hằng số
int circumference = 20.4;
double wheelBase = 15.5;

// Điều khiển - Giai đoạn
bool begin = false;
int phase = 0;
bool reset = false;

// Cảm biến
MiniR4Analog<(18U), (19U)> greyscaleSensor;
MatrixLaserV2& laserSensorSide = MiniR4.I2C3.MXLaserV2;
MatrixLaserV2& laserSensorBack = MiniR4.I2C4.MXLaserV2;

// Debug
Adafruit_SSD1306& screen = MiniR4.OLED;

// Motor
MiniR4DriveDC<1> drivetrain;
MiniR4DC<4> motor_1;

MiniR4RC<3> claw;
MiniR4RC<1> arm;

// Nút
MiniR4BTN<1> btnDown;
MiniR4BTN<2> btnUp;

MiniR4Digital<13, 10> frameSwitchUp;
MiniR4Digital<12, 11> frameSwitchDown;

// Vị trí
MiniR4Motion pos;

int dir = 1;  // -1: trái, 1: phải

// Di chuyển
int basePower = 85;

// Color
int color[6];

// ------------------------------------------------

// ---------------------Class----------------------

// Note: Đơn vị là cm
struct Dis {
  double value;

  explicit Dis(double value)
    : value(value - 2) {}
};

struct Ms {
  int value;

  explicit Ms(int value)
    : value(value) {}
};


// ------------------------------------------------

// ---------------------Hàm------------------------

// Tính toán
double toRadian(double angle) {
  return angle * 3.141 / 180;
}

// Di chuyển
double prevAngle = 0;

void move(int target = 0) {
  drivetrain.MoveGyro(basePower, target);
}
void move(int powerL, int powerR) {
  drivetrain.MoveSync(powerL, powerR);
}
void move(Ms time, int target = 0, int power = basePower, bool brake = true) {
  prevAngle = pos.getEuler(MiniR4Motion::AxisType::Yaw);

  drivetrain.MoveGyroTime(power, target, time.value / 1000, brake);
}
void move(Dis distance, int target = 0, int power = basePower, bool brake = true) {
  prevAngle = pos.getEuler(MiniR4Motion::AxisType::Yaw);

  double deg = distance.value / circumference * 360 - 14;

  drivetrain.MoveGyroDegs(power, target, deg, brake);
}

void turn(int target, int mode = 1, int power = basePower, bool brake = true) {
  prevAngle = pos.getEuler(MiniR4Motion::AxisType::Yaw);

  drivetrain.TurnGyro(power, target, mode, brake);

  delay(50);
}

void moveArc(double r, double angle, int powerR = basePower) {
  const double Kp_r = 0.05;
  const double Kp_angle = 0.75;
  const double Kd_angle = 0.05;

  while (true) {
    double angleError = angle - getAngle();

    int gyro = getGyroZ();

    if (abs(angleError) < 2) {
      drivetrain.brake(true);

      break;
    }

    int32_t speed[4];
    pos.getAllSpeed(speed);

    double denominator = speed[2] + speed[1];

    double cur_r;

    if (abs(denominator) < 0.001) cur_r = r;
    else cur_r = (wheelBase / 2.0) * (speed[2] - speed[1]) / denominator;

    double rError = (r - cur_r) * Kp_r;

    double anglePower = abs(angleError) * Kp_angle - Kd_angle * abs(gyro);

    anglePower = constrain(anglePower, 10, powerR);


    int powerL = anglePower * (r - wheelBase / 2) / (r + wheelBase / 2);

    move(powerL - rError, anglePower + rError);
  }
}

// Vị trí và hướng
double getAngle() {
  return pos.getEuler(MiniR4Motion::AxisType::Yaw);
}

double getDistance() {
  return circumference * (drivetrain.getDegrees() + 14) / 360 + 2;
}

double getLaserDisSide() {
  return laserSensorSide.getDistance() * 0.1;
}

double getLaserDisBack() {
  return laserSensorBack.getDistance() * 0.1;
}

double getGyroZ() {
  return pos.getGyro(MiniR4Motion::AxisType::Z);
}


// Dò line
int greyscaleSetpoint = 930;

void lineDetector(int angle = 0, int power = basePower, double distance = 5) {
  int greyscale;

  while (true) {
    greyscale = greyscaleSensor.getAIL();

    drivetrain.MoveGyro(power, 0);

    if (greyscale > greyscaleSetpoint) break;
  }

  drivetrain.brake(false);

  move(Dis{ distance }, -getAngle());

  int powerMotor = constrain(basePower * 90 / abs(angle), 20, 100);

  turn(angle, 1, powerMotor);

  delay(100);
}

// Color
void color_detector(int t = 0) {
  while (true) {
    if (MiniR4.I2C4.MXColorV3.getRaw_R() < 93 && MiniR4.I2C4.MXColorV3.getRaw_G() < 93 && MiniR4.I2C4.MXColorV3.getRaw_B() < 93) {
      drivetrain.brake(false);
      break;
    } else {
      move(50, 50);
    }
  }


  color[1 + 3 * t] = MiniR4.I2C4.MXColorV3.getColorID();
  while (getAngle() < 20) {
    move(30, -30);
  }
  drivetrain.brake(true);
  color[0 + 3 * t] = MiniR4.I2C4.MXColorV3.getColorID();
  while (getAngle() > -20) {
    move(-30, 30);
  }
  drivetrain.brake(true);
  color[2 + 3 * t] = MiniR4.I2C4.MXColorV3.getColorID();
  return;
}

// Debug
void print(int x, int y, auto content) {
  screen.setCursor(x, y);
  screen.print(content);
}

// ------------------------------------------------

// ---------------------Main-----------------------

// Cơ chế làm nhiệm vụ
bool hasOpened = false;
void toggleFrame(bool reset = false, bool again = false) {
  if (again || (!hasOpened && !reset)) {
    unsigned long startTime = millis();

    while (frameSwitchDown.getL() == 0) {
      if (millis() - startTime >= 1000) break;

      motor_1.setPower(-35);
    }

    motor_1.setBrake(true);

    hasOpened = true;
  } else {
    while (frameSwitchUp.getL() == 0)
      motor_1.setPower(45);

    motor_1.setBrake(true);

    hasOpened = false;
  }
}

bool hasLiftedArm = true;
void toggleArm(int angle = -1) {
  if (angle == -1)
    if (hasLiftedArm) {
      arm.setAngle(150);

      hasLiftedArm = false;
    } else {
      arm.setAngle(10);

      hasLiftedArm = true;
    }
  else arm.setAngle(angle);
}

bool hasOpenedClaw = false;
void toggleClaw(int angle = -1) {
  if (angle == -1)
    if (hasOpenedClaw) {
      claw.setAngle(15);

      hasOpenedClaw = true;
    } else {
      claw.setAngle(144);

      hasOpenedClaw = false;
    }
  else claw.setAngle(angle);
}

void shake(unsigned long timeout = 2500) {
  unsigned long startTime = millis();

  while (getAngle() > -30)
    move(-basePower * 0.7, -10);

  while (getAngle() < 0)
    move(basePower * 0.7, 10);

  drivetrain.brake(false);
  delay(100);

  while (getAngle() < 30)
    move(-10, -basePower * 0.7);

  while (getAngle() > 0)
    move(10, basePower * 0.7);

  drivetrain.brake(false);

  turn(-getAngle());

  delay(100);
}

void getBrick(double curAngle) {
  double angle = curAngle - getAngle();

  while (getLaserDisBack() > 22.5)
    drivetrain.MoveGyro(-basePower / 2, angle);

  drivetrain.brake(false);

  toggleFrame();

  move(Dis{ 10 }, 0, basePower * 0.5);

  shake();

  move(Ms{ 2000 }, -getAngle(), -basePower / 2.5);

  pos.resetIMUValues();
}

void shortcut(int dir, double error) {
  while (getLaserDisSide() > 35) move();
  while (getLaserDisSide() < 35) move();

  drivetrain.brake(true);

  turn(50 * dir, 0);

  delay(100);

  move(Dis{ 32 });

  lineDetector(40 * dir, basePower, 8);
}

void phaseAm1() {
  toggleArm();

  delay(100);

  toggleClaw();

  move(Dis{20});

  turn(55);

  move(Dis{50});

  lineDetector(35);

  
}

void phase0() {
  moveArc(-14, 90, basePower / 2);

  drivetrain.brake(false);

  double angle = 90 - getAngle();

  while (getLaserDisSide() > 18) move(angle);
  while (getLaserDisSide() < 18) move();
  while (getLaserDisSide() > 18) move();
  while (getLaserDisSide() < 18) move();

  drivetrain.brake(true);

  delay(100);

  turn(-90 - getAngle());

  move(Dis{ 9 }, 0, -basePower * 0.5, false);

  toggleFrame();

  move(Dis{ 5 }, 0, -basePower * 0.5, false);

  toggleFrame(false, true);

  turn(82 - getAngle());

  angle = getAngle();

  delay(100);

  move(Dis{ 40 }, 0, -basePower, false);

  move(Dis{ 15 });

  delay(100);

  turn(-10 - getAngle());

  angle += getAngle();

  delay(100);

  double distance = 24 / cos(toRadian(angle)) - getLaserDisSide() * tan(toRadian(angle));

  move(Dis{ distance - 5 });

  delay(100);

  lineDetector(92 - angle, basePower * 0.5);

  phase++;
}

void phase1() {
  while (getLaserDisSide() > 18)
    move();

  while (getLaserDisSide() < 18)
    move();

  drivetrain.brake(false);

  //19
  move(Dis{ 20 });

  turn(90 - getAngle());

  delay(100);

  move(Dis{ 15 }, 0, -basePower / 2, false);

  turn(-25 - getAngle());

  delay(100);

  toggleFrame();

  turn(-35 - (-25 - getAngle()));

  delay(100);

  move(Dis{ 18 });

  lineDetector(150, basePower, 8);

  getBrick(150);

  delay(100);

  move(Dis{ 20 });

  moveArc(-20, 90 - getAngle());

  move(Dis{ 17.5 }, 90 - getAngle());

  turn(-45 - getAngle());

  delay(100);

  move(Dis{ 8 });

  lineDetector(135, basePower, 8);

  move(Dis{ 19 }, 135 - getAngle(), -basePower);
  move(Ms{ 1500 }, 0, -basePower / 3);

  delay(100);

  move(Dis{ 1.5 }, 0, basePower / 2);

  toggleFrame(false, true);

  move(Dis{ 6.7 * 1.3 }, 0, basePower / 2);

  turn(41, 0);

  move(Dis{ 30 }, 0, -basePower / 2);

  toggleFrame();

  move(Dis{ 12 }, 0, basePower / 2);

  lineDetector(-41, basePower, 8);

  phase++;
}

void phase2() {
  move(Dis{ 17 }, -41 - getAngle());

  turn(-45);

  delay(100);

  move(Dis{ 20 });

  lineDetector(-135, basePower, 5);

  getBrick(-135);

  move(Dis{ 5 });

  turn(-50 - getAngle());

  delay(100);

  move(Dis{ 18 }, -50 - getAngle());

  lineDetector(53, basePower, 8);

  shortcut(-1, 3);

  while (getLaserDisSide() > 40)  drivetrain.MoveGyro(-basePower, 0);
  drivetrain.brake(true);
  toggleFrame();
  move(Dis{ 13 }, 0, -basePower);

  phase++;
}

void setup() {
  MiniR4.begin();
  MiniR4.Motion.begin();
  laserSensorSide.begin();
  laserSensorBack.begin();

  // Debug
  Serial.begin(9600);

  screen.clearDisplay();

  // Pin
  MiniR4.PWR.setBattCell(2);

  // Drivetrain
  drivetrain.begin(2, 3, false, true);

  // Thiết lập lại giá trị IMU, PID
  pos.resetIMUValues();

  drivetrain.setMoveGyroPID(6.01, 0, 2.15);
  drivetrain.setMoveSyncPID(0.02, 0, 0.04);
  drivetrain.setTurnGyroPID(30, 0.027, 7);
}

void loop() {
  // Debug
  if (begin == false) {
    screen.clearDisplay();

    print(0, 0, getAngle());
    print(40, 0, greyscaleSensor.getAIL());
    print(80, 0, getLaserDisSide());
    print(80, 10, getLaserDisBack());

    screen.display();
  }

  if (btnDown.getState()) {
    // NOTE: bỏ đống này vô resetFn
    drivetrain.brake(true);
    drivetrain.resetCounter();

    pos.resetIMUValues();
    prevAngle = 0;

    screen.clearDisplay();

    toggleFrame(true);
    claw.setAngle(15);
    arm.setAngle(10);

    begin = false;
    reset = true;
    phase = 0;
  }

  if (btnUp.getState()) {
    begin = true;
    reset = false;
  }

  if (begin == true) {
    // Debug
    screen.clearDisplay();

    print(0, 0, getAngle());
    print(40, 0, greyscaleSensor.getAIL());
    print(0, 10, getLaserDisBack());
    // print(40, 10, getAngle());

    screen.display();

    delay(500);

    // Giai đoạn
    if (phase == 0) phase0();
    if (phase == 1) phase1();
    if (phase == 2) phase2();
  }
}
