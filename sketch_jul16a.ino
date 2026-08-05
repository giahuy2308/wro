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
MatrixLaserV2& laserSensor = MiniR4.I2C3.MXLaserV2;

// Debug
Adafruit_SSD1306& screen = MiniR4.OLED;

// Motor
MiniR4DriveDC<1> drivetrain;
MiniR4DC<4> motor_1;

MiniR4RC<3> claw;
MiniR4RC<2> arm;

// Nút
MiniR4BTN<1> btnDown;
MiniR4BTN<2> btnUp;

MiniR4Digital<13, 10> frameSwitchUp;
MiniR4Digital<12, 11> frameSwitchDown;

// Vị trí
MiniR4Motion pos;

int dir = 1;  // -1: trái, 1: phải

// Di chuyển
int basePower = 54;

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

double getLaserDis() {
  return laserSensor.getDistance() / 10 - 0.45;
}

double getGyroZ() {
  return pos.getGyro(MiniR4Motion::AxisType::Z);
}


// Dò line
int greyscaleSetpoint = 950;

void lineDetector(int angle = 0, int power = basePower) {
  int greyscale;
  int counter = 1;

  double headingError = angle - getAngle();

  while (true) {
    greyscale = greyscaleSensor.getAIL();
    move(basePower, basePower);
    if (greyscale > greyscaleSetpoint) break;
  }

  drivetrain.brake(false);

  move(Dis{ 9 });

  turn(headingError);

  return;
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
    // motor_1.setPower(-35);

    while (frameSwitchDown.getL() == 0) {
      motor_1.setPower(-35);

      delay(500);
    }

    motor_1.setBrake(true);

    hasOpened = true;
  } else {
    while (frameSwitchUp.getL() == 0)
      motor_1.setPower(35);

    motor_1.setBrake(true);

    hasOpened = false;
  }
}

bool hasLiftedArm = true;
void toggleArm(int angle) {
  if (isnan(angle))
    if (hasLiftedArm) {
      arm.setAngle(144);

      hasLiftedArm = false;
    } else {
      arm.setAngle(0);

      hasLiftedArm = true;
    }
  else arm.setAngle(angle);
}

bool hasOpenedClaw = false;
void toggleClaw(int angle) {
  if (isnan(angle))
    if (hasOpenedClaw) {
      claw.setAngle(0);

      hasOpenedClaw = true;
    } else {
      claw.setAngle(144);

      hasOpenedClaw = false;
    }
  else claw.setAngle(angle);
}

void shake(int times = 2) {

  turn(-15);

  double accError = getAngle() + 15;

  delay(200);

  for (int i = 0; i < times; i++) {
    turn(30);
    accEcrror += getAngle() - 30;
    delay(100);

    turn(-30);
    accEcrror += getAngle() + 30;
    delay(100);
  }

  turn(15 - accError);
  delay(100);
}

void getBrick(Dis distance) {
  delay(200);

  move(Dis{ distance }, 0, -basePower, false);

  toggleFrame();

  shake();

  move(Dis{ 7 }, 0, -basePower / 2);
}

void phase0() {
  moveArc(-14, 90, basePower / 2);

  drivetrain.brake(false);

  double angle = 90 - getAngle();

  while (getLaserDis() > 16)
    move(angle);

  while (getLaserDis() < 16)
    move();

  while (getLaserDis() > 16)
    move();

  while (getLaserDis() < 16)
    move();

  drivetrain.brake(false);

  delay(200);

  turn(-90 - getAngle());

  move(Dis{ 6 }, 0, -basePower * 0.5, false);

  toggleFrame();

  move(Dis{ 6 }, 0, -basePower * 0.5, false);

  toggleFrame(false, true);

  turn(82 - getAngle());

  double lastAngle = getAngle();

  move(Dis{ 40 }, 0, -basePower, false);

  move(Dis{ 15 });

  delay(200);

  turn(-10);

  delay(200);

  angle = lastAngle + getAngle();

  double radius = 9 / (2 * sin(angle));

  double distance = 24 / cos(toRadian(angle)) - getLaserDis() * tan(toRadian(angle));

  move(Dis{ distance - 5 });

  delay(200);

  lineDetector(92 - angle, basePower * 0.5);

  delay(100);

  phase++;
}

void phase1() {
  while (getLaserDis() > 16)
    move();

  while (getLaserDis() < 16)
    move();

  drivetrain.brake(false);

  move(Dis{ 30 });

  delay(200);

  turn(90 - getAngle());

  delay(200);

  move(Dis{ 15 }, 0, -basePower * 0.5, false);

  toggleFrame();

  turn(-50 - getAngle());

  delay(200);

  move(Dis{ 18 });

  lineDetector(130 - getAngle());

  getBrick(Dis{ 13 });

  move(Dis{ 25 });

  moveArc(-24, 90);

  phase++;
}

void phase2() {
  move(Dis{ 22 });

  turn(-45);

  delay(200);

  double angle = getAngle();

  move(Dis{ 30 });

  angle += getAngle();

  lineDetector(0 - angle);

  getBrick(Dis{ 15 });

  moveArc(30, 90);

  while (getLaserDis() > 28) move();

  move(Dis{ 3 });

  moveArc(-25, 90);

  while (getLaserDis() > 11) move();

  move(Dis{ 34 });

  turn(-90 - getAngle());

  while (getLaserDis() > 16) move(-basePower, -basePower);

  move(Dis{ 5 }, 0, -basePower);

  toggleFrame();

  move(Dis{ 20 });

  phase++;
}

void setup() {
  MiniR4.begin();
  MiniR4.Motion.begin();
  laserSensor.begin();

  // Debug
  Serial.begin(9600);

  screen.clearDisplay();

  // Pin
  MiniR4.PWR.setBattCell(2);

  // Drivetrain
  drivetrain.begin(2, 3, false, true);

  // Thiết lập lại giá trị IMU, PID
  pos.resetIMUValues();

  drivetrain.setMoveGyroPID(5, 0, 6);
  drivetrain.setMoveSyncPID(0.02, 0, 0.04);
  drivetrain.setTurnGyroPID(30, 0.027, 7);
}

void loop() {
  // Debug
  if (begin == false) {
    screen.clearDisplay();

    print(0, 0, getAngle());
    print(40, 0, greyscaleSensor.getAIL());
    print(80, 0, getLaserDis());

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
    claw.setAngle(0);

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
    print(0, 10, getLaserDis());
    // print(40, 10, getAngle());

    screen.display();

    delay(500);

    // Giai đoạn
    if (phase == 0) phase0();
    if (phase == 1) phase1();
  }
}
