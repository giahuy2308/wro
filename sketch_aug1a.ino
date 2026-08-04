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

// Nút
MiniR4BTN<1> btnDown;
MiniR4BTN<2> btnUp;

MiniR4Digital<13, 10> frameSwitch;

// Vị trí
MiniR4Motion pos;

int dir = 1; // -1: trái, 1: phải  

// Di chuyển
int basePower = 60;

// ------------------------------------------------

// ---------------------Class----------------------

struct Dis {
    double value;

    explicit Dis(double value) : value(value - 2) {}
};

struct Ms {
    int value;

    explicit Ms(int value) : value(value) {}
};


// ------------------------------------------------

// ---------------------Hàm------------------------

// Tính toán
double toRadian(double angle) {
  return angle * 3.141 / 180;
}

// Cơ chế làm nhiệm vụ
bool isOpen = false;
void toggleFrame(bool reset = false, bool again = false) {
  if (again || (!isOpen && !reset)) {
      motor_1.setPower(-35);

      delay(again ? 200 : 550);

      motor_1.setBrake(true);

      isOpen = true;
  } 
  else {
      while (frameSwitch.getL() == 0)
          motor_1.setPower(35);

      motor_1.setBrake(true);

      isOpen = false;
  }
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

void moveArc(double r, int angle, int powerR = basePower) {
  const double Kp_r = 0.05;
  const double Kp_angle = 0.75;
  const double Kd_angle = 0.05;

  while (true) {
    int angleError = angle - getAngle();

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
double getIMUAngle() {
  return pos.getEuler(MiniR4Motion::AxisType::Yaw);
}
double getAngle() {
  double angle = getIMUAngle() + prevAngle;

  return angle;
}

double getDistance() {
  return circumference * (drivetrain.getDegrees() + 14) / 360 + 2;
}

double getLaserDis() {
  return laserSensor.getDistance() - 4.5;
}

double getGyroZ() {
  return pos.getGyro(MiniR4Motion::AxisType::Z);
}

// Dò line
void lineDetector(int lineAngle = 0) {
  int greyscale;
  double headingError = lineAngle - getIMUAngle();
  while (true) {
    greyscale = greyscaleSensor.getAIL();
    moveForward();
    if (greyscale > greyscaleSetpoint) {
      break;
    }
  }
  drivetrain.brake(false);
  // Serial.print(lineAngle);
  // Serial.print(" ");
  // Serial.println(headingError);

  moveForward(Dis{ 10 / cos((90 - headingError) * PI / 180) });
  turn(headingError);

  return;
}

// Debug
void print(int x, int y, auto content) {
  screen.setCursor(x, y);
  screen.print(content);
}

// ------------------------------------------------

// ---------------------Main-----------------------

void phase0() {
  moveArc(-15, 90, 30);

  drivetrain.brake(false);

  int angle = getAngle() - 90;

  while (getLaserDis() > 175) 
    move(angle);
  
  while (getLaserDis() < 175)
    move();
 
  while (getLaserDis() > 175)
    move();

  while (getLaserDis() < 175)
    move();

  drivetrain.brake(false);

  delay(200);

  turn(-90 - getIMUAngle());

  move(Dis{6}, 0, -basePower * 0.5, false);

  toggleFrame();

  move(Dis{6}, 0, -basePower * 0.5, false);

  toggleFrame(false, true);

  turn(85 - getIMUAngle());

  move(Dis{40}, 0, -basePower, false);

  move(Dis{5});

  double lastAngle = getAngle();

  delay(200);

  turn(-10);

  delay(200);
  
  double radian = toRadian(lastAngle + getAngle());

  int distance = 25 / cos(radian) - getLaserDis() / 10 * tan(radian);

  move(Dis{distance});

  phase++;
}

void phase1 () {

}

void setup() {
  MiniR4.begin();
  MiniR4.Motion.begin();
  laserSensor.begin();

  // Debug
  Serial.begin(9600);

  screen.clearDisplay();

  // Servo
  claw.setHWDir(false);

  // Pin
  MiniR4.PWR.setBattCell(2);

  // Drivetrain
  drivetrain.begin(2, 3, false, true);

  // Thiết lập lại giá trị IMU, PID
  pos.resetIMUValues();

  drivetrain.setMoveGyroPID(6, 0, 5);
  drivetrain.setMoveSyncPID(0.02, 0, 0.04);
  drivetrain.setTurnGyroPID(30, 0.027, 7);
}

void loop() {
  // Debug
  if (begin == false) {
    screen.clearDisplay();

    print(0, 0, getAngle());
    print(40, 0, greyscaleSensor.getAIL());
    print(40, 0, getLaserDis());

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
    claw.setAngle(10);

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
    print(40, 10, getAngle());

    screen.display();

    // Giai đoạn
    if (phase == 0) phase0();
    if (phase == 1) phase1();

    // if (phase == 0) {
    //   if (MiniR4.D4.getL() == 1) {
    //     delay(100);

    //     motor_1.setBrake(true);

    //     delay(500);

    //     phase++;
    //   }

    //   motor_1.setPower(35);
    // }

    // if (phase == 1) {
    //   motor_1.setPower(-20);

    //   delay(690);

    //   motor_1.setBrake(true);

    //   phase++;
    // }
  }
}
