#include "MatrixMiniR4.h"

// -------------------Biến Global------------------

// Hằng số
int circumference = 20.4;
int wheelBase = 15;

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

// Servo
MiniR4RC<3> claw;

// Nút
MiniR4BTN<1> btn_down;
MiniR4BTN<2> btn_up;

// Vị trí
MiniR4Motion pos;

int dir = 1;  // -1: trái, 1: phải

// Di chuyển
int basePower = 30;

// color
int color[6];

// ------------------------------------------------

// ---------------------Class----------------------

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

// Điều khiển
void resetFn() {
  while (MiniR4.D4.getL() == 0) {
    motor_1.setPower(35);
  }

  motor_1.setBrake(true);
}

// Di chuyển
double prevAngle = 0;

void moveForward(int target = 0) {
  drivetrain.MoveGyro(basePower, target);
}
void moveForward(int powerL, int powerR) {
  drivetrain.MoveSync(powerL, powerR);
}
void moveForward(Ms time, int power = basePower, int target = 0, bool brake = true) {
  prevAngle = pos.getEuler(MiniR4Motion::AxisType::Yaw);

  drivetrain.MoveGyroTime(power, target, time.value / 1000, brake);
}
void moveForward(Dis distance, int power = basePower, int target = 0, bool brake = true) {
  prevAngle = pos.getEuler(MiniR4Motion::AxisType::Yaw);

  double deg = distance.value / circumference * 360 - 14;

  drivetrain.MoveGyroDegs(power, target, deg, brake);
}

void turn(int target, int mode = 1, int power = basePower, bool brake = true) {
  prevAngle = pos.getEuler(MiniR4Motion::AxisType::Yaw);

  drivetrain.TurnGyro(power, target, mode, brake);

  delay(50);
}

void moveArc(int r, int target, int powerR = basePower) {
  if (getAngle() >= target) {
    phase++;

    drivetrain.brake(true);

    return;
  }

  int32_t speed[4];

  pos.getAllSpeed(speed);

  int powerL = powerR * (r - wheelBase / 2) / (r + wheelBase / 2);

  double cur_r = (wheelBase / 2) * abs(speed[2] - speed[1]) / (speed[2] + speed[1]);

  double error = (r - cur_r) * 0.01;

  moveForward(powerL - error, powerR + error);
}

// Vị trí
double getIMUAngle() {
  return pos.getEuler(MiniR4Motion::AxisType::Yaw);
}
double getAngle() {
  double angle = getIMUAngle() + prevAngle;

  return angle;
}

int getDistance() {
  return circumference * (drivetrain.getDegrees() + 14) / 360 + 2;
}




// Dò line
int lastError;
int greyscaleSetpoint = 950;

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

void lineFollow(int target = 0) {
  int greyscale = greyscaleSensor.getAIL();

  double headingError = getIMUAngle() - target;

  if (greyscale > greyscaleSetpoint && abs(headingError) < 3) {
    if (headingError > 0) dir = 1;
    else dir = -1;

    moveForward();

    return;
  }

  drivetrain.brake(false);

  int gsError = (greyscaleSetpoint - greyscale) * dir;
  // int correction = gsError * 0.05 + (gsError - lastError) * 0.15;
  int correction = gsError * 0.3 + (gsError - lastError) * 0.25 + headingError * 0.5;


  moveForward(basePower - correction, basePower + correction);
  // moveForward(-20,-20);

  lastError = gsError;
}

// Motor
void grabTool(int basePower = 45, int time = 500) {
  motor_1.setPower(45);

  delay(500);
  motor_1.setBrake(true);
  return;
}

// Debug
void print(int x, int y, auto content) {
  screen.setCursor(x, y);
  screen.print(content);
}

// ------------------------------------------------

// ---------------------Main-----------------------

void color_detector(int t = 0) {
  while (true) {
    if (MiniR4.I2C4.MXColorV3.getRaw_R() < 93 && MiniR4.I2C4.MXColorV3.getRaw_G() < 93 && MiniR4.I2C4.MXColorV3.getRaw_B() < 93) {
      drivetrain.brake(false);
      break;
    } else {
      moveForward(50, 50);
    }
  }


  color[1 + 3 * t] = MiniR4.I2C4.MXColorV3.getColorID();
  while (getAngle() < 20) {
    moveForward(30, -30);
  }
  drivetrain.brake(true);
  color[0 + 3 * t] = MiniR4.I2C4.MXColorV3.getColorID();
  while (getAngle() > -20) {
    moveForward(-30, 30);
  }
  drivetrain.brake(true);
  color[2 + 3 * t] = MiniR4.I2C4.MXColorV3.getColorID();
  return;
}


void phase0() {
  while (getAngle() < 90) {
    if (getAngle() > 60 && greyscaleSensor.getAIL() > 850) {
      drivetrain.resetCounter();
      break;
    }

    int power = 20 + (90 - getAngle()) * 0.2;

    moveForward(power * 2.5, power);

    drivetrain.resetCounter();
  }

  drivetrain.brake(false);

  while (getDistance() < 50)
    lineFollow();

  drivetrain.brake(false);

  while (laserSensor.getDistance() < 190)
    moveForward();

  while (laserSensor.getDistance() > 190)
    moveForward();

  while (laserSensor.getDistance() < 190)
    moveForward();

  drivetrain.brake(false);

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

  // Motor
  motor_1.begin();

  // Claw
  claw.begin();
  claw.setHWDir(true);

  // Thiết lập lại giá trị IMU, PID
  pos.resetIMUValues();

  drivetrain.setMoveGyroPID(6, 0, 5);
  drivetrain.setTurnGyroPID(30, 0.027, 7);

  //color
  MiniR4.I2C4.MXColorV3.begin();
  MiniR4.I2C4.MXColorV3.setWhiteBalance(282, 395, 336);
}



void loop() {
  // Debug
  if (begin == false) {
    screen.clearDisplay();

    print(10, 10, getAngle());
    print(50, 10, greyscaleSensor.getAIL());
    print(90, 10, getIMUAngle());

    screen.display();
  }

  if (btn_down.getState()) {
    // if (reset == false)
    //   resetFn();

    // NOTE: bỏ đống này vô resetFn
    drivetrain.brake(true);
    drivetrain.resetCounter();

    pos.resetIMUValues();
    claw.setAngle(0);
    prevAngle = 0;

    screen.clearDisplay();

    begin = false;
    reset = true;
    phase = 0;
  }

  if (btn_up.getState()) {
    begin = true;
    reset = false;
  }

  if (begin == true) {
    // Debug
    screen.clearDisplay();

    print(10, 10, getAngle());
    print(50, 10, greyscaleSensor.getAIL());
    print(90, 10, getIMUAngle());


    screen.display();

    // Giai đoạn
    // lineFollow();
    if (phase == 0) {

      claw.setAngle(60);
      phase++;
    };

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
