typedef enum {
  LEFT, 
  RIGHT
} Dir;


typedef struct {
  short speed;
  Dir dir;
} MotorState;

class Motor {
  short pinA;
  short pinB;
  short speedPin;
  short deadZone;
  short zeroThreshold;
  
public:
  Motor(short pinA, short pinB, short speedPin, short deadZone = 0, short zeroThreshold = 5);
  
  void init();
  void left(int speed);
  void right(int speed);
  void stop();
  void setSpeed(int speed);
  void setMotor(int speed, Dir dir);
  void setMotor(int speed);
};
