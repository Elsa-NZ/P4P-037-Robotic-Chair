#ifndef OPERATING_FSM_H
#define OPERATING_FSM_H

class OperatingFSM {
 private:
  enum State {
    IDLE,
    CONTROLLER1,
    CONTROLLER2,
    CONTROLLER3,
    CONTROLLER4,
    CONTROLLER5,
    CONTROLLER6,
    CONTROLLER7
  };
  State currentState;
  unsigned long switchStateTimer;

  void idle();
  void controller1();  // note: rename these controller with descriptive names
                       // based on their functionality
  void controller2();
  void controller3();
  void controller4();
  void controller5();
  void controller6();
  void controller7();

  unsigned long starttime;  // Variable to store the start time for trajectory calculations

 public:
  OperatingFSM();
  void updateFSM();
  void setState(State newState);
};

#endif  // OPERATING_FSM_H