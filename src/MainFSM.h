// Header for main FSM of chair system

#ifndef MAIN_FSM_H
#define MAIN_FSM_H

class MainFSM {
 private:
  enum State { IDLE, OPERATING, HOMING, ERROR };
  State currentState;
  float timeInState;
  bool isHomed;

  bool outsideSafeRange();

  void homing();
  void operating();
  void error();
  void idle();

 public:
  MainFSM();
  void updateFSM();
  void setState(State newState);
};

#endif  // MAIN_FSM_H
