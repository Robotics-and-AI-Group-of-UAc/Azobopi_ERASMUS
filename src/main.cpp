#include "main.h"

void Encoders_Interrupt(void)
{
  byte next_state, table_input;

  // Encoder 1
  next_state     = digitalRead(ENC1_A) << 1;
  next_state    |= digitalRead(ENC1_B);
  table_input    = (encoder1_state << 2) | next_state;
  encoder1_pos  -= encoder_table[table_input];
  encoder1_state = next_state;

  // Encoder 2
  next_state     = digitalRead(ENC2_A) << 1;
  next_state    |= digitalRead(ENC2_B);
  table_input    = (encoder2_state << 2) | next_state;
  encoder2_pos  += encoder_table[table_input];
  encoder2_state = next_state;

  counterPID += 1;
}

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  Encoders_Interrupt();
  portEXIT_CRITICAL_ISR(&timerMux);
}

void startTimer()
{
  if (timer == NULL) {
    timer = timerBegin(2, 4000, true);
    timerAttachInterrupt(timer, &onTimer, true);
  }
  timerAlarmWrite(timer, 1, true);
  timerAlarmEnable(timer);
}

void stopTimer()
{
  if (timer != NULL) {
    timerAlarmDisable(timer);
    timerDetachInterrupt(timer);
    timerEnd(timer);
    timer        = NULL;
    encoder1_pos = 0;
    encoder2_pos = 0;
  }
}

void setLed(int r, int g, int b)
{
  DEBUG_PRINTLN_FCT("exc setLED fct");
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

void read_direction_buttons()
{
  DEBUG_PRINTLN_FCT("exc read_direction_buttons fct");
  button_left.loop();

  if (button_left.isPressed()) {  
    mov = TURN_LEFT;
    DEBUG_PRINTLN_ACT("button left is pressed");
    if (mov != 0) {
      recorded_button[nr_comm] = mov;
      nr_comm++;
    }
    mov = 0;
  }
  button_forwards.loop();

  if (button_forwards.isPressed()) {
    mov = FORWARD;
    DEBUG_PRINTLN_ACT("button forward is pressed");
    if (mov != 0) {
      recorded_button[nr_comm] = mov;
      nr_comm++;
    }
    mov = 0;
  }
  button_right.loop();

  if (button_right.isPressed()) {
    mov = TURN_RIGHT;
    DEBUG_PRINTLN_ACT("button right is pressed");
    if (mov != 0) {
      recorded_button[nr_comm] = mov;
      nr_comm++;
    }
    mov = 0;
  }
  button_backwards.loop();

  if (button_backwards.isPressed()) {
    mov = BACKWARD;
    DEBUG_PRINTLN_ACT("button backwards is pressed");
    if (mov != 0) {
      recorded_button[nr_comm] = mov;
      nr_comm++;
    }
    mov = 0;
  }
} // read_cmd_buttons

void init(void)
{
  DEBUG_PRINTLN_FCT("exc init fct");
  setLed(0, 0, 255); // set blue

  if (button_command_count > 3) { // reset command button counter if command button is pressed more then one time
    button_command.resetCount();
  } // reset

  if (button_command_count == 1) {  // if command button is pressed once
    // Initialize state
    nr_comm = 0;                                         // start the command reading
    memset(recorded_button, 0, sizeof(recorded_button)); // initialize to zero the commands vector
    machine_state = READ_COMM_ST;
  }

  if (button_command_count == 0) {
    on_execute_comm_st = 0;
  }
}

void readComm(void)
{
  DEBUG_PRINTLN_FCT("exc readComm fct");
  setLed(255, 255, 0);               // yellow

  if (nr_comm < MAX_NR_COMMANDS) { // it only keeps the first max_nr_commands...
    read_direction_buttons();
  }

  // -- wait for the button_command_count = 2 and nr_commands != 0
  if (button_command_count == 2 and nr_comm != 0) {
    machine_state = START_EXEC_ST;
  }
}

void startExec(void)
{
  DEBUG_PRINTLN_FCT("exc startExec fct");
  setLed(0, 255, 0); // green
  button_forwards.loop();

  if (button_command_count > 2) {
    if (on_execute_comm_st == 1) {
      button_command.resetCount();
    }
    machine_state = INIT_ST;
  }

  if (button_forwards.isPressed()) {
    comm_index         = nr_comm;
    on_execute_comm_st = 1; // executed at least once ... needed???
    machine_state      = EXEC_ST;
  }
}

void exec(void)
{
  DEBUG_PRINTLN_FCT("exc exec fct");
  setLed(255, 51, 255);               // pink
  comm_index--;

  if (comm_index >= 0) { // avoid getting nonsense data
    int action = recorded_button[(nr_comm - 1) - comm_index];
    if (action == FORWARD) {
      machine_state = FORWARD_ST;
    } else if (action == BACKWARD) {
      machine_state = BACK_ST;
    } else if (action == TURN_LEFT) {
      machine_state = TURN_LEFT_ST;
    } else if (action == TURN_RIGHT) {
      machine_state = TURN_RIGHT_ST;
    }
  }

  if (comm_index < 0) {             // no more commands
    button_forwards.loop();

    if (button_forwards.isReleased()) { // wait till button releases state
      machine_state = START_EXEC_ST;
    }
  }
}

void turnRight(void)
{
  DEBUG_PRINTLN_FCT("exc turnRight fct");
  DEBUG_PRINTLN_ACT("turn right");
  if ((abs(encoder1_pos) < SETPOINT_TURN) &&
      (abs(encoder2_pos < SETPOINT_TURN)))
  {
    startTimer();

    int vel = kspeed * (speedL + val_outputL);
    int ver = kspeed * (speedR + val_outputR);
    MotorControl.motorForward(0, vel);
    MotorControl.motorReverse(1, ver);

    if (counterPID > 50) {
      portENTER_CRITICAL_ISR(&counterMux);
      counterPID = 0;
      portEXIT_CRITICAL_ISR(&counterMux);
      enc_readL = encoder1_pos;
      enc_readR = encoder2_pos;
      pidleft.Compute();
      pidright.Compute();
    }
  } else {
    stopTimer();
    time_now = millis();

    stop_next_state = EXEC_ST;
    machine_state   = STOP_ST;
  }
}

void turnLeft(void)
{
  DEBUG_PRINTLN_FCT("exc turnLeft fct");
  DEBUG_PRINTLN_ACT("turn left");
  if ((abs(encoder1_pos) < SETPOINT_TURN) &&
      (abs(encoder2_pos < SETPOINT_TURN)))
  {
    startTimer();
    int vel = kspeed * (speedL + val_outputL);
    int ver = kspeed * (speedR + val_outputR);
    MotorControl.motorReverse(0, vel);
    MotorControl.motorForward(1, ver);

    if (counterPID > 50) {
      portENTER_CRITICAL_ISR(&counterMux);
      counterPID = 0;
      portEXIT_CRITICAL_ISR(&counterMux);
      enc_readL = encoder1_pos;
      enc_readR = encoder2_pos;
      pidleft.Compute();
      pidright.Compute();
    }
  } else {
    stopTimer();
    time_now = millis();

    stop_next_state = EXEC_ST;
    machine_state   = STOP_ST;
  }
}

void forward(void)
{
  DEBUG_PRINTLN_FCT("exc forward fct");
  DEBUG_PRINTLN_ACT("drive forward");
  if ((abs(encoder1_pos) < SETPOINT_RUN) &&
      (abs(encoder2_pos) < SETPOINT_RUN)) {
    startTimer();

    int vel = kspeed * (speedL + val_outputL);
    int ver = kspeed * (speedR + val_outputR);
    MotorControl.motorForward(0, vel);
    MotorControl.motorForward(1, ver);

    if (counterPID > freq) {
      portENTER_CRITICAL_ISR(&counterMux);
      counterPID = 0;
      portEXIT_CRITICAL_ISR(&counterMux);
      enc_readL = encoder1_pos;
      enc_readR = encoder2_pos;
      pidleft.Compute();
      pidright.Compute();
    }
  } else {
    stopTimer();
    time_now = millis();

    stop_next_state = EXEC_ST;
    machine_state   = STOP_ST;
  }
}

void back(void)
{
  DEBUG_PRINTLN_FCT("exc back fct");
  DEBUG_PRINTLN_ACT("drive back");
  if ((abs(encoder1_pos) < SETPOINT_RUN) &&
      (abs(encoder2_pos) < SETPOINT_RUN)) {
    startTimer();

    int vel = kspeed * (speedL + val_outputL);
    int ver = kspeed * (speedR + val_outputR);
    MotorControl.motorReverse(0, vel);
    MotorControl.motorReverse(1, ver);

    if (counterPID > freq) {
      portENTER_CRITICAL_ISR(&counterMux);
      counterPID = 0;
      portEXIT_CRITICAL_ISR(&counterMux);
      enc_readL = encoder1_pos;
      enc_readR = encoder2_pos;
      pidleft.Compute();
      pidright.Compute();

      /*DEBUG_PRINT(enc_readL);
       * DEBUG_PRINT(",");
       * DEBUG_PRINT(val_outputL);
       * DEBUG_PRINT(",");
       * DEBUG_PRINT(vel);
       * DEBUG_PRINT(",");
       * DEBUG_PRINT(enc_readR);
       * DEBUG_PRINT(",");
       * DEBUG_PRINT(val_outputR);
       * DEBUG_PRINT(",");
       * DEBUG_PRINTLN(ver);*/
    }
  } else {
    stopTimer();
    time_now = millis();

    stop_next_state = EXEC_ST;
    machine_state   = STOP_ST;
  }
}

void stop(void)
{
  DEBUG_PRINTLN_FCT("exc stop fct");
  DEBUG_PRINTLN_ACT("stop exec");
  setLed(255, 0, 0);               // red
  //put code to stop the motors here
  MotorControl.motorsStop();

  if (millis() >= time_now + STOP_DELAY) {
    machine_state = stop_next_state;
  }
}

void fsm(void)
{
  DEBUG_PRINTLN_FCT("exc fsm fct");

  button_command.loop(); // loop() for button_command
  button_command_count = button_command.getCount(); // get count of how often command button was pressed
  
  DEBUG_PRINT_VAR("button_command_count: ");
  DEBUG_PRINTLN_VAR(button_command_count);
  
  DEBUG_PRINT_VAR("nr_comm: ");
  DEBUG_PRINTLN_VAR(nr_comm);
  
  switch (machine_state) {
  case INIT_ST: // execute init state
    last_machine_state = machine_state;
    init(); 
    break;

  case READ_COMM_ST: // execute read comment state
    last_machine_state = machine_state;
    readComm();
    break;

  case START_EXEC_ST: // execute start execution state
    last_machine_state = machine_state;
    startExec();
    break;

  case EXEC_ST: // execute execute state
    last_machine_state = machine_state;
    exec();
    break;

  case FORWARD_ST: // execute forward state
    last_machine_state = machine_state;
    forward();
    break;

  case BACK_ST: // execute forward state
    last_machine_state = machine_state;
    back();
    break;

  case TURN_RIGHT_ST: // execute turn right state
    last_machine_state = machine_state;
    turnRight();
    break;

  case TURN_LEFT_ST: // execute turn left state
    last_machine_state = machine_state;
    turnLeft();
    break;

  case STOP_ST:  //execute stop state
    //last_machine_state = machine_state;
    stop();
    break;
  
  
  /* // ?????is this code really needed, what is the purpose of the void state?????
  case VOID_ST: // execute void state 
    last_machine_state = machine_state;
    // put code here
    break;
  */
  }
} // fsm

void show_state(void){ // show state
  if (machine_state != last_machine_state){
    DEBUG_PRINTLN_STATE(machine_state);
  }
} // show state

void set_stop_state_interrupt(void){ // function that is called when stop button is pressed to change machine state
  machine_state = STOP_ST;
} // set_stop_state_interrupt

void setup()
{
  Serial.begin(9600); // setup serial monitor
  DEBUG_PRINTLN_FCT("exc microcontroller setup fct");
  
  //setup interrupts // very unstable button input with the interrupts
 // attachInterrupt(digitalPinToInterrupt(button_stop), set_stop_state_interrupt, RISING); //attach interrupt
  //sei();                                                               //activate global interrupts

  // setup ez buttons debounce time to 50 milliseconds
  button_command.setDebounceTime(button_debounce_time);
  button_command.setCountMode(COUNT_FALLING);
  button_left.setDebounceTime(button_debounce_time);
  button_forwards.setDebounceTime(button_debounce_time);
  button_right.setDebounceTime(button_debounce_time);
  button_backwards.setDebounceTime(button_debounce_time);

  // Neopixel setup
  pixels.begin(); // INITIALIZE NeoPixel strip object
  pixels.clear(); // Set all pixel colors to 'off'
  
  // Motor setup
  // Encoders Pins
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);

  // Motor Pins
  MotorControl.attachMotors(25, 26, 32, 33); //ROBOT José trocar 25 por 27

  machine_state = INIT_ST; // set machine to init state
}

void loop()
{
  DEBUG_PRINTLN_FCT("exc microcontoller loop fct");
  fsm(); // execute finite state machine
  show_state();
}