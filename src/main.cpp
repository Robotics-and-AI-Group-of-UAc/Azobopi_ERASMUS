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

void showBitmap(unsigned char bitmap_oled[]) {
  display.clearDisplay();
 
  display.drawBitmap(0, 0, bitmap_oled, bitmap_height, bitmap_width, WHITE);
  display.display();
}

void setLed(int r, int g, int b) // function to set NEO Pixel LED
{
  DEBUG_PRINTLN_FCT("exc setLED fct"); // debug print
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.setPixelColor(0, pixels.Color(r, g, b)); // set pixel color
  pixels.show(); // show pixel color
}

void read_direction_buttons() // function to read direction buttons
{
  DEBUG_PRINTLN_FCT("exc read_direction_buttons fct"); //debug print
  button_left.loop(); //read left button

  if (button_left.isPressed()) {   //check if left button is pressed
    mov = TURN_LEFT; //set mov to turn left
    DEBUG_PRINTLN_ACT("button left is pressed"); // debug print
    tone(PIN_SPEAKER,NOTE_C6,100); // play single note for user feedback
    if (mov != 0) { // check if mov is not 0
      recorded_button[nr_comm] = mov; //write mov to array of recorded buttons(movements)
      nr_comm++; //add 1 to the total number of movements
    }
    mov = 0; //reset mov to 0 
  }
  button_forwards.loop(); //read forwards button

  if (button_forwards.isPressed()) { //check if forwards button is pressed
    mov = FORWARD; //set mov to forward
    DEBUG_PRINTLN_ACT("button forward is pressed"); //debug print
    tone(PIN_SPEAKER,NOTE_C6,100); // play single note for user feedback
    if (mov != 0) { // check if mov is not 0
      recorded_button[nr_comm] = mov; //write mov to array of recorded buttons(movements)
      nr_comm++; //add 1 to the total number of movements
    }
    mov = 0; //reset mov to 0 
  }
  button_right.loop(); //read right button

  if (button_right.isPressed()) { //check if right button is pressed
    mov = TURN_RIGHT; //set mov to turn right
    DEBUG_PRINTLN_ACT("button right is pressed"); //debug print
    tone(PIN_SPEAKER,NOTE_C6,100); // play single note for user feedback
    if (mov != 0) { // check if mov is not 0
      recorded_button[nr_comm] = mov; //write mov to array of recorded buttons(movements)
      nr_comm++; //add 1 to the total number of movements
    }
    mov = 0; //reset mov to 0 
  }
  button_backwards.loop(); //read backwards button

  if (button_backwards.isPressed()) { //check if backwards button is pressed
    mov = BACKWARD; //set mov to backward
    DEBUG_PRINTLN_ACT("button backwards is pressed"); //debug print
    tone(PIN_SPEAKER,NOTE_C6,100); // play single note for user feedback
    if (mov != 0) { // check if mov is not 0
      recorded_button[nr_comm] = mov; //write mov to array of recorded buttons(movements)
      nr_comm++; //add 1 to the total number of movements
    }
    mov = 0; //reset mov to 0 
  }
} // read_cmd_buttons

void init(void) // function to init the the robo
{
  DEBUG_PRINTLN_FCT("exc init fct");
  setLed(0, 0, 255); // set LED to blue

  if (button_command_count > 3) { // reset command button counter if command button is pressed more then one time
    button_command.resetCount();
  } // reset

  if (button_command_count == 1) {  // if command button is pressed once
    // Initialize state
    nr_comm = 0;                                         // start the command reading
    memset(recorded_button, 0, sizeof(recorded_button)); // initialize to zero the commands vector
    machine_state = READ_COMM_ST; // set machine state to read comm
  }

  if (button_command_count == 0) {
    on_execute_comm_st = 0;
  }
}

void readComm(void) // funciton to read movement commands
{
  DEBUG_PRINTLN_FCT("exc readComm fct"); // debug print
  setLed(255, 255, 0);               // set LED to yellow

  if (nr_comm < MAX_NR_COMMANDS) { // it only keeps the first max_nr_commands...
    read_direction_buttons(); // call read dir func
  }

  // -- wait for the button_command_count = 2 and nr_commands != 0
  if (button_command_count == 2 and nr_comm != 0) {
    machine_state = START_EXEC_ST;
  }
}

void startExec(void) // function to start execution of commands
{
  DEBUG_PRINTLN_FCT("exc startExec fct"); // debug print
  setLed(0, 255, 0); // set LED to green
  showBitmap(image_data_EYES_MIDDLE);
  button_forwards.loop(); 

  if (button_command_count > 2) {
    if (on_execute_comm_st == 1) {
      button_command.resetCount();
    }
    machine_state = INIT_ST; // set machine state
  }

  if (button_forwards.isPressed()) { // check if button forward is pressed
    comm_index         = nr_comm; // set comm_index to number of commands
    on_execute_comm_st = 1; // executed at least once ... needed???
    machine_state      = EXEC_ST; // set machine state to exectute_state
  }
}

void exec(void) // function to execut the movement commands
{
  DEBUG_PRINTLN_FCT("exc exec fct"); // debug print
  comm_index--; // comm index -1

  if (comm_index >= 0) { // avoid getting nonsense data
    setLed(255, 51, 255);               // set led to pink
    int action = recorded_button[(nr_comm - 1) - comm_index];  // get current action
    if (action == FORWARD) {  //set state to execute movement action
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
    finish_melody();
    button_forwards.loop();         

    if (button_forwards.isReleased()) { // wait till button releases state
      machine_state = START_EXEC_ST;
    }
  }
}

void turnRight(void) // function to turn right
{
  DEBUG_PRINTLN_FCT("exc turnRight fct");
  DEBUG_PRINTLN_ACT("turn right");
  showBitmap(image_data_EYES_LEFT);
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

void turnLeft(void) // function to turn left
{
  DEBUG_PRINTLN_FCT("exc turnLeft fct");
  DEBUG_PRINTLN_ACT("turn left");
  showBitmap(image_data_EYES_RIGHT);
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

void forward(void) // function to drive forwards
{
  DEBUG_PRINTLN_FCT("exc forward fct");
  DEBUG_PRINTLN_ACT("drive forward");
  showBitmap(image_data_EYES_DOWN);
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
    }
  } else {
    stopTimer();
    time_now = millis();

    stop_next_state = EXEC_ST;
    machine_state   = STOP_ST;
  }
}

void back(void) // function to drive backwards
{
  DEBUG_PRINTLN_FCT("exc back fct");
  DEBUG_PRINTLN_ACT("drive back");
  showBitmap(image_data_EYES_UP);
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

void stop(void) // function that is called between movemnts
{
  DEBUG_PRINTLN_FCT("exc stop fct"); // debug print
  DEBUG_PRINTLN_ACT("stop"); // debug print
  MotorControl.motorsStop(); // stop motors

  if (millis() >= time_now + STOP_DELAY) {
    machine_state = stop_next_state; 
  }
}

void stop_exec(void) // function to stop execution of current run
{
  DEBUG_PRINTLN_FCT("exc stop_exec fct"); // debug print
  DEBUG_PRINTLN_ACT("stop exec"); // debug print
    
  setLed(255, 0, 0); // set led to red

  MotorControl.motorsStop(); // stop motors
  machine_state = EXEC_ST; // set machine state to init
  comm_index = 0; // set comm index to 0 to restart at command 0 on the next run

  showBitmap(image_data_DISTRESSED_EYES);
  stop_melody();
}

void set_stop_state(void){ // function that is called when stop button is pressed to change machine state
  
  if ((machine_state != INIT_ST) && (machine_state != START_EXEC_ST) && (machine_state != READ_COMM_ST)){
    DEBUG_PRINTLN_FCT("exc set_stop_state fct"); // debug print
    button_stop.loop(); // loop() for button_stop
    button_stop_count = button_stop.getCount(); // get count of how often command button was pressed
    
    if (button_stop_count >= 1) // check if stop button is pressed more then 0 times
    {
      machine_state = STOP_EXEC_ST; // set machine state
      button_stop_count = 0; // reset button stop counter
      button_stop.resetCount(); // reset button stop
    }
  } 
} // set_stop_state

void fsm(void) // finite state machine
{
  DEBUG_PRINTLN_FCT("exc fsm fct");
  
  set_stop_state(); // call fct to check if stop button was pressed

  button_command.loop(); // loop() for button_command
  button_command_count = button_command.getCount(); // get count of how often command button was pressed
  
  DEBUG_PRINT_VAR("button_command_count: "); // debug print
  DEBUG_PRINTLN_VAR(button_command_count); // debug print
  
  DEBUG_PRINT_VAR("nr_comm: "); // debug print
  DEBUG_PRINTLN_VAR(nr_comm); // debug print
  
  switch (machine_state) { // switch to current machine state
  case INIT_ST: // execute init state
    last_machine_state = machine_state; // set last machine state
    init();  //execute func
    break;

  case READ_COMM_ST: // execute read comment state
    last_machine_state = machine_state; // set last machine state
    readComm(); //execute func
    break;

  case START_EXEC_ST: // execute start execution state
    last_machine_state = machine_state; // set last machine state
    startExec(); //execute func
    break;

  case EXEC_ST: // execute execute state
    last_machine_state = machine_state; // set last machine state
    exec(); //execute func
    break;

  case FORWARD_ST: // execute forward state
    last_machine_state = machine_state; // set last machine state
    forward(); //execute func
    break;

  case BACK_ST: // execute forward state
    last_machine_state = machine_state; // set last machine state
    back(); //execute func
    break;

  case TURN_RIGHT_ST: // execute turn right state
    last_machine_state = machine_state; // set last machine state
    turnRight(); //execute func
    break;

  case TURN_LEFT_ST: // execute turn left state
    last_machine_state = machine_state; // set last machine state
    turnLeft(); //execute func
    break;

  case STOP_ST:  //execute stop state
    last_machine_state = machine_state; // set last machine state
    stop(); //execute func
    break;
  
  case STOP_EXEC_ST: // execute STOP_EXEC state 
    last_machine_state = machine_state; // set last machine state
    stop_exec(); //execute func
    break;

  case TUNE_ST: // execute tune state 
    last_machine_state = machine_state; // set last machine state
    //tune(); // function to be written
    break;

  case VOID_ST: // execute void state 
    // put code here
    break;
  }
} // fsm

void show_state(void){ // show state function is used for debuging
  if (machine_state != last_machine_state){ // show new state if the state has changed
    DEBUG_PRINTLN_STATE(machine_state); // print current state
  }
} // show state

void setup() // microcontroller setup runs once
{
  Serial.begin(9600); // setup serial monitor
  DEBUG_PRINTLN_FCT("exc microcontroller setup fct"); // debug print
  
  //display setup 
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  display.clearDisplay(); // Clear the buffer
  showBitmap(bitmap_uac_logo); // show uac logo
  delay(500); // show UAC logo for 0.5 sec
  showBitmap(image_data_EYES_MIDDLE);

  //play startup melody
  startup_melody();
  
  // setup ez buttons debounce time
  button_command.setDebounceTime(button_debounce_time); 
  button_command.setCountMode(COUNT_FALLING);
  button_left.setDebounceTime(button_debounce_time);
  button_forwards.setDebounceTime(button_debounce_time);
  button_right.setDebounceTime(button_debounce_time);
  button_backwards.setDebounceTime(button_debounce_time);
  button_stop.setDebounceTime(button_debounce_time);
  button_stop.setCountMode(COUNT_RISING);

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

void loop() // microcontroller loop function 
{
  DEBUG_PRINTLN_FCT("exc microcontoller loop fct"); // debug print
  fsm(); // execute finite state machine
  show_state(); // execute show state fct for debugging
}