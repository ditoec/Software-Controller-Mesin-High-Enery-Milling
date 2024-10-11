// Arduino HEM Control, Nano Center Indonesia @2024
// Variable Table
// holdingRegister ----
// holdingRegisters[0] -> Total_Milling_Time_Minutes
// holdingRegisters[1] -> Total_Milling_Time_Seconds
// holdingRegisters[2] -> Idle_Time_Minutes
// holdingRegisters[3] -> Idle_Time_Seconds
// holdingRegisters[4] -> Grinding_Time_Minutes
// holdingRegisters[5] -> Grinding_Time_Seconds
// holdingRegisters[6] -> Maximum_Temperature
// holdingRegisters[7] -> CurrentTotalTime
// holdingRegisters[8] and holdingRegister[9] -> MinMaxTotalTime (holdingRegisters[8] set to 0)
// holdingRegisters[10] -> Elapsed_Minutes
// holdingRegisters[11] -> Elapsed_Seconds
// holdingRegisters[12] -> Remaining_Minutes
// holdingRegisters[13] -> Remaining_Seconds
// holdingRegisters[14] -> Current_Temp
// holdingRegisters[15] -> Cooldown_Differential
// holdingRegisters[16] and [17] -> MinMaxTempScale
// holdingRegisters[18] and [19] -> TempFlash

// coilsRegisters ---
// coilsRegisters[0] -> PLC_System_Check
// coilsRegisters[1] -> Temperature_Sensor
// coilsRegisters[2] -> Door_Sensor
// coilsRegisters[3] -> Everything_OK_Button
// coilsRegisters[4] -> Action_Started
// coilsRegisters[5] -> Milling_Done
// coilsRegisters[6] -> Milling_Done_Pressed
// coilsRegisters[7] -> Paused
// coilsRegisters[8] -> Emergency_Button_Pressed
// coilsRegisters[9] -> Door_Sensor_Open
// coilsRegisters[10] -> Temp_Exceed

#include <ModbusRTUSlave.h>
#include <Adafruit_MLX90614.h>

ModbusRTUSlave modbus_rtu_slave(Serial);
// Temperature sensor
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

const uint8_t slaveID = 1;
const uint32_t baud = 9600;

uint16_t holdingRegisters[20] = {0};
bool coilsRegisters[20] = {false};
uint16_t sensor = 0;

// Variables for storing time in seconds
unsigned long totalMillingTimeSeconds = 0;
unsigned long idleTimeSeconds = 0;
unsigned long grindingTimeSeconds = 0;

// Variables for tracking elapsed time when coilsRegisters[4] turns true
unsigned long startTime = 0;
unsigned long elapsedTime = 0;
unsigned long remainingTime = 0;
unsigned long pauseTime = 0;  
unsigned long pauseDuration = 0;  

bool timerStarted = false;
bool isPaused = false;
bool isGrinding = false;
unsigned long phaseStartTime = 0;
bool wasGrindingBeforePause = false; 
bool tempPauseActive = false;
bool tempExceeded = false;
bool millingDone = false;
bool doorSensorOpen = false;
bool emergencyButtonPressed = false;

void setup() {
  // Set default value for holdingRegisters[6]
  holdingRegisters[6] = 100;
  holdingRegisters[16] = 20;
  holdingRegisters[18] = 20;
  holdingRegisters[15] = 10;

  // Initialize the temperature sensor
  mlx.begin();

  // Emergency Button Pin
  pinMode(7, INPUT);

  // Relay Pin
  pinMode(5, OUTPUT);

  // Door Sensor PIN
  pinMode(9, INPUT_PULLUP);

  // Configure holding registers and coils
  modbus_rtu_slave.configureHoldingRegisters(holdingRegisters, 20);
  modbus_rtu_slave.configureCoils(coilsRegisters, 20);
  modbus_rtu_slave.begin(slaveID, baud, SERIAL_8N1);
}

void loop() {
  // Read the temperature
  float tempC = mlx.readObjectTempC();
  holdingRegisters[14] = (uint16_t)tempC;
  holdingRegisters[17] = holdingRegisters[6];
  holdingRegisters[19] = holdingRegisters[6] - holdingRegisters[15];

  if (tempC > 0) {
    coilsRegisters[1] = true;  
  } else {
    coilsRegisters[1] = false; 
  }

  // Check if temperature exceeds maximum temperature
  if (tempC > holdingRegisters[6]) {
    tempPauseActive = true;
    tempExceeded = true;
  }

  // Resume milling when temperature drops by 10 degrees below maximum
  if (tempPauseActive && tempC <= (holdingRegisters[6] - holdingRegisters[15])) {
    tempPauseActive = false;
    tempExceeded = false;
  }

  // Calculate totalMillingTimeSeconds, idleTimeSeconds, and grindingTimeSeconds
  totalMillingTimeSeconds = (holdingRegisters[0] * 60) + holdingRegisters[1];
  idleTimeSeconds = (holdingRegisters[2] * 60) + holdingRegisters[3];
  grindingTimeSeconds = (holdingRegisters[4] * 60) + holdingRegisters[5];

  if (totalMillingTimeSeconds != 0) {
    coilsRegisters[0] = true;
  }

  if (coilsRegisters[0] && coilsRegisters[1] && coilsRegisters[2]) {
    coilsRegisters[3] = true;
  }

  // Check for total milling time and phase transition logic
  if (coilsRegisters[4] == true && !isPaused) {
    if (!timerStarted) {
      startTime = millis();
      timerStarted = true;
      pauseDuration = 0;  
      phaseStartTime = startTime; 
      isGrinding = true;  
    }

    // Calculate elapsed time and remaining time based on total time
    elapsedTime = (millis() - startTime - pauseDuration) / 1000;
    

    // Ensure the elapsed time and remaining time are clamped to the totalMillingTimeSeconds
    if (elapsedTime >= totalMillingTimeSeconds) {
      elapsedTime = totalMillingTimeSeconds;
      if (digitalRead(9) == HIGH) {  // Only mark milling done if the door is closed
        millingDone = true;  // Set milling done
      }
    }
    remainingTime = totalMillingTimeSeconds - elapsedTime;

    if (remainingTime > 0) {
      unsigned long phaseElapsedTime = (millis() - phaseStartTime) / 1000;

      if (isGrinding) {
        // Send relay high during the grinding phase
        digitalWrite(5, HIGH);

        // Grinding phase: check if grindingTimeSeconds has passed
        if (phaseElapsedTime >= grindingTimeSeconds) {
          // Switch to idle phase, reset phaseStartTime for idle phase
          isGrinding = false;
          phaseStartTime = millis();
        }
      } else {
        // Send relay low during the idle phase
        digitalWrite(5, LOW);

        // Idle phase: check if idleTimeSeconds has passed
        if (phaseElapsedTime >= idleTimeSeconds) {
          // Switch back to grinding phase, reset phaseStartTime for grinding phase
          isGrinding = true;
          phaseStartTime = millis();
        }
      }
    }

    // Update holding registers for elapsed and remaining times
    holdingRegisters[10] = elapsedTime / 60;  // Elapsed time minutes
    holdingRegisters[11] = elapsedTime % 60;  // Elapsed time seconds
    holdingRegisters[12] = remainingTime / 60;  // Remaining time minutes
    holdingRegisters[13] = remainingTime % 60;  // Remaining time seconds

    // Update progress bar registers
    holdingRegisters[7] = elapsedTime;
    holdingRegisters[8] = 0;
    holdingRegisters[9] = totalMillingTimeSeconds;
  }

  // Logic for door sensor
  if (digitalRead(9) == LOW) {
    // Door is open, pause the system
    doorSensorOpen = true;  
    coilsRegisters[2] = false;  
  } else if (digitalRead(9) == HIGH) {
    // Door is closed, resume the system
    doorSensorOpen = false;
    coilsRegisters[2] = true;
  }

  // Handle pause when coilsRegisters[7] is true or temperature exceeds max
  if (coilsRegisters[7] == true || doorSensorOpen || tempPauseActive) {
    if (!isPaused) {
      pauseTime = millis();
      isPaused = true;
      wasGrindingBeforePause = isGrinding;
      isGrinding = false;
      digitalWrite(5, LOW);  // Stop relay during pause
    }
  } else if (isPaused && !tempPauseActive && !coilsRegisters[7] && !doorSensorOpen) {
    pauseDuration += millis() - pauseTime;
    isPaused = false;
    isGrinding = wasGrindingBeforePause;

    // Ensure relay state reflects the correct phase after resuming
    if (isGrinding) {
      digitalWrite(5, HIGH);  // Resume grinding phase
    } else {
      digitalWrite(5, LOW);  // Stay in idle phase
    }

    // Reset the phase start time based on the elapsed phase time before the pause
    phaseStartTime = millis() - (pauseTime - phaseStartTime);
  }

  // Reset logic when coilsRegisters[6] is true (Reset operation)
  if (coilsRegisters[6] == true) {
    timerStarted = false;
    elapsedTime = 0;
    remainingTime = 0;
    pauseDuration = 0;
    isPaused = false;
    tempPauseActive = false;
    holdingRegisters[10] = 0;
    holdingRegisters[11] = 0;
    holdingRegisters[12] = 0;
    holdingRegisters[13] = 0;
    coilsRegisters[4] = false;
    millingDone = false;
    coilsRegisters[7] = false;
    coilsRegisters[6] = false;
    coilsRegisters[3] = false;
    isGrinding = false;
    phaseStartTime = 0;
    digitalWrite(5, LOW);
    tempExceeded = false;
  }

  // Emergency Button Logic
  if (digitalRead(7) == HIGH) {
    emergencyButtonPressed = true;
    timerStarted = false;
    elapsedTime = 0;
    remainingTime = 0;
    pauseDuration = 0;
    isPaused = false;
    tempPauseActive = false;
    holdingRegisters[10] = 0;
    holdingRegisters[11] = 0;
    holdingRegisters[12] = 0;
    holdingRegisters[13] = 0;
    coilsRegisters[4] = false;
    millingDone = false;
    coilsRegisters[7] = false;
    coilsRegisters[6] = false;
    coilsRegisters[3] = false;
    isGrinding = false;
    phaseStartTime = 0;
    digitalWrite(5, LOW);
    coilsRegisters[10] = false;
  } else {
    emergencyButtonPressed = false;
  }

  // For managing popups priority
  managePopups();

  // Poll the Modbus RTU slave
  modbus_rtu_slave.poll();
}

void managePopups() {
  // Reset all popup coils except coilsRegisters[7] (Paused)
  coilsRegisters[5] = false;  // Milling_Done
  coilsRegisters[8] = false;  // Emergency_Button_Pressed
  coilsRegisters[9] = false;  // Door_Sensor_Open
  coilsRegisters[10] = false; // Temp_Exceed
  // Do not reset coilsRegisters[7]; it represents Paused state

  // Set the popup based on priority
  if (emergencyButtonPressed) {
    coilsRegisters[8] = true; // Highest priority (Emergency)
  } else if (doorSensorOpen) {
    coilsRegisters[9] = true;
  } else if (millingDone) {
    coilsRegisters[5] = true;
  } else if (tempExceeded) {
    coilsRegisters[10] = true;
  } else if (coilsRegisters[7] == true) {
    // Paused popup remains active
    coilsRegisters[7] == true;
  }
}

