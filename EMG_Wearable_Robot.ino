  #include <AccelStepper.h>
  #include <Wire.h>
  #include <FspTimer.h>

  // ================= STEPPER =================
  #define STEP_PIN 3
  #define DIR_PIN 2
  #define M0_PIN 7
  #define M1_PIN 6
  #define M2_PIN 5

  AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

  const int stepsPerRevolution = 200;
  int microstepSetting = 32;
  long stepsPerFullTurn = stepsPerRevolution * microstepSetting;

  // ================= AS5600 =================
  #define AS5600_ADDR 0x36

  unsigned long lastEncoderMillis = 0;
  const unsigned long encoderInterval = 10; // 100 Hz

  float degAngle = 0.0;
  float startAngle = 0.0;
  float correctedAngle = 0.0;
  long numberOfTurns = 0;
  int previousQuadrant = 0;

  // ================= EMG READ DATA =================
  #define EMG_RATE 2000
  #define EMG_BUFFER_SIZE 256

  FspTimer emgTimer;

  volatile uint16_t emgBuffer[EMG_BUFFER_SIZE];
  volatile uint16_t emgWriteIndex = 0;
  volatile uint16_t emgReadIndex  = 0;
  volatile bool emgOverflow = false;

  // ================= EMG CONTROL =================
  float emgEnvelope = 0.0;
  long targetPosition = 0;

  // ================= TIMER CALLBACK =================
  void emgTimerCallback(timer_callback_args_t __attribute__((unused)) *p_args)
  {
    uint16_t sample = analogRead(A0);

    uint16_t nextIndex = (emgWriteIndex + 1) % EMG_BUFFER_SIZE;

    if (nextIndex != emgReadIndex) {
      emgBuffer[emgWriteIndex] = sample;
      emgWriteIndex = nextIndex;
    } else {
      emgOverflow = true; // buffer penuh
    }
  }

  // ================= TIMER INIT =================
  bool beginEMGTimer(float rate)
  {
    uint8_t timer_type = GPT_TIMER;
    int8_t tindex = FspTimer::get_available_timer(timer_type);

    if (tindex < 0) return false;

    if (!emgTimer.begin(TIMER_MODE_PERIODIC,
                        timer_type,
                        tindex,
                        rate,
                        0.0f,
                        emgTimerCallback))
      return false;

    if (!emgTimer.setup_overflow_irq()) return false;
    if (!emgTimer.open()) return false;
    if (!emgTimer.start()) return false;

    return true;
  }

  // ================= SETUP =================
  void setup()
  { 
    Serial.begin(2000000);
    Wire.begin();

    pinMode(M0_PIN, OUTPUT);
    pinMode(M1_PIN, OUTPUT);
    pinMode(M2_PIN, OUTPUT);

    digitalWrite(M0_PIN, HIGH);
    digitalWrite(M1_PIN, HIGH);
    digitalWrite(M2_PIN, HIGH);

    stepper.setMaxSpeed(5000);
    stepper.setAcceleration(4000);

    delay(500);

    ReadRawAngle();
    startAngle = degAngle;
    previousQuadrant = getQuadrant(0);

    // START DETERMINISTIC EMG TIMER
    if (!beginEMGTimer(EMG_RATE)) {
      Serial.println("EMG Timer Failed");
      while (1);
    }
  }

  // ================= LOOP =================
  void loop()
  {
    // ===== EMG TRANSMISSION FROM BUFFER =====
    while (emgReadIndex != emgWriteIndex)
    {
      uint16_t sample = emgBuffer[emgReadIndex];
      emgReadIndex = (emgReadIndex + 1) % EMG_BUFFER_SIZE;

      Serial.write(0xAA);
      Serial.write((uint8_t*)&sample, 2);
    }

    // ===== RECEIVE ENVELOPE =====
    while (Serial.available() >= 5)
    {
      if (Serial.peek() == 0xCC)
      {
        Serial.read();
        uint8_t buffer[4];
        Serial.readBytes(buffer, 4);
        memcpy(&emgEnvelope, buffer, 4);
      }
      else
      {
        Serial.read();
      }
    }

    // ===== MOTOR CONTROL =====
    float envAbs = abs(emgEnvelope);
    const float ENV_THRESHOLD = 0.06;  //Treshold Kondisi Normal
    // const float ENV_THRESHOLD = 0.05;  //Treshold Isometric Contraction

    if (envAbs < ENV_THRESHOLD) {
      targetPosition = 0;
    } else {
      float envActive =
        (envAbs - ENV_THRESHOLD) / (1.0 - ENV_THRESHOLD);

      targetPosition =
        (long)(envActive * stepsPerFullTurn * 5);
    }

    stepper.moveTo(targetPosition);
    stepper.run();

    // ===== ENCODER 100 Hz =====
    if (millis() - lastEncoderMillis >= encoderInterval)
    {
      lastEncoderMillis += encoderInterval;

      ReadRawAngle();

      correctedAngle =
        normalizeAngle(degAngle - startAngle);

      int quadrant =
        getQuadrant(correctedAngle);

      if (previousQuadrant == 4 && quadrant == 1)
        numberOfTurns++;

      if (previousQuadrant == 1 && quadrant == 4)
        numberOfTurns--;

      float continuousAngle =
        numberOfTurns * 360.0 + correctedAngle;

      previousQuadrant = quadrant;

      Serial.write(0xBB);
      Serial.write((uint8_t*)&continuousAngle, 4);
    }
  }

  // ================= AS5600 =================
  void ReadRawAngle()
  {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0D);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDR, 1);
    int lowbyte = Wire.read();

    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0C);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDR, 1);
    int highbyte = Wire.read();

    int rawAngle = (highbyte << 8) | lowbyte;
    degAngle = rawAngle * 0.087890625f;
  }

  float normalizeAngle(float a)
  {
    while (a >= 360) a -= 360;
    while (a < 0) a += 360;
    return a;
  }

  int getQuadrant(float angle)
  {
    if (angle < 90) return 1;
    if (angle < 180) return 2;
    if (angle < 270) return 3;
    return 4;
  }
