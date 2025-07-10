/*
 * J1850 PWM Data Reader for Macchina M2 (Arduino Due)
 * DATA ONLY VERSION - No Statistics or Debug Output
 * 
 * Version: 13.3 - Fixed EOF Detection
 * Date: 2025
 */

// ============================================================================
// SAM3X COMPATIBILITY
// ============================================================================
// #define Serial SerialUSB


#define PS_J1850_9141   49
#define J1850_PWM_VPW   50
// ============================================================================
// J1850 PWM TIMING CONSTANTS (Based on SAE J1850 Standard Table 3)
// CRITICAL: All timing is based on RISING EDGES (passive to active transitions)

// Bit timing - measured from rising edge to rising edge
#define BIT_TIME_MIN_US   22      // Tp3: Bit time (rising edge to rising edge) ≥22μs, ≤27μs
#define BIT_TIME_MAX_US   27

// Active pulse widths - measured from rising edge to falling edge
#define BIT_1_ACTIVE_MIN_US    6   // Tp1: Active phase "1" ≥6μs, ≤11μs
#define BIT_1_ACTIVE_MAX_US   11
#define BIT_0_ACTIVE_MIN_US   14   // Tp2: Active phase "0" ≥14μs, ≤19μs  
#define BIT_0_ACTIVE_MAX_US   19

// SOF/EOD timing
#define SOF_EOD_MIN_US    46      // Tp4: SOF/EOD total time ≥46μs, ≤63μs
#define SOF_EOD_MAX_US    63

// EOF and other timing
// Note: Tp4 may refer to total SOF/EOD sequence time, not active pulse width
// Current implementation treats this as active pulse width - verify against actual signals
#define EOF_MIN_US        70      // Tp5: EOF time ≥70μs
#define EOF_MAX_US        100     // Extended for real-world tolerance
#define SOF_ACTIVE_MIN_US      30  // Tp7: Active SOF ≥30μs, ≤35μs
#define SOF_ACTIVE_MAX_US      35

// ============================================================================
#define J1850_PWM_RX               51

// ============================================================================
// FRAME STRUCTURE
// ============================================================================
#define MIN_VALID_BITS     24      // Minimum for a valid frame
#define MAX_VALID_BITS     96      // 12 bytes * 8 bits = 96 data bits max
#define MAX_BITS          101      // Maximum frame length SOF to EOF inclusive
#define MAX_FRAMES         20
#define MAX_BYTES          (MAX_BITS / 8 + 1)

struct Frame {
  uint8_t data[MAX_BYTES];
  uint8_t bitLength;
  uint32_t timestamp;
  bool hasValidSOF;
  bool hasValidEOF;
};

volatile Frame frameQueue[MAX_FRAMES];
volatile uint8_t queueHead = 0;
volatile uint8_t queueTail = 0;

volatile uint8_t bitBuffer[MAX_BITS];
volatile uint8_t bitIndex = 0;
volatile uint32_t lastTime = 0;
volatile uint32_t lastRisingEdge = 0;
volatile uint32_t lastFallingEdge = 0;
volatile uint32_t messageStartTime = 0;
volatile bool lastState = false;
volatile bool collectingMessage = false;
volatile bool eofDetected = false;

unsigned long messagesShown = 0;
uint32_t lastActivityTime = 0;



#define TC           TC0
#define CHANNEL      0
#define IRQ          TC0_IRQn
#define ID_TC        ID_TC0

volatile bool timerFired = false;
volatile uint32_t timerStartUs = 0;
volatile uint32_t timerEndUs = 0;


void TC0_Handler() {
  Tc *tc = TC;
  tc->TC_CHANNEL[CHANNEL].TC_SR; // Đọc để xóa cờ ngắt
  tc->TC_CHANNEL[CHANNEL].TC_CCR = TC_CCR_CLKDIS;  // ✅ Dừng timer ngay sau ngắt
  if (bitIndex >= MIN_VALID_BITS) {
    queueMessageFromISR();
  } else {
    resetMessageCollection();
  }
}

// Khởi tạo timer
void initTimer() {
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC);

  Tc *tc = TC;
  TcChannel *ch = &tc->TC_CHANNEL[CHANNEL];

  ch->TC_CCR = TC_CCR_CLKDIS;
  ch->TC_IDR = 0xFFFFFFFF;
  ch->TC_SR;

  ch->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1
             | TC_CMR_WAVE
             | TC_CMR_WAVSEL_UP
             | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET;

  NVIC_EnableIRQ(IRQ);
}

// Bắt đầu timer với khoảng thời gian us
uint32_t startTimer_cnt =0;
void startTimer(uint32_t us) {
  startTimer_cnt++;
  Tc *tc = TC;
  TcChannel *ch = &tc->TC_CHANNEL[CHANNEL];

  uint32_t ticks = (42000000 / 1000000) * us; // 42 tick mỗi us

  ch->TC_RC = ticks;
  ch->TC_IER = TC_IER_CPCS;
  ch->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

// ============================================================================
// SETUP FUNCTION
// ============================================================================
void setup() {
  Serial.begin(115200);
  // while (!Serial);
  
  // Small delay to ensure serial is ready
  delay(1000);
  Serial.println("J1850 PWM Reader Starting...");

  setupJ1850Hardware();

  // Setup J1850_PWM_RX as input WITHOUT pull-up
  pinMode(J1850_PWM_RX, INPUT);

  // Initialize timing
  lastTime = micros();
  lastActivityTime = millis();
  initTimer();
  attachInterrupt(J1850_PWM_RX, handlePWMInput, CHANGE);

  Serial.println("Ready to receive J1850 PWM data...");
  Serial.println("Expected format: XX XX XX XX XX XX");
}

void loop() {
  static uint32_t lastCheck = 0;

  if ((millis() - lastCheck) > 1) {
    processQueuedFrame();
    checkMessageTimeout();
    lastCheck = millis();
  }

}

void setupJ1850Hardware() {
  pinMode(PS_J1850_9141, OUTPUT);
  pinMode(J1850_PWM_VPW, OUTPUT);
  
  digitalWrite(PS_J1850_9141, HIGH);  // LOW  = no power at +12V_SW/+5V_SW
                                      // HIGH = power at +12V_SW/+5V_SW

  digitalWrite(J1850_PWM_VPW, HIGH);       // LOW  = ~7.9v (VPW)
                                      // HIGH = ~5.9V (PWM)

  delay(500);
}

// ============================================================================
// INTERRUPT SERVICE ROUTINE
// ============================================================================
void handlePWMInput() {
  bool newState = digitalRead(J1850_PWM_RX);
  uint32_t now = micros();

  // Ignore if state hasn't changed or duration is too short (noise filter)
  if (newState == lastState) return;
  
  lastState = newState;

  lastTime = now;


  // return;
  if (newState) {
    // RISING EDGE (passive to active) - PRIMARY TIMING REFERENCE
    lastRisingEdge = now;
    
    // Check for bit timing (rising edge to rising edge)
    if (collectingMessage && lastFallingEdge > 0) {
      uint32_t bitTime = now - lastFallingEdge;
      if (bitTime >= BIT_TIME_MIN_US && bitTime <= BIT_TIME_MAX_US) {
        // Valid bit timing - the active pulse width will determine bit value
        // We'll decode the bit on the falling edge
      } else if (bitTime > BIT_TIME_MAX_US) {
        // Gap too long - end of message or start of new frame
        // if (bitIndex >= MIN_VALID_BITS) {
        //   queueMessageFromISR();
        // } else {
        //   resetMessageCollection();
        // }
      }
    }
    
    // Handle EOF completion
    if (eofDetected && collectingMessage) {
      if (bitIndex >= MIN_VALID_BITS) {
        queueMessageFromISR();
      } else {
        resetMessageCollection();
      }
      eofDetected = false;
    }
    
  } else {
    // FALLING EDGE (active to passive) - DECODE ACTIVE PULSE WIDTH
    lastFallingEdge = now;
    
    if (lastRisingEdge == 0) return; // No valid rising edge reference
    
    uint32_t activePulseWidth = now - lastRisingEdge;
    

    // Check for Start of Frame (SOF) - Tp7: Active SOF
    if (activePulseWidth >= SOF_ACTIVE_MIN_US && activePulseWidth <= SOF_ACTIVE_MAX_US) {
      handleSOFDetection(lastRisingEdge);
      return;
    }

    // Check for End of Frame (EOF) - Tp5: EOF time
    if (activePulseWidth >= EOF_MIN_US && activePulseWidth <= EOF_MAX_US) {
      handleEOFDetection(lastRisingEdge);
      return;
    }

    // Process data bits if we're collecting a message
    if (collectingMessage) {
      // Decode bit based on active pulse width - Tp1 and Tp2
      if (activePulseWidth >= BIT_1_ACTIVE_MIN_US && activePulseWidth <= BIT_1_ACTIVE_MAX_US) {
        
        addBitToMessage(1, lastRisingEdge);
        startTimer(72 - 8); // Restart timer for next bit
      } else if (activePulseWidth >= BIT_0_ACTIVE_MIN_US && activePulseWidth <= BIT_0_ACTIVE_MAX_US) {
        addBitToMessage(0, lastRisingEdge);
        startTimer(72 -16); // Restart timer for next bit
      }
    }
  }
}

// ============================================================================
// SIGNAL HANDLING
// ============================================================================
void handleSOFDetection(uint32_t now) {
  // If we were collecting a message, finish it first
  // if (collectingMessage && bitIndex >= MIN_VALID_BITS) {
  //   queueMessageFromISR();
  // } else {
  //   resetMessageCollection();
  // }

  startNewMessage(now);
  
  // SOF is typically a '1' bit - add it to the message
  // The SOF active pulse itself represents the first bit
  // addBitToMessage(1, now);
}

void handleEOFDetection(uint32_t now) {
  eofDetected = true;

  // FIXED: Immediately queue the message if we have valid data
  if (collectingMessage && bitIndex >= MIN_VALID_BITS) {
    // Don't wait for rising edge, queue immediately
    queueMessageFromISR();
  } else {
    resetMessageCollection();
  }
  
  // Reset EOF detection
  eofDetected = false;
}

// ============================================================================
// MESSAGE COLLECTION
// ============================================================================
void addBitToMessage(uint8_t bit, uint32_t now) {
  if (!collectingMessage) {
    // Should not happen - bits should only come after SOF
    return;
  }

  if (bitIndex < MAX_BITS) {
    bitBuffer[bitIndex++] = bit;
    lastRisingEdge = now;

    // Auto-queue if we hit max bits
    if (bitIndex >= MAX_BITS) {
      queueMessageFromISR();
    }
  }
}

void startNewMessage(uint32_t now) {
  collectingMessage = true;
  bitIndex = 0;
  messageStartTime = now;
  lastRisingEdge = now;
  eofDetected = false;
}

void resetMessageCollection() {

  collectingMessage = false;
  bitIndex = 0;
  messageStartTime = 0;
  lastRisingEdge = 0;
  lastFallingEdge = 0;
  eofDetected = false;
}

void checkMessageTimeout() {
  if (collectingMessage && messageStartTime > 0) {
    // Message timeout logic removed
  }
}

void queueMessageFromISR() {
  uint8_t nextTail = (queueTail + 1) % MAX_FRAMES;
  if (nextTail == queueHead || bitIndex < MIN_VALID_BITS) {
    resetMessageCollection();
    return;
  }

  volatile Frame& f = frameQueue[queueTail];
  f.bitLength = bitIndex;
  f.timestamp = millis();
  f.hasValidSOF = true;  // We only start collecting after valid SOF
  f.hasValidEOF = eofDetected;

  // // Clear data array
  // for (uint8_t i = 0; i < MAX_BYTES; i++) f.data[i] = 0;

  // Pack bits into bytes (MSB first within each byte)
  for (uint8_t i = 0; i < bitIndex; i++) {
    uint8_t byteIndex = i / 8;
    uint8_t bitPosition = 7 - (i % 8);
    if (bitBuffer[i]) {
      f.data[byteIndex] |= (1 << bitPosition);
    }
  }

  queueTail = nextTail;
  resetMessageCollection();
}

// ============================================================================
// FRAME PROCESSING
// ============================================================================
void processQueuedFrame() {
  if (queueHead == queueTail) return;

  Frame f;
  noInterrupts();
  volatile Frame& vf = frameQueue[queueHead];
  f.bitLength = vf.bitLength;
  f.timestamp = vf.timestamp;
  f.hasValidSOF = vf.hasValidSOF;
  f.hasValidEOF = vf.hasValidEOF;
  for (uint8_t i = 0; i < MAX_BYTES; i++) {
    f.data[i] = vf.data[i];
  }
  queueHead = (queueHead + 1) % MAX_FRAMES;
  interrupts();

  // Only print frames with reasonable bit counts
  if (f.bitLength >= MIN_VALID_BITS && f.bitLength <= MAX_VALID_BITS) {
    uint8_t byteCount = (f.bitLength + 7) / 8;
    printFrame(f, byteCount);
    messagesShown++;
    lastActivityTime = millis();
  }
}

// ============================================================================
// OUTPUT
// ============================================================================
void printFrame(const Frame& f, uint8_t byteCount) {
  for (uint8_t i = 0; i < byteCount; i++) {
    if (f.data[i] < 0x10) Serial.print("0");
    Serial.print(f.data[i], HEX);
    if (i < byteCount - 1) Serial.print(" ");
  }
  Serial.println();
}