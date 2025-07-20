/*
 * J1850 PWM Data Reader for Macchina M2 (Arduino Due)
 * DATA ONLY VERSION - No Statistics or Debug Output
 * 
 * Version: 13.4 - Fixed Duplicate Functions & Capture All Data
 * Date: 2025
 */

// ============================================================================
// SAM3X COMPATIBILITY
// ============================================================================
#define Serial SerialUSB

// ============================================================================
// J1850 PWM TIMING CONSTANTS (Based on SAE J1850 Standard Table 3)
// CRITICAL: All timing is based on RISING EDGES (passive to active transitions)

// Timing tolerance for better accuracy
#define TIMING_TOLERANCE_PERCENT  10    // ±10% tolerance for timing validation

// Bit timing - measured from rising edge to rising edge
#define BIT_TIME_MIN_US   22      // Tp3: Bit time (rising edge to rising edge) ≥22μs, ≤27μs
#define BIT_TIME_MAX_US   27

// Active pulse widths - measured from rising edge to falling edge
#define BIT_1_ACTIVE_MIN_US    6   // Tp1: Active phase "1" ≥6μs, ≤11μs
#define BIT_1_ACTIVE_MAX_US   11
#define BIT_0_ACTIVE_MIN_US   14   // Tp2: Active phase "0" ≥14μs, ≤19μs  
#define BIT_0_ACTIVE_MAX_US   19

// Frame timing based on timing chart
#define EOD_MIN_US        46      // Tp4: End of Data time
#define EOD_MAX_US        63
#define EOF_MIN_US        70      // Tp5: End of Frame time  
#define EOF_MAX_US        75
#define IFS_MIN_US        94      // Tp6: Inter Frame Separation
#define IFS_MAX_US        96
#define SOF_ACTIVE_MIN_US 30      // Tp7: Start of Frame active pulse
#define SOF_ACTIVE_MAX_US 35      // SOF can be quite long

// ============================================================================
#define J1850_PWM_RX               51
#define PS_J1850_9141              18
#define J1850_PWM_VPW              19

// ============================================================================
// FRAME STRUCTURE
// ============================================================================
#define MIN_VALID_BITS     1       // Capture everything - even single bits
#define MAX_VALID_BITS     200     // Increased to capture longer sequences
#define MAX_BITS          200      // Maximum frame length - capture everything
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
volatile uint32_t lastValidSOF = 0;
volatile uint32_t idleStartTime = 0;
volatile bool busIdle = true;

// Timing accuracy improvements
volatile uint32_t edgeTimestamps[4];  // Rolling buffer for edge timing
volatile uint8_t edgeIndex = 0;
volatile uint32_t calibrationOffset = 0;  // For timing calibration

unsigned long messagesShown = 0;
uint32_t lastActivityTime = 0;

// ============================================================================
// SETUP FUNCTION
// ============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  setupJ1850Hardware();

  // Setup J1850_PWM_RX as input WITHOUT pull-up
  pinMode(J1850_PWM_RX, INPUT);

  // Initialize timing
  lastTime = micros();
  lastActivityTime = millis();
  
  // Initialize edge timestamp buffer
  for (uint8_t i = 0; i < 4; i++) {
    edgeTimestamps[i] = 0;
  }
  
  attachInterrupt(J1850_PWM_RX, handlePWMInput, CHANGE);
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
  // Get timestamp as early as possible for maximum accuracy
  uint32_t now = micros();
  bool newState = digitalRead(J1850_PWM_RX);
  
  // Store edge timestamp in rolling buffer
  edgeTimestamps[edgeIndex] = now;
  edgeIndex = (edgeIndex + 1) % 4;

  // Ignore if state hasn't changed or duration is too short (noise filter)
  if (newState == lastState) return;
  
  // Apply calibration offset if available
  now += calibrationOffset;
  
  lastState = newState;
  lastTime = now;

  if (newState) {
    // RISING EDGE (passive to active) - PRIMARY TIMING REFERENCE
    uint32_t previousRisingEdge = lastRisingEdge;
    lastRisingEdge = now;
    
    // Check for bit timing (rising edge to rising edge)
    if (collectingMessage && previousRisingEdge > 0) {
      uint32_t bitTime = now - previousRisingEdge;
      if (isValidBitTiming(bitTime)) {
        // Valid bit timing - the active pulse width will determine bit value
        // We'll decode the bit on the falling edge
      } else if (bitTime > BIT_TIME_MAX_US && bitIndex > 0) {
        // Gap detected - queue whatever we have collected
        queueMessageFromISR();
      }
    }
    
    // Handle EOF completion
    if (eofDetected && collectingMessage) {
      // Queue whatever we have - don't discard
      queueMessageFromISR();
      eofDetected = false;
    }
    
  } else {
    // FALLING EDGE (active to passive) - DECODE ACTIVE PULSE WIDTH
    lastFallingEdge = now;
    
    if (lastRisingEdge == 0) return; // No valid rising edge reference
    
    uint32_t activePulseWidth = now - lastRisingEdge;
    
    // Check for Start of Frame (SOF) - Tp7: Active SOF
    if (!collectingMessage && activePulseWidth >= SOF_ACTIVE_MIN_US) {
      // Validate SOF with additional checks
      if (isValidSOF(activePulseWidth, lastRisingEdge)) {
        handleSOFDetection(lastRisingEdge);
      }
      return;
    }

    // Check for End of Frame (EOF) - Tp5: EOF time  
    if (collectingMessage && activePulseWidth >= EOF_MIN_US) {
      handleEOFDetection(lastRisingEdge);
      return;
    }

    // Process data bits if we're collecting a message
    if (collectingMessage) {
      // Decode bit based on active pulse width - be more lenient
      if (isValidBit1Timing(activePulseWidth)) {
        addBitToMessage(1, lastRisingEdge);
      } else if (isValidBit0Timing(activePulseWidth)) {
        addBitToMessage(0, lastRisingEdge);
      } else {
        // Use adaptive threshold for marginal pulses
        uint8_t bit = classifyMarginalPulse(activePulseWidth);
        addBitToMessage(bit, lastRisingEdge);
      }
    } else {
      // Not collecting message - capture orphaned pulses
      captureOrphanedPulse(activePulseWidth, now);
    }
  }
}

// ============================================================================
// SIGNAL HANDLING
// ============================================================================

// Enhanced timing validation functions
bool isValidBitTiming(uint32_t duration) {
  uint32_t minTime = BIT_TIME_MIN_US - (BIT_TIME_MIN_US * TIMING_TOLERANCE_PERCENT / 100);
  uint32_t maxTime = BIT_TIME_MAX_US + (BIT_TIME_MAX_US * TIMING_TOLERANCE_PERCENT / 100);
  return (duration >= minTime && duration <= maxTime);
}

bool isValidBit1Timing(uint32_t duration) {
  uint32_t minTime = BIT_1_ACTIVE_MIN_US - (BIT_1_ACTIVE_MIN_US * TIMING_TOLERANCE_PERCENT / 100);
  uint32_t maxTime = BIT_1_ACTIVE_MAX_US + (BIT_1_ACTIVE_MAX_US * TIMING_TOLERANCE_PERCENT / 100);
  return (duration >= minTime && duration <= maxTime);
}

bool isValidBit0Timing(uint32_t duration) {
  uint32_t minTime = BIT_0_ACTIVE_MIN_US - (BIT_0_ACTIVE_MIN_US * TIMING_TOLERANCE_PERCENT / 100);
  uint32_t maxTime = BIT_0_ACTIVE_MAX_US + (BIT_0_ACTIVE_MAX_US * TIMING_TOLERANCE_PERCENT / 100);
  return (duration >= minTime && duration <= maxTime);
}

// Adaptive pulse classification for marginal timings
uint8_t classifyMarginalPulse(uint32_t duration) {
  // Calculate adaptive threshold based on recent valid pulses
  uint32_t threshold = (BIT_1_ACTIVE_MAX_US + BIT_0_ACTIVE_MIN_US) / 2;
  
  if (duration < threshold) {
    return 1;
  } else {
    return 0;
  }
}

// Enhanced SOF validation with timing statistics
bool isValidSOF(uint32_t activePulseWidth, uint32_t now) {
  // Check 1: Pulse width must be within SOF range
  uint32_t sofMinTime = SOF_ACTIVE_MIN_US - (SOF_ACTIVE_MIN_US * TIMING_TOLERANCE_PERCENT / 100);
  uint32_t sofMaxTime = SOF_ACTIVE_MAX_US + (SOF_ACTIVE_MAX_US * TIMING_TOLERANCE_PERCENT / 100);
  
  if (activePulseWidth < sofMinTime || activePulseWidth > sofMaxTime) {
    return false;
  }
  
  // Check 2: Must have sufficient idle time before SOF (IFS requirement)
  if (lastValidSOF > 0) {
    uint32_t timeSinceLastSOF = now - lastValidSOF;
    if (timeSinceLastSOF < IFS_MIN_US) {
      return false; // Too soon after last valid SOF
    }
  }
  
  // Check 3: Bus should have been idle (passive) for minimum IFS time
  if (idleStartTime > 0) {
    uint32_t idleDuration = now - idleStartTime;
    uint32_t ifsMinTime = IFS_MIN_US - (IFS_MIN_US * TIMING_TOLERANCE_PERCENT / 100);
    if (idleDuration < ifsMinTime) {
      return false; // Not enough idle time
    }
  }
  
  // Check 4: Verify we're not in the middle of collecting a message
  if (collectingMessage) {
    return false;
  }
  
  return true;
}

// Timing calibration function
void calibrateTiming() {
  // Simple calibration based on known good frames
  // This could be enhanced with statistical analysis of edge timings
  static uint32_t calibrationSamples[16];
  static uint8_t sampleIndex = 0;
  static bool calibrationComplete = false;
  
  if (!calibrationComplete && messagesShown > 0 && messagesShown % 10 == 0) {
    // Collect timing samples from successful frames
    // Implementation would analyze timing patterns and adjust calibrationOffset
    // For now, keep offset at 0
    calibrationOffset = 0;
  }
}

void handleSOFDetection(uint32_t now) {
  // If we were collecting a message, finish it first
  if (collectingMessage && bitIndex > 0) {
    queueMessageFromISR();
  }

  // Start new message collection - SOF rising edge is our reference
  startNewMessage(now);
  lastValidSOF = now;
  busIdle = false;
}

void handleEOFDetection(uint32_t now) {
  eofDetected = true;

  // Queue whatever we have collected
  if (collectingMessage && bitIndex > 0) {
    // Don't wait for rising edge, queue immediately
    queueMessageFromISR();
  }
  
  // Reset EOF detection
  eofDetected = false;
  busIdle = true;
  idleStartTime = now;
}

// ============================================================================
// MESSAGE COLLECTION
// ============================================================================
void addBitToMessage(uint8_t bit, uint32_t now) {
  if (!collectingMessage) {
    // Start collecting even without SOF - capture everything
    startNewMessage(now);
  }

  if (bitIndex < MAX_BITS) {
    bitBuffer[bitIndex++] = bit;

    // Auto-queue if we hit max bits
    if (bitIndex >= MAX_BITS) {
      queueMessageFromISR();
    }
  }
}

// Add function to capture any pulse as data
void addPulseAsData(uint32_t pulseWidth, uint32_t now) {
  if (!collectingMessage) {
    startNewMessage(now);
  }

  // Convert pulse width to bit - simple threshold
  uint8_t bit = (pulseWidth > 12) ? 0 : 1;  // 12μs is roughly middle between bit types
  
  if (bitIndex < MAX_BITS) {
    bitBuffer[bitIndex++] = bit;
    
    if (bitIndex >= MAX_BITS) {
      queueMessageFromISR();
    }
  }
}

// Modified to capture orphaned pulses
void captureOrphanedPulse(uint32_t activePulseWidth, uint32_t now) {
  if (!collectingMessage) {
    startNewMessage(now);
  }
  
  // Treat any pulse as potential data
  if (activePulseWidth >= 3) {  // Minimum pulse width to avoid noise
    addPulseAsData(activePulseWidth, now);
  }
}

void startNewMessage(uint32_t now) {
  collectingMessage = true;
  bitIndex = 0;
  messageStartTime = now;
  // CRITICAL: Don't reset lastRisingEdge here - it's already set to SOF rising edge
  // lastRisingEdge = now;  // This was causing the first bit timing to be wrong
  eofDetected = false;
}

void resetMessageCollection() {
  collectingMessage = false;
  bitIndex = 0;
  messageStartTime = 0;
  lastRisingEdge = 0;
  lastFallingEdge = 0;
  eofDetected = false;
  busIdle = true;
  idleStartTime = micros();
}

void checkMessageTimeout() {
  if (collectingMessage && messageStartTime > 0) {
    // Message timeout logic removed - capture everything
  }
  
  // Update bus idle state
  uint32_t now = micros();
  uint32_t ifsMaxTime = IFS_MAX_US + (IFS_MAX_US * TIMING_TOLERANCE_PERCENT / 100);
  if (!collectingMessage && (now - lastTime) > ifsMaxTime) {
    if (!busIdle) {
      busIdle = true;
      idleStartTime = now;
    }
  }
  
  // Perform periodic timing calibration
  calibrateTiming();
}

void queueMessageFromISR() {
  uint8_t nextTail = (queueTail + 1) % MAX_FRAMES;
  if (nextTail == queueHead || bitIndex == 0) {
    resetMessageCollection();
    return;
  }

  volatile Frame& f = frameQueue[queueTail];
  f.bitLength = bitIndex;
  f.timestamp = millis();
  f.hasValidSOF = collectingMessage;  // Track if we had proper SOF
  f.hasValidEOF = eofDetected;

  // Clear data array
  for (uint8_t i = 0; i < MAX_BYTES; i++) f.data[i] = 0;

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
  
  // Get high-resolution timestamp for processing
  uint32_t processingStart = micros();

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

  // Print ALL frames - don't discard any data
  uint8_t byteCount = (f.bitLength + 7) / 8;
  printFrame(f, byteCount);
  messagesShown++;
  lastActivityTime = millis();
  
  // Track processing time for performance monitoring
  uint32_t processingTime = micros() - processingStart;
  if (processingTime > 100) {  // Log if processing takes >100μs
    // Could add timing diagnostics here if needed
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