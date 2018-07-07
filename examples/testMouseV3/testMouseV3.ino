#include <PMW3360DM_T2QU.h>
#include <Mouse.h>


#define ncsPin A4               // nCS Pin to communicate with mouse through SPI
#define mySerial SerialUSB      // SerialUSB for Arduino Zero, Serial for Feather M0, 32u4...
#define WAIT_FOR_SERIAL         // Wait until Serial Upens
#define USE_MOUSE_LIBRARY       // Emulate Mouse only for 32u4 or SAMD21 microcontrollers.
//#define MOTION_INTERRUPT_PIN 5  // Motion interrupt Pin


// Global definitions
PMW3360DM_T2QU mouse;
volatile bool movementFlag = false;
void mouse_ISR () { movementFlag = true; }


void setup() 
{
  mySerial.begin(115200); 
  #ifdef WAIT_FOR_SERIAL
    while(!mySerial);
  #endif
  mySerial.println("Setup started");
  
  mouse.begin(ncsPin, &mySerial);
  mouse.printRegisters();
  mouse.printDefaultRegisters();

  #ifdef USE_MOUSE_LIBRARY
    Mouse.begin();
  #endif
  
  #ifdef MOTION_INTERRUPT_PIN
    pinMode(MOTION_INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(MOTION_INTERRUPT_PIN, mouse_ISR, FALLING);
  #endif
  
  mySerial.println("Setup complete");
}

void loop() 
{
  #ifdef MOTION_INTERRUPT_PIN
    if (movementFlag) 
    {
      detachInterrupt(MOTION_INTERRUPT_PIN);
  #endif // MOTION_INTERRUPT_PIN
      mouse.motionBurstRead();
      //mouse.printMotionBurstData();
    
      #ifdef USE_MOUSE_LIBRARY
        Mouse.move(mouse.x(), mouse.y(), 0);
      #else
        if (mouse.x() != 0 || mouse.y() != 0)
          mySerial.println("x = " + (String)mouse.x() + " | y = "+ (String)mouse.y()); 
        
      }
      #endif // USE_MOUSE_LIBRARY
      
  #ifdef MOTION_INTERRUPT_PIN
      movementFlag = false;
      attachInterrupt(MOTION_INTERRUPT_PIN, mouse_ISR, FALLING);
    }
  #endif // MOTION_INTERRUPT_PIN
    
  
// --- Tests ---
  // Function mouse.updatePointer(); not implemeted yet.
  /*
  uint8_t val = mouse.spi_read8(0x02);
  //if (val == 0x00)
  {
    uint8_t deltaXL= mouse.spi_read8(0x03);
    uint8_t deltaXH= mouse.spi_read8(0x04);
    uint8_t deltaYL= mouse.spi_read8(0x05);
    uint8_t deltaYH= mouse.spi_read8(0x06);

    mySerial.print("Motion: 0x"); mySerial.print(val, HEX);  
    mySerial.print(" deltaXL: 0x"); mySerial.print(deltaXL, HEX);
    mySerial.print(" deltaXH: 0x"); mySerial.print(deltaXH, HEX);
    mySerial.print(" deltaYL: 0x"); mySerial.print(deltaYL, HEX);
    mySerial.print(" deltaYH: 0x"); mySerial.print(deltaYH, HEX);
    mySerial.println();
  }
  */
  /*
  mySerial.print("Motion:    0x"); mySerial.println(mouse.spi_read8(REG_Motion));
  mySerial.print("Delta_X_L: "); mySerial.println((int) mouse.spi_read8(REG_Delta_X_L));
  mySerial.print("Delta_Y_L: "); mySerial.println((int) mouse.spi_read8(REG_Delta_Y_L));
 */
   // mouse.motionBurstRead();
   // mouse.printMotionBurstData();
   // mySerial.print("SQUAL: 0x"); mySerial.println(mouse.spi_read8(REG_SQUAL), HEX);
   // delay(500);
}
