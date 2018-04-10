#include <PMW3360DM_T2QU.h>
#include <Mouse.h>

const int motionInterruptPin = 5;
const int ncsPin = 10;

PMW3360DM_T2QU mouse;

unsigned long currTime;
unsigned long timer;
unsigned long pollTimer;
volatile bool movementFlag = false;

void mouse_ISR ()
{
  movementFlag = true;
}

void setup() 
{
  Serial.begin(115200); 
  //while(!Serial);
  Serial.println("Setup start");
  Mouse.begin();
  mouse.begin(ncsPin);

  pinMode(motionInterruptPin, INPUT_PULLUP);
  //attachInterrupt(motionInterruptPin, mouse_ISR, FALLING);

  mouse.displayRegisters();
  mouse.printDefaultRegisters();
  

  currTime = millis();
  pollTimer = 1000;
  delay(5000);
  
  Serial.println("Setup complete");
}

void loop() 
{
  // Function mouse.updatePointer(); not implemeted yet.
  /*
  uint8_t val = mouse.spi_read8(0x02);
  //if (val == 0x00)
  {
    uint8_t deltaXL= mouse.spi_read8(0x03);
    uint8_t deltaXH= mouse.spi_read8(0x04);
    uint8_t deltaYL= mouse.spi_read8(0x05);
    uint8_t deltaYH= mouse.spi_read8(0x06);

    Serial.print("Motion: 0x"); Serial.print(val, HEX);  
    Serial.print(" deltaXL: 0x"); Serial.print(deltaXL, HEX);
    Serial.print(" deltaXH: 0x"); Serial.print(deltaXH, HEX);
    Serial.print(" deltaYL: 0x"); Serial.print(deltaYL, HEX);
    Serial.print(" deltaYH: 0x"); Serial.print(deltaYH, HEX);
    Serial.println();
  }
  */
/*
  Serial.print("Motion:    0x"); Serial.println(mouse.spi_read8(REG_Motion));
  Serial.print("Delta_X_L: "); Serial.println((int) mouse.spi_read8(REG_Delta_X_L));
  Serial.print("Delta_Y_L: "); Serial.println((int) mouse.spi_read8(REG_Delta_Y_L));
 */
 //mouse.motionBurstRead();
 //mouse.printMotionBurstData();
    

  if (true)
  {
    mouse.motionBurstRead();
     
    //uncomment below if using Teensy from PJRC to move mouse on screen
    Mouse.move(mouse.x(), mouse.y(), 0);
    /*
     if (mouse.x() != 0 || mouse.y() != 0)
      {
        Serial.print("x = "); Serial.print((String)mouse.x() + " | ");
        Serial.print("y = "); Serial.print((String)mouse.y() + "\n"); 
      }
     }
*/
    movementFlag = false;
  }
  
/*
   Serial.print("SQUAL: 0x"); Serial.println(mouse.spi_read8(REG_SQUAL), HEX);
   delay(500);
   */
}
