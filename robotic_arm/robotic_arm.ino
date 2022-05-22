#include <Servo.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servo1;
Servo servo2;
Servo servo3;

void getServoCoords();
void writeMessage();
int myAtoi(const char* str);

void setup() {
  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);
  
  Serial.begin(115200);
  Serial.print("Ready!\n");
  
  lcd.init();
  lcd.backlight();
  lcd.clear();
}

int pos_servo1 = 90;
int pos_servo2 = 90;
int pos_servo3 = 90;
String serialString;
char serial_char[16];
char *p;
char lcd_buffer[16];

void loop() {
  // Wait for serial
  while (!Serial.available());
  
  // Get coordinates of servomotors from serial
  getServoCoords();

  // Move servomotors
  servo1.write(pos_servo1);
  servo2.write(pos_servo2);
  servo3.write(pos_servo3);

  // Write message to LCD Screen
  writeMessage();
  
}

void getServoCoords() {
  serialString = Serial.readString();
  strncpy(serial_char, serialString.c_str(), 16);
  serial_char[15] = '\0';
  p = strtok(serial_char, " ");
  pos_servo1 = myAtoi(p);
  p = strtok(NULL, " ");
  pos_servo2 = myAtoi(p);
  p = strtok(NULL, " ");
  pos_servo3 = myAtoi(p);
}

void writeMessage() {
  lcd.clear();
  lcd.setCursor(0, 0);
  sprintf(lcd_buffer, "X: %d", pos_servo1);
  lcd.print(lcd_buffer);
  
  lcd.setCursor(10, 0);
  sprintf(lcd_buffer, "Y: %d", pos_servo2);
  lcd.print(lcd_buffer);
  
  sprintf(lcd_buffer, "Z: %d", pos_servo3);
  lcd.setCursor(5, 1);
  lcd.print(lcd_buffer);
}


int myAtoi(const char* str) {
    int sign = 1, base = 0, i = 0;
   
    while (str[i] == ' ')
        i++;
    
    if (str[i] == '-' || str[i] == '+')
        sign = 1 - 2 * (str[i++] == '-');
   
    while (str[i] >= '0' && str[i] <= '9')
        base = 10 * base + (str[i++] - '0');
        
    return base * sign;
}
