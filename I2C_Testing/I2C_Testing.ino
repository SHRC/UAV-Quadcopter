#include <util/twi.h>

#define F_SCL 100000UL // SCL frequency
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)
#define GYRO_CALIB 1.0 // Need to test for this
#define TORQUE_CALIB 1.0 // Need to test for this

int16_t rawAccelData[3];
int16_t rawMagneData[3];
int16_t rawGyroData[3];

int16_t base[3];

int16_t quadNorm[3]; // Unit vector normal to the plane of the quadcopter in global terms
int16_t oriX[2]; // Unit vector pointing in the direction of the East side of the quadcopter
int16_t oriY[2]; // Unit vector pointing in the direction of the North side of the quadcopter

int16_t * torque;

void setup(){
  I2C_init();
  accel_init();
  magne_init();
  gyro_init();
  Serial.begin(115200);
  Serial.println("test");
}

void loop(void){
//  getAccelVec((uint16_t *)rawAccelData);
//  Serial.print(rawAccelData[0]);
//  Serial.print(", ");
//  Serial.print(rawAccelData[1]);
//  Serial.print(", ");
//  Serial.println(rawAccelData[2]);
//  
//  getMagneVec((uint16_t *)rawMagneData);
//  Serial.print(rawMagneData[0]);
//  Serial.print(", ");
//  Serial.print(rawMagneData[1]);
//  Serial.print(", ");
//  Serial.println(rawMagneData[2]);
//  
//  getGyroVec((uint16_t *)rawGyroData);
//  Serial.print(rawGyroData[0]);
//  Serial.print(", ");
//  Serial.print(rawGyroData[1]);
//  Serial.print(", ");
//  Serial.println(rawGyroData[2]);
//  Serial.println();

  Serial.print("Hello World");
  getQuadNorm();
  Serial.print(quadNorm[0]);
  Serial.print(", ");
  Serial.print(quadNorm[1]);
  Serial.print(", ");
  Serial.print(quadNorm[2]);
  Serial.println();
  
  delay(50);
}


void I2C_init(void){
  DDRC &= ~(0b01100000);
  PORTC |= 0b01100000;
  TWBR = TWBR_val;
}

uint8_t I2C_start(uint8_t address){
  // reset TWI control register
  TWCR = 0;
  // transmit START condition
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
  // wait for end of transmission
  while( !(TWCR & (1<<TWINT)) );

  // check if the start condition was successfully transmitted
  if((TWSR & 0xF8) != TW_START){ return 1; }

  // load slave address into data register
  TWDR = address;
  // start transmission of address
  TWCR = (1<<TWINT) | (1<<TWEN);
  // wait for end of transmission
  while( !(TWCR & (1<<TWINT)) );

  // check if the device has acknowledged the READ / WRITE mode
  uint8_t twst = TW_STATUS & 0xF8;
  if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;
                                                                                       
  return 0;                                                                            
}                                                                                            
                                                                                             
uint8_t I2C_write(uint8_t data){                                                             
  // load data into data register                                                      
  TWDR = data;                                                                         
  // start transmission of data                                                        
  TWCR = (1<<TWINT) | (1<<TWEN);
  // wait for end of transmission
  while( !(TWCR & (1<<TWINT)) );

  if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }

  return 0;
}

uint8_t I2C_read_ack(void){

  // start TWI module and acknowledge data after reception
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  // wait for end of transmission
  while( !(TWCR & (1<<TWINT)) );
  // return received data from TWDR
  return TWDR;
}

uint8_t I2C_read_nack(void){

  // start receiving without acknowledging reception
  TWCR = (1<<TWINT) | (1<<TWEN);
  // wait for end of transmission
  while( !(TWCR & (1<<TWINT)) );
  // return received data from TWDR
  return TWDR;
}

void I2C_stop(void){
  // transmit STOP condition
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}

void accel_init(void){
  I2C_start(0b00110010);
  I2C_write(0x20);
  I2C_write(0b01010111);
  I2C_stop();
}

void magne_init(void){
  I2C_start(0b00111100);
  I2C_write(0x01);
  I2C_write(0x20);
  I2C_write(0x00);
  I2C_stop();
}

void gyro_init(void){
  I2C_start(0b11010110);
  I2C_write(0xA0);
  I2C_write(0b00001111);
  I2C_write(0b01000000);
  I2C_stop();
}
  

void getAccelVec(uint16_t *data){
  I2C_start(0b00110010);
  I2C_write(0xA8);
  I2C_start(0b00110011);
  data[0] = I2C_read_ack();
  data[0] |= (I2C_read_ack() << 8);
  data[1] = I2C_read_ack();
  data[1] |= (I2C_read_ack() << 8);
  data[2] = I2C_read_ack();
  data[2] |= (I2C_read_nack() << 8);
  I2C_stop();
}

void getMagneVec(uint16_t *data){
  I2C_start(0b00111100);
  I2C_write(0x03);
  I2C_start(0b00111101);
  data[0] = (I2C_read_ack() << 8);
  data[0] |= I2C_read_ack();
  data[2] = (I2C_read_ack() << 8);
  data[2] |= I2C_read_ack();
  data[1] = (I2C_read_ack() << 8);
  data[1] |= I2C_read_nack();
  I2C_stop();
}

void getGyroVec(uint16_t *data){
  I2C_start(0b11010110);
  I2C_write(0xA8);
  I2C_start(0b11010111);
  data[0] = I2C_read_ack();
  data[0] |= (I2C_read_ack() << 8);
  data[1] = I2C_read_ack();
  data[1] |= (I2C_read_ack() << 8);
  data[2] = I2C_read_ack();
  data[2] |= (I2C_read_nack() << 8);
  I2C_stop();
}

void getQuadNorm() {
  quadNorm[0] = -rawAccelData[0];
  quadNorm[1] = rawAccelData[1];
  quadNorm[2] = -rawAccelData[2];
}

void getOriX(int dt) {
 int16_t xx = oriX[0];
 int16_t xy = oriX[1];
 
 int16_t yx = oriY[0];
 int16_t yy = oriY[1];
 
 float dtheta = GYRO_CALIB * rawGyroData[1] * dt;
 oriX[0] = xx * cos(dtheta) - xy * sin(dtheta);
 oriX[1] = xx * sin(dtheta) + xy * cos(dtheta);
 
 oriY[0] = yx * cos(dtheta) - yy * sin(dtheta);
 oriY[1] = yx * sin(dtheta) + yy * cos(dtheta);
}


void getTorque(const int16_t* move) {
  int16_t resultant[] {base[0] + move[0], base[1] + move[1], base[2] + move[2]};
  torque = cross(quadNorm, resultant);
  torque[0] *= TORQUE_CALIB;
  torque[1] *= TORQUE_CALIB;
  torque[2] *= TORQUE_CALIB;
}

int16_t* cross(const int16_t* v1, const int16_t* v2) {
  int16_t answer[3];
  answer[0] = v1[1] * v2[2] - v1[2] * v2[1];
  answer[1] = v1[2] * v2[0] - v1[0] * v2[2];
  answer[2] = v1[0] * v2[1] - v1[1] * v2[0];
  return answer;
}











