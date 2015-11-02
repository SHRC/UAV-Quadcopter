#include <util/twi.h>
#define F_SCL 100000UL // SCL frequency
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)

int16_t rawAccelData[3];
int16_t rawMagneData[3];
int16_t rawGyroData[3];

void setup(){
  I2C_init();
  accel_init();
  magne_init();
  gyro_init();
  Serial.begin(115200);
  Serial.println("test");
}

void loop(void){
  getAccelVec((uint16_t *)rawAccelData);
  Serial.print(rawAccelData[0]);
  Serial.print(", ");
  Serial.print(rawAccelData[1]);
  Serial.print(", ");
  Serial.println(rawAccelData[2]);
  
  getMagneVec((uint16_t *)rawMagneData);
  Serial.print(rawMagneData[0]);
  Serial.print(", ");
  Serial.print(rawMagneData[1]);
  Serial.print(", ");
  Serial.println(rawMagneData[2]);
  
  getGyroVec((uint16_t *)rawGyroData);
  Serial.print(rawGyroData[0]);
  Serial.print(", ");
  Serial.print(rawGyroData[1]);
  Serial.print(", ");
  Serial.println(rawGyroData[2]);
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

