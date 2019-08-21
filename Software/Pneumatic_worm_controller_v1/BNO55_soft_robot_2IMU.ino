#include <Wire.h>
//#include <i2c_t3.h>

#define BNO055_ADDRESS_A                                        0x28
#define BNO055_ADDRESS_B                                        0x29
#define BNO055_ID                                               0xA0

/* Page id register definition */
#define  BNO055_PAGE_ID_ADDR                                    0X07

/* PAGE0 REGISTER DEFINITION START*/
#define BNO055_CHIP_ID_ADDR                                     0x00
#define  BNO055_ACCEL_REV_ID_ADDR                               0x01
#define BNO055_MAG_REV_ID_ADDR                                  0x02
#define BNO055_GYRO_REV_ID_ADDR                                 0x03
#define BNO055_SW_REV_ID_LSB_ADDR                               0x04
#define BNO055_SW_REV_ID_MSB_ADDR                               0x05
#define BNO055_BL_REV_ID_ADDR                                   0X06

/* Accel data register */
#define BNO055_ACCEL_DATA_X_LSB_ADDR                            0X08
#define BNO055_ACCEL_DATA_X_MSB_ADDR                            0X09
#define BNO055_ACCEL_DATA_Y_LSB_ADDR                            0X0A
#define BNO055_ACCEL_DATA_Y_MSB_ADDR                            0X0B
#define BNO055_ACCEL_DATA_Z_LSB_ADDR                            0X0C
#define BNO055_ACCEL_DATA_Z_MSB_ADDR                            0X0D

/* Mag data register */
#define  BNO055_MAG_DATA_X_LSB_ADDR                              0X0E
#define BNO055_MAG_DATA_X_MSB_ADDR                               0X0F
#define BNO055_MAG_DATA_Y_LSB_ADDR                               0X10
#define BNO055_MAG_DATA_Y_MSB_ADDR                               0X11
#define BNO055_MAG_DATA_Z_LSB_ADDR                               0X12
#define BNO055_MAG_DATA_Z_MSB_ADDR                               0X13

/* Gyro data registers */
#define BNO055_GYRO_DATA_X_LSB_ADDR                              0X14
#define BNO055_GYRO_DATA_X_MSB_ADDR                              0X15
#define BNO055_GYRO_DATA_Y_LSB_ADDR                              0X16
#define BNO055_GYRO_DATA_Y_MSB_ADDR                              0X17
#define BNO055_GYRO_DATA_Z_LSB_ADDR                              0X18
#define BNO055_GYRO_DATA_Z_MSB_ADDR                              0X19

/* Euler data registers */
#define BNO055_EULER_H_LSB_ADDR                                  0X1A
#define BNO055_EULER_H_MSB_ADDR                                  0X1B
#define BNO055_EULER_R_LSB_ADDR                                  0X1C
#define BNO055_EULER_R_MSB_ADDR                                  0X1D
#define BNO055_EULER_P_LSB_ADDR                                  0X1E
#define BNO055_EULER_P_MSB_ADDR                                  0X1F

/* Quaternion data registers */
#define BNO055_QUATERNION_DATA_W_LSB_ADDR                        0X20
#define BNO055_QUATERNION_DATA_W_MSB_ADDR                        0X21
#define BNO055_QUATERNION_DATA_X_LSB_ADDR                        0X22
#define BNO055_QUATERNION_DATA_X_MSB_ADDR                        0X23
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR                        0X24
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR                        0X25
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR                        0X26
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR                        0X27

/* Linear acceleration data registers */
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR                      0X28
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR                      0X29
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR                      0X2A
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR                      0X2B
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR                      0X2C
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR                      0X2D

/* Gravity data registers */
#define BNO055_GRAVITY_DATA_X_LSB_ADDR                           0X2E
#define BNO055_GRAVITY_DATA_X_MSB_ADDR                           0X2F
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR                           0X30
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR                           0X31
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR                           0X32
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR                           0X33

/* Temperature data register */
#define BNO055_TEMP_ADDR                                         0X34

/* Status registers */
#define BNO055_CALIB_STAT_ADDR                                   0X35
#define BNO055_SELFTEST_RESULT_ADDR                              0X36
#define BNO055_INTR_STAT_ADDR                                    0X37

#define BNO055_SYS_CLK_STAT_ADDR                                 0X38
#define BNO055_SYS_STAT_ADDR                                     0X39
#define BNO055_SYS_ERR_ADDR                                      0X3A

/* Unit selection register */
#define BNO055_UNIT_SEL_ADDR                                     0X3B
#define BNO055_DATA_SELECT_ADDR                                  0X3C

/* Mode registers */
#define BNO055_OPR_MODE_ADDR                                     0X3D
#define BNO055_PWR_MODE_ADDR                                     0X3E

#define BNO055_SYS_TRIGGER_ADDR                                  0X3F
#define BNO055_TEMP_SOURCE_ADDR                                  0X40

/* Axis remap registers */
#define BNO055_AXIS_MAP_CONFIG_ADDR                              0X41
#define BNO055_AXIS_MAP_SIGN_ADDR                                0X42

/* SIC registers */
#define BNO055_SIC_MATRIX_0_LSB_ADDR                             0X43
#define BNO055_SIC_MATRIX_0_MSB_ADDR                             0X44
#define BNO055_SIC_MATRIX_1_LSB_ADDR                             0X45
#define BNO055_SIC_MATRIX_1_MSB_ADDR                             0X46
#define BNO055_SIC_MATRIX_2_LSB_ADDR                             0X47
#define BNO055_SIC_MATRIX_2_MSB_ADDR                             0X48
#define BNO055_SIC_MATRIX_3_LSB_ADDR                             0X49
#define BNO055_SIC_MATRIX_3_MSB_ADDR                             0X4A
#define BNO055_SIC_MATRIX_4_LSB_ADDR                             0X4B
#define BNO055_SIC_MATRIX_4_MSB_ADDR                             0X4C
#define BNO055_SIC_MATRIX_5_LSB_ADDR                             0X4D
#define BNO055_SIC_MATRIX_5_MSB_ADDR                             0X4E
#define BNO055_SIC_MATRIX_6_LSB_ADDR                             0X4F
#define BNO055_SIC_MATRIX_6_MSB_ADDR                             0X50
#define BNO055_SIC_MATRIX_7_LSB_ADDR                             0X51
#define BNO055_SIC_MATRIX_7_MSB_ADDR                             0X52
#define BNO055_SIC_MATRIX_8_LSB_ADDR                             0X53
#define BNO055_SIC_MATRIX_8_MSB_ADDR                             0X54

/* Accelerometer Offset registers */
#define ACCEL_OFFSET_X_LSB_ADDR                                  0X55
#define ACCEL_OFFSET_X_MSB_ADDR                                  0X56
#define ACCEL_OFFSET_Y_LSB_ADDR                                  0X57
#define ACCEL_OFFSET_Y_MSB_ADDR                                  0X58
#define ACCEL_OFFSET_Z_LSB_ADDR                                  0X59
#define ACCEL_OFFSET_Z_MSB_ADDR                                  0X5A

/* Magnetometer Offset registers */
#define MAG_OFFSET_X_LSB_ADDR                                    0X5B
#define MAG_OFFSET_X_MSB_ADDR                                    0X5C
#define MAG_OFFSET_Y_LSB_ADDR                                    0X5D
#define MAG_OFFSET_Y_MSB_ADDR                                    0X5E
#define MAG_OFFSET_Z_LSB_ADDR                                    0X5F
#define MAG_OFFSET_Z_MSB_ADDR                                    0X60

/* Gyroscope Offset register s*/
#define GYRO_OFFSET_X_LSB_ADDR                                   0X61
#define GYRO_OFFSET_X_MSB_ADDR                                   0X62
#define GYRO_OFFSET_Y_LSB_ADDR                                   0X63
#define GYRO_OFFSET_Y_MSB_ADDR                                   0X64
#define GYRO_OFFSET_Z_LSB_ADDR                                   0X65
#define GYRO_OFFSET_Z_MSB_ADDR                                   0X66

/* Radius registers */
#define ACCEL_RADIUS_LSB_ADDR                                    0X67
#define ACCEL_RADIUS_MSB_ADDR                                    0X68
#define MAG_RADIUS_LSB_ADDR                                      0X69
#define MAG_RADIUS_MSB_ADDR                                      0X6A



#define POWER_MODE_NORMAL                                        0X00
#define POWER_MODE_LOWPOWER                                      0X01
#define POWER_MODE_SUSPEND                                       0X02


/* Operation mode settings*/
#define OPERATION_MODE_CONFIG                                    0X00
#define OPERATION_MODE_ACCONLY                                   0X01
#define OPERATION_MODE_MAGONLY                                   0X02
#define OPERATION_MODE_GYRONLY                                   0X03
#define OPERATION_MODE_ACCMAG                                    0X04
#define OPERATION_MODE_ACCGYRO                                   0X05
#define OPERATION_MODE_MAGGYRO                                   0X06
#define OPERATION_MODE_AMG                                       0X07
#define OPERATION_MODE_IMUPLUS                                   0X08
#define OPERATION_MODE_COMPASS                                   0X09
#define OPERATION_MODE_M4G                                       0X0A
#define OPERATION_MODE_NDOF_FMC_OFF                              0X0B
#define OPERATION_MODE_NDOF                                      0X0C

#define REMAP_CONFIG_P0                                          0x21
#define REMAP_CONFIG_P1                                          0x24 // default
#define REMAP_CONFIG_P2                                          0x24
#define REMAP_CONFIG_P3                                          0x21
#define REMAP_CONFIG_P4                                          0x24
#define REMAP_CONFIG_P5                                          0x21
#define REMAP_CONFIG_P6                                          0x21
#define REMAP_CONFIG_P7                                          0x24

#define REMAP_SIGN_P0                                            0x04
#define REMAP_SIGN_P1                                            0x00 // default
#define REMAP_SIGN_P2                                            0x06
#define REMAP_SIGN_P3                                            0x02
#define REMAP_SIGN_P4                                            0x03
#define REMAP_SIGN_P5                                            0x01
#define REMAP_SIGN_P6                                            0x07
#define REMAP_SIGN_P7                                            0x05


uint16_t wg, xg, yg, zg;
uint16_t wg2, xg2, yg2, zg2;
uint16_t xe1;
int16_t  ye1, ze1; 
uint16_t xe2;
int16_t ye2, ze2; 

void setup() {
  // put your setup code here, to run once:

  Serial.begin(0);
  Serial.println("Start");
  Wire1.begin(BNO055_ADDRESS_A);
  Wire.begin(BNO055_ADDRESS_A); 
  BNO055_initiate(1);
  BNO055_initiate(2);


}

void loop() {
  // put your main code here, to run repeatedly:
 //Serial.println("looping"); 
 delay(25);
 BNO055_getEulerAngles(1);
 BNO055_getEulerAngles(2);
 /*
 BNO055_getQuaternion(1);

 Serial.print("1"); 
 Serial.print(",");
 Serial.print(wg);
 Serial.print(","); 
 Serial.print(xg);
 Serial.print(","); 
 Serial.print(yg); 
 Serial.print(",");
 Serial.println(zg);

 BNO055_getQuaternion(2);

 
 Serial.print("2"); 
 Serial.print(",");
 Serial.print(wg2);
 Serial.print(","); 
 Serial.print(xg2);
 Serial.print(","); 
 Serial.print(yg2); 
 Serial.print(",");
 Serial.println(zg2);
*/
// BNO055_initiate(1);

  /*
  uint8_t id = read8(BNO055_CHIP_ID_ADDR,2);
  if (id==BNO055_ID) { 
      Serial.println("BNO055 2 found\n");
    }
    else { 
      Serial.println("BNO055 2 not found!\n");
      return 0; 
    }



  uint8_t id2 = read8(BNO055_CHIP_ID_ADDR,1);
  if (id2==BNO055_ID) { 
      Serial.println("BNO055 1 found\n");
    }
    else { 
      Serial.println("BNO055 1 not found!\n");
      return 0; 
    }
    */
    
}

int16_t * BNO055_getEulerAngles(int whichIMU) {
 // uint8_t * buffer;
  uint8_t buffer[6];
  memset (buffer, 0, 6);
  static int16_t angles[3]; 
  int16_t x, y, z;
  x = y = z = 0;
  readLen(BNO055_EULER_H_LSB_ADDR, buffer, 6, whichIMU);
  x = (((int16_t)buffer[1]) << 8) | ((int16_t)buffer[0]);
  y = (((int16_t)buffer[3]) << 8) | ((int16_t)buffer[2]);
  z = (((int16_t)buffer[5]) << 8) | ((int16_t)buffer[4]);
  
  angles[0] =x; // scale*w; 
  angles[1] =y;// scale*x; 
  angles[2] =z;// scale*y; 

  if(whichIMU ==1) {
    xe1 = x; 
    ye1 = y; 
    ze1 = z; 
    Serial.print("1"); 
    Serial.print(",");
    Serial.print(xe1/16);
    Serial.print(","); 
    Serial.print(ye1/16);
    Serial.print(","); 
    Serial.println(ze1/16); 
  }

  if(whichIMU ==2) {
    xe2 = x; 
    ye2 = y; 
    ze2 = z; 
    Serial.print("2"); 
    Serial.print(",");
    Serial.print(xe2/16);
    Serial.print(","); 
    Serial.print(ye2/16);
    Serial.print(","); 
    Serial.println(ze2/16); 
  }
  
//    SEGGER_RTT_printf(0,"euler:    %d,%d,%d\n",x,y,x);
  return angles;
}

void BNO055_getQuaternion(int whichIMU)
{
 // uint8_t * buffer;
  uint8_t buffer[8];
  memset (buffer, 0, 8);
  static int16_t quat[4]; 

  int16_t x, y, z, w;
  x = y = z = w = 0;

  /* Read quat data (8 bytes) */
  readLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8,whichIMU);
  w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
  x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
  y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
  z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

    //    SEGGER_RTT_printf(0,"quanternion:    %d,%d,%d,%d\n",w,x,y,z);

  /* Assign to Quaternion */
  /* See http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
     3.6.5.5 Orientation (Quaternion)  */
  const double scale = (1.0 / (1<<14));
  quat[0] =w; // scale*w; 
  quat[1] =x;// scale*x; 
  quat[2] =y;// scale*y; 
  quat[3] =z;// scale*z;

  if(whichIMU ==1) {
    wg = w; 
    xg = x; 
    yg = y; 
    zg = z;
  }

  if(whichIMU ==2) {
    wg2 = w; 
    xg2 = x; 
    yg2 = y; 
    zg2 = z;
  }
  
  
  //imu::Quaternion quat(scale * w, scale * x, scale * y, scale * z);
}

bool BNO055_initiate(int whichIMU) { 
  
  uint8_t id = read8(BNO055_CHIP_ID_ADDR,whichIMU);
  if (id==BNO055_ID) { 
      Serial.println("BNO055 found\n");
    }
    else { 
      Serial.println("BNO055 not found!\n");
      return 0; 
    }

   write8(BNO055_OPR_MODE_ADDR,OPERATION_MODE_CONFIG,whichIMU);
   Serial.println("BNO055 set to config mode\n");
   delay(10);    
    
    delay(100);
    Serial.println("BNO055 reset ok!\n");   
    write8(BNO055_PWR_MODE_ADDR,POWER_MODE_NORMAL,whichIMU);    
    Serial.println("BNO055 set to normal power mode ok!\n");
    delay(10);

    write8(BNO055_PAGE_ID_ADDR,0x0,whichIMU);

    write8(BNO055_SYS_TRIGGER_ADDR, 0x0,whichIMU);
    delay(10);

    write8(BNO055_OPR_MODE_ADDR,OPERATION_MODE_NDOF,whichIMU);
    
    delay(20);
    Serial.println("BNO055 initialization done\n");
    return true; 
    
}//end initialize BNO055


void write8(uint8_t reg, uint8_t value, int whichIMU) {
  if (whichIMU == 1) {
      Wire.beginTransmission(BNO055_ADDRESS_A);
      Wire.write(reg);
      Wire.write(value); 
      Wire.endTransmission();
  }
  if (whichIMU == 2) { 
      Wire1.beginTransmission(BNO055_ADDRESS_A);
      Wire1.write(reg);
      Wire1.write(value); 
      Wire1.endTransmission();
  }  
  return 0;
}


byte read8(uint8_t reg, int whichIMU) {
  byte value = 0;
  if(whichIMU==1) { 
    Wire.beginTransmission(BNO055_ADDRESS_A);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(BNO055_ADDRESS_A, 1);
    value = Wire.read();
  }

  else if(whichIMU==2) { 
    Wire1.beginTransmission(BNO055_ADDRESS_A);
    bool ok = Wire1.write(byte(reg));
    if(ok==true) Serial.println("sucess to write");
    Wire1.endTransmission();
    
    int numbBytes = Wire1.requestFrom(BNO055_ADDRESS_A, 1);
    value = Wire1.read();
    Serial.println(numbBytes);
  }

  
  return value;
}


bool readLen(uint8_t reg, byte *buffer,
                              uint8_t len,int whichIMU) {
  if(whichIMU==1) {                              
    Wire.beginTransmission(BNO055_ADDRESS_A);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom(BNO055_ADDRESS_A, (byte)len);
  
    for (uint8_t i = 0; i < len; i++) {
      buffer[i] = Wire.read();
    }
  }//end imu1

  if(whichIMU==2) {                              
    Wire1.beginTransmission(BNO055_ADDRESS_A);
    Wire1.write((uint8_t)reg);
    Wire1.endTransmission();
    Wire1.requestFrom(BNO055_ADDRESS_A, (byte)len);
  
    for (uint8_t i = 0; i < len; i++) {
      buffer[i] = Wire1.read();
    }
  }//end imu1

  /* ToDo: Check for errors! */
  return true;
}
