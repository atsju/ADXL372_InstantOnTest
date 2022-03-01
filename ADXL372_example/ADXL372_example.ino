#include "SPI.h"
#include "Communication.h"
#include "adxl372.h"

/*Acceleremoter configuration*/
#define ACT_VALUE          30     /* Activity threshold value */

#define INACT_VALUE        30     /* Inactivity threshold value */

#define ACT_TIMER          1    /* Activity timer value in multiples of 3.3ms */

#define INACT_TIMER        1     /* Inactivity timer value in multiples of 26ms */

/**
 * @brief Number of points in ADXL FIFO
 *
 * 255 is multiple of 3 (XYZ) and represents half of FIFO
 */
#define FIFO_LENGTH       12

struct adxl372_device adxl372;
unsigned char devId_AD;
unsigned char devId_MST;
unsigned char devID_Product;
unsigned char revID;
unsigned char status1;
AccelTriplet_t accel_data;
short samples[512];
unsigned short fifo_entries;

typedef struct {
  float x;
  float y;
  float z;
}acceleration_G_t;

acceleration_G_t data_G;

void Set_Impact_Detection(void)
{
  adxl372_Reset(&adxl372);
  adxl372_Set_Op_mode(&adxl372, FULL_BW_MEASUREMENT);
  delay(500); //wait 370ms for data settlement
  
  adxl372_Set_Op_mode(&adxl372, STAND_BY);
  
  adxl372_Set_Autosleep(&adxl372, false);
  
  adxl372_Set_BandWidth(&adxl372, BW_3200Hz);
  
  adxl372_Set_ODR(&adxl372, ODR_6400Hz);
  
  adxl372_Set_WakeUp_Rate(&adxl372, WUR_52ms);
  
  adxl372_Set_Act_Proc_Mode(&adxl372, LOOPED);
  
  /* Set Instant On threshold */
  adxl372_Set_InstaOn_Thresh(&adxl372, ADXL_INSTAON_LOW_THRESH); //Low threshold 10-15 G
  
  /*Put fifo in Peak Detect and Stream Mode */
  adxl372_Configure_FIFO(&adxl372, FIFO_LENGTH, STREAMED, XYZ_FIFO);
  
  /* Set activity/inactivity threshold */
  adxl372_Set_Activity_Threshold(&adxl372, ACT_VALUE, true, true);
  adxl372_Set_Inactivity_Threshold(&adxl372, INACT_VALUE, true, true);
  
  /* Set activity/inactivity time settings */
  adxl372_Set_Activity_Time(&adxl372, ACT_TIMER);
  adxl372_Set_Inactivity_Time(&adxl372, INACT_TIMER);
  
  /* Configure interrupts*/
  adxl372_Set_Interrupts1(&adxl372, INTx_MAP_FIFO_FULL);
  
  /* Set filter settle time */
  adxl372_Set_Filter_Settle(&adxl372, FILTER_SETTLE_16);
  
  /* Set operation mode to Instant-On */
  adxl372_Set_Op_mode(&adxl372, INSTANT_ON);
}

void setup() {
  Serial.begin(9600);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0); //CPHA = CPOL = 0    MODE = 0
  delay(1000);

  pinMode(CS_PIN, OUTPUT);
  pinMode(INT1_ACC_PIN, INPUT);
  pinMode(INT2_ACC_PIN, INPUT);
  
  //adxl372_Reset(&adxl372);
  adxl372_Get_DevID_AD(&adxl372, &devId_AD);
  adxl372_Get_DevID_MST(&adxl372, &devId_MST);
  adxl372_Get_DevID_Product(&adxl372, &devID_Product);
  adxl372_Get_RevID(&adxl372, &revID);

  Serial.print("Device id: ");
  Serial.println(devId_AD, HEX);
  Serial.print("Mems id: ");
  Serial.println(devId_MST, HEX);
  Serial.print("Part id: ");
  Serial.println(devID_Product, HEX);
  Serial.print("Revision: ");
  Serial.println(revID, HEX);

  Set_Impact_Detection();
}

void loop() {
  
  if (digitalRead(INT1_ACC_PIN)) {
    //delay(500);
    //adxl372_Get_Accel_data(&adxl372, &accel_data);
    
    adxl372_Get_Status_Register(&adxl372, &status1);
    
    adxl372_Get_FIFO_data(&adxl372, samples, FIFO_LENGTH);


    // wait for end of shock because Arduino is slow with prints
    // let FIFO overrun or whatever. We will fix the overrun later
    delay(500);
    
    // this is status read earlier
    Serial.println("");
    Serial.print("Status1 read before measurements: ");
    Serial.println(status1, HEX);

    for(size_t i=0; i<FIFO_LENGTH; i++)
    {
      if(i%3 == 0)
      {
        Serial.println("");
      }
      Serial.print(samples[i]); Serial.print(" ");  
    }
    Serial.println("");

    // got back to instant ON and clean FIFO
    adxl372_Set_Op_mode(&adxl372, INSTANT_ON);
    adxl372_Get_Fifo_Entries(&adxl372, &fifo_entries);
    Serial.print("Fifo entries before cleaning: ");
    Serial.println(fifo_entries);

    short cnt = 0;
    // read 1 by 1 because SPI fails with many samples
    while(fifo_entries != 0)
    {
      adxl372_Get_FIFO_data(&adxl372, samples, 1);
      adxl372_Get_Fifo_Entries(&adxl372, &fifo_entries);
      cnt++;
    }
    Serial.print("Nb read samples to empty FIFO:"); Serial.println(cnt); 
    
    
    adxl372_Get_Fifo_Entries(&adxl372, &fifo_entries);
    Serial.print("Fifo entries after all read: ");
    Serial.println(fifo_entries);
    
    adxl372_Get_Status_Register(&adxl372, &status1);
    Serial.println("");
    Serial.print("Status1 read after fifo cleanup: ");
    Serial.println(status1, HEX);

    adxl372_Get_Status_Register(&adxl372, &status1);
    Serial.println("");
    Serial.print("Status1 read again after fifo cleanup: ");
    Serial.println(status1, HEX);

    Serial.println("-----------------------------------");
  }

}
