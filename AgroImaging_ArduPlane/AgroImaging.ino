
#define CAM_TRIGGER_PIN 17
#define TRIGGER_HIGH_TIME 1  // milliseconds
#define TRIGGER_FREQ 1  // Hz 

#define LOG_TRIGGER_CAMERA_MSG 0x0C

float RH = 60;
float T_C = 61;

long previousMillis = 0;        // will store last time LED was updated
int8_t Tetracam_trigger = 0;
long Tetracam_trigger_time_ms = 0;

static int trigger_flag = 1;
int val = 0;
float V = 0;
static long prev_millis = 0;
long time_millis = 0;
static AP_HAL::AnalogSource *TTC_Busy_line;

void AgroImagingSetup(void) {
  TTC_Busy_line->set_pin(1);

  
}

void AgroImagingExecute_Fast(void){
  
  time_millis = millis();
  float V = TTC_Busy_line->voltage_average() ;
  
  if( V >= 1.0 &&  trigger_flag == 1 && (time_millis - prev_millis) > 200){
   prev_millis = time_millis; 
   Tetracam_trigger = 1;
   Tetracam_trigger_time_ms = millis();
   Roll_servo = 100;
   trigger_flag = 0;
//   digitalWrite(A_LED_PIN, LED_ON);
  }
  
  
  
  
  
//  val = analogRead(1);
//  V = (val/1023.0)*5.0;

//  
//  if( V >= 2.0 &&  trigger_flag == 1 && (time_millis - prev_millis) > 200){
//   prev_millis = time_millis; 
//   Tetracam_trigger = 1;
//   Tetracam_trigger_time_ms = millis();
//   Roll_servo = 100;
//   trigger_flag = 0;
//   time_stamp = millis(); 
//   digitalWrite(A_LED_PIN, LED_ON);
//  }
  
  

  
}


void AgroImagingExecute(void) {
  
//  if ( (digitalRead(CAM_TRIGGER_PIN) == HIGH) && rising_flag == 1) {
////    Serial.print("RISING EDGE!!");
////    Serial.print("\n");
//    Log_Write_TriggerCamera();
//    digitalWrite(16,HIGH);
//    
//    trigger_flag = 0;
//  }
//  
//  if (digitalRead(CAM_TRIGGER_PIN) == LOW && trigger_flag == 0){
//    trigger_flag = 1;
//    digitalWrite(16,LOW);
////    Serial.print("DOWN EDGE!!");
////    Serial.print("\n");
//  }
  

  
  
  //Log_Write_TriggerCamera();
  
//  if( V <= 1.5 &&  trigger_flag  == 0 ){
//    trigger_flag = 1;
//    Tetracam_trigger = 0;
//    Roll_servo = 0;
//    digitalWrite(A_LED_PIN, LED_OFF);
//  }
// 

}

void GetHumidTemp(void)
{
//   int8_t _status;
//   unsigned int H_dat, T_dat;
//
//   
//
//      _status = fetch_humidity_temperature(&H_dat, &T_dat);
//      
////      switch(_status)
////      {
////          case 0:  Serial.println("Normal.");
////                   break;
////          case 1:  Serial.println("Stale Data.");
////                   break;
////          case 2:  Serial.println("In command mode.");
////                   break;
////          default: Serial.println("Diagnostic."); 
////                   break; 
////      }       
//    
//      RH = (float) H_dat * 6.10e-3;
//      T_C = (float) T_dat * 1.007e-2 - 40.0;

//      print_float(RH, 1);
//      Serial.print("  ");
//      print_float(T_C, 2);
//      Serial.println();
//      delay(1000);
}

int8_t fetch_humidity_temperature(unsigned int *p_H_dat, unsigned int *p_T_dat)
{
//      uint8_t address;
//      int8_t Hum_H, Hum_L, Temp_H, Temp_L, _status;
//      unsigned int H_dat, T_dat;
//      address = 0x27;
//      I2c.beginTransmission(0x27); 
//      I2c.endTransmission();
//      delay(1);
//
//
//      I2c.read(0x27,4);
//      Hum_H = I2c.receive();
//      Hum_L = I2c.receive();
//      Temp_H = I2c.receive();
//      Temp_L = I2c.receive();
//      I2c.endTransmission();
//      
//      _status = (Hum_H >> 6) & 0x03;
//      Hum_H = Hum_H & 0x3f;
//      H_dat = (((unsigned int)Hum_H) << 8) | Hum_L;
//      T_dat = (((unsigned int)Temp_H) << 8) | Temp_L;
//      T_dat = T_dat / 4;
//      *p_H_dat = H_dat;
//      *p_T_dat = T_dat;
//      return(_status);
}

//void TriggerCamera(void) {
//  unsigned long currentMillis = millis();
//  if( currentMillis - previousMillis > (1/TRIGGER_FREQ)*1000 ) {
//    previousMillis = currentMillis;  
//    // save the last time you blinked the LED 
//    digitalWrite(CAM_TRIGGER_PIN, HIGH);
//    delay(TRIGGER_HIGH_TIME);
//    digitalWrite(CAM_TRIGGER_PIN, LOW);
//  }
//}

//// Write an AgroImaging data packet to memory.
//static void Log_Write_TriggerCamera(void)
//{
//    DataFlash.Writeint8_t(HEAD_int8_t1);
//    DataFlash.Writeint8_t(HEAD_int8_t2);
//    DataFlash.Writeint8_t(LOG_TRIGGER_CAMERA_MSG);
//    
//    DataFlash.WriteLong(((long)CPU_time_ms));
//    DataFlash.WriteLong(((long)GPS_time_ms));
//    
//    DataFlash.Writeint8_t(g_gps->fix);
//    DataFlash.Writeint8_t(g_gps->num_sats);
//    DataFlash.WriteLong((long)loc.lat);
//    DataFlash.WriteLong((long)loc.lng);
//    DataFlash.WriteLong((int32_t)(100.0*Z_position_GPS));
//    
//    DataFlash.WriteLong((int32_t)(100.0*ToDeg(roll)));
//    DataFlash.WriteLong((int32_t)(100.0*ToDeg(pitch)));
//    DataFlash.WriteLong((int32_t)(100.0*ToDeg(yaw)));
//    
//    DataFlash.WriteLong((int32_t)(100.0*Z_position_Baro));
//    DataFlash.WriteInt((int16_t)(100.0*ground_speed));
//    DataFlash.WriteLong((int32_t)(100.0*ToDeg(ground_course)));
//    
//    DataFlash.WriteLong((int32_t)(100.0*RH));
//    DataFlash.WriteLong((int32_t)(100.0*T_C));
//    
//    DataFlash.Writeint8_t(Tetracam_trigger);
//    DataFlash.WriteLong(((long)Tetracam_trigger_time_ms));
//    
//       
//    DataFlash.Writeint8_t(END_int8_t);
//}
//
//
//// Read an attitude packet
//static void Log_Read_TriggerCamera()
//{
//  
//    static int flag = 1;
//    if (flag == 1){
//      // Printing Header for AgroImaging data log
//      Serial.printf_P(PSTR("TRIGGERCAM, CPU_time_ms, GPS_time_ms, "));
//      Serial.printf_P(PSTR("GPS_Fix, Num_Sats, Latitude_deg, Longitude_deg, Z_position_GPS_cm, "));
//      Serial.printf_P(PSTR("roll_centiDeg, pitch_centiDeg, yaw_centiDeg, " ));
//      Serial.printf_P(PSTR("Z_position_Baro_cm, GroundSpeed_cm_s, GroundCourse_centiDeg, " ));
//      Serial.printf_P(PSTR("Relative_Humidity_centi, Temperature_centiDegC, " ));
//      Serial.printf_P(PSTR("Tetracam_trigger, Tetracam_trigger_ms, " ));
//      
//      
//      Serial.printf_P(PSTR("\n"));
//      flag = 0;
//    }
//    Serial.printf_P(PSTR("TRIGGERCAM, "));
//    
//    int8_t b[6];
//    int16_t i[26];
//    int32_t l[16];
//    
//    l[0] = DataFlash.ReadLong();  // CPU time
//    l[1] = DataFlash.ReadLong();  // GPS time
//    Serial.printf_P(PSTR("%lu, %lu, "),
//                    (long)l[0], (long)l[1]);
//    
//    b[0] = DataFlash.Readint8_t(); // GPS fix
//    b[1] = DataFlash.Readint8_t(); // GPS num sats
//    l[13] = DataFlash.ReadLong(); // Latitude
//    l[14] = DataFlash.ReadLong(); // Longitude
//    l[5] = DataFlash.ReadLong();  // Z_position_GPS
//    Serial.printf_P(PSTR("%d, %d, %4.7f, %4.7f, %ld, "), (int)b[0], (int)b[1], l[13]/t7, l[14]/t7, l[5]);
//
//    l[7] = DataFlash.ReadLong(); // roll
//    l[8] = DataFlash.ReadLong(); // pitch
//    l[9] = DataFlash.ReadLong(); // yaw
//    
//    Serial.printf_P(PSTR("%ld, %ld, %ld, "), l[7], l[8], l[9]);
//    
//    l[4] = DataFlash.ReadLong();  // Z_position_Baro
//    i[6] = DataFlash.ReadInt();  // ground speed
//    l[6] = DataFlash.ReadLong(); // ground course
//    
//    Serial.printf_P(PSTR("%ld, %d, %ld, "), l[4], i[6], l[6]);
//    
//    l[8] = DataFlash.ReadLong(); // Relative Humidty
//    l[9] = DataFlash.ReadLong(); // Temperature
//    
//    Serial.printf_P(PSTR("%ld, %ld, "), l[8], l[9]);
//    
//    b[0] = DataFlash.Readint8_t(); // Tetracam trigger
//    l[0] = DataFlash.ReadLong();  // CPU time
//    Serial.printf_P(PSTR("%d, %lu, "), (int)b[0], (long)l[0]);
//    
//
//    Serial.printf_P(PSTR("\n"));
//}


