
/******************************************************************
 Created with PROGRAMINO IDE for Arduino - 28.12.2018 12:45:52
 Project     :MySensors Node Schuur ingangen RS485
 Libraries   :MySensors & Bounce2
 Author      :MySensors community & dzjr
 Description :
******************************************************************/

//#define MY_DEBUG

//RS 485 transport zaken
#define MY_RS485                  //RS-485 transport
#define MY_RS485_DE_PIN 11        //De DE pin benoemen, normaal is dit PIN-3
#define MY_RS485_BAUD_RATE 9600


//eerst wat administratie en dergelijk
#define SN "Schuur Alarm" //Software Naam
#define SV "1.0"      //Software versie
#define MY_NODE_ID 111 //Node nummer



//libaries activeren
#include <MySensors.h>
#include <Bounce2.h>



#define CHILD_ID_BIN1       11        //Child ID Schootcontact
#define CHILD_ID_BIN2       12        //Child ID Deurcontact
#define CHILD_ID_BIN3       13        //Child ID BIN-3
#define CHILD_ID_BIN4       14        //Child ID BIN-4
#define CHILD_ID_BIN5       15        //Child ID BIN-5
#define CHILD_ID_BIN6       16        //Child ID BIN-6
#define CHILD_ID_MQ135      20
#define CHILD_ID_MQ135_GAS  21        //Child ID MQ-135 sensor        
#define CHILD_ID_MQ135_NH3  22        //Child ID MQ-135 sensor     
#define CHILD_ID_MQ135_CH3  23        //Child ID MQ-135 sensor     
#define CHILD_ID_MQ135_TRIP 24        //Child ID MQ-135 grenswaarde
#define CHILD_ID_MQ2        25
#define CHILD_ID_MQ2_CO     26        //Child ID MQ-2 sensor
#define CHILD_ID_MQ2_ROOK   27        //Child ID MQ-2 sensor
#define CHILD_ID_MQ2_TRIP   28        //Child ID MQ-2 CO setpoint
#define CHILD_ID_PIR_ALARM  31        //Child ID PIR Beweging
#define CHILD_ID_PIR_FOUT   32        //Child ID PIR Sabotage of anders
#define CHILD_ID_ALARM_UIT  33        //Child ID alarm niet actief vanaf controller
#define CHILD_ID_RESET      35        //Child ID reset alarm vanaf controller
#define CHILD_ID_ALARM_EXT  36        //Child ID ALARM uit controller
#define CHILD_ID_DEURBEL    37        //Child ID voor de deurbel huis
#define CHILD_ID_BUZZ       38        //Child ID voor Buzzer status


#define MQ_135_PIN        A0         //Ingang MQ135 sensor
#define MQ_2_PIN          A2          //Ingang MQ2 sensor
#define PIR_ALARM_PIN     A5          //Pin voor het alarmcontact uit de PIR
#define PIR_FOUT_PIN  A6          //Pin voor het sabotage contact uit de PIR
//#define PIR_AUX_PIN       A7 //A6     //Pin voor het AUX of antimask contact uit de PIR
#define BIN1_PIN          2           //Pin voor Schootcontact
#define BIN2_PIN          3           //Pin Voor deurcontact
#define BIN3_PIN          4           //Pin reset knop
#define BIN4_PIN          5           //Pin licht status
#define BIN5_PIN          6           //Pin voor poortcontact
#define BIN6_PIN          7           //Pin voor schootcontact poort
// 8,9,10 & 11 zijn in gebruik voor de RS-485 interface
#define BEL_PIN           13          // Pin voor de deurbel
#define BUZZ_PIN          12          // Pin voor alarmbuzzer

//Binaire ingangen.
//bouncen!!!
Bounce debouncer1 = Bounce();
Bounce debouncer2 = Bounce();
Bounce debouncer3 = Bounce();
Bounce debouncer4 = Bounce();
Bounce debouncer5 = Bounce();
Bounce debouncer6 = Bounce();
//instellen oude waarde
int deuropenold=-1;
int schootold=-1;
int BIN3old=-1;
int BIN4old=-1;
int BIN5old=-1;
int BIN6old=-1;

boolean PIR_Inbraak=LOW;
boolean PIR_Inbraak_old=LOW;
boolean LichtAanold = LOW;
boolean PIR_ALARM=LOW;
boolean PIR_FOUT=LOW;
boolean PIR_FOUT_old=LOW;
boolean PIR_Sabotage = LOW;
boolean PIR_AUX = LOW;
boolean ALARM_UIT = LOW;
boolean ALARM_EXT = LOW;
//int AlarmMQ2 = LOW;
boolean AlarmStMQ2 = LOW;
boolean AlarmMQ135 = LOW;
boolean AlarmStMQ135 = LOW;
boolean lastAlarmMQ135 = LOW;
boolean ALARMSt_EXT =LOW;
boolean PIR_FOUTSt = LOW;
boolean Resetold = LOW;
boolean DEURBEL = LOW;
boolean AlarmOn = LOW;
boolean AlarmStatus =  LOW;
boolean AlarmOld = LOW;
boolean ResetIn = LOW;
bool initialValueSent = false;

MyMessage bin1_msg (CHILD_ID_BIN1, V_LOCK_STATUS);         // msg schootcontact
MyMessage bin2_msg (CHILD_ID_BIN2, V_TRIPPED);             // msg deurcontact
MyMessage bin3_msg (CHILD_ID_BIN3, V_STATUS);              // msg Reset knop
MyMessage bin4_msg (CHILD_ID_BIN4, V_STATUS);              // msg Licht aan
MyMessage bin5_msg (CHILD_ID_BIN5, V_LOCK_STATUS);         // msg schootcontact poort
MyMessage bin6_msg (CHILD_ID_BIN6, V_TRIPPED);             // msg poortcontact
MyMessage pcMsg_mq135(CHILD_ID_MQ135,V_VAR1);
MyMessage mq135gas_msg  (CHILD_ID_MQ135_GAS, V_LEVEL);
MyMessage mq135NH4_msg  (CHILD_ID_MQ135_NH3, V_LEVEL);
MyMessage mq135CH3_msg  (CHILD_ID_MQ135_CH3, V_LEVEL);
MyMessage mq135trip_msg (CHILD_ID_MQ135_TRIP, V_TRIPPED); // msg MQ135 drempelwaarde
MyMessage pcMsg_mq2(CHILD_ID_MQ2,V_VAR1);
MyMessage mq2co_msg (CHILD_ID_MQ2_CO, V_LEVEL);      // msg CO MQ-2 sensor
MyMessage mq2rook_msg (CHILD_ID_MQ2_ROOK, V_LEVEL); // msg Rook MQ-2 sensor
MyMessage mq2trip_msg (CHILD_ID_MQ2_TRIP, V_TRIPPED); //msg Tripped van MQ-2 sensor
MyMessage pir_alarm_msg (CHILD_ID_PIR_ALARM, V_TRIPPED);   // msg PIR Alarm
MyMessage pir_fout_msg (CHILD_ID_PIR_FOUT, V_TRIPPED);     // msg PIR Fout melding AM of sabotage
MyMessage alarm_uit_msg (CHILD_ID_ALARM_UIT, V_ARMED);     // msg Alarm niet actief uit controller
MyMessage alarm_ext_msg (CHILD_ID_ALARM_EXT, V_TRIPPED);   // msg inkomend alarm vanaf controller
MyMessage reset_msg (CHILD_ID_RESET, V_STATUS);            // msg voor externe reset
MyMessage deurbel_msg (CHILD_ID_DEURBEL, V_STATUS);        //msg voor de inkomende deurbel
MyMessage buzz_msg (CHILD_ID_BUZZ, V_STATUS);              // msg voor buzzer status





//MQ-2 gegevens (gekopieerd van MySensors Build-GAS)
#define         RL_VALUE                     (990)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (983)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
//which is derived from the chart in datasheet
/***********************Software Related Macros************************************/
#define         CALIBRATION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interval(in milliseconds) between each samples in the
//calibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interval(in milliseconds) between each samples in
//normal operation
/**********************Application Related Macros**********************************/
#define         GAS_LPG                      (0)
#define         GAS_H2                       (1)
#define         GAS_CO_sec                   (2)
#define         GAS_Smoke                    (3) 
#define         GAS_NH4                      (4)
#define         GAS_CH3                      (5)
#define         GAS_CO                       (6)
#define         GAS_C2H5OH                   (7)
#define         GAS_CH3_2CO                  (8)
#define         GAS_CO2                      (9)

/*****************************Globals***********************************************/
// Gekopieerd van empierre GITHUB
float           COCurve[2]      =  {37793.94418, -3.24294658};   //MQ2
float           H2Curve[2]      =  {957.1355042, -2.07442628};   //MQ2
float           LPGCurve[2]     =  {591.6128784, -1.679699732};  //MQ2
float           SmokeCurve[2]   =  {3426.376355, -2.225037973};  //MQ2
float           CO2Curve[2]     =  {113.7105289, -3.019713765};  //MQ135
float           CO_secCurve[2]  =  {726.7809737, -4.040111669};  //MQ135
float           NH4Curve[2]     =  {84.07117895, -4.41107687};   //MQ135
float           C2H50H_Curve[2] =  {74.77989144, 3.010328075};   //MQ135 
float           CH3Curve[2]     =  {47.01770503, -3.281901967};  //MQ135
float           CH3_2COCurve[2] =  {7.010800878, -2.122018939};  //MQ135
float           Ro              = 1000;

//VARIABLES
float Ro0 = 4.300;    //MQ2     3.83 this has to be tuned 10K Ohm
float RL0 = 2.897;    //MQ2     Elecfreacks Octopus
float Ro4 = 2.428;   //2.511;    //MQ135   2.51 this has to be tuned 10K Ohm
float RL4 = 0.358; //0.990;    //MQ135   FC-22
int val = 0;          // variable to store the value coming from the sensor


float valMQCO =0.0;
float lastMQCO =10.0;
float valMQRook =0.0;
float lastMQRook =10.0;
float valMQGas =0.0;
float lastMQGas =10.0;
float valMQNH4 =0.0;
float lastMQNH4 =10.0;
float valMQCH3 = 0.0;
float lastMQCH3 =10.0;
float AlarmMQ2 = LOW;
float lastAlarmMQ2 = 10;
float AlarmMQ5 = LOW;
float valMQCO2 = 0.0;

//float lastAlarmMQ5 = 0;

#define MQ2CO_SendTime    9000ul // every 15 minutes a sensor update
uint32_t lastsend_mq2CO;
#define MQ2SMOKE_SendTime    9000ul // every 15 minutes a sensor update
uint32_t lastsend_mq2SMOKE;
#define SendTimeMQ135    9000ul // every 15 minutes a sensor update
uint32_t lastSendMQ135;




void before()
{
  //wait(wachtTime);
}
void setup()
{
   Serial.print("Ro -->\n    MQ2:"); 
  Ro0 = MQCalibration(MQ_2_PIN,10,RL0,SmokeCurve);
 Serial.println(Ro0);
  send(pcMsg_mq2.set((long int)ceil(Ro0)));
 Serial.print("    MQ135:"); 
   Ro4 = MQCalibration(MQ_135_PIN,10,RL4,CO_secCurve);
   //Ro4 = -56.14;
 Serial.println(Ro4);
    send(pcMsg_mq135.set((long int)ceil(Ro4)));


  
  
  pinMode(BIN1_PIN,INPUT);
  pinMode(BIN2_PIN,INPUT);
  pinMode(BIN3_PIN,INPUT);
  pinMode(BIN4_PIN,INPUT);
  pinMode(BIN5_PIN,INPUT);
  pinMode(BIN6_PIN,INPUT);
  pinMode(BEL_PIN,OUTPUT);
  pinMode(BUZZ_PIN,OUTPUT);
  
  debouncer1.attach(BIN1_PIN);
  debouncer1.interval(15);
  debouncer2.attach(BIN2_PIN);
  debouncer2.interval(15);
  debouncer3.attach(BIN3_PIN);
  debouncer3.interval(15);
  debouncer4.attach(BIN4_PIN);
  debouncer4.interval(15);
  debouncer5.attach(BIN5_PIN);
  debouncer5.interval(15);
  debouncer6.attach(BIN6_PIN);
  debouncer6.interval(15);
  

}

void presentation()
{
  sendSketchInfo(SN,SV);

  {
  // Registreer alle sensors naar de Gateway
  present(CHILD_ID_BIN1, S_LOCK, "Schootcontact");
  present(CHILD_ID_BIN2, S_DOOR, "Deurcontact");
  present(CHILD_ID_BIN3, S_BINARY, "Reset knop");
  present(CHILD_ID_BIN4, S_BINARY, "Licht aan");
  present(CHILD_ID_BIN5, S_LOCK, "Schoor Poort");
  present(CHILD_ID_BIN6, S_DOOR, "Poortcontact");  
  
  present(CHILD_ID_MQ2_CO, S_AIR_QUALITY, "MQ-2 CO meting");
  present(CHILD_ID_MQ2_ROOK, S_AIR_QUALITY, "MQ-2 Rook meting"); 
  present(CHILD_ID_MQ2_TRIP, S_SMOKE, "MQ-2 TRIPPED");
  present(CHILD_ID_MQ135_GAS, S_AIR_QUALITY, "MQ-135 Gas");
  present(CHILD_ID_MQ135_NH3, S_AIR_QUALITY, "MQ-135 Alcohol"); 
  present(CHILD_ID_MQ135_CH3, S_AIR_QUALITY, "MQ-135 Amonnia");
  present(CHILD_ID_MQ135_TRIP, S_SMOKE, "MQ-135 TRIPPED");  
  present(CHILD_ID_PIR_ALARM, S_MOTION, "Beweging Schuur");
  present(CHILD_ID_PIR_FOUT, S_MOTION, "Sabotage of storing PIR");
  present(CHILD_ID_ALARM_UIT, S_MOTION, "ALARM Actief");
  present(CHILD_ID_RESET, S_BINARY, "Reset Schuur alarm");
  present(CHILD_ID_ALARM_EXT, S_SMOKE, "Extern alarm ");
  present(CHILD_ID_RESET, S_BINARY, "Reset Schuur alarm");
  present(CHILD_ID_DEURBEL, S_BINARY, "Deurbel naar schuur");
  present(CHILD_ID_BUZZ, S_BINARY, "Buzzer in schuur");
  }
}


void sendBinaryIn ()
{
  debouncer5.update();
  debouncer6.update();
   
  // Get the update value
  int BIN5 = debouncer5.read();
  int BIN6 = debouncer6.read(); 
 
  

  //int BIN5 = digitalRead(BIN5_PIN);
  
  //int BIN6 = digitalRead(BIN6_PIN);
  if (BIN5 != BIN5old) {
     // Send in the new value
     send(bin5_msg.set(BIN5==HIGH ? 1 : 0));
     BIN5old = BIN5;
  }

  if (BIN6 != BIN6old) {
     // Send in the new value
     send(bin6_msg.set(BIN6==HIGH ? 1 : 0));
     BIN6old = BIN6;
  }
  
  
}

void sendALARM()
{
    debouncer1.update();
    debouncer2.update();
    
    int PIR_ruw = analogRead(PIR_ALARM_PIN);
    int deuropen = debouncer1.read();
    int schoot = debouncer2.read();
      
  //int deuropen = digitalRead(BIN5_PIN);
    
  //int schoot = digitalRead(BIN5_PIN);

  if (!initialValueSent) {
    send(bin1_msg.set(deuropen==HIGH ? 1 : 0));
    send(bin2_msg.set(schoot==HIGH ? 1 : 0));
    request(CHILD_ID_BIN1, V_LOCK_STATUS);
    wait(2000, C_SET, V_LOCK_STATUS);
    request(CHILD_ID_BIN2, V_TRIPPED);
    wait(2000, C_SET, V_TRIPPED);
    
  }
  
    if (PIR_ruw > 600)  {
    (PIR_ALARM = HIGH);   
    }
    else {
    (PIR_ALARM = LOW);
    }
    if ((PIR_ALARM == HIGH || deuropen == LOW)  && ALARM_UIT == LOW) {
        PIR_Inbraak = HIGH;
        }
    
    if (PIR_Inbraak != PIR_Inbraak_old) {
    send(pir_alarm_msg.set(PIR_Inbraak==HIGH ? 1 : 0));
    PIR_Inbraak_old = PIR_Inbraak;
    }
  if (deuropen != deuropenold) {
     // Send in the new value
     send(bin1_msg.set(deuropen==HIGH ? 1 : 0));
     deuropenold = deuropen;
  }
  if (schoot != schootold) {
     // Send in the new value
     send(bin2_msg.set(schoot==HIGH ? 1 : 0));
     schootold = schoot;
  }  
    }

void sendSabotage()
{
    int PIR_Alarm_ruw = analogRead(PIR_ALARM_PIN);
  int PIR_Sabotage_ruw = analogRead(PIR_FOUT_PIN);
      if (PIR_Sabotage_ruw > 600)  {
    (PIR_Sabotage = HIGH);   
    }
    else {
    (PIR_Sabotage = LOW);
    }
      if (PIR_Alarm_ruw < 400 ||PIR_Sabotage_ruw < 400) //kabel defect detectie 
      {
    (PIR_AUX = HIGH);   
    }
    else {
    (PIR_AUX = LOW);
    }
  
    if ((PIR_Sabotage == HIGH || PIR_AUX == HIGH)&& ALARM_UIT == LOW){
        PIR_FOUT = HIGH;
        }
    else {
    PIR_FOUT = LOW;
    }
    
    if (PIR_FOUT != PIR_FOUT_old){
    send(pir_fout_msg.set(PIR_FOUT==HIGH ? 1 : 0));
    PIR_FOUT_old = PIR_FOUT;
    }
}
void sendMQ2()
{
    uint32_t currentMQ2 = millis(); 
    uint16_t valMQCO = (MQGetGasPercentage(MQRead(MQ_2_PIN,RL0),Ro0,GAS_CO,MQ_2_PIN) );
  
///   uint16_t valMQCO2 = (MQGetGasPercentage(MQRead(MQ_2_PIN,RL0),Ro0,GAS_CO,MQ_2_PIN) );
//   send(mq135gas_msg.set((int16_t)ceil(valMQCO2)));  
    //MQGetGasPercentage(MQRead(MQ2_PIN)/Ro,GAS_CO);

    /*Serial.println(val);

    Serial.print("LPG:");
    Serial.print(MQGetGasPercentage(MQRead(MQ2_PIN)/Ro,GAS_LPG) );
    Serial.print( "ppm" );
    Serial.print("    ");
    Serial.print("CO:");
    Serial.print(MQGetGasPercentage(MQRead(MQ2_PIN)/Ro,GAS_CO) );
    Serial.print( "ppm" );
    Serial.print("    ");
    Serial.print("SMOKE:");
    Serial.print(MQGetGasPercentage(MQRead(MQ2_PIN)/Ro,GAS_SMOKE) );
    Serial.print( "ppm" );
    Serial.print("\n");*/

    if ((valMQCO != lastMQCO)||(currentMQ2-lastsend_mq2CO > MQ2CO_SendTime)) {
        lastsend_mq2CO=currentMQ2; 
        send(mq2co_msg.set((int16_t)ceil(valMQCO)));
        lastMQCO = ceil(valMQCO);
    }


   

 uint16_t valMQRook = (MQGetGasPercentage(MQRead(MQ_2_PIN,RL0),Ro0,GAS_Smoke,MQ_2_PIN) );
 //MQGetGasPercentage(MQRead(MQ2_PIN)/Ro,GAS_SMOKE);

   if (!initialValueSent) {
    send(mq2rook_msg.set((int16_t)ceil(valMQRook)));
    request(CHILD_ID_BIN1, V_LEVEL);
    wait(2000, C_SET, V_LEVEL);
    
  }
 
    if ((valMQRook != lastMQRook )||(currentMQ2-lastsend_mq2SMOKE > MQ2SMOKE_SendTime)) {
        lastsend_mq2SMOKE=currentMQ2;
        send(mq2rook_msg.set((int16_t)ceil(valMQRook)));
        lastMQRook = ceil(valMQRook);
    }
    
    
    int MQ2_Value = analogRead(MQ_2_PIN);
  


    if (MQ2_Value > 200) {
      (AlarmMQ2 = HIGH); }
      else {
      (AlarmMQ2 = LOW); }
   
      if (AlarmMQ2 != lastAlarmMQ2 ) {
        send(mq2trip_msg.set(AlarmMQ2==HIGH ? 1 : 0));
        lastAlarmMQ2 = ceil(AlarmMQ2);
  
        }
      
    
    
}

void sendMQ135()
{

   uint16_t valMQCO2 = (MQGetGasPercentage(MQRead(MQ_135_PIN,RL4),Ro4,GAS_CO2,MQ_135_PIN) );
   uint16_t valMQC0  = (MQGetGasPercentage(MQRead(MQ_135_PIN,RL4),Ro4,GAS_CO_sec,MQ_135_PIN) );
   uint16_t valMQCH3 = (MQGetGasPercentage(MQRead(MQ_135_PIN,RL4),Ro4,GAS_CH3,MQ_135_PIN) );
   uint16_t valMQNH4 = (MQGetGasPercentage(MQRead(MQ_135_PIN,RL4),Ro4,GAS_NH4,MQ_135_PIN) );
   
   
   
unsigned long currentMQ135 = millis();  
      // Only send values at a maximum frequency or woken up from sleep
    if (currentMQ135 - lastSendMQ135 > SendTimeMQ135 )
    {
    lastSendMQ135 = currentMQ135;


//   uint16_t valMQCO2 = (MQGetGasPercentage(MQRead(MQ_135_PIN,RL0),Ro0,GAS_CO2,MQ_135_PIN) );
   send(mq135gas_msg.set((int16_t)ceil(valMQCO2)));
   //send(mq135co_msg.set((int16_t)ceil(valMQCO)));
  send(mq135CH3_msg.set((int16_t)ceil(valMQCH3)));
  send(mq135NH4_msg.set((int16_t)ceil(valMQNH4)));
  // send(pcMsg_mq135.set((long int)MQ_135_PIN));
}

      int MQ135_Value = analogRead(MQ_135_PIN);
      if (MQ135_Value > 200) {
      (AlarmMQ135 = HIGH); }
      else {
      (AlarmMQ135 = LOW); }
   
      if (AlarmMQ135 != lastAlarmMQ135 ) {
        send(mq135trip_msg.set(AlarmMQ135==HIGH ? 1 : 0));
        lastAlarmMQ135 = ceil(AlarmMQ135);
    }

  
  }

  


void deurbel(){

  debouncer4.update();
    
  // Get the update value
  int LichtAan = debouncer4.read();
  //int LichtAan = digitalRead(BIN4_PIN);
   if (!initialValueSent) {
    send(bin4_msg.set(LichtAan==HIGH ? 1 : 0));
    send(alarm_ext_msg.set(ALARM_EXT));  
    request(CHILD_ID_BIN1, V_STATUS);
    wait(2000, C_SET, V_STATUS);
        request(CHILD_ID_ALARM_UIT, V_ARMED);
    wait(2000, C_SET, V_ARMED);
    
    
  }
  
  if (LichtAan == HIGH && DEURBEL == HIGH){
      digitalWrite(BEL_PIN, HIGH);
      }
   else{
      digitalWrite(BEL_PIN, LOW);
      }

  if (LichtAan != LichtAanold) {
     // Send in the new value
     send(bin4_msg.set(LichtAan==HIGH ? 1 : 0));
     LichtAanold = LichtAan;
      }


}
void sendBuzzer()
  {
  
  debouncer3.update();
  int ResetBuzz = debouncer3.read();
    
//  int ResetBuzz = digitalRead(BIN3_PIN);    
  
  if (ResetBuzz != Resetold) {
     // Send in the new value
     send(reset_msg.set(ResetBuzz==HIGH ? 1 : 0));
     Resetold = ResetBuzz;
  }
  
  
  if (AlarmMQ2 == HIGH  && ResetBuzz == LOW) {
    AlarmStMQ2 = HIGH ;}
  if (AlarmMQ135 == HIGH  && ResetBuzz == LOW) {
    AlarmStMQ135 = HIGH ;}
  if (ALARM_EXT == HIGH  && ResetBuzz == LOW) {
    ALARMSt_EXT = HIGH ;}
  if (PIR_FOUT == HIGH  && ResetBuzz == LOW) {
    PIR_FOUTSt = HIGH ;}   
    
  if (AlarmMQ2 == LOW){
    AlarmStMQ2 = LOW;}
  if (AlarmMQ135 == LOW){
    AlarmStMQ135 = LOW;}
   if (ALARM_EXT == LOW){
    ALARMSt_EXT = LOW;}       
   if (PIR_FOUT == LOW){
    PIR_FOUTSt = LOW;}  
    
    if (AlarmMQ2 == HIGH && AlarmStMQ2 == LOW)  { 
    AlarmOn= HIGH;}
    else if 
    (AlarmMQ135 == HIGH && AlarmStMQ135 == LOW)  { 
    AlarmOn= HIGH;}
    else if 
    (ALARM_EXT == HIGH && ALARMSt_EXT == LOW)  { 
    AlarmOn= HIGH;}
    else if 
    (PIR_FOUT == HIGH && PIR_FOUTSt == LOW)  { 
    AlarmOn= HIGH;}    
    else
    {(AlarmOn = LOW); }
     
    if (AlarmStatus == HIGH || ResetBuzz == LOW) {
    AlarmOn = LOW;}
    
    
  if (AlarmOn != AlarmOld) {
     // Send in the new value
     send(buzz_msg.set(AlarmOn==HIGH ? 1 : 0));
     AlarmOld = AlarmOn;
  }  
  } 
  
void sendHeartbeat()
{
}

void loop()
{/*
unsigned long currentTime = millis();  
      // Only send values at a maximum frequency or woken up from sleep
    if (currentTime - lastSend > SEND_FREQUENCY)
    
    lastSend = currentTime;
  */  
    
{
  sendBinaryIn();
  sendALARM();
  sendSabotage();
  deurbel();
  sendBuzzer();
  sendMQ2();
  sendMQ135();
}
}




/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage across the load resistor and its resistance, the resistance of the sensor could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc,float rl_value)
{
  return  (long)((long)(1024*1000*(long)rl_value)/raw_adc-(long)rl_value);
;
}
 
/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air.        .
************************************************************************************/ 
float MQCalibration(int mq_pin, double ppm, double rl_value,float *pcurve )
{
  int i;
  float val=0;

  for (i=0;i<CALIBRATION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin),rl_value);
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBRATION_SAMPLE_TIMES;                   //calculate the average value
  //Ro = Rs * sqrt(a/ppm, b) = Rs * exp( ln(a/ppm) / b )

  return  (long)val*exp((log(pcurve[0]/ppm)/pcurve[1]));

}
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQRead(int mq_pin,float rl_value)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin),rl_value);
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 
/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
int MQGetGasPercentage(float rs_ro_ratio, float ro, int gas_id, int sensor_id)
{
  if (sensor_id == MQ_2_PIN ) {
    if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,ro,COCurve);      //MQ2
    } else if ( gas_id == GAS_H2 ) {
     return MQGetPercentage(rs_ro_ratio,ro,H2Curve);      //MQ2
    } else if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,ro,LPGCurve);     //MQ2
    } else if ( gas_id == GAS_Smoke ) {
     return MQGetPercentage(rs_ro_ratio,ro,SmokeCurve);   //MQ2
    }    
  }
 
   if (sensor_id == MQ_135_PIN ){
    if ( gas_id == GAS_CO2 ) {
     return MQGetPercentage(rs_ro_ratio,ro,CO2Curve);     //MQ135
    } else if ( gas_id == GAS_NH4 ) {
     return MQGetPercentage(rs_ro_ratio,ro,NH4Curve);     //MQ135
    } else if ( gas_id == GAS_C2H5OH ) {
     return MQGetPercentage(rs_ro_ratio,ro,C2H50H_Curve); //MQ135
    } else if ( gas_id == GAS_CH3 ) {
     return MQGetPercentage(rs_ro_ratio,ro,CH3Curve);     //MQ135
    } else if ( gas_id == GAS_CH3_2CO ) {
     return MQGetPercentage(rs_ro_ratio,ro,CH3_2COCurve); //MQ135
    } else if ( gas_id == GAS_CO_sec ) {
     return MQGetPercentage(rs_ro_ratio,ro,CO_secCurve);  //MQ135
    }
  return 0;
}
}
 
/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
int  MQGetPercentage(float rs_ro_ratio, float ro, float *pcurve)
{
  return (double)(pcurve[0] * pow(((double)rs_ro_ratio/ro), pcurve[1]));
}





void receive(const MyMessage &message)
{
 if (message.isAck()) {
     Serial.println("This is an ack from gateway");
  }
    
   //deurbel gaat ontvangen
      if (message.type==V_STATUS) { 
         if (!initialValueSent) {
      Serial.println("Receiving initial value from controller");
      initialValueSent = true;
    }
      if ( message.sensor==CHILD_ID_DEURBEL) {
    // Only switch if the state is new
    if (message.getBool() != DEURBEL) {
      DEURBEL = message.getBool();
       // Send the current state
      send(deurbel_msg.set(DEURBEL));  
      }
    }
}
 
  //alarm niet actief ontvangen
      if (message.type==V_ARMED) { 
      
      if ( message.sensor==CHILD_ID_ALARM_UIT) {
    // Only switch if the state is new
    if (message.getBool() != ALARM_UIT) {
      ALARM_UIT = message.getBool();
       // Send the current state
      send(alarm_uit_msg.set(ALARM_UIT));  
      }
    }
}

  //alarm niet actief ontvangen
      if (message.type==V_TRIPPED) { 
      if ( message.sensor==CHILD_ID_ALARM_EXT) {
    // Only switch if the state is new
    if (message.getBool() != ALARM_EXT) {
      ALARM_EXT = message.getBool();
       // Send the current state
      send(alarm_ext_msg.set(ALARM_EXT));  
      }
    }
}
    if (message.type==V_STATUS) { 
      if ( message.sensor==CHILD_ID_RESET) {
    // Only switch if the state is new
    if (message.getBool() != ResetIn) {
      ResetIn = message.getBool();
       // Send the current state
      send(reset_msg.set(ResetIn));  
      }
    }
}

  
  
}





//Eind van het verhaal
