/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/

//Stale ADC
const long AREF = 5000;
const float ADC_MAXVAL = 1023.0;
const int ANALOG_INPUTS = 8;

const int PROCENT = 100;

const int LED = 13;

//Stale pomiaru napiecia zasilania Vcc i Vbatt
const long Vcc_5V_VALUE = 5020;
const long Vcc_ZENER_VALUE = 3250;

const long Vbatt_RR_MULTIPLIER = 2;

//Stale zwiazane z usrednianiem
const int SIZE = 1; //max SIZE == 419
int val_tab[SIZE][ANALOG_INPUTS];
int iter_last;

//Podlaczenie pinow
const int POT1_PIN = A0;
const int POT1_ADC_INPUT = 0;
const int POT1_MAX_VAL = 4990;
const int POT1_VCC_PIN = 5;

const int POT2_PIN = A7;
const int POT2_ADC_INPUT = 7;
const int POT2_VCC_PIN = 4;
const int POT2_MAX_VAL = 5000;

const int Imcu_PIN = A1;
const int Imcu_ADC_INPUT = 1;
const int Idrv_PIN = A6;
const int Idrv_ADC_INPUT = 6;

const int Vcc_PIN = A2;
const int Vcc_ADC_INPUT = 2;

const int Vbatt_PIN = A3;
const int Vbatt_ADC_INPUT = 3;

//Podlaczenie sterownika silnika
const int PWM_MAX_PIN = 9;
const int DIR_MAX_PIN = 8;
const int nFAULT_MAX_PIN = 7;
const int nEN_MAX_PIN = 6;


//Parametry czujnikow pradu
const int Imcu_CUR_SENSIVITY = 132;//.6;//mV per Amper
const int Imcu_CUR_IMBALANCE = -0.00;
const int Idrv_CUR_SENSIVITY = 132;//.6;//mV per Amper
const int Idrv_CUR_IMBALANCE = -0.00;



// the setup routine runs once when you press reset:
void setup() {
  pinMode(LED,OUTPUT);
  pinMode(nFAULT_MAX_PIN,INPUT_PULLUP);
  
  pinMode(POT1_VCC_PIN, OUTPUT);
  digitalWrite(POT1_VCC_PIN, HIGH);
  pinMode(POT2_VCC_PIN, OUTPUT);
  digitalWrite(POT2_VCC_PIN, HIGH);

  pinMode(PWM_MAX_PIN, OUTPUT);
  pinMode(DIR_MAX_PIN, OUTPUT);
  //pinMode(eEN_MAX_PIN, OUTPUT);

  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:

  int val[ANALOG_INPUTS];
  {
    const int AVR_CNT = 100; //usrednianie pradu po AVR_CNT probek
    long suma[ANALOG_INPUTS];
    for(int input_no=0; input_no<ANALOG_INPUTS; input_no++) suma[input_no] = 0;
    for(int i=0; i<AVR_CNT; i++)
      for(int input_no=0; input_no<ANALOG_INPUTS; input_no++)
        suma[input_no]+=analogRead(input_no);
    for(int input_no=0; input_no<ANALOG_INPUTS; input_no++)
      val[input_no] = (suma[input_no]+AVR_CNT/2)/AVR_CNT;
  }

  if(iter_last < SIZE)
    for(int input_no=0; input_no<ANALOG_INPUTS; input_no++)
      val_tab[iter_last][input_no] = val[input_no];
  iter_last = (iter_last+1)%SIZE;
  
  float A[ANALOG_INPUTS];
  for(int input_no=0; input_no<ANALOG_INPUTS; input_no++){
    long suma = 0;
    for(int i=0;i<SIZE;i++)
      suma+=val_tab[i][input_no];
    A[input_no] = (suma * AREF) / (ADC_MAXVAL * SIZE);
  }

//----------------------------------------------------------POTs and PWM
  int pot1 = PROCENT * A[POT1_ADC_INPUT]/POT1_MAX_VAL;
  int pot2 = PROCENT * A[POT2_ADC_INPUT]/POT2_MAX_VAL;
const long MAX_PWM = 255L;
  int pwm = ((pot1 - PROCENT/2) * MAX_PWM) / (PROCENT/2);
  bool dir = pwm>0 ? 1 : 0 ;
  pwm = pwm>0 ? pwm : -pwm;

  digitalWrite(DIR_MAX_PIN, dir);
  if(pwm == 255) digitalWrite(PWM_MAX_PIN, HIGH);
  else if(pwm >128) analogWrite(PWM_MAX_PIN, pwm);
  else digitalWrite(PWM_MAX_PIN, LOW);



  digitalWrite(LED, not digitalRead(nFAULT_MAX_PIN));
    

//-----------------------------------------------------------Imcu
  int Imcu = A[Imcu_ADC_INPUT]-AREF/2;
  Imcu = ( Imcu*1000L ) / Imcu_CUR_SENSIVITY + Imcu_CUR_IMBALANCE;
  
  int Idrv = A[Idrv_ADC_INPUT]-AREF/2;
  Idrv = ( Idrv*1000L ) / Idrv_CUR_SENSIVITY + Idrv_CUR_IMBALANCE;

//-----------------------------------------------------------VCC and Vbatt
  int Vcc = Vcc_5V_VALUE + Vcc_ZENER_VALUE - A[Vcc_ADC_INPUT];
  int Vbatt = A[Vbatt_ADC_INPUT] * Vbatt_RR_MULTIPLIER;
  
//----------------------------------------------------------SERIAL PRINT
  Serial.print("POT1: ");
  Serial.print(pot1);
  Serial.print("%,\tPOT2: ");
  Serial.print(pot2);
  Serial.print("%,\tPWM: ");
  Serial.print(pwm);
  Serial.print( dir ? " ->" : " <-");
  
  Serial.print("\tImcu: ");  
  Serial.print(Imcu);
  
  Serial.print("mA\tIdrv: ");
  Serial.print(Idrv);

  //Serial.print("mA,\tADC[Vcc_ADC_INPUT: ");
  //Serial.print(analogRead(Vcc_ADC_INPUT));
  Serial.print("mA,\tVCC: ");
  Serial.print(Vcc);
  
  Serial.print("mV,\tVbatt: ");
  Serial.print(Vbatt);
  
  Serial.print("mV\n");


  delay(1);        // delay in between reads for stability
}
