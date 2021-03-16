#include <math.h>

const int led_pin=11;
const int sig_pin = A0;

double value = 0.0;
const double gain = 0.25;
double prev = 0.0;
const unsigned long relax_time = 500;
const double treshold = -10;
unsigned long last_time = 0;

const unsigned int interval = 10.;
const unsigned int wait_time = 20;
double Amin = 100000.0;
double Amax = -100000.0;
double tmax, tmin, t_rising, t_falling;
bool rising_edge, falling_edge;
double integral = 0.0;
double d_value;
bool detect = false;

double R = 43.0; // Resistor in low-pass-filter in ligth-emittin diod
double tau = 10.;

double t, t1, t2;
double sensorValue;

const int calib_size = 100; // Calibration time
double noise = 0.0;

const double devider = 0.0033;

double median = 1.;

// the setup routine runs once when you press reset:
void setup() {
    // initialize serial communication at 9600 bits per second:
    Serial.begin(115200);
    Serial.flush();
    //Serial.println("VAL, DET, RIS, FALL"); // for debug 3rd algorith
    //Serial.println("SensVal, Val, SrcSig");
    pinMode(2, OUTPUT);
    pinMode(led_pin, OUTPUT);
  
    //// Calibration
    double calib_arr[2][calib_size];
    double sensorValue;
    double STD_2 = 0.0;
    double mean = 0.0;
    for (int i=0; i<calib_size; i++)
    {
      sensorValue = analogRead(sig_pin)/100.0;
      calib_arr[0][i] = millis();
      delay(interval);
      calib_arr[1][i] = sensorValue;
      mean += sensorValue;
      
      /*
      Serial.print(i);
      Serial.print(" ,");
      Serial.println(sensorValue);
      */
    }
    mean /= (double)calib_size;
    //Serial.print("mean is ");
    //Serial.println(mean);
    for (int i=0; i<calib_size; i++)
    {
      //Accumulate squared deviation from meaт
      STD_2 += (calib_arr[1][i] - mean)*(calib_arr[1][i] - mean);
      //Serial.println(STD_2);
    }
    //Serial.println(STD_2);
    STD_2 /= (double)(calib_size-1);
    //Serial.println(STD_2);
    STD_2 = sqrt(STD_2);
    //Serial.println(STD_2);
    //STD_2 /= (double)sqrt(calib_size);
    noise = STD_2*100.0;
    //Serial.print("The noise size is ");
    //Serial.println(noise);
    //// Noise is STD = sqrt(RMS)
  
    // Calibration are going not so good so decided to set noise manually
    noise = 4;
  
}

void loop() {
    if (Serial.available() > 0){
      char in = Serial.read();
      if (in == 'A') // it's a message sending from python code
        Serial.println(millis());
    }

    //Controliing a IR diod 
    digitalWrite(led_pin, HIGH);
    
    ////// First algorith - use a filter (by Sergei Stetskii)
    /*
    delay(10);
    int sensorValue = analogRead(sig_pin); // Входной сигнал
    double diff = (double)sensorValue - value; // Производная входного сигнала
    value += diff * gain; //Некий аналог low-pass фильтра 
    double overMedian = min(4 * median, diff * diff); //Здесь мы немного ограничиваем входной сигнал, чтобы пики не влияли на среднеквадратичное 
    median = (1 - devider) * median + devider * overMedian;
    double RMS = sqrt(median) * 1.6; //Среднеквадратичное
    
    double overRMS = max(RMS, abs(diff)); 
    double sourceSignal = RMS - overRMS; //Отсекаем весь среднеквадратичный шум
    */
    ////////////////////////////////////////////////
    
    //////// Алгоритм обнаружения пролета
    /*
    if ((diff < treshold) && (millis() >= last_time + relax_time) ){
      last_time = millis();
      //Serial.println("Detected!");
      //Serial.println(diff);
      //Serial.println(millis());
    }
    // debug
    Serial.print(sensorValue);
    Serial.print(" "); 
    Serial.print(value);
    Serial.print(" "); 
    Serial.println(sourceSignal+900);
    
    
    
    //////////////////////////////////////////////////////
      
    ///////////////  Второй алгоритм - по разнице амплитуд
    /*
    sensorValue = analogRead(sig_pin); // Входной сигнал 
    
    t1 = t2;
    t2 = millis();
    t += t2 - t1;
    
    //// equation for low-pass-filter
    d_value = 1/tau * (sensorValue - value)*(t2-t1);
    value += d_value;
    
    if (sensorValue > Amax) {
      Amax = sensorValue;
      tmax = t;
    }
    
    if (sensorValue < Amin) {
      Amin = sensorValue;
      tmin = t;
    }
    */
    //////////////////////////////////////////////////
    
    /*
    if (abs(Amin - Amax) > 3*noise ) {
      Amax = -10000;
      Amin = 10000;
      if ( tmax < tmin )
      {
        rising_edge = true;  
        t_rising = t;
      }
      else if ( tmax > tmin ) 
      {
        falling_edge = true;
        t_falling = t;
      }
      
      if (t_falling>0 & t_rising>0 )
      {
        // Found rising and falling edges
        Serial.print("Detected : time is ");
        Serial.println(t); 
        //Serial.print(", ampitude is ");
        //Serial.println(abs(Amax - Amin));
        Serial.print(", noise is ");
        Serial.println(noise);
        
        delay(10*interval);  
        tmax=0;
        tmin=0;
        t_rising=0;
        t_falling=0;
      }
    }
    */

    //////////////////////// Третий алгоритм - имитация обычного low-pass фильтра
    
    sensorValue = analogRead(sig_pin); // Входной сигнал 
    
    t1 = t2;
    t2 = millis();
    t += t2 - t1;
      
    //// equation for low-pass-filter
    d_value = 1/tau * (sensorValue - value)*(t2-t1);
    value += d_value;
    
    //delay(interval);

    //debug in plotter
    //Serial.print(sensorValue);
    //Serial.print(" ");
    //Serial.println(value);
    //Serial.print(" ");
    //Serial.print(detect*100);
    //Serial.print(" ");
    //Serial.print(rising_edge*200);
    //Serial.print(" ");
    //Serial.println(falling_edge*300);
    detect = false;
    
    /////// Обработка сигнала
    
    
    if (value > Amax & t>3000) {
      Amax = value; 
      tmax = t;
    }
    if (value < Amin & t>3000) {
      Amin = value; 
      tmin = t;
    }
    
    //////////////////////////////////////////
    
    
    //Serial.print(Amax);
    //Serial.print(" ");
    //Serial.println(Amin);
    //Serial.println(t);
    //Serial.println(tmin);
    //Serial.println(tmax);
    
    
    
    if (abs(Amax - Amin) > 3*noise ) {
      if (falling_edge)
      {
        if ( tmax > tmin ) {
          rising_edge  = true;
        }
      }  
      else if ( tmax < tmin )
      {
        falling_edge = true;
      }

      tmax = 0.0;
      tmin = 0.0;
      if ( rising_edge & falling_edge  )
      {
        // Found rising and falling edges
        
        //Serial.print("Detected : time is ");
        //Serial.println(t); 
        //Serial.print(", ampitude is ");
        //Serial.println(abs(Amax - Amin));
        //Serial.print(", noise is ");
        //Serial.println(noise);
        
        detect = true;
        // Send time by serial port to GUI
        Serial.println(millis());
        delay(1000); // for example  
        t2= millis();
        tmax=0.0;
        tmin=0.0;
        rising_edge = false;
        falling_edge = false;
      }
      Amax = -10000;
      Amin = 10000;
    
    }
    
    /////////////////////////////////////////////
    
    /////////////////////////////////////////////

    /////////////// Строки для дебага ///////////
    
    //Serial.println(sensorValue);
    //Serial.println(diff);
    //Serial.print(int(100*diff));
    //Serial.print(", ");
    //Serial.println(value);
    //Serial.print(", ");
    //Serial.println(sourceSignal * (diff / abs(diff)));  // Немного дебага
    //Serial.println(sourceSignal);
}
