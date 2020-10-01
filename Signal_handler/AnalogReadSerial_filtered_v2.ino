double value = 0;
const double gain = 0.25;
double prev = 0;
const unsigned long relax_time = 500;
const double treshold = -10;
unsigned long last_time = 0;

const double devider = 0.0033;

double median = 1.;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  
}

void loop() {
  if (Serial.available() > 0){
    char in = Serial.read();
    if (in == 'A')
      Serial.println(millis());
  }
  int sensorValue = analogRead(A0); // Входной сигнал
  double diff = (double)sensorValue - value; // Производная входного сигнала
  value += diff * gain; //Некий аналог low-pass фильтра 
  double overMedian = min(4 * median, diff * diff); //Здесь мы немного ограничиваем входной сигнал, чтобы пики не влияли на среднеквадратичное 
  median = (1 - devider) * median + devider * overMedian;
  double RMS = sqrt(median) * 1.6; //Среднеквадратичное
  
  double overRMS = max(RMS, abs(diff)); 
  double sourceSignal = RMS - overRMS; //Отсекаем весь среднеквадратичный шум
  
  if ((sourceSignal < treshold) && (millis() >= last_time + relax_time) ){
    last_time = millis();
    //Serial.println("Detected!");
    //Serial.println(millis());
    digitalWrite(2, HIGH);
    //delay(50);
    digitalWrite(2, LOW);
  }
  //Serial.print(int(100*diff));
  //Serial.print(", ");
  Serial.println(value);
  //Serial.print(", ");
  //Serial.println(sourceSignal * (diff / abs(diff)));  // Немного дебага
}
