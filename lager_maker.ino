/*
Arduino thermostatic relay controller
*/

#define THERM_OFFSET 5.0 //adjust the temp reading
#define THERM_RESISTOR 9950 //as measured w/ a multimeter (thermistor only) - should be around 10k
#define VDIV_R1 10700 //as measured w/ a multimeter for "voltage" circuit - should be around 10k
#define VDIV_R2 340 //as measured w/ a multimeter for "voltage" circuit - should be around 330

//time constants (all in milliseconds)
const unsigned long second1 = 1000;
const unsigned long minute1 = second1 * 60;
const unsigned long hour1 = minute1 * 60;
const unsigned long day1 = hour1 * 24;
const unsigned long week1 = day1 * 7;

//component constants
//for Steinhart-Hart equation
const double A = 0.0011468093968495;
const double B = 0.00023524863530327;
const double C = 5.0580813554846e-8;

//for beta parameter equation
const int betaParam = 4038.0;
double rinfinit = 0.013186042797417;

const double dissFactor = 6.5; // mW/dec C – dissipation factor

//circuit constants
const int thermPin = 0;     //pin the thermistor is attached to
const int tmp36Pin = 1;     //pin the tmp36 sensor is attached to
const int voltagePin = 2;   //
const int relayPin = 13;    //pin that turns the relay on or off
const int buttonPin = 2;    //pin for button
const int speakerPin = 9;   //pin for buzzer

//set to false if controlling heater
//set to true if controlling cooler
const boolean cool = true;

//temperature constants
//adjust these if cycling on/off too fast/slow
const int degreeBuffer = 1;
const unsigned long readDelay = 2 * second1;

//stage definition - do not modify
typedef struct {
  int tempStart;
  int tempEnd;
  unsigned long duration;
} stage;

//setup stage details here
const int stageNum = 5;
stage stages[stageNum] = {
  //primary fermentation
  { 56, 56, 2* week1 },
  //split secondary fermentation into 3 stages
  { 56, 49, week1 },
  { 49, 42, week1 },
  { 42, 35, week1 },
  //conditioning (ready to drink)
  { 35, 35, 3 * week1 }
};

//button hold constants
const long debounce = 20; // ms debounce period to prevent flickering when pressing or releasing the button
const long holdTime = 3 * second1; // how long to wait for press+hold event

//notes & tones
const char names[] = { 'c', 'd', 'e', 'f', 'g', 'a', 'b', 'C' };
const int tones[] = { 1915, 1700, 1519, 1432, 1275, 1136, 1014, 956 };

//runtime variables
double voltage = 5.0;
int stageCurrent = -1; //start at -1, nextStage() will advance us to stage[0] (1st stage)
boolean theEnd = false;
unsigned long stageStart;
unsigned long lastRead = 0;
double tempStep;
double currentTemp;
int targetTemp;
int buttonState;
int buttonLastState = 0; // buffered value of the button's previous state
long btnDnTime; // time the button was pressed down
long btnUpTime; // time the button was released
boolean ignoreUp = true; // whether to ignore the button release because the click+hold was triggered

void setup() {
  
  pinMode(relayPin, OUTPUT);      
  pinMode(buttonPin, INPUT);  
  pinMode(speakerPin, OUTPUT);
  pinMode(voltagePin, INPUT);
  
  // We'll send debugging information via the Serial monitor
  Serial.begin(9600); 

  //set the voltage to 1.1v and then use a divider to determine our actual voltage  
  analogReference(INTERNAL);
  
  //read the voltage a few times until we get two of the same readings
  voltage = readVoltage();
  double oldVoltage = 0.0;
  while(oldVoltage != voltage) {
     delay(4 * second1); // wait
     oldVoltage = voltage;
     voltage = readVoltage(); 
  }
  Serial.print("Settled on Voltage=");
  Serial.println(voltage, 2);
  
  //set back to (around) 5.0v
  analogReference(DEFAULT); 
  // or if you want to use 3.3v for a tmp36
  //analogReference(EXTERNAL);
    
  //start with the first stage
  nextStage();
}

double readVoltage(){
  //use a voltage divider (R1 = 10k, R2 = 330) against 1.1v 
  //to figure out what our actual "5v" reference voltage is
  int sensorValue = analogRead(voltagePin);  
  return (sensorValue/1024.0)*((VDIV_R1 / VDIV_R2) * 1.1);
}


void loop() {
  //check & do button functions
  buttonRead();

  if(stageStart + stages[stageCurrent].duration > millis()) { //active stage
    unsigned long elapsed = millis() - lastRead;
      
    if(elapsed > readDelay) {
      lastRead = millis();
      
      //Serial.println("tmp36");
      //tmp36Read();
      Serial.println("therm");
      currentTemp = thermRead();
      regulateTemp();
    }
  } else { //end of stage
    if(stageCurrent == stageNum - 1) { //end of all stages
      if(!theEnd) {
        theEnd = true;
        Serial.println("End of last stage");
      }        
    } else {
      nextStage();
      report(); 
    }    
  }
}

double tmp36Read() {
  int sensorValue = analogRead(tmp36Pin);  
  // converting that reading to voltage, which is based off the reference voltage
  double tmpVolt = sensorValue * voltage;
  tmpVolt /= 1024.0; 
  return toF((tmpVolt - 0.5) * 100);  //converting from 10 mv per degree wit 500 mV offset
                                      //to degrees ((volatge - 500mV) times 100)  
}

double steinhartHart(double R) {
  //Steinhart-Hart thermistor equation, this gives us temperature in Kelvin
  return 1 / (A + B * log(R) + C * pow(log(R), 3));   
}

double beta(double R) {
  return betaParam/log(R/rinfinit);
}

double dissipation(double R) {
   return pow(voltage, 2)/(dissFactor * R);
}

double thermRead() { 
  int sensorValue = analogRead(thermPin);

  double Vout = sensorValue * (voltage/1024); //convert the ADC reading from the analog pin into a voltage. 
  //We'll need this to calculate the thermistor's resistance next
  
  //calculate resistance from the analogread value. See this page for more info: http://en.wikipedia.org/wiki/Voltage_divider
  double R1 = ((THERM_RESISTOR * voltage) / (Vout)) - THERM_RESISTOR;
  
  //show the beta calc'ed temp
  double temp = beta(R1) - dissipation(R1);
  temp += -273.15; // Convert Kelvin to Celcius
  //Serial.println("Beta");
  return toF(temp);  
  
  //show the SH calc'ed temp
  /*
  temp = steinhartHart(R1) - dissipation(R1);   
  temp += -273.15; // Convert Kelvin to Celcius  
  Serial.println("Steinhart-Hart");
  return toF(temp);
  */
}

double toF(double temperatureC) {
  Serial.print(temperatureC); Serial.println(" degrees C");
 
  // now convert to Fahrenheight
  double temp = (temperatureC * 9.0 / 5.0) + 32.0 + THERM_OFFSET;
  Serial.print(temp, 2); Serial.println(" degrees F (corrected)");
  return temp;
}

void regulateTemp() {
  calculateTemp();
  
  if(abs(currentTemp - targetTemp) < degreeBuffer) { //If approx. temp is within range of desired temp
    digitalWrite(relayPin, LOW); //err on this side of energy savings
    return;
  }    
        
  if(currentTemp < targetTemp) { //If below target temp
    if(cool)
      digitalWrite(relayPin, LOW); 
    else
      digitalWrite(relayPin, HIGH); 
  } else if(currentTemp > targetTemp) { //If above target temp
    if(cool)
      digitalWrite(relayPin, HIGH);
    else
      digitalWrite(relayPin, LOW); 
  }
}

void calculateTemp() {
  //do casting here or the compiler will do it for you - incorrectly
  targetTemp = stages[stageCurrent].tempStart + ((double)(millis() - stageStart) * tempStep);
  Serial.print("Target: "); Serial.println(targetTemp);  
}

void nextStage() {
  if(stageCurrent >= stageNum - 1) {
     stageCurrent = 0; 
  } else {
     stageCurrent++; 
  }
    
  theEnd = false; //incase we're restarting after the last stage
  Serial.print("Current Stage: "); Serial.println(stageCurrent + 1);
  stageStart = millis();
  Serial.print("Temp start: "); Serial.println(stages[stageCurrent].tempStart);
  Serial.print("Temp end: "); Serial.println(stages[stageCurrent].tempEnd);
  //do casting here or the compiler will do it for you - incorrectly
  tempStep = (double)(stages[stageCurrent].tempEnd - stages[stageCurrent].tempStart) / stages[stageCurrent].duration;
  calculateTemp();
}

//button function
void buttonRead() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // Test for button pressed and store the down time
  if(buttonState == LOW && buttonLastState == HIGH && (millis() - btnUpTime) > debounce) {
    btnDnTime = millis();
  }

  // Test for button release and store the up time
  if(buttonState == HIGH && buttonLastState == LOW && (millis() - btnDnTime) > debounce)
  {
    if(ignoreUp == false)
      report(); //do short press activity
    else 
      ignoreUp = false;
    btnUpTime = millis();
  }

  // Test for button held down for longer than the hold time
  if(buttonState == LOW && (millis() - btnDnTime) > holdTime)
  {
    nextStage();
    report();
    ignoreUp = true;
    btnDnTime = millis();
  }

  buttonLastState = buttonState;
  
}

//speaker functions
void report() {
  if(theEnd) {
    //two quick tones if we're at the end
    playNote('f', 50);
    delay(50);
    playNote('f', 50);
  } else {
    //two long F tones for the stage number
    soundOff(stageCurrent + 1, 'f', second1);
  
    //current temp
    //long tones for first digit
    //short tones for second digit
    //low C
    delay(second1);
    int firstDigit = (int)currentTemp / 10;
    soundOff(firstDigit, 'c', second1);
    delay(.75 * second1);
    int secondDigit = (int)currentTemp % 10;
    soundOff(secondDigit, 'c', .5 * second1);
    
    //target temp
    //long tones for first digit
    //short tones for second digit
    //high C
    delay(second1);
    firstDigit = (int)targetTemp / 10;
    soundOff(firstDigit, 'C', second1);
    delay(.75 * second1);
    secondDigit = (int)targetTemp % 10;
    soundOff(secondDigit, 'C', .5 * second1);
  }
}

void soundOff(int tones, char note, int duration) {
  for(int x = 0; x < tones; x++) {
    playNote(note, duration); 
    delay(.5 * second1);
  }
}

void playTone(int tone, int duration) {
  for(long i = 0; i < duration * 1000L; i += tone * 2) {
    digitalWrite(speakerPin, HIGH);
    delayMicroseconds(tone);
    digitalWrite(speakerPin, LOW);
    delayMicroseconds(tone);
  }
}

void playNote(char note, int duration) {  
  // play the tone corresponding to the note name
  for(int i = 0; i < 8; i++) {
    if(names[i] == note) {
      playTone(tones[i], duration);
    }
  }
}
