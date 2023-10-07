
#include <Wire.h>  //Biblioteka umożliwiająca komunikację za pomocą magistrali I(kwadrat)C
#include <ZumoShield.h>  //Biblioteka od producenta ZUMO, zawiera funkcje pozwalające sterowanie poszczególnymi elementami robota
#include <IRremote.h>  //Biblioteka umożliwiająca odbiór danych z fal podczerwonych wysyłanych z pilota

int IR_pin = 6;  //przyporządkowanie pinu 6 do odbiornika fal podczerwonych, współpracującego z pilotem
IRrecv IRR (IR_pin);   //Utworzenie obiektu za pomocą którego możliwe jest odwoływanie się do funkcji i danych związanych z pilotem
bool tryb[4]; //tablica zmiennych typu bool, określająca w którym trybie znajduje się robot 

#define QTR_THRESHOLD  1500 // definicja czułości czujnika linii

// Definicja charakterystycznych wartości prędkości 
#define REVERSE_SPEED     200 // 0 zatrzymanie, 400 pełna dzida
#define TURN_SPEED        200
#define FORWARD_SPEED     200
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     300 // ms
#define MAX_SPEED         400   
#define LED_PIN 13  //dioda LED na płytce ZUMO shield związana jest z pinem 13
#define NUM_SENSORS 6 //liczba czujników linii
ZumoMotors motors; //stworzenie obiektu, pozwalającego odwoływać się do funkcji związanych z prędkością silników
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN); //stworzenie obiektu, pozwalającego odwoływać się do funkcji związanych z czujnikami linii
unsigned int sensor_values[NUM_SENSORS]; //tablica przechowująca wartości odczytane przez czujniki linii
int lastError = 0; //zmienna służacą do korekcji prędkości w trybie podążania za linią

void setup() {
  Serial.begin (9600);   //umożliwienie komunikacji z portem szeregowym
  IRR.enableIRIn ();       //Rozpoczęcie procesu odbierania zakodowanych danych z pilota. Zezwolenie na przerwania.   
  Serial.print ("IR Ready ... @Pin");
  Serial.println (IR_pin);
  pinMode(LED_PIN, OUTPUT); //Ustawienie pinu 13 (LED_PIN) jako wyjście
  sensors.init(QTR_NO_EMITTER_PIN); //Inicjalizacja modułu z czujnikami linii
  tryb[0]=false; 
  tryb[1]=false;
  tryb[2]=false;
  tryb[3]=false; //Wyzerowanie tablic związanych z aktualnym trybem. Początkowo robot nie znajduję się w żadnym trybie
}

void loop() {
if (IRR.decode ()) { //Funkcja decode() zwraca wartość true, gdy zostanie odebrany sygnał z pilota
    Serial.println (IRR.results.value, HEX); //Wyświetlenie odczytanej wartości na monitorze portu szeregowego w kodzie szesnastkowy
    if (IRR.results.value == 0xFF6897) { //results.value przechowuje ostatnio odczytaną wartość z pilota. FF6897 (przycisk 0 na pilocie) powoduje zmianę trybu 0, który odpowiada za świecenie diody  
    tryb[0]=!tryb[0];
    if(tryb[0]==true) digitalWrite(LED_PIN, HIGH); 
    if(tryb[0]==false) digitalWrite(LED_PIN, LOW); 
    }
  
    if (IRR.results.value == 0xFF629D) {  //FF629D odpowiada przyciskowi CH. Tryb[2] - tryb śledzenia linii
    motors.setSpeeds(0, 0);
    tryb[2]=!tryb[2];
    tryb[1]=false;
    tryb[3]=false;
    
    if (tryb[2]){
      //Rozpoczęcie kalibracji
    digitalWrite(13, HIGH);     // Załączenie diody
    delay(1000); //Odczekanie sekundy
    for(int i = 0; i < 80; i++) {
    if ((i > 10 && i <= 30) || (i > 50 && i <= 70))
      motors.setSpeeds(-200, 200);
    else
      motors.setSpeeds(200, -200);
      sensors.calibrate();
    delay(20);
    //Łączny czas trwania kalibracji- 1600ms
    }
    motors.setSpeeds(0,0); //Zahamowanie silników
    digitalWrite(13, LOW);     // Wyłączenie diody
    } 
    else {
    motors.setSpeeds(0, 0);}
    }
      
    if (IRR.results.value == 0xFFA25D){   //FFA25D odpowiada przyciskowi CH-. Tryb[1] - tryb wykrywania linii granicy
    tryb[1]=!tryb[1];
    tryb[2]=false;
    tryb[3]=false;
    motors.setSpeeds(0, 0);
    }
    
    if(IRR.results.value == 0xFFE21D){  //FFE21D odpowiada przyciskowi CH+. Tryb[3] - tryb sterowania manualnego
    motors.setSpeeds(0, 0);
    tryb[3]=!tryb[3];
    tryb[2]=false;
    tryb[1]=false;
    }
  
    if((IRR.results.value == 0xFF18E7)&&tryb[3]){ //FF18E7 odpowiada jeździe do przodu. Warunek &&tryb[3] powoduję, że klawisze 2,4,6,8 powodują jakiekolwiek działanie wyłącznie w trybie 3
    do{
    IRR.resume ();   //reset i przygotowanie pilota na odbior kolejnego sygnalu
    IRR.results.value = 0x0; //Wyzerowanie ostaniej wartości przetrzymywanej w pamięci
    delay(150); //opóźnienie wymagane 
    if (IRR.decode ()) { 
    Serial.println (IRR.results.value, HEX);
    motors.setSpeeds(MAX_SPEED, MAX_SPEED);} //MAX SPEED = 400
    }while(IRR.results.value == 0xFFFFFFFF); //Przytrzymanie jakiekolwiek przycisku powoduje cykliczne (co okolo 100ms) wysyłanie wartości FFFFFFFF 
    motors.setSpeeds(0, 0); //Zahamowanie silników
    }
  
    if((IRR.results.value == 0xFF4AB5)&&tryb[3]){ // jazda do tyłu
    do{
    IRR.resume ();
    IRR.results.value = 0x0;
    delay(150);
    if (IRR.decode ()) {
    Serial.println (IRR.results.value, HEX);
    motors.setSpeeds(-MAX_SPEED, -MAX_SPEED);} //MAX SPEED na minusie, jazda do tyłu
    }while(IRR.results.value == 0xFFFFFFFF);
    motors.setSpeeds(0, 0);
    }
  
    //if pilot 4
    if((IRR.results.value == 0xFF10EF)&&tryb[3]){ //obrót w miejscu przeciwnie do wskazówek zegara
    do{
    IRR.resume ();
    IRR.results.value = 0x0;
    delay(150);
    if (IRR.decode ()) {
    Serial.println (IRR.results.value, HEX);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);} //lewa gąsienica obraca się do tyłu, prawa do przodu, TURN_SPEED = 200
    }while(IRR.results.value == 0xFFFFFFFF);
    motors.setSpeeds(0, 0); 
    }
    
    //if pilot 6
    if((IRR.results.value == 0xFF5AA5)&&tryb[3]){ //obrót w miejscu zgodnie do wskazówek zegara
    do{
    IRR.resume ();
    IRR.results.value = 0x0;
    delay(150);
    if (IRR.decode ()) {
    Serial.println (IRR.results.value, HEX);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);} //lewa gąsienica obraca się do przodu, prawa do tyłu, TURN_SPEED = 200
    }while(IRR.results.value == 0xFFFFFFFF);
    motors.setSpeeds(0, 0);
    }

    IRR.resume ();   //reset i przygotowanie pilota na odbior kolejnego sygnalu
}
 if(tryb[1]){
    sensors.read(sensor_values);
    
    if (sensor_values[0] > QTR_THRESHOLD) {
    // if leftmost sensor detects line, reverse and turn to the right
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);}
    
    else if (sensor_values[5] > QTR_THRESHOLD)  {
    // if rightmost sensor detects line, reverse and turn to the left
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);}
    else  {
    // otherwise, go straight
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);}
 }
 
  if(tryb[2]){
    int position = sensors.readLine(sensor_values);     // Wykrycie i zapamiętanie pozycji oraz grubości linii, szczytanie dnaych z modułu czujników linii
    
    int error = position - 2500;    //Zmienna error przechowuje odległość (w zunitach) od środka linii, wartość 2500 dobrana heurystycznie. 
    // Może przyjmować wartości zarówno dodatnie jak i ujemne, w zależności od kierunku, w którym nastąpiło odchylenie od środka linii.


    //Korekcja prędkości w stosunku do odległości od linii opiera się na zasadzie regulatora PID. Korekcja prędkości obliczana jest
    //porzez sumę aktualnej od środka linii (podzielonej przez 4) i różnicy aktualnego odchylenia a odchylenia zmierzonego w ostatniej pętli programu (pomnożone przez 6).
    //Wartości 4 i 6 dobrane heurystycznie
    int speedDifference = error / 4 + 6 * (error - lastError);

    lastError = error; //przypisanie wartości aktualnego ochylenia (error) zmiennej (lastError) , która zostanie wykorzystana w kolejnym obiegu programu


    int m1Speed = MAX_SPEED + speedDifference; //Korekcja prędkości ma przeciwny znak dla każdego z silników
    int m2Speed = MAX_SPEED - speedDifference; //Prędkość nie może przekroczyć 400, w większości przypadków, jeden z silników jedzie z prędkością maksymalną, drugi z mniejszą od maksymalnej,w efekcie następuje jazda po łuku.


    if (m1Speed < 0) //Blokada silników przed cofaniem- mogłoby to nastąpić, gdyby zmienna error przyjęła dużą wartość
    m1Speed = 0;
    if (m2Speed < 0)
    m2Speed = 0;
    if (m1Speed > MAX_SPEED) //Ograniczenie maksymalnej prędkości Zumo do 400
    m1Speed = MAX_SPEED;
    if (m2Speed > MAX_SPEED)
    m2Speed = MAX_SPEED;
    motors.setSpeeds(m1Speed, m2Speed); //Zadanie silnikom określonej prędkości obrotowej po uwzględnieniu korekty
  }
}