#include <EEPROM.h>

#include<avr/interrupt.h>
#include<avr/io.h>
#include <avr/wdt.h>

#define Reset_AVR() wdt_enable(WDTO_30MS); while(1) {}

/////NOTA EEPROM/////
/*
  Attenzione si salvano tutti i dati nel Primo e secondo Byte della EEPROM quindi inidirizzo 0 e 1, le scritture previste sono circa 10000
  cioè 1/10 dei cicli totali previsti, ciò significa bruciare 1/10 di vita prevista della per ogni cella della EEPROM
*/

byte threshold_LOW = 24;
byte threshold_HIGH = 32;

byte const winSizeT = 4;          //dimensione finestra per calcolo media pressione e temperatura
byte const winSizeP = 2;
byte const winSizeV = 10;
byte winT = 0;
byte winP = 0;
byte winV = 0;
float mediaT[winSizeT];
float mediaP[winSizeP];
float mediaV[winSizeV];
float tempP = 0;
float tempT = 0;
float tempV = 0;

byte dp = 15;           //delta per controllo emergenza di pressione e/o temperatura
byte dt = 15;

byte ritardo=150; //ritado di circa 3.3 s
//NOT USED

byte presMAX = 70;      //massimo valore di pressione impostabile
byte presMIN = 0;       //minimo valore di pressione impostabile
byte tempMAX = 90;      //massimo valore di temperatura impostabile
byte tempMIN = 0;       //minimo valore di temperatura impostabile

byte setpres = 30;       //pressione desiderata da impostare a 1
byte settemp = 70;       //temperatura desiderata da impostare a 1
byte errt = 1;         //errore ammissibile sulla temperatura closed loop
byte errp = 0.5;         //errore amissibile sulla pressione  closed loop

float p_act = 0;            //valore di pressione di linea
float T_mis = 0;          //valore di temperatura olio misurato

byte AIcurp = A13;       //Pin di connessione sensore 4-20 mA pressione
byte AIcurt = A12;  //Pin di connessione sensore 4-20 mA temperatura
//byte AIcurv = A11;  //Pin di connessione sensore 4-20 mA tensione
byte AOp = 44;          //contorllo pressione 0-5 V pin 44 o 45
byte AIpres_fb = A11;    //TO BE CHECKED

float q = 0;              //variabili per scalatura valori sensori
float m = 0;
float portatamaxT = 90;
float portataminT = 10;
float portatamaxP = 60;
float portataminP = 0;
float portatamaxV = 30;
float portataminV = 0;
float poffset = 0;        //offset sensore di pressione

//VARIABILI PID
float kp = 6.5;           //k proporzionale pressione
float ki = 1;         //k integrale pressione
float prop = 0;         //valore proporzionale
float intg = 0;         //valore integrativo
float intg_precP = 0;   //memoria valore integrativo pressione
int controlP = 0;      //valore controllo di pressione
float err = 0;          //errore di pressione o temperatura
float p_acc = 0;

bool next = 0;          //bit di cambio di stato
bool on = 0;            //bit di ciclo attivo

//PIN COMANDO CENTRALINA

byte PA = 12;
byte PB = 11;
byte Consenso = 22;
byte Allarme = A0;    //CHECK WHERE IS CONNCTED
byte Chiller = 30;
byte Resistenza = 31;
byte AbilitaP = 13;
byte valve1 = 23;

unsigned long N_cicli = 0;            //contacicli
word N_cicli_salvato = 0;    //cicli memorizzati in eeprom
unsigned long N_cicli_prova = 10000000;   //durata prova
word c = 1000;                //delta  cicli al quale si memorizza


byte lifebit = 0;       //variabile di conteggio tempo prima di verificare che l'interfaccia grafica sia attiva
byte OVF = 0;           //variabile per conteggio overflow
byte OVF_slow = 1;      //regolazione frequenza di fino
byte OVFP = 0;          //conterollo pressione
byte OVFP_slow = 15;
byte OVFT = 0;          //conterollo pressione
byte OVFT_slow = 30;

char* stringerror = "";

bool tminsup = false;     //variabili utilizzate per l'attesa dell'arrivo ad un certo livello di pres o temp in
bool pminsup = false;     //modo da evitare emergenze in fase di preparazione

bool startciclo = false;  //variabile di avvio o fermo del ciclo di prova
bool tempcontrol = 0;       //variabile di attivazione controllo temperatura
bool prescontrol = 0;     //variabile di atttivazione controllo pressione
bool errore = 0;          //memorizza errori
bool active = 0;          //variabile di stato
bool allarme = 0;         //memorizza allarmi
bool pa = false;          ///variabili di controllo attuatori centralina
bool pb = false;
bool chil = false;
bool res = false;
bool allarm_str_bool = 0;     //variabile utilizzata per controllo di flusso codice

float calcanalog(byte pin, char mod);       //funzione che restituisce valore di pressione o temperatura acquisendo un ingresso

void arresto();           //dichiarazione funzioni

void PID();

void sendData(float val, char type);

char car;         //variabile carattere per comunicazione
char car2;         //variabile carattere per comunicazione

word txDelay = 200; //attesa tra una trasmissione e l'altra


//////////////////INTERRUPT TIMER ROUTINE/////////////////////////
///////TIMER4 VERIFICA ATTIVITA' INTERFACCIA DI CONTROLLO//////////

ISR(TIMER4_OVF_vect)
{

  if (errore == 0)
  {
    ++lifebit;
    if (lifebit > 45) {           //45 circa uguale a 10 secondi (txDelay=0.2s * 45 = 9s)
      stringerror = "Connection down";
      allarm_str_bool = 1;
      active = 1;  //addormento il pid dopo dieci secondi
      //arresto();
    }
  }
}



//////////////////INTERRUPT TIMER3 ROUTINE/////////////////////////
/////////TIMER3 CONTROLLO TEMPERATURA E PRESSIONE/////////////////

ISR(TIMER3_OVF_vect)
{
  
  allarme = digitalRead(Allarme);
  if (errore == 0) {
    if (startciclo) {     //contaclicli
      ++OVF;
      p_acc = calcanalog(AIpres_fb, 'V');
      if ((p_acc > threshold_HIGH)&&(OVF==ritardo)) {
        OVF=0;
        ++N_cicli;
        digitalWrite(PB, LOW);    //DIVENTERA PB

        
      } else if ((p_acc < threshold_LOW)&&(OVF==ritardo)) {
        OVF=0;
        ++N_cicli;
        digitalWrite(PB, HIGH);     //DIVENTERA PB
      }
      if ((N_cicli % c) == 0) {
        Serial.print(N_cicli);
        EEPROM.put(2, ((N_cicli/2) / c));
      }
      if ((N_cicli/2) >= N_cicli_prova) {
        startciclo = 0;
        digitalWrite(PB, LOW); //DIVENTERA PB
      }
      
    }


    OVFP++;
    OVFT++;
    if (OVFP == OVFP_slow)        //CONTROLLO PRESSIONE
    {  
     
      //invio valore effettivo temperatura
      //sendData(22.34, 'P'); 
      OVFP = 0;
      p_act = calcanalog(AIcurp, 'P');    //leggo la pressione in bar
      if (p_act > setpres - errp) {
        pminsup = true;
      }
      if ((p_act >= setpres + errp) ||  (p_act <= setpres - errp))    //controllo se la pressione non rientra nel range di errore accettabile
      {
        if (((p_act > setpres + dp) || (p_act < setpres - dp)) && (active == 1) && (pminsup == 1))   //controllo fuori pressione
        { //eliminare active???
          stringerror = "MAX pres superata";
          allarm_str_bool = 1;
          //arresto();
        }
        if (prescontrol) {              //controllo pressione automatica
          intg_precP = controlP;
          PID();      //calcola valore controlP
          OCR5C = controlP; //attuazione PWM
        }
      }
    }
    if (OVFT == OVFT_slow)    //CONTROLLO TEMPERATURA
    {
      OVFT = 0;
      sendData((N_cicli/2),'Z');
      sendData(T_mis, 'T'); //invio valore effettivo pressione
      sendData(p_act, 'P'); 
      sendData(settemp, 't');    //invio valore setT
      sendData(setpres, 'p');    //invio valore setP
      sendData(controlP, 'K');   //invio valore PID
      sendData(ki, 'G');   //invio valore Kp
      sendData(kp, 'N');   //invio valore Ki
      //sendData(N_cicli,'Z');    //to be removed
      sendData(ritardo,'D');
      sendData(N_cicli_prova/1000,'C');
      
      T_mis = calcanalog(AIcurt, 'T');

      if (T_mis > settemp - errt) {
        tminsup = true;
      }
      if (tempcontrol) {    //check se controllo temperatura abilitato
        T_mis = calcanalog(AIcurt, 'T');
        if (((T_mis > (settemp + dt)) || (T_mis < (settemp - dt))) && (active == 1) && (tminsup == 1))
        {
          stringerror = "Temp out of limits";
          allarm_str_bool = 1;
          //arresto();
        }
        if (T_mis <= settemp - errt) {
          digitalWrite(Resistenza, HIGH); //Accensione resistenza
          digitalWrite(Chiller, LOW);
        }
        else if (T_mis >= settemp + errt) {
          digitalWrite(Chiller, HIGH);    //accendo chiller
          digitalWrite(Resistenza, LOW);
        } else {
          digitalWrite(Resistenza, LOW);   //spengo la resisstenza se mi serve riaccendo subito se non mi serve resta spenta
          digitalWrite(Chiller, LOW);
        }
      }
    }
  }
  else {
    //arresto();
  }
}



/////////////VOID SETUP/////////////

void setup()
{

  for (int i = 0; i < winSizeP; ++i) {    //inizializzo gli array
    mediaP[i] = 0;
  }

  for (int i = 0; i < winSizeT; ++i) {
    mediaT[i] = 0;
  }

  for (int i = 0; i < winSizeV; ++i) {
    mediaV[i] = 0;
  }

  errore = 0;
  pinMode(PA, OUTPUT);
  pinMode(PB, OUTPUT);
  pinMode(Consenso, OUTPUT);
  pinMode(Allarme, INPUT);
  pinMode(Chiller, OUTPUT);
  pinMode(Resistenza, OUTPUT);
  pinMode(AbilitaP, OUTPUT);

  pinMode(AOp, OUTPUT);

  Serial.begin(9600);

  digitalWrite(Consenso, LOW);
  digitalWrite(Chiller, LOW);
  digitalWrite(Resistenza, LOW);
  digitalWrite(AbilitaP, LOW);
  digitalWrite(PA, LOW);
  digitalWrite(PB, LOW);

  TCCR5A = 0xAB;                 //timer for PWM
  TCCR5B = 0x09;                 //
  OCR5C = 0;

  /////////////////////RECUPERO VALORI EEPROM///////////////////////////
 
  EEPROM.get(2, N_cicli_salvato);     //verificare indirizzo di scrittura
  N_cicli = N_cicli_salvato * c*2;
  Serial.print(N_cicli);
  digitalWrite(Consenso, HIGH);
  Serial.flush();       //pulizia buffer comunicazione
  Serial1.flush();
  Serial2.flush();
  Serial3.flush();

  sendData(1, 'c');    //invia il consenso all'avviamento

  while (!(Serial.available())) {}  //attendo la conferma di avviamento

  car=Serial.read();
  car=' ';
  //sendData(setpres, 'p');        //invio set pressione

  //sendData(settemp, 't');        //invio set temperatura


  ////////////////SETTAGGIO VALORI INIZIALE DI CICLO//////////////////


  ////////////////AVVIO PROVA//////////////////

  digitalWrite(AbilitaP, HIGH);
  //timer 3 controllo pressione e temperatura
  TIMSK3 = 0x01;                  //  interrupt enable timer3
  TCCR3A = 0x00;                  //  timer3 funzionamento normale
  TCCR3B = 0x02;                  //  8 (30 OVF/s)
  //timer 4 controllo lifebbit
  TIMSK4 = 0x01;                  //  interrupt enable timer4
  TCCR4A = 0x00;                  //  timer4 funzionamento normale
  TCCR4B = 0x03;                  //  64 (3.814697265625 OVF/s)
  //AVVIO IL TIMER
  sei();                          //  abilitazione globalinterrupt
  ASSR = 0x20;                    //  clock da oscillatore esterno da 16MHz

  controlP = 420;                 //regolo di base a circa 30 bar
}



////////////////VOID LOOP////////////////

void loop()
{

  if (!errore)

  {
    if (pa) {
      digitalWrite(PB, LOW);
      digitalWrite(PA, HIGH);
    }
    else {
      digitalWrite(PA, LOW);
    }
    if (pb) {
      digitalWrite(PA, LOW);
      digitalWrite(PB, HIGH);
    }
    else {
      digitalWrite(PB, LOW);
    }
    if (res) {
      digitalWrite(Chiller, LOW);
      digitalWrite(Resistenza, HIGH); //Accensione resistenza
    }
    else if (!tempcontrol) {
      digitalWrite(Resistenza, LOW);
    }
    if (chil) {
      digitalWrite(Resistenza, LOW); //Accensione chiller
      digitalWrite(Chiller, HIGH);
    }
    else {
      digitalWrite(Chiller, LOW);
    }

    // INIZIO RICEZIONE
    car = ' ';
    car2 = ' ';
    if (Serial.available() > 0)     //attende un carattere
    {
      car = Serial.read();          //legge carattere
      if (car == 'W') {

        digitalWrite(Resistenza, LOW);
        digitalWrite(Chiller, LOW);
        tempcontrol = (!tempcontrol);
      }
      else if (car == 'k') {
        prescontrol = (!prescontrol);
      }
      else if (car == 'X') {
        startciclo = (!startciclo);
      }
      else if (car == 'S') {
        stringerror = "Ciclo Arrestato";
        allarm_str_bool = 1;
        //arresto();
      }
      else if (car == 'A') {
        pa = (!pa);
        pb = 0;
      }
      else if (car == 'B') {
        pb = (!pb);
        pa = 0;
      }
      else if (car == 'C') {
        chil = (!chil);
        res = 0;
      }
      else if (car == 'R') {
        res = (!res);
        chil = 0;
      }
      else if (car == 'O') {
        lifebit = 0;
      }
      else if (car == 'T') {
        while (Serial.available() < 1);       //attende fino a che non arriva un carattere
        car = Serial.read();
        if (car == '+') {
          settemp++;
        }
        else if (car == '-') {
          settemp--;
        }
      }
      else if (car == 'P') {
        while (Serial.available() < 1);
        car = Serial.read();
        if (car == '+') {
          setpres++;
        }
        else if (car == '-') {
          setpres--;
        }
      } else if (car == 'K') {
        while (Serial.available() < 1);
        car = Serial.read();
        if (car == '+') {
          controlP++;
        }
        else if (car == '-')  {
          controlP--;
        }
      } else if (car == 'G') {
        while (Serial.available() < 2);
        car = Serial.read();
        car2 = Serial.read();
        if ((int(car2)/10)>5)
          {kp = float(car) + (float(car2)/100)+0.01;}
          else{kp = float(car) + (float(car2)/100);}
      } else if (car == 'N') {
        while (Serial.available() < 2);
        car = Serial.read();
        car2 = Serial.read();
        ki = float(car) + (float(car2)/100);

      } else if (car == 'Z') {
        while (Serial.available() < 2);
        car = Serial.read();
        car2 = Serial.read();
        N_cicli_prova = (car + (car2 << 8));
        N_cicli_prova=N_cicli_prova*1000;
      }
      else if (car == 'D') {
        while (Serial.available() < 2);
        car = Serial.read();
        car2 = Serial.read();
        ritardo = (car + (car2 << 8));
      }
    }
    if (!(digitalRead(Allarme)))        //check allarmi
    {
      stringerror = "Allarme centralina";
      allarm_str_bool = 1;
      //arresto();
    }
  }
}



/////////////FUNZIONE CALCOLO INGRESSI ANALOGICI ///////////////

float calcanalog(byte pin, char mod)
{
  float P = analogRead(pin);    //10us
  if (mod == 'P')               //sensore centralina misura pressione linea
  {
    if (P == 0)
    {
      stringerror = "I sensor fault";
      allarm_str_bool = 1;
      //arresto();
    }
    m = (portatamaxP - portataminP) / 775.8;    //le rette sono calibrate sugli esatti valori misurati, vedi file excell
    q = (portatamaxP - (m * 970));
    P = ((P * m) + q);
    ///////////////CALCOLO VALORE MEDIO/////////////////
    if (P > 0) {        //medio solo valori maggiori di zero
      mediaP[winP] = P;
    }
    ++winP;
    if (winP == winSizeP) {       //winP variabile di incremento ciclo
      winP = 0;
    }
    float temp1 = 0;
    float count = 0;
    for (int i; i < winSizeP; ++i) {
      temp1 = mediaP[i];        //estraggo valore per valore
      if (temp1 > 0) {          //all'inizio devo fare in modo di non considerare le celle dell'array vuote
        tempP = (tempP + mediaP[i]);    //sommo tutti i valori
        ++count;                  // conto quanti valori ho sommato
      }
    }
    P = tempP / count;        //calcolo effettivo media
    tempP = 0;
  }

  if (mod == 'T')               //sensore di temperatura in corrente temperatura centralina
  {
    if (P == 0)
    {
      stringerror = "I sensor fault";
      allarm_str_bool = 1;
      //arresto();
    }
    m = (portatamaxT - portataminT) / 773;
    q = (portatamaxT - (m * 970));
    P = ((P * m) + q);

    if (P > 0) {
      mediaT[winT] = P;
    }
    ++winT;
    if (winT == winSizeT) {
      winT = 0;
    }
    float temp1 = 0;
    float count = 0;
    for (int i; i < winSizeT; ++i) {
      temp1 = mediaT[i];
      if (temp1 > 0) {
        tempT = (tempT + mediaT[i]);
        ++count;
      }
    }
    P = tempT / count;
    tempT = 0;
  }

  if (mod == 'V')               //sensore di pressione in tensione pressione perdita giunto
  {
    if (P == 0)
    {
      stringerror = "V sensor fault";
      allarm_str_bool = 1;
      //arresto();
    }
    m = (portatamaxV - portataminV) / 818.4;
    q = (portataminV - (m * 102.3));
    P = ((P * m) + q);

    if (P > 0) {        //medio solo valori maggiori di zero
      mediaV[winV] = P;
    }
    ++winV;
    if (winV == winSizeV) {       //winP variabile di incremento ciclo
      winV = 0;
    }
    float temp1 = 0;
    float count = 0;
    for (int i; i < winSizeV; ++i) {
      temp1 = mediaV[i];        //estraggo valore per valore
      if (temp1 > 0) {          //all'inizio devo fare in modo di non considerare le celle dell'array vuote
        tempV = (tempV + mediaV[i]);    //sommo tutti i valori
        ++count;
      }
    }
    //IMPLEMETARE MEDIA MOBILE
    P = tempV / count;
    tempV = 0;
  }
  return P;
}


/////////////FUNZIONE CALCOLO VALORE PID ///////////////

void PID()
{

  err = (setpres) - (p_act);                                     //calcolo l'errore
  prop = kp * err;
  intg = intg_precP + (ki * err);                            //tengo traccia solo dell'errore precedente
  intg_precP = intg;
  if (intg > 1024) {
    intg = 1024;
  }
  controlP = int((prop + intg));                             //dal valore float ricavo il intero tra 0...1024 corrispondente a 0..5 V in out
  if (controlP > 1024) {
    controlP = 1024;
  }
  if (controlP < 0) {
    controlP = 0;
  }
}



///////////VOID ARRESTO////////////

void arresto()
{
  errore = 1;                         //attivo il bit di errore per fermare tutte le altre azioni del SW
  digitalWrite(PA, LOW);              //disattivo tutte le uscite
  digitalWrite(PB, LOW);
  digitalWrite(Consenso, LOW);
  digitalWrite(Chiller, LOW);
  digitalWrite(Resistenza, LOW);
  digitalWrite(AbilitaP, LOW);
  OCR5C = 0;
  int i = 0;
  if (allarm_str_bool == 1) {
    //Serial.write('J');   //mando messaggi per informare l'HMI di arrivo stringa
    Serial.write('E');
    while (stringerror[i] != '\0') { //conto dimensione stringa
      ++i;
    }
    allarm_str_bool = 0;               //bit utilizzato per suddividere in due stati la routine d'errore
  }
  for (int j = 0; j < i; ++j) {     //invio stringa per carattere
    Serial.write(stringerror[j]);
  }

  if (Serial.available() > 0) {       //ora tutto è disabilitato grazie al bit errore
    car = Serial.read();              //qui cicla fin che non riceve un carattere specifico
  }
  if ((car == 'X') && (allarme == 1)) { //se il carattere è corretto e non ci sono allarmi, allora il sistema si riavvia
    Reset_AVR();
  }
}

void sendData(float val, char type) {
 
  byte km = 0;
  byte kl = 0;
  byte ka = 0;
  
  //il type c è utilizzato per iniziare la connessione e quindi è sufficiente mandi solo un carattere
  if (type == 'c') {
    Serial.write(type);
  }
  //tutti i valori float G=kp; N=ki; K=PID; T=T_mis; P=p_act
  //Sono invieti su due byte, uno per la parte intera ed uno per la parte decimale
  if ((type == 'G') || (type == 'N') || (type == 'P') || (type == 'T')) {
    kl = int((val - int(val)) * 100);
    km = int(val);                //invio caratteri per informare HMI di arrivo di determinate cosa
    Serial.write(type);
    Serial.write(kl);   //valore decimale
    Serial.write(km);   //valore intero
 
  }
  //Tutti i valori intere compresi tra 0 e 255 vengono inviati su un sunglo byte
  if ((type == 't') || (type == 'p')||(type=='D')) {
    val = int(val);
    km = val;
  // Serial.print("eseguito pezzo di codice su condizione K");                 //invio caratteri per informare HMI di arrivo di determinate cosa
    Serial.write(type);
    Serial.write(km);
  }

  if  ((type == 'Z')||(type == 'K')||(type=='C')) {
    long temp = val;
    if (type=='C'){val=val/1000;}
    km = temp;
    kl = (temp >> 8);
    ka = (temp >> 16);               //invio caratteri per informare HMI di arrivo di determinate cosa
    Serial.write(type);
    Serial.write(km);
    Serial.write(kl);
    if (type=='Z'){Serial.write(ka);}
    
  }

  /*
    else {
    km = (val - int(val)) * 100;    //estraggo la parte decimale e mi porto 2 cifre significative nella parte intera
    kl = val;                        //mi estraggo al parte intera tagliando la virgola
    Serial.write(c);                  //invio caratteri per informare HMI di arrivo di determinate cosa
    Serial.write(type);
    Serial.write(km);
    Serial.write(kl);
    }*/
//Serial.print("\n");
//Serial.print("eseguito pezzo di codice dei print");
  /*Serial.print("\n");Serial.print("type="); Serial.print(type); Serial.print("\n");
  Serial.print("val="); Serial.print(val, DEC); Serial.print("\n");
  Serial.print("km="); Serial.print(km, DEC); Serial.print("\n");
  Serial.print("kl="); Serial.print(kl, DEC); Serial.print("\n");
  Serial.print("ka="); Serial.print(ka, DEC); Serial.print("\n");
  Serial.print("\n");/*


  ////Serial.print("temp=");//Serial.print(val); //Serial.print("\n");
  /*Serial.print(c, DEC); Serial.print("\n");
    Serial.print(type, DEC); Serial.print("\n");
    Serial.print(km, DEC); Serial.print("\n");
    Serial.print(kl, DEC); Serial.print("\n"); Serial.print("\n");*/



  /*if (type == 'S') {
    Serial.write(c);
    Serial.write(type);
    }*/

  /*if (type == 'E') {
    Serial.write(c);
    Serial.write(type);
    }*/
}

