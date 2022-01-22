// Author windundsternne V.0.99
// Das Programm liest Control und Sense Daten vom Bus
// fhem. Updates können OTA durchgeführt werden.
// Ist bestimmt nicht fehlerfrei läuft aber seit einer Weile stabil
// Hardware
// Webmos d1 mini Pro + HER MAX485 module
// Stromversorgung über Kessel 24V 7800 Spannungsregler
// Neben den Kesselwerten werden nich Brennerlaufzeit Gesamtenergie
// berechnet mit der der Pelletverbrauch bestimmt werden kann (ca. 4.3kg/kwh bei einem EF2)

// Individualisierungen

//#define STASSID "MYSSID"          // Wlan SSID
//#define STAPSK  "MYWLANPW"        // Wlan PW
//#define MQTTSERVER "192.168.0.5"  // IP MQTT-Server

//  individuelle Konfig ausserhalb des GIT
#ifndef STASSID
#include "conf.h"
#endif



#define OTAHOST "kwbeasyfire"     // unnter dem Namen  Netzwerkschnittstelle in der ArduionIDE
#define OUTTOPIC "kwb"
#define INTOPIC "cmd"
#define MQNAME "kwb"


// Ende Individualisierungen


#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <stdio.h>
#include <time.h>

#define STATE_WAIT_FOR_HEADER 1
#define STATE_READ_MSG  2
#define MSG_TYPE_CTRL 1
#define MSG_TYPE_SENSE 2
#define FALSE 0
#define TRUE 1
#define RX        2    //4 // D3    //Serial Receive pin
// Aktuell ist der nicht angeschlossen, da  nicht
// gesteuert werden soll
#define TX        4    //2 // 2 // D4    //Serial Transmit pin
#define RTS_pin   5 // D1    //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW


const char* mqtt_server = MQTTSERVER; // IP des MQTT Servers - z.B. fhem
const char* ssid = STASSID;
const char* password = STAPSK;

// Globals

long UD = 0;
long SL = 0 ; // Schneckenlaufzeit
long ZD = 0;
int deltatime = 5 ;
long UDtimer = 0;
long HANAtimer = 0; // Timer zur Ausgabe der HANA Ratio
long NAz = 0, HAz = 0;

int wifistatus = 0;

extern long bytecounter;
extern long framecounter;
extern long errorcounter;
long last_errorcount = 0;

// Stat impulse
unsigned long id[300];
int ic = 0;
unsigned long ifc[300];
unsigned long idl[300];

//  gemessen 308gr auf 229 UD = 1.3449
//  Empirische aus 26KW Leistung 1.3338
//  24,7KW/4.8 mit 2338 UD -> 2.200
//  22KW * 0.8 * 9,4h  *  4kg /h  / 1642 UD =


// gemessen am HA
// 310gr 207UD = 1,495 g/UD
// 200gr auf 135 UDs = 1.48
// 298gr auf 207 UDs = 1.44
// mach ca. 2.4 UD /s  bei Norm Betrieb 187 UD in 3 Min
// nied Fallrohrstand 160UD auf 200gr = 1.25
// Nebenantrieb/Schnecke: 1990gr mit 373 s = 5.33gr/s

double lfaktor = 1.0; // Umrechnung Leistung

// Hauptantrieb (Antrieb im Kessel)  = Umdr. HA
double UDfaktor = 1.49; // gr Pellet pro UD HA
double UDsek =  1.73; // 2.45; // Umdrechungen HA / sek
double NAfaktor = 5.4;

unsigned long count = 0;
unsigned long bytecount = 0;
unsigned long waitcount = 0;
unsigned long longwaitcount = 0;
unsigned long keeptime = 0, timerd = 0, timer1 = 0, timer2 = 0, timer3 = 0, timeru = 0 , timerboot = 0, timerimpuls = 0, timerha = 0;
unsigned long timerprell = 0, timerhazeit = 0;
unsigned long timerschnecke = 0;
unsigned long timerpause = 0;
unsigned long kwhtimer = 0; // Zeit seit letzer KW Messung

struct ef2
{
  double kwh = 0.1;
  double Leistung = 0.1;
  double Rauchgastemperatur = 0.0;
  double Proztemperatur = 0.0;
  double Unterdruck = 0.0;
  double Brennerstunden = 0.0;
  double Kesseltemperatur = 0.0;
  double Geblaese = 0.0;
  double Saugzug = 0.0;
  int  Reinigung = 0;
  int Zuendung = 0;
  int Drehrost = 0;
  int Schneckenlaufzeit = 0;
  int Schneckengesamtlaufzeit = 0;
  int KeineStoerung = 0;
  int Raumaustragung = 0;
  int Hauptantriebimpuls = 0; // Impulszähler
  int Hauptantrieb = 0 ; // Motor läuft
  int HauptantriebUD = 0; // Umdrehungen Stoker
  int Hauptantriebzeit = 0;
  int Pumpepuffer = 0;
  int RLAVentil = 0;
  int ext = 1;
  double d[10];
  double photo = 0.0;
};

struct ef2 Kessel, oKessel; // akt. und "letzter" Kesselzustandd

WiFiClient espClient;
PubSubClient client(espClient);

#include <SoftwareSerial.h>  // https://github.com/PaulStoffregen/SoftwareSerial

SoftwareSerial RS485Serial(RX, TX);

#include "espinclude.h"

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

// Wifi einschalten,, Verbinden und mqtt Connecten
// OTA einrichten

void wifi_on()
{


  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    // Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(OTAHOST);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  client.setServer(mqtt_server, 1883);
  //Serial.println("Ready");
  //Serial.print("IP address: ");
  //Serial.println(WiFi.localIP());
  wifistatus = 1;
}


// WIFi Abschalten
void wifi_off()
{

  mqttreconnect();
  client.publish("Info", "WIFI Off");
  delay(1999);
  WiFi.mode(WIFI_OFF);
  wifistatus = 0;
}

////////////////////// Setup //////////////////////////

void setup() {
  char msg[64];
  char rec[1000];

  // Serial.begin(19200);
  // Serial.println("Booting");

  wifi_on();

  pinMode(RTS_pin, OUTPUT);
  pinMode(RX, INPUT);
  // Start the Modbus serial Port RS485
  RS485Serial.begin(19200);
  // RS485 Einlesen
  digitalWrite(RTS_pin, RS485Receive);      // Init Receive

  ArduinoOTA.begin();
  mqttreconnect();
  client.setKeepAlive(5);
  ArduinoOTA.handle();

  client.publish("info", "Booting");

  sprintf(msg, "%d", deltatime );
  client.publish("deltamin", msg);

  sprintf(msg, "%.4f", UDfaktor );
  client.publish("UDfaktor", msg);

  sprintf(msg, "%.3f", 0);
  client.publish("kwh", msg);

  for (int r = 0; r < 10; r++)
  {
    Kessel.d[r] = 0.0;
  }

  // 10 Werte ausgeben um zu schauen ob die ser. tut und beim MQ alles ankommt...
  for (int r = 0; r < 10; r++)
  {
    rec[r] = readbyte();
  }

  // Speedtest

  unsigned long ts = millis();
  double x;

  // Setze CPU Speed auf 80mHz
  REG_CLR_BIT(0x3ff00014, BIT(0));
  os_update_cpu_frequency(80);
  ts = millis();
  x = 1;

  for (int i = 0; i < 10000; i++) {
    x = x + 1.2;
  }
  sprintf(msg, "%d %f", millis() - ts, x );
  client.publish("Speedtest", msg);

  // Setze CPU Speed auf 16mHz
  REG_SET_BIT(0x3ff00014, BIT(0));
  os_update_cpu_frequency(160);
  ts = millis();
  x = 1;
  for (int i = 0; i < 10000; i++) {
    x = x + 1.2;
  }
  sprintf(msg, "%d %f", millis() - ts, x );
  client.publish("Speedtest", msg);

  int r = 0;
  sprintf(msg, "bytes read RS485: %d %d %d %d %d %d %d %d %d %d", r, rec[r++], rec[r++], rec[r++], rec[r++], rec[r++], rec[r++], rec[r++], rec[r++], rec[r++], rec[r++] );
  client.publish("rec", msg);

  //sprintf(msg, "startrec: bytecount:%d/%d/%d", bytecount, waitcount, longwaitcount);
  //client.publish("info", msg);

  timerboot = millis();
  timer1 = millis();
  timeru = millis();
  keeptime = millis();
  timerd = millis();


  // Impulsstat
  timerpause = millis();
  id[0] = millis();
  ic++;


}



//////////////////////////////////////////////////////////////////////////
///////////////////// L O O P ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void loop() {
  unsigned char anData[256];
  int nDataLen;
  int byte;
  char msg[500], data[500];
  int nID;;
  int i, r ;
  int value;
  unsigned long  milli = 0;

  // Setze CPU Speed auf 16mHz
  //REG_SET_BIT(0x3ff00014, BIT(0));
  //os_update_cpu_frequency(160);


  // Datenframe vom RS485 einlesen
  if (readframe(anData, nID, nDataLen))
  {
    milli = millis();


    ///////////////////////////////////
    // Control MSG  und Neu

    if (nID == 33)
    {
      //mqttreconnect();
      // sprintf(msg, " ID %d /Checksum %d/%d nDatalen:%d", nID, nChecksum, nCrc,nDataLen );
      //client.publish("Received", "messeq");

      //for (i = 0; i < nDataLen; i++) c[i] = anData[i];

      Kessel.Leistung = lfaktor *  getval2(anData, 12, 2, 0.05, 0);
      Kessel.Pumpepuffer = getbit(anData, 2, 7);
      Kessel.Zuendung = getbit(anData, 16, 2);
      Kessel.KeineStoerung = getbit(anData, 3, 0);
      Kessel.Reinigung = getbit(anData, 3, 7);
      Kessel.Drehrost = getbit(anData, 3, 6);
      Kessel.Raumaustragung = getbit(anData, 9, 2);
      Kessel.RLAVentil = getbit(anData, 2, 3);

      // kwh summieren
      double deltat = (milli - kwhtimer) / (3600.0 * 1000.0); // in h

      if (Kessel.Leistung > 1) {
        Kessel.Brennerstunden += deltat; // Wenn der Kessel läuft
      }
      Kessel.kwh += Kessel.Leistung * deltat;
      kwhtimer = milli;
    }

    ///////////////////////////
    // Sense Paket empfangen

    if (nID == 32)
    {

      //for (i = 0; i < nDataLen; i++) s[i] = anData[i];
      // Photodiode
      Kessel.photo =  getval2(anData, 32, 2, 0.1, 1);
      Kessel.Kesseltemperatur = getval2(anData, 12, 2, 0.1, 1);
      Kessel.Rauchgastemperatur = getval2(anData, 20, 2, 0.1, 1);
      Kessel.Proztemperatur = getval2(anData, 22, 2, 0.1, 1);
      Kessel.Unterdruck = getval2(anData, 34, 2, 0.1, 1);
      Kessel.Saugzug = getval2(anData, 69, 2, 0.6, 0);
      Kessel.Geblaese = getval2(anData, 71, 2, 0.6, 0);
      Kessel.ext = getbit(anData, 4, 7);
      Kessel.Hauptantriebimpuls = getbit(anData, 3, 7);

      if (Kessel.Hauptantriebimpuls != oKessel.Hauptantriebimpuls )
      { // Hauptantrieb läuft und produziert Impulse

        Kessel.HauptantriebUD++; // vollst. Takte zählen

        idl[ic] += errorcounter - last_errorcount;
        // impulsdauer speichern Millisekunden
        id[ic] = milli;
        ifc[ic] = framecounter;
        ic = (ic + 1) % 300;
        idl[ic] = 0;

        last_errorcount = errorcounter;

        if ((milli - timerimpuls) < 1000)  // Wenn der letzte impuls weniger als 1s her ist läuft der HA
        {
          if (Kessel.Hauptantrieb == 0)
          {
            Kessel.Hauptantrieb = 1;
            timerhazeit = milli;
          }
        }
        timerimpuls = milli;

        oKessel.Hauptantriebimpuls = Kessel.Hauptantriebimpuls;

      } // Impulsende


      if ((milli - timerimpuls) >= 500) // Wenn der letzte impuls länger  als 1s her ist, ist der HA aus
      {
        if (Kessel.Hauptantrieb)
        {
          Kessel.Hauptantrieb = 0;
          Kessel.Hauptantriebzeit += (milli - timerhazeit);
        }

      }



      // Debug sense ausgeben
      //if (0)
      //{
      //  for (int j = 0; j < 15; j += 4)
      //  {
      //    sprintf(msg, "%d %s %s %s %s", j, inttobin(anData[j]), inttobin(anData[j + 1]), inttobin(anData[j + 2]), inttobin(anData[j + 3]));
      //    client.publish("sensedata", msg);
      //  }
      //}
      //sprintf(msg, "-Frame:%d> type:%d[%d] / nlen:%d / nid:%d / nCounter / %d / nDatalen: %d /Checksum %d/%d", count, nType, nState, nLen, nID, nCounter, nDataLen, nChecksum, nCrc );
      //client.publish("info", msg);
      //}
    } // Ende Sense Paket auslesen

    // Raumaustragung = SchneckeBunker

    if ((Kessel.Raumaustragung == 0) && (oKessel.Raumaustragung == 1))
    {
      Kessel.Schneckenlaufzeit = (milli - timerschnecke) / 1000;
      if (Kessel.Schneckenlaufzeit > 800) Kessel.Schneckenlaufzeit = 0;

      Kessel.Schneckengesamtlaufzeit += Kessel.Schneckenlaufzeit;
    }
    if ((Kessel.Raumaustragung == 1) && (oKessel.Raumaustragung == 0))
    {
      timerschnecke = milli;
    }
    if ((Kessel.Raumaustragung == 0) && (oKessel.Raumaustragung == 0))
    {
      Kessel.Schneckenlaufzeit = 0;
    }
    //oKessel.Raumaustragung = Kessel.Raumaustragung;

  } // Ende Paket mit korr. Prüfsumme empfangen


  // Wichtig!!!
  // alle x Sekunden ## Update, damit OTA und Ping tun
  if (milli > (timeru + 5 * 1000))
  {
    idl[ic] += 100;
    timeru = milli;
    ArduinoOTA.handle();
    client.loop(); // Zeit für den Callback+MQTT Ping
  }


  // manche Werte direkt ausgeben,  wenn eine Änderung da ist
  // Wenn die Schnecke stehen geblieben ist und vorher lief Schneckenaufzeitausgeben
  if  (Kessel.Raumaustragung != oKessel.Raumaustragung)
  {
    if (Kessel.Raumaustragung == 0)
    {
      // live reporting Schneckenlaufzeitausgabe deaktiieren
      //mqttreconnect();
      //sprintf(msg, "%d", Kessel.Schneckenlaufzeit);
      //client.publish("Schneckenlaufzeit", msg);
      oKessel.Schneckenlaufzeit = Kessel.Schneckenlaufzeit;
    }
    oKessel.Raumaustragung = Kessel.Raumaustragung;

  }
  // braucht man nicht wirklich immer im Log
  //    if (Kessel.Drehrost != oKessel.Drehrost)
  //      {
  //        mqttreconnect();
  //        sprintf(msg, "%d", Kessel.Drehrost);
  //        client.publish("Drehrost", msg);
  //        oKessel.Drehrost=Kessel.Drehrost;
  //      }

  //  if (Kessel.ext != oKessel.ext)
  //  {
  //    if (Kessel.ext)
  //      // Wenn Anforderung da ist WIFI abschalten
  //      wifi_off();
  //    else
  //      wifi_on();
  //  }

  if (Kessel.Reinigung != oKessel.Reinigung)
  {

    mqttreconnect();
    sprintf(msg, "%d", Kessel.Reinigung);
    client.publish("Reinigung", msg);
    oKessel.Reinigung = Kessel.Reinigung;

  }

  if (Kessel.Zuendung != oKessel.Zuendung)
  {
    mqttreconnect();
    sprintf(msg, "%d", Kessel.Zuendung);
    client.publish("Zuendung", msg);
    oKessel.Zuendung = Kessel.Zuendung;
  }


  //////////////////// 5 min Block /////////////////////////////
  // Alle anderen Änderungen werden erst ausgegeben wenn
  // timer1 abgelaufen
  // wenn Änderung alle x*60 Sekunden Ausgabe an MQTT

  if (milli > (timer1 + deltatime * 60 * 1000))
  {

    // Wenn sich etwas an den Daten getan hat
    if (memcmp(&oKessel, &Kessel, sizeof(Kessel)))
    {
      mqttreconnect();

      for (i = 1; i < ic; i++)
      {
        sprintf(msg, "%d %d %d %d %d", i, id[i], idl[i], ifc[i], id[i] - id[i - 1]);
        //sprintf(msg, "%d %d %d", i, id[i], idl[i]);
        client.publish("id", msg);
      }
      ic = 1; id[0] = millis();

      sprintf(msg, "%d", (bytecounter * 1000) / (milli - timer1));
      client.publish("bytecounter", msg);
      bytecounter = 0;

      sprintf(msg, "%d", framecounter);
      client.publish("framecounter", msg);
      framecounter = 0;

      sprintf(msg, "%d", errorcounter);
      client.publish("errorcounter", msg);

      sprintf(msg, "%d", longwaitcount);
      client.publish("longwaitcount", msg);
      errorcounter = 0;


      //sprintf(msg, "%d", Kessel.HauptantriebUD);
      //client.publish("HauptantriebUD", msg);

      //sprintf(msg, "%d", Kessel.Hauptantriebzeit / 1000);
      //client.publish("Hauptantriebzeit", msg);

      if (Kessel.Hauptantriebzeit)
      {
        //sprintf(msg, "%f", (float)(1000 * Kessel.HauptantriebUD)/((float) Kessel.Hauptantriebzeit )) ;
        //client.publish("HauptantriebUDproSekunde", msg);
      }

      //sprintf(msg, "%d", (int)(((float)Kessel.Hauptantriebzeit) * UDfaktor * UDsek)/1000); // Ca. * Gramm/UD * UD/sek
      //client.publish("PelletsHA", msg);

      sprintf(msg, "%d", (int)((float)Kessel.HauptantriebUD * UDfaktor ) ); //  Gramm/UD * UD
      //client.publish("PelletsUDHA", msg);

      // Gesamtverbrauch in GR
      client.publish("PelletsUDHA", msg);

      // Messung Über Schneckenantrieb
      sprintf(msg, "%d", (int)(((float)Kessel.Schneckengesamtlaufzeit * NAfaktor)  ));

      client.publish("Pellets", msg);

      // akt Verbrauch berechnen
      if (Kessel.HauptantriebUD - UD)
      {
        int d, p;
        d = (Kessel.HauptantriebUD - UD) * 3600 * 1000 / (milli - timerd);
        p = (int)( ((float)(Kessel.Hauptantriebzeit - ZD)) * UDfaktor * UDsek / 1000.0 * 3600.0 * 1000.0 / (milli - timerd));

        sprintf(msg, "%d", Kessel.HauptantriebUD - UD);
        //client.publish("deltaUD", msg);
        //sprintf(msg, "%d", d);
        //client.publish("deltaUDh", msg);
        sprintf(msg, "%d", (int)(d * UDfaktor));
        //client.publish("deltaPelletsUDh", msg);
        client.publish("deltaPelletsh", msg);

        // Verbrauch pro Stunde gemessen über NA
        //sprintf(msg, "%d", (int) ((float)(Kessel.Schneckengesamtlaufzeit - SL) * NAfaktor * 1000.0 * 3600.0 / ( milli - timerd)));
        //client.publish("deltaPelletsNAh", msg);


        SL = Kessel.Schneckengesamtlaufzeit;

        UD = Kessel.HauptantriebUD;
        //sprintf(msg, "%d", (millis() - timerd) / 1000);
        //client.publish("deltat", msg);
        ZD = Kessel.Hauptantriebzeit;
        timerd = milli;
      }

      //////////////////////////////////////////////
      // Berechnung HA/NA Verhältnis
      // Alle xx Min  berechnen (-> ca. 1.9 wenn der sinkt gibt es Förderprobleme)

      if (milli > ( HANAtimer + 30 * 60  * 1000))
      {
        HANAtimer = milli;
        double v;

        if ((Kessel.Hauptantriebzeit - HAz) && (Kessel.Schneckengesamtlaufzeit - NAz)) // Wenn der HA lief
        {
          v = (float) (Kessel.Hauptantriebzeit - HAz) / ((float)(Kessel.Schneckengesamtlaufzeit - NAz) * 1000.0  ) ;
          sprintf(msg, "%f", v);
          client.publish("HANA", msg);
          NAz = Kessel.Schneckengesamtlaufzeit;
          HAz = Kessel.Hauptantriebzeit;
        }
      }

      if (Kessel.Schneckengesamtlaufzeit != oKessel.Schneckengesamtlaufzeit)
      {
        oKessel.Schneckengesamtlaufzeit = Kessel.Schneckengesamtlaufzeit;
        // braucht man nur zum debuggen
        //sprintf(msg, "%d", Kessel.Schneckengesamtlaufzeit);
        //client.publish("Schneckengesamtlaufzeit", msg);

        if (Kessel.Schneckengesamtlaufzeit)
        {
          sprintf(msg, "%.2f", ((float)Kessel.Hauptantriebzeit) / 1000 / ((float)Kessel.Schneckengesamtlaufzeit));
          client.publish("SchneckeSchnecke", msg);
        }


      }

      if (Kessel.KeineStoerung != oKessel.KeineStoerung)
      {
        sprintf(msg, "%d", 1 - Kessel.KeineStoerung);
        client.publish("Stoerung", msg);
      }

      if (Kessel.Leistung != oKessel.Leistung )
      {

        oKessel.Leistung = Kessel.Leistung;
        sprintf(msg, "%.1f", Kessel.Leistung);
        client.publish("Leistung", msg);
        if (Kessel.Leistung < 1.0 )
        {
          client.publish("deltaPelletsh", "0");
          //client.publish("deltaPelletsUDh", "0");
          //client.publish("deltaPelletsHAh", "0");
        }
      }

      if (Kessel.kwh != oKessel.kwh)
      {
        sprintf(msg, "%.3f", Kessel.kwh);
        client.publish("kwh", msg);
      }

      if (Kessel.Brennerstunden != oKessel.Brennerstunden)
      {
        sprintf(msg, "%.3f", Kessel.Brennerstunden);
        client.publish("Brennerstunden", msg);
      }


      if (Kessel.Pumpepuffer != oKessel.Pumpepuffer)
      {
        sprintf(msg, "%d", Kessel.Pumpepuffer );
        client.publish("Pumpepuffer", msg);
      }

      if (Kessel.Raumaustragung != oKessel.Raumaustragung)
      {
        sprintf(msg, "%d", Kessel.Raumaustragung);
        client.publish("Raumaustragung", msg);
      }

      if (Kessel.Kesseltemperatur != oKessel.Kesseltemperatur)
      {
        sprintf(msg, "%.1f", Kessel.Kesseltemperatur);
        client.publish("Kesseltemperatur", msg);
      }
      if (Kessel.Rauchgastemperatur != oKessel.Rauchgastemperatur)
      {
        sprintf(msg, "%.1f", Kessel.Rauchgastemperatur);
        client.publish("Rauchgastemperatur", msg);
      }

      if (Kessel.Saugzug != oKessel.Saugzug)
      {
        sprintf(msg, "%.0f", Kessel.Saugzug);
        client.publish("Saugzug", msg);
        sprintf(msg, "%.1f", Kessel.Unterdruck );
        client.publish("Unterdruck", msg);
      }
      if (Kessel.Geblaese != oKessel.Geblaese)
      {
        sprintf(msg, "%.0f", Kessel.Geblaese);
        client.publish("Geblaese", msg);
        if (Kessel.Geblaese > 10.0)
          client.publish("Kessel", "brennt");
        else
          client.publish("Kessel", "aus");
      }

      sprintf(msg, "%d", Kessel.ext);
      client.publish("Anforderung", msg);

      if (Kessel.photo != oKessel.photo)
      {
        sprintf(msg, "%d", ((int) (Kessel.photo + 255.0) * 100) >> 9);
        client.publish("photodiode", msg);
      }

      memcpy(&oKessel, &Kessel, sizeof Kessel);

    }

    timer1 = milli;

  } // timer1 Ausgabe alle 5min

}
