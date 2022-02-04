
# esp-kwb-mqttlogger

Um aus unserem KWB Easyfire2 mehr Daten zu Optimierung und Verbrauch herauszubekommen, hab ich mich an die Entwicklung eines kleinen Datenloggers auf Basis eines ESP Wemos D1 mini gemacht. 

Dieser läuft nun seit knapp 2 Jahren stabil, analysiert die Daten des Kessels und loggt diese auf einen MQTT Server unter fhem.esp-kwb-mqttlogger

Damit können neben den aktuellen Verbrauchsswerten auch Störungen und der Gesamtverbrauch ausgegeben werden.

Aktuell sind dies:

<Bildfhem>![fhemwerte.png](https://github.com/windundsterne/esp-kwb-mqttlogger/blob/main/Bilder/fhemwerte.png?raw=true)

Die Daten können verwendet werden, um nette Graphen oder auch Alarme zu generieren:

<Bildftui>![ftuigraph.png](https://github.com/windundsterne/esp-kwb-mqttlogger/blob/main/Bilder/ftuigraph.png?raw=true)
    
## Hardware:

* wemos d1 min pro 
* MAX485 
* TL7800
    
Die Verdrahtung ist recht simpel, über den Modbus des Kessels liest  der MAX485 Werte, damit er die Kesselsteuerung nicht verwirrt, im Grunde könnte man damit aber auch Steuern…. 
Der Kessel verfügt schon über einen entsprechenden RS485 Stecker, der auch die 24V Versorgungspannung liefert. Diese wird mit einem TL7800 und einem kleinen Kühlkörper für den Webmos und den MX485 angepasst.
    
Die erste Version nutzte pin2 des Wemos, um über einen SoftwareSerial Daten des RS485 zu lesen. Die aktuelle Version verwendet stattdesse den RX und den integrierten UART (die alte Version kann noch über einen #define SWSERIAL 2 bei Bedarf umgeschaltet werden - macht aber eigentlich keinen Sinn. 

Materialkosten liegen bei ca. 15€ 

<Bildlogger>![PXL_20201004_160445885.jpg](https://github.com/windundsterne/esp-kwb-mqttlogger/blob/main/Bilder/PXL_20201004_160445885.jpg?raw=true)

## Software:

Arduiono Programm (C++) esp-kwb-mqttlogger. Dieses liest die Daten kontinuierlich über den RS485 des Kessels ein und überträgt die per WLAN an den MQTT-FHEM.
Es untersützt einen OTA-Update. 

Nach der Installation der entsprechenen Bibs die  define am Anfang des Codes unter individualisierungen einkommentieren (Wlan SSID und Passwort, MQTT IP anpassen...) und auf den Webmos schreiben. 
          
Großen Dank an die Vorarbeit von Dirk Abel im Forum: https://www.mikrocontroller.net/topic/274137 aus dessen Protokollbeschreibung viele Werte übernommen werden konnte. 

Viel Spaß beim basteln
Philip
