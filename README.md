
# esp-kwb-mqttlogger

Um aus unserem KWB Easyfire2 mehr Daten zu Optimierung und Verbrauch herauszubekommen, hab ich mich an die Entwicklung eines kleinen Datenloggern auf Basis eines ESP Wemos D1 mini gemacht. 

Dieser läuft nun seit Knapp 2 Jahren stabil und analysiert die Daten des Kessels und loggt diese auf einen MQTT Server unter fhem.esp-kwb-mqttlogger

Damit können neben den aktuellen Verbrauchsswerten auch Störungen und der Gesamtverbrauch ausgegeben werden.

Aktuell sind dies:

<Bildfhem>![IMG_2605.jpg](/148/BilderHaus148/148-VorReno/IMG_2605.jpg?fileId=3765#mimetype=image%2Fjpeg&hasPreview=true)

Die Daten können verwendet werden, um nette Graphen oder auch Alarme zu generieren

<WERTE Fhem>

## Hardware:

* wemos d1 min pro 
* MAX485 
* TL7800

## Software:

Arduiono Programm (C++) esp-kwb-mqttlogger. Dieses liest die Daten kontinuierlich über den RS485 des Kessels ein  und überträgt die per WLAN an den MQTT-FHEM. 

Die Verdrahtung ist recht simpel, über den Modbus des Kessels liest  der MAX485 Werte, damit er die Kesselsteuerung nicht verwirrt, im Grunde könnte man damit aber auch Steuern…. 

Materialkosten liegen bei ca. 15€ 
  
Großen Dank an die Vorarbeit von Dirk Abel im Forum: https://www.mikrocontroller.net/topic/274137 aus dessen Protokollbeschreibung viele Werte übernommen werden konnte. 

Viel Spaß beim basteln
