esp-kwb-mqttlogger

Um aus unserem KWB Easyfire2 mehr Daten zu Optimierung und Verbrauch herauszubekommen, 
hab ich mich an die Entwicklung eines kleinen Datenloggern auf Basis eines ESP Wemos D1 mini gemacht. 
Dieser läuft nun seit Knapp 2 Jahren stabil und analysiert die Daten des Kessels und loggt diese auf einen 
MQTT Server unter fhem.

Damit können neben den aktuellen Verbrauchsswerten auch Störungen und der Gesamtverbrauch ausgegeben werden.
Aktuell sind dies:
<Bildfhem>

Die Daten können verwendet werden um nette Graphen oder auch Alarme zu generieren
<ftui>

Hardware:
wemos d1 min pro 
MAX485 
TL7800

Software:
Arduiono Programm (C++) esp-kwb-mqttlogger. Dieses liest die Daten kontinuierlich über den RS485 des Kessels ein 
und überträgt die per WLAN an den MQTT-FHEM. 

Die Verdrahtung ist recht simpel, über den Modbus des Kessels liest  der 
MAX485 Werte, damit er die Kesselsteuerung nicht verwirrt, im Grunde könnte man damit aber auch Steuern…. 

Materialkosten liegen bei ca. 15€ 

Viel Spaß beim basteln