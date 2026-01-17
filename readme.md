# VfR NOKK

## Bedienung

### Status LED

| LED | Bedeutung |
| --- | --- |
| rot blinken | Fahrzeug nicht bereit |
| rot | Fahrzeug bereit (Drehschalter Mitte) |
| grün | Fahrzeug fährt mit Vor-Ort Bedienung (Drehschalter links) |
| blau | Fahrzeug fährt mit Fernbedienung (Drehschalter rechts) |
| blau blinken | Fahrzeug Fernbedienung nicht verbunden (Drehschalter rechts) |
| weiß  blinken | Fehler im Fahrzeug |

### Fahrzeug in Betrieb nehmen

1. Hauptschalter ausschalten
2. Prüfen: Fahrzeug links anheben, Radantrieb links lässt sich **nicht frei** drehen
3. Prüfen: Fahrzeug rechts anheben, Radantrieb rechts lässt sich **nicht frei** drehen
4. Hauptschalter einschalten
5. Not-Aus lösen
6. Prüfen: Status LED leuchtet
7. Prüfen: Fahrzeug links anheben, Radantrieb links lässt sich **frei** drehen
8. Prüfen: Fahrzeug rechts anheben, Radantrieb rechts lässt sich **frei** drehen
9. Not-Aus betätigen
10. Prüfen: Status LED leutet nicht

### Fahren auf Fahrzeug

1. Not-Aus betätigen
2. Aufsteigen
3. Not-Aus lösen
4. Drehschalter in Mittelstellung
5. Prüfen: Status LED leuchtet rot
6. Drehschalter nach **links**
7. Prüfen: Status LED leutet grün
8. Joystick betätigen: Fahrzeug bewegt sich
9. Joystick loslassen: Fahrzeug bremt bis Stillstand
10. Not-Aus betätigen: Fahrzeug schaltet ab, Radantrieb ist gehemmt
11. Taster drücken: Licht/Beine schaltet ein/aus

### Controller verbinden

1. Not-Aus lösen
2. Joycon beliebige Taste drücken
3. Prüfen: LEDs am Joycon blinken langsam (verbunden mit Controller)
4. Falls nicht verbunden, Handschlaufe abziehen und Sync Taste lange drücken bis verbunden

### Fahren mit Fernbedienung

1. Not-Aus lösen
2. Drehschalter in Mittelstellung
3. Prüfen: Status LED leuchtet rot
4. Drehschalter nach **rechts**
5. Prüfen: Status LED leutet blau
6. Falls LED blau blinkt, dann Controller verbinden
7. Joystick betätigen: Fahrzeug bewegt sich
8. Joystick loslassen: Fahrzeug bremt bis Stillstand
9. Taste X drücken: Licht/Beine schaltet ein/aus

### Fahrzeug laden

1. Hauptschalter ausschalten
2. Not-Aus betätigen
3. Ladegerät einstecken
4. Prüfen: Ladegerät LED leuchtet rot
5. Warten bis: Ladegerät LED leuchtet grün

## Dokumentation

Die Stromversorgung und Motorregelung befindet sich zwischen den Hauptantriebsrädern. Gespeist wird die Elektrik aus einem 36V Akku. Der Hauptschalter S2 (Kippschalter) und der Not-Aus Schalter S1 geben das Relais K1 frei. Beide Schalter müssen geschlossen sein, damit das System versorgt wird. Ist einer der beiden Schalter geöffnet, wird das Fahrzeug nicht versorgt und stattdessen der Ladeeingang mit dem Akku verbunden.

Im Bedienpanel vor dem Bedienersitz befinden sich der Not-Aus Schalter S1 und die Hauptsteuerung, die aus den Bedienereingaben am Bedienpanel bzw. der Joycon Bluetooth Fernbedienung die Stellgrößen für den Hauptantrieb, die Beleuchtung und den Nebenmotor (Beinantrieb) errechnet.

Wenn das System nicht versorgt wird (S1 oder S2 offen), werden beide Hauptantriebe gehemmt. Das ist notwendig, um das Fahrzeug passiv zu bremsen, wenn der Not-Aus während der Fahrt betätigt wird. Die Hemmung findet über die Relais K2 und K3 statt. Wenn das System nicht versorgt wird, schließen K2 bzw. K3 die drei Phasen des jeweiligen Motors über einen Brückengleichrichter kurz. Dadurch wird die Generatorleistung der Motoren über die Motorwicklungen (ca. 0,3 Ohm) in Wärme abgeführt, und das Fahrzeug so gebremst.

![Schaltplan](/doc/schaltplan.png)

### Pinbelegung ESP32 Board

| Belegung              | Ader  | PIN       |       | PIN           | Ader  | Belegung          |
| ---                   | ---   | ---       | ---   | ---           | ---   | ---               |
| -                     | -     | EN        |       | GPIO23        | OR    | Pad-2 (MOSI)      |
| -                     | -     | GPIO36    |       | GPIO22        | -     | -                 |
| -                     | -     | GPIO39    |       | GPIO1         | -     | (blockiert UART0) |
| Joystick analog X     | RD    | GPIO34    |       | GPIO3         | -     | (blockiert UART0) |
| Joystick analog Y     | WH    | GPIO35    |       | GPIO21        | RD    | Taster            |
| -                     | -     | GPIO32    |       | GPIO19        | GN    | Pad-1 (MISO)      |
| -                     | -     | GPIO33    |       | GPIO18        | YE    | Pad-7 (CLOCK)     |
| Reserve               | WH    | GPIO25    |       | GPIO5         | OR    | Pad-6 (CS)        |
| PWM Licht             | BU    | GPIO26    |       | GPIO17        | WH    | UART ODESC TX     |
| UART Modbus RX        | WH    | GPIO27    |       | GPIO16        | OR    | UART ODESC RX     |
| UART Modbus TX        | GN    | GPIO14    |       | GPIO4         | RD    | Status LED rot    |
| Drehschalter links    | BK    | GPIO12    |       | GPIO2         | BU    | Status LED blau   |
| Drehschalter rechts   | BK/WH | GPIO13    |       | GPIO15        | GN    | Status LED grün   |
| 0V (Eingang)          | BN    | GND       |       | GND           | BK    | 0V                |
| 5V (Eingang)          | WH    | VIN       |       | 3.3V          | RD    | 3.3V              |

### Pinbelegung RJ-45 Anschlussleitung

| PIN | Farbe | Belegung |
| --- | --- | --- |
| 1 | Orange/Weiß | UART ODESC RX |
| 2 | Orange | UART ODESC TX |
| 3 | Grün/Weiß | Modbus RS485 A+ |
| 4 | Blau | PWM Licht |
| 5 | Blau/Weiß | (Reserve) |
| 6 | Grün | Modbus RS485 B- |
| 7 | Braun/weiß | 5V (Eingang) |
| 8 | Braun | 0V (Eingang) |
