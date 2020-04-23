## Beschreibung des Codes

### Auswahl der Programmiersprache
Zuerst haben wir uns noch überlegt, die Berechnung der Winkel am PC zu erledingen und danach diese über die Serielle Schnittstelle an den Arduino zu übertragen.
Da wir dann auch gemerkt haben dass es gut wäre, wenn der Arm auch ohne PC funktionsfähig sein sollte, war diese Idee wieder vom Tisch und wir mussten die Berechnung in C++ implementieren.
Zum Glück läuft der normale C++ Code auch auf dem Arduino wenn man nicht allzu viele Bibliotheken verwendet.

### ino_builder.py
Dieses Python-Skript hat Herr Bader geschrieben, um das PC-C++ Projekt in den Arduino-Sketch zu kopieren.
So konnten wir auf dem richtigen PC entwickeln und testen und haben den Code dann auch gleich im Arduino-Sketch und können ihn dort verwenden.
Uns ist durchaus bekannt, dass man auch im Arduino-Sketch mehrere Dateien inkludieren kann.
Dieser Weg erschien uns aber einfacher, da man sonst alle Dateien doppelt hätte und die Verwirrung grösser wäre.
Zuerst werden alle .hpp und alle .cpp Dateien gelesen und in einer Liste gespeichert.
Dann werden die Header topologisch sortiert, sodass jede Deklaration erst nach allen ihren Abhängigkeiten kommt. Bei den .cpp-Dateien ist dies nicht nötig.
Dann werden noch einige Anpassungen gemacht, um das Programm auf dem Arduino lauffähig zu machen. 
Zum Beispiel wird folgende Zeile Standard-C++
```c++
cout << abc << xyz << EOL;
```
Zu diesem Arduino-C++ Code konvertiert:
```c++
Serial.print(abc);
Serial.println(xyz);
```

### interactive_calc3d.py
Dieses Skript war nicht von Anfang an geplant. Herr Bader hat es geschrieben, um seine Berechnungen der inversen Kinematik besser debuggen zu können.
Es besteht aus vier Slidern, mit welchen man X, Y, Z und Omega (Winkel des Greifers relativ zum Horizont) einstellen kann. Darunter hat es eine Anzeige der Ausgabe der Berechnung als Text.
Zudem hat es noch eine zweidimensionale Darstellung des Roboterarms, welche auch laufend aktualisiert wird.

### Inverse Kinematik
Inverse Kinematik ist ein komplexes Thema. Es ist erstaunlich schwierig, das nachzuprogrammieren was ein Mensch ohne zu überlegen kann, nämlich die Berechnung der einzelnen Gelenkwinkel aus der Zielposition.
Wir haben dies bereits beim Design berücksichtigt und darum sind die Achsen der Gelenke 2, 3 und 4 alle parallel. So kann der Hauptteil der Berechung in einer Ebene, also zweidimensional berechnet werden.
Die Berechnung wird in `RobotArm::calc3d` und `RobotArm::internal_calc2d` mithilfe des Satzes von Pythagoras und Trigonometrie durchgeführt.
 
Da die meisten Gelenke mithilfe von Verbindungsstangen (Viergelenkgetriebe) bewegt werden, ist dort ebenfalls eine Berechnung nötig, um vom Gelenkwinkel auf den Servowinkel zu kommen.
Diese Funktionalität ist in `Coupling::getServoAngle` implementiert, ebenfalls mithilfe von Trigonometrie.
Die Funktion `coupling_calculator()` in `main.cpp` kann die Länge des Verbinders und des Gelenkradius aus dem Winkelbereich des Gelenkes berechnen.
Da diese Funktion nicht laufzeitkritisch ist, ist die Implementation nicht sonderlich effizient, dafür sind die Formeln viel einfacher als sie ohne Brute-Force wären.

### main.cpp
In dieser Datei findet man alle Funkionen, die nur auf dem PC gebraucht werden. Hauptsächlich sind es Funktionen, die dem Testing dienen. Sie wird nicht in den Arduino-Sketch kopiert, da sie dort nicht gebraucht wird und nur stören würde.
