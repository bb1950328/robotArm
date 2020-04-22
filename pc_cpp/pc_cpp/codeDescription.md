## Beschreibung des Codes

Zuerst haben wir uns noch überlegt, die Berechnung der Winkel am PC zu erledingen und danach diese über die Serielle Schnittstelle an den Arduino zu übertragen.
Da wir dann auch gemerkt haben dass es gut wäre, wenn der Arm auch ohne PC funktionsfähig sein sollte. Damit war diese Idee wieder vom Tisch und wir mussten die Berechnung in C++ implementieren.
Zum Glück läuft der normale C++ Code auch auf dem Arduino wenn man nicht allzu viele Bibliotheken verwendet.
Darum hat Herr Bader ein Python-Skript geschrieben, welches alle Header- und Source-Dateien einliest und dann in der richtigen Reihenfolge in den Arduino-Sketch kopiert.
So konnten wir auf dem richtigen PC entwickeln und testen und den Code mit einem Klick in den Arduino-Sketch kopieren und dort verwenden.
Uns ist durchaus bekannt, dass man auch im Arduino-Sketch mehrere Dateien inkludieren kann, aber dieser Weg erschien uns einfacher, da man sonst alle Dateien doppelt hätte und die Verwirrung gross wäre.
Das Konvertierungs-Skript (`ino_builder.py`) führt nach dem Einlesen der Dateien und einer topologischen Sortierung noch ein paar Anpassungen durch, um den Code auf dem Arduino lauffähig zu machen.
Zum Beispiel wird folgende Zeile Standard-C++
```c++
cout << abc << xyz << EOL;
```
Zu diesem Arduino-C++ Code konvertiert:
```c++
Serial.print(abc);
Serial.println(xyz);
```