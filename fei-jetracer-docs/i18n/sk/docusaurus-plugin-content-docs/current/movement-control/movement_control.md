---
sidebar_position: 6
---

# Ovládanie pohybu robota

## Krok 1: Pridanie používateľskej skupiny pre sériový port

Zadajte príkaz ls /dev v Jetson Nano na kontrolu, či je ovládacia doska správne pripojená k Jetsonu a či sa našli zariadenia ttyACM0 a ttyACM1 【kde ttyACM0 sa používa na komunikáciu s mikrokontrolérom a ttyACM1 sa používa na komunikáciu s radarom】.

```bash
ls /dev
```

Zadajte nasledujúci príkaz na pridanie používateľskej skupiny pre sériový port, inak nebudete mať oprávnenie na ovládanie sériového portu. Pri použití nakonfigurovaného systému boli už povolenia pridané, takže tento krok môžete preskočiť.

```bash
ls -l /dev/ttyACM*             #Zobrazenie používateľskej skupiny sériového portu ako dialout
id -Gn                         #Zobrazenie používateľskej skupiny, do ktorej patrí aktuálny používateľ, prvá je aktuálny používateľ
sudo adduser jetbot dialout    #Pridanie aktuálneho používateľa do používateľskej skupiny dialout, kde sa nachádza sériový port
```

Zadajte príkaz "sudo reboot" na reštartovanie, heslo je jetson 【Poznámka: Vyššie uvedený príkaz na pridanie nadobudne účinnosť až po reštartovaní, a keď nie je k dispozícii povolenie sériového portu, uzol podvozku robota bude po spustení stále hlásiť chybu】.

```bash
sudo reboot
```

## Krok 2: Spustenie uzla na ovládanie podvozku robota

Pripojte sa cez SSH k robotovi, otvorte terminál a zadajte nasledujúci príkaz na spustenie hlavného uzla robota.

```bash
roscore
```

Nasledujúci príkaz roslaunch tiež automaticky spustí hlavný uzol, ale účelom samostatného spustenia hlavného uzla je udržať trvalé pripojenie k virtuálke. V opačnom prípade sa hlavný uzol automaticky zatvorí, keď sa zatvorí uzol podvozku, čo spôsobí odpojenie virtuálneho stroja.
Zadajte nasledujúci príkaz na spustenie uzla podvozku robota.

```bash
roslaunch jetracer jetracer.launch
```

Otvorte terminál virtuálneho stroja Ubuntu a zadajte nasledujúci príkaz do terminálu na overenie, či je viac-počítačová komunikácia normálna.

```bash
rostopic list
```

Ak je toto obsah, ktorý získate po spustení príkazu, znamená to, že komunikácia medzi zariadeniami je úspešná:

- /cmd_vel je téma o pohybe robota a je ovládaná robotom.
- /imu je téma IMU robota.
- /motor/\* indikuje skutočnú kódovanú rýchlosť a nastavenú rýchlosť ľavého a pravého motora.
- /odom kóduje odometer pre robota.
- /odom_combined je fúzny odometer robota, ktorý sa získava kombináciou kódovaného odometra s údajmi IMU.

## Krok 3: Spustenie publikovania témy pre ovládanie

Môžete publikovať témy prostredníctvom virtuálneho stroja Ubuntu na ovládanie pohybu robota.

```bash
rosrun rqt_publisher rqt_publisher
```

V kontextovom okne vyberte tému /cmd_vel a kliknite na "+" na vytvorenie novej témy. /cmd_vel je téma na ovládanie pohybu robota, linear->x označuje lineárnu rýchlosť robota s rozsahom -1,2m/s až 1,2m/s, angular->z predstavuje uhol natočenia prednej pneumatiky robota, nie uhlovú rýchlosť, s rozsahom od -0,6 radiánov do 0,6 radiánov.
Zmeňte hodnotu linear.x na 0,5, kliknite pravým tlačidlom myši a vyberte publish selected Once na publikovanie témy. Robot sa posunie dopredu o 0,5 metra a potom sa zastaví. Ak zmeníte linear.x na 0 a angular.z na 0,6, robot sa otočí doľava s maximálnym uhlom pohybu.

Poznámka: Téma, ktorú tu otvárame, je cmd_vel, vyberte typ Twist. Ak vyberiete iné témy, auto sa nemusí pohnúť.

## Krok 4: Ovládanie pohybu robota klávesnicou

Otvorte nový terminál, spustite uzol na ovládanie klávesnicou na ovládanie pohybu robota pomocou klávesnice.

```bash
roslaunch jetracer keyboard.launch
```

Po úspešnom spustení sa zobrazí rozhranie zobrazujúce mapovanie ovládacích prvkov a pohyb robota môžete ovládať stláčaním klávesnice.

## Krok 5: Pripojenie virtuálneho stroja k ovládaniu gamepadom

Pripojte USB prijímač gamepadu k hostiteľskému počítaču. Zobrazí sa okno, vyberte Connect to Virtual Machine -> Ubuntu JetRacer

Spustite nasledujúci príkaz vo virtuálnom stroji na otestovanie, či je gamepad pripojený normálne (mali by ste vidieť vstupné zariadenie js0).

```bash
ls /dev/input/
```

Otestujte vstup gamepadu pomocou nasledujúceho príkazu

```bash
jstest /dev/input/js0
```

Stlačte rôzne tlačidlá na gamepade a sledujte zmeny zodpovedajúcich hodnôt.

Spustite uzol joysticku.

```bash
roslaunch jetracer joy.launch
```

Zapnite gamepad, stlačte tlačidlo HOME a rozsvieti sa červené svetlo, potom stlačte a podržte tlačidlo L1 v ľavom hornom rohu a súčasne ovládajte ľavý joystick doľava a doprava na ovládanie natočenia servomotorov robota a pravý joystick hore a dole na ovládanie pohybu robota dopredu a dozadu. Po uvoľnení tlačidla L1 sa robot zastaví.

## Riešenie problémov

### Robot sa nepohybuje

1. Overte, či je batéria nabitá
2. Uistite sa, že riadiaci uzol beží na Jetson Nano
3. Skontrolujte, či sa v termináli nezobrazujú chybové hlásenia

### Gamepad nie je rozpoznaný

1. Overte, či je gamepad správne pripojený k hostiteľskému PC
2. Uistite sa, že VM má prístup k USB zariadeniu
3. Skontrolujte, či sa používa správne zariadenie joysticku (napr. /dev/input/js0)

## Referencie

Pre podrobnejšie pokyny si pozrite [oficiálny návod Waveshare JetRacer ROS AI Kit Tutorial V: Ovládanie pohybu robota](https://www.waveshare.com/wiki/JetRacer_ROS_AI_Kit_Tutorial_V%3A_Robot_Movement_Control).
