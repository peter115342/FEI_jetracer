---
sidebar_position: 5
---

# Nastavenie komunikácie medzi viacerými zariadeniami

Po [inštalácii Ubuntu VM na váš hostiteľský PC](/docs/installation/installation_host) je ďalším krokom konfigurácia komunikácie medzi viacerými zariadeniami - medzi vašim hostiteľským VM a Jetson Nano. To umožňuje vášmu vývojovému počítaču diaľkovo komunikovať s JetRacerom.

## Prehľad

ROS (Robot Operating System) je navrhnutý tak, aby fungoval naprieč viacerými zariadeniami, čo vám umožňuje:

- Spúšťať výpočtovo náročné operácie na vašom výkonnejšom hostiteľskom PC
- Diaľkovo ovládať JetRacer
- Vizualizovať údaje zo senzorov a ladiace informácie na vašom vývojovom počítači
- Bezproblémovo nasadzovať kód medzi vývojovým a robotickým prostredím

V tomto návode sa naučíte:

1. Konfigurovať sieťové nastavenia na oboch zariadeniach
2. Nastaviť premenné prostredia ROS
3. Otestovať komunikáciu medzi viacerými zariadeniami
4. Riešiť bežné problémy

## Predpoklady

- Jetson Nano s nainštalovaným ROS a pripojený k vašej sieti
- Ubuntu VM bežiace na vašom hostiteľskom PC
- Obe zariadenia pripojené k rovnakej sieti

## Krok 1: Zistenie IP adries

Najprv potrebujete poznať IP adresy oboch zariadení:

### Na Jetson Nano:

Pozrite sa na OLED displej základnej dosky, kde uvidíte IP adresu (napr. `192.168.1.100`).

### Na Ubuntu VM:

```bash
ifconfig
```

Hľadajte rozhranie `ens33` alebo podobné a poznamenajte si IP adresu (napr. `192.168.1.200`).

## Krok 2: Konfigurácia súboru hosts na oboch zariadeniach

Na oboch zariadeniach budete musieť upraviť súbor hosts, aby ste umožnili rozlíšenie hostiteľských mien:

### Na Jetson Nano:

```bash
sudo nano /etc/hosts
```

Pridajte nasledujúce riadky:

```
127.0.0.1       localhost
127.0.1.1       jetson
192.168.1.100   jetson    # IP adresa vášho Jetson Nano
192.168.1.200   ubuntu    # IP adresa vašej Ubuntu VM
```

Uložte a ukončite (Ctrl+X, potom Y, potom Enter).

### Na Ubuntu VM:

```bash
sudo nano /etc/hosts
```

Pridajte nasledujúce riadky:

```
127.0.0.1       localhost
127.0.1.1       ubuntu
192.168.1.100   jetson    # IP adresa vášho Jetson Nano
192.168.1.200   ubuntu    # IP adresa vašej Ubuntu VM
```

Uložte a ukončite.

## Krok 3: Konfigurácia premenných prostredia ROS

ROS potrebuje vedieť, ktoré zariadenie je master a ako komunikovať medzi zariadeniami:

### Na Jetson Nano (ROS Master):

Upravte súbor `.bashrc`:

```bash
nano ~/.bashrc
```

Pridajte tieto riadky na koniec:

```bash
export ROS_MASTER_URI=http://jetson:11311
export ROS_HOSTNAME=jetson
```

Aplikujte zmeny:

```bash
source ~/.bashrc
```

### Na Ubuntu VM:

Upravte súbor `.bashrc`:

```bash
nano ~/.bashrc
```

Pridajte tieto riadky na koniec:

```bash
export ROS_MASTER_URI=http://jetson:11311
export ROS_HOSTNAME=ubuntu
```

Aplikujte zmeny:

```bash
source ~/.bashrc
```

## Krok 4: Test komunikácie

Teraz otestujeme, či komunikácia medzi viacerými zariadeniami funguje:

### Na Jetson Nano:

Spustite ROS master:

```bash
roscore
```

### Na Ubuntu VM:

Skontrolujte, či vidíte ROS master:

```bash
rostopic list
```

Ak vidíte zoznam tém (aj keby to bolo len `/rosout` a `/rosout_agg`), komunikácia funguje!

## Krok 5: Spustenie testovacieho príkladu

Spustime jednoduchý príklad publisher/subscriber na overenie komunikácie:

### Na Jetson Nano:

Spustite príklad prijímacieho uzla:

```bash
rosrun rospy_tutorials listener
```

### Na Ubuntu VM:

Spustite príklad publikujúceho uzla:

```bash
rosrun rospy_tutorials talker
```

Mali by ste vidieť správy "hello world", ktoré sa opakovane zobrazujú na strane Jetsonu.

## Riešenie problémov

### Nie je možné pripojiť sa k ROS Master

1. Overte, či sa zariadenia navzájom pingujú:

   ```bash
   ping jetson  # Z Ubuntu VM
   ping ubuntu  # Z Jetson Nano
   ```

2. Overte premenné prostredia ROS:
   ```bash
   echo $ROS_MASTER_URI
   echo $ROS_HOSTNAME
   ```

### Časové limity pripojenia

1. Uistite sa, že obe zariadenia sú v rovnakej sieti
2. Skontrolujte, či sa nezmenili IP adresy (bežné pri DHCP)
3. Aktualizujte súbor hosts, ak sa IP adresy zmenili

## Referencie

Pre podrobnejšie inštrukcie si pozrite [oficiálny návod Waveshare JetRacer ROS AI Kit Tutorial IV](https://www.waveshare.com/wiki/JetRacer_ROS_AI_Kit_Tutorial_IV%3A_Configure_Multi-machine_Communication).
