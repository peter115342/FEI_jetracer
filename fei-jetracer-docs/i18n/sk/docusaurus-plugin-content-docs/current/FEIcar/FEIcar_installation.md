---
sidebar_position: 8
---

# Inštalácia FEI DonkeyCar na Jetson Nano

Tento návod vás prevedie inštaláciou našej prispôsobenej verzie DonkeyCar na Jetson Nano pre projekt FEI JetRacer. Toto nastavenie umožňuje možnosti autonómneho riadenia pomocou strojového učenia a počítačového videnia.

## Prehľad

DonkeyCar je open-source platforma na stavanie a pretekanie autonómnych RC áut. Naša verzia zahŕňa:

- Prípravu na detekciu objektov pomocou YOLOv4-tiny
- Vylepšené webové rozhranie

> **Poznámka**: Predvolený DonkeyCar je už nainštalovaný v predvolenom nastavení, ak používate nakonfigurovaný image, takže v tomto návode preskakujeme niektoré kroky.

## Krok 1: Nastavenie virtuálneho prostredia

Vytvorte a aktivujte virtuálne prostredie Python na izoláciu inštalácie DonkeyCar:

```bash
pip3 install virtualenv
```

```bash
python3 -m virtualenv -p python3 env --system-site-packages
```

```bash
source env/bin/activate
```

> **Dôležité**: Všetky následné operácie DonkeyCar musia byť vykonané v rámci tohto virtuálneho prostredia. Nezabudnite spustiť `source env/bin/activate` zakaždým, keď otvoríte novú reláciu terminálu.

## Krok 2: Inštalácia FEI DonkeyCar

Naklonujte a nainštalujte našu prispôsobenú verziu DonkeyCar:

```bash
git clone -b 4_5_1_FEI https://github.com/peter115342/FEIcar
```

```bash
cd donkeycar
```

```bash
pip install -e .[nano]
```

## Krok 3: Vytvorenie aplikácie vášho auta

Vytvorte novú inštanciu aplikácie DonkeyCar:

```bash
donkey createcar --path ~/mycar
```

Tento príkaz vytvorí nový adresár `~/mycar` so všetkými potrebnými súbormi na spustenie vášho autonómneho auta. V našom FEIcar forku je kamera už predkonfigurovaná na prácu s naším JetRacer kitom.

## Overenie

Otestujte vašu inštaláciu spustením webového rozhrania DonkeyCar:

```bash
cd ~/mycar
```

```bash
python manage.py drive
```

Otvorte webový prehliadač a navigujte na `http://<jetson-ip>:8887` pre prístup k ovládaciemu rozhraniu.

## Riešenie problémov

### Problémy s virtuálnym prostredím

Ak sa stretnete s chybami importu, uistite sa, že ste vo virtuálnom prostredí:

```bash
source env/bin/activate
```

### Kamera nebola detegovaná

Ak kamera nefunguje:

1. Skontrolujte pripojenie kamery
2. Overte, že je kamera povolená v nastaveniach Jetsonu
3. Otestujte kameru pomocou:

```bash
nvgstcapture-1.0
```

### Problémy s pamäťou

Ak sa stretnete s chybami nedostatku pamäte počas inštalácie:

1. Zvýšte swap priestor
2. Zatvorte ostatné aplikácie
3. Inštalujte balíky po jednom

### Chyby oprávnení

Pre problémy súvisiace s oprávneniami:

```bash
sudo chown -R $USER:$USER ~/mycar
```

## Referencie

- [Pôvodný návod Waveshare DonkeyCar](https://www.waveshare.com/wiki/DonkeyCar_for_JetRacer_ROS_Tutorial_I%3A_Install_Jetson_Nano)
- [Oficiálna dokumentácia DonkeyCar](http://docs.donkeycar.com/)
