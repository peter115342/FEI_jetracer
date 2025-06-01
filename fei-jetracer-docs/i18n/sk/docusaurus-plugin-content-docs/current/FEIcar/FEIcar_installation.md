---
sidebar_position: 9
---

# Inštalácia FEI DonkeyCar(FEIcar) na Jetson Nano

Tento návod vás prevedie inštaláciou našej prispôsobenej verzie DonkeyCar na Jetson Nano pre projekt FEI JetRacer. Toto nastavenie umožňuje možnosti autonómneho riadenia pomocou strojového učenia a počítačového videnia.

## Rýchly štart - Predkonfigurovaný obraz (Odporúčané)

**Ak chcete preskočiť celý proces inštalácie**, poskytujeme predkonfigurovaný obraz SD karty s už nastaveným všetkým:

**Odkaz na stiahnutie**: [FEI JetRacer kompletný obraz](https://drive.google.com/file/d/1OVulgYBTdY4HOwtRhksuTYHidAfxd2wF/view?usp=sharing)

### Čo je zahrnuté v obraze

Predkonfigurovaný obraz obsahuje:

- Ubuntu 18.04 s JetPack 4.5
- FEI DonkeyCar s predvytvorenou mycar aplikáciou
- OpenCV s podporou CUDA a cuDNN
- YOLOv4-tiny pre detekciu objektov
- Všetky závislosti a knižnice predpripravené
- Pripravenú konfiguráciu pre JetRacer hardware

### Nahratie obrazu na SD kartu

1. **Stiahnite obraz** (približne 8GB komprimovaný)
2. **Rozbaľte obraz** ak je komprimovaný
3. **Nahrajte na SD kartu** pomocou jedného z týchto nástrojov:

   **Možnosť A: Raspberry Pi Imager (Odporúčané)**

   - Stiahnite [Raspberry Pi Imager](https://www.raspberrypi.org/software/)
   - Vyberte "Use custom image" a zvoľte stiahnutý súbor
   - Vyberte vašu SD kartu (32GB alebo väčšiu)
   - Kliknite "Write"

   **Možnosť B: Balena Etcher**

   - Stiahnite [Balena Etcher](https://www.balena.io/etcher/)
   - Vyberte súbor obrazu
   - Vyberte vašu SD kartu
   - Kliknite "Flash"

   **Možnosť C: Príkazový riadok (Linux/macOS)**

   ```bash
   # Nájdite vaše SD karta zariadenie (buďte opatrní!)
   lsblk

   # Nahrajte obraz (nahraďte /dev/sdX vaším SD karta zariadením)
   sudo dd if=fei-jetracer-image.img of=/dev/sdX bs=4M status=progress
   ```

4. **Vložte SD kartu** do vášho Jetson Nano a spustite

### Nastavenie prvého spustenia

Po nahratí a spustení:

1. Systém automaticky zmení veľkosť súborového systému
2. Predvolené prihlasovacie údaje:
   - Používateľské meno: `jetson`
   - Heslo: `jetson`
3. DonkeyCar prostredie je pripravené v `~/mycar`
4. Aktivujte virtuálne prostredie:
   ```bash
   source ~/env/bin/activate
   ```

**Ak použijete tento predkonfigurovaný obraz, môžete preskočiť zvyšok tohto návodu a taktiež [Nastavenie detekcie objektov](/docs/FEIcar/object_detection).**

---

## Manuálna inštalácia (Pokročilí používatelia)

Ak uprednostňujete inštaláciu všetkého manuálne alebo chcete pochopiť proces, pokračujte s nasledujúcimi krokmi.

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

## Krok 2: Inštalácia FEIcar

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

> **Poznámka**: Funkčnosť detekcie objektov bude dostupná až po dokončení [návodu na nastavenie detekcie objektov](./object_detection). Základné rozhranie DonkeyCar bude fungovať bez detekcie objektov, ale pokročilé funkcie ako YOLO detekcia objektov vyžadujú dodatočné nastavenie.

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
