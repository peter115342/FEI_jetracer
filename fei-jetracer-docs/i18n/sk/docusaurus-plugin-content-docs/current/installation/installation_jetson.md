---
sidebar_position: 3
---

# Inštalácia image pre Jetson Nano

Po [dokončení montáže](/docs/assembly/assembly) vášho JetRacera je ďalším krokom nastavenie softvérového prostredia inštaláciou image Jetson Nano.

## Prehľad

Jetson Nano vyžaduje správne nakonfigurovaný operačný systém založený na Ubuntu s potrebnými ovládačmi a softvérovými balíkmi na ovládanie komponentov JetRacera. Tento návod vás prevedie:

1. Programovaním image Jetson Nano na SD kartu
2. Nastavením Wi-Fi pripojenia
3. Zriadením vzdialeného prístupu k vášmu Jetson Nano
4. Overením inštalácie

## Krok 1: Nahranie image Jetson Nano

> **Poznámka:** Ak ste dostali vašu JetRacer súpravu s už pripravenou SD kartou, môžete tento krok preskočiť.

Pre inštaláciu operačného systému:

1. Zaobstarajte si SD kartu (odporúča sa minimálne 64GB)
2. Stiahnite si obraz JetRacer ROS z [Google Drive](https://drive.google.com/file/d/16OBLRNlrZaSkhVcC4xJ6VugtChmZw1_B/view?usp=sharing)
3. Naformátujte vašu SD kartu pomocou [SDFormatter](https://files.waveshare.com/upload/3/31/Panasonic_SDFormatter_%289%29.zip)
   > **Dôležité:** Odpojte ostatné úložné zariadenia, aby ste predišli formátovaniu nesprávneho zariadenia
4. Použite [Rufus](https://rufus.ie/en/) alebo [Balena Etcher](https://etcher.balena.io/) na nahranie ISO image na SD kartu
5. Bezpečne vysuňte SD kartu z počítača

## Krok 2: Pripojenie Jetson Nano k Wi-Fi

Jetson Nano môžete pripojiť k Wi-Fi buď s displejom alebo bez neho.

### Možnosť A: S displejom

1. Vložte SD kartu do slotu pre SD kartu na Jetson Nano (nachádza sa na zadnej strane modulu)
2. Pripojte periférne zariadenia (monitor, klávesnicu, myš) k Jetson Nano
3. Zapnite JetRacer pomocou vypínača
4. Prihláste sa s predvolenými prihlasovacími údajmi:
   - Používateľské meno: `jetson`
   - Heslo: `jetson`
5. Pripojte sa k Wi-Fi pomocou správcu sietí v Ubuntu rozhraní

### Možnosť B: Bez displeja (bezobslužné nastavenie)

Ak nemáte prístup k displeju:

1. Vytvorte textový súbor s názvom `wpa_supplicant.conf` s nasledujúcim obsahom:

   ```
   country=US
   ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
   update_config=1

   network={
       ssid="VAS_WIFI_NAZOV"
       psk="VASE_WIFI_HESLO"
       key_mgmt=WPA-PSK
   }
   ```

2. Nahraďte `VAS_WIFI_NAZOV` a `VASE_WIFI_HESLO` vašimi skutočnými Wi-Fi údajmi
3. Umiestnite tento súbor do koreňového adresára SD karty
4. Vložte SD kartu do Jetson Nano a zapnite ho

## Krok 3: Vzdialený prístup

### SSH vzdialené prihlásenie

1. Zistite IP adresu vášho Jetson Nano z OLED displeja na základnej doske alebo skontrolujte pripojené zariadenia na vašom routeri
2. Pripojte sa cez SSH z terminálu vášho počítača:
   ```bash
   ssh jetson@<IP-ADRESA>
   ```
3. Zadajte heslo po výzve (predvolené: `jetson`)

## Krok 4: Overenie inštalácie

Na overenie, či všetko funguje správne:

1. Prihláste sa do vášho Jetson Nano
2. Skontrolujte systémové informácie:
   ```bash
   uname -a
   ```
   Malo by sa zobraziť, že beží Linux na architektúre aarch64
3. Overte inštaláciu CUDA:
   ```bash
   nvcc --version
   ```
4. Skontrolujte inštaláciu ROS:
   ```bash
   rosversion -d
   ```

## Ďalšie kroky

Teraz, keď ste úspešne nainštalovali a pristúpili k vášmu Jetson Nano, ste pripravení pokračovať s inštaláciou Ubuntu virtuálneho stroja na vašom hostiteľskom PC.

## Riešenie problémov

- **Nie je možné pripojiť sa k Wi-Fi**: Skúste najprv použiť káblové pripojenie cez Ethernet, potom nastavte Wi-Fi
- **SSH pripojenie odmietnuté**: Uistite sa, že Jetson Nano sa úplne naštartoval (pri prvom spustení to môže trvať až 2 minúty)
- **Problémy s výkonom**: Jetson Nano sa môže počas prevádzky zahriať; uistite sa, že ventilátor chladiča je pripojený a beží

## Referencie

Pre podrobnejšie inštrukcie si pozrite [oficiálny návod Waveshare JetRacer ROS AI Kit Tutorial II](https://www.waveshare.com/wiki/JetRacer_ROS_AI_Kit_Tutorial_II%3A_Install_Jetson_Nano_Image).
