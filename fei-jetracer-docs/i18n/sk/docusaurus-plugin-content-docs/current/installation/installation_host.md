---
sidebar_position: 4
---

# Inštalácia Ubuntu virtuálneho stroja na hostiteľskom PC

Po [nastavení Jetson Nano](/docs/installation/installation_jetson) je ďalším krokom inštalácia Ubuntu virtuálneho stroja na vašom hostiteľskom počítači. Tento virtuálny stroj bude slúžiť ako vaše vývojové prostredie pre programovanie a ovládanie JetRacera.

## Prehľad

V tomto návode sa naučíte:

1. Stiahnuť a nainštalovať VMware
2. Stiahnuť predkonfigurovaný obraz Ubuntu VM
3. Importovať a nakonfigurovať virtuálny stroj
4. Pripojiť VM k vášmu JetRaceru

## Krok 1: Inštalácia VMware

Odkedy VMware prevzala spoločnosť Broadcom, inštalácia sa stala trochu zdĺhavou a musíte si vytvoriť účet Broadcom

1. Prejdite na [Broadcom Registráciu](https://profile.broadcom.com/web/registration) a vytvorte nový Broadcom účet
2. Počas registrácie budete požiadaní o pracovnú pozíciu, vyberte "other"
3. Po registrácii budete požiadaní o vytvorenie profilu. To nebudeme robiť, kliknite na: "I'll do it later".
4. Po registrácii prejdite na tento odkaz:
   [VMware Workstation Player 16.2.2](https://support.broadcom.com/group/ecx/productfiles?subFamily=VMware%20Workstation%20Player&displayGroup=VMware%20Workstation%20Player%2016&release=16.2.2&os=&servicePk=208715&language=EN&freeDownloads=true)
   Budete požiadaní o prihlásenie, pri prihlasovaní použite svoju e-mailovú adresu, ktorú ste použili pri vytváraní účtu, ako používateľské meno.
5. Stiahnite VMware Workstation 16.2.2 Player pre 64-bitové operačné systémy Windows, táto verzia bola testovaná a preukázalo sa, že funguje s našou VM.

## Krok 2: Stiahnutie image Ubuntu VM

1. Stiahnite predkonfigurovaný obraz Ubuntu VM z [Google Drive](https://drive.google.com/file/d/1u98h-GooQYXppDZCbRxvvfh2Zw3-cUcq/view?usp=sharing)
2. Rozbaľte stiahnutý .zip súbor na miesto vo vašom počítači (najlepšie s aspoň 20GB voľného miesta)

## Krok 3: Import VM do VMware

1. Otvorte VMware Workstation Player
2. Kliknite na rozbaľovacie menu Player v ľavom hornom rohu > File > Open
3. Vyhľadajte a vyberte rozbalený .vmx súbor
4. Kliknite na "Open"
5. Ak sa zobrazí výzva, či bol virtuálny stroj presunutý alebo skopírovaný, vyberte "I copied it"

## Krok 4: Konfigurácia virtuálneho stroja

### Konfigurácia systému

1. Vyberte importovaný VM a kliknite na "Edit virtual machine settings"
2. V časti "Memory" prideľte aspoň 4GB RAM (odporúča sa 8GB)
3. V časti "Processors" prideľte aspoň 2 procesorové jadrá
4. Kliknite na "OK" pre uloženie nastavení

### Konfigurácia siete

1. Prejdite na "Edit" → "Virtual Network Editor" (možno budete potrebovať administrátorské práva)
2. Uistite sa, že je povolené premostenie siete (Bridge networking) pre prístup na internet (malo by byť automatické)
3. Kliknite na "OK" pre uloženie nastavení

## Krok 5: Spustenie a prihlásenie do VM

1. Vyberte VM a kliknite na "Play virtual machine"
2. Počkajte, kým sa Ubuntu spustí (pri prvom spustení to môže trvať niekoľko minút)
3. Prihláste sa s predvolenými prihlasovacími údajmi:
   - Používateľské meno: `jetson`
   - Heslo: `jetson`

## Riešenie problémov

- **VM je pomalé**: Zvýšte pridelenú RAM a počet jadier CPU v nastaveniach VM
- **Nie je možné pripojiť sa na internet**: Uistite sa, že sieť je nastavená na Bridged(automatic)
- **VMware chyby pri spustení**: Uistite sa, že je na vašom PC povolená virtualizácia

## Ďalšie kroky

Teraz, keď máte nastavený Jetson Nano aj Ubuntu VM, ste pripravení pokračovať s ovládaním pohybov vášho JetRacera.

## Referencie

Pre podrobnejšie inštrukcie si pozrite [oficiálny návod Waveshare JetRacer ROS AI Kit Tutorial III](https://www.waveshare.com/wiki/JetRacer_ROS_AI_Kit_Tutorial_III%3A_Install_Ubuntu_Virtual_Image).
