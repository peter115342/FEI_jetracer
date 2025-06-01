---
sidebar_position: 9
---

# Detekcia objektov s OpenCV a CUDA

Tento návod pokrýva nastavenie schopností detekcie objektov na vašom FEI JetRacer pomocou OpenCV s CUDA akceleráciou a YOLOv4-Tiny pre výkon v reálnom čase.

## Prehľad

Použijeme:

- **OpenCV s CUDA**: Pre akcelerované operácie počítačového videnia
- **YOLOv4-Tiny**: Ľahký model detekcie objektov optimalizovaný pre embedded zariadenia
- **cuDNN**: NVIDIA knižnica pre akceleráciu hlbokého učenia

## Poznámka

**Ak používate náš predkonfigurovaný JetRacer image, časť tohto nastavenia je už kompletná!**

**Môžete preskočiť kroky inštalácie OpenCV s podporou CUDA nižšie a prejsť priamo na [Overenie inštalácie](#overenie-inštalácie).**

## Manuálna inštalácia

### Krok 1: Inštalácia OpenCV s podporou CUDA

Použijeme inštalačný skript Qengineering, ktorý automatizuje komplexný proces buildovania OpenCV s podporou CUDA a cuDNN.

#### Stiahnutie inštalačného skriptu

```bash
cd ~
```

```bash
git clone https://github.com/Qengineering/Install-OpenCV-Jetson-Nano.git
```

```bash
cd Install-OpenCV-Jetson-Nano
```

#### Príprava vášho systému

Pred spustením inštalačného skriptu sa uistite, že je váš systém pripravený:

```bash
sudo apt update && sudo apt upgrade -y
```

```bash
sudo apt autoremove -y
```

Uvoľnite miesto na disku a zvýšte swap:

```bash
sudo systemctl disable nvzramconfig
```

```bash
sudo fallocate -l 4G /var/swapfile
```

```bash
sudo chmod 600 /var/swapfile
```

```bash
sudo mkswap /var/swapfile
```

```bash
sudo swapon /var/swapfile
```

```bash
sudo swapon -s
```

#### Spustenie inštalačného skriptu

> **Upozornenie**: Tento proces môže trvať 2-4 hodiny. Uistite sa o stabilnom napájaní a neprerušujte proces.

```bash
chmod +x ./OpenCV-4-11-0.sh
```

```bash
./OpenCV-4-11-0.sh
```

Skript bude:

- Inštalovať všetky potrebné závislosti
- Stiahnuť zdrojový kód OpenCV
- Konfigurovať build s podporou CUDA
- Kompilovať OpenCV (toto trvá najdlhšie)
- Inštalovať skompilované knižnice

#### Overenie inštalácie

Po dokončení inštalácie overte OpenCV s podporou CUDA:

```bash
python3 -c "import cv2; print('OpenCV version:', cv2.__version__); print('CUDA devices:', cv2.cuda.getCudaEnabledDeviceCount())"
```

Mali by ste vidieť výstup podobný tomuto:

```
OpenCV version: 4.11.1
CUDA devices: 1
```

### Krok 2: Konfigurácia prostredia

Uistite sa, že je inštalácia OpenCV správne nakonfigurovaná vo vašom DonkeyCar prostredí:

```bash
source ~/env/bin/activate
```

```bash
cd ~/mycar
```

Otestujte OpenCV vo vašom virtuálnom prostredí:

```bash
python3 -c "import cv2; print('OpenCV loaded successfully with version:', cv2.__version__)"
```

## Krok 3: Nastavenie YOLOv4-Tiny

Teraz pridáme model detekcie objektov YOLOv4-Tiny do vášho DonkeyCar nastavenia.

### Vytvorenie adresára modelov

```bash
cd ~/mycar
```

```bash
mkdir -p models
```

```bash
cd models
```

### Stiahnutie súborov YOLOv4-Tiny

Stiahnite predtrénované súbory modelu YOLOv4-Tiny:

```bash
wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4-tiny.weights
```

```bash
wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg
```

```bash
wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/data/coco.names
```

### Overenie súborov modelu

Skontrolujte, či sú všetky požadované súbory prítomné:

```bash
ls -la ~/mycar/models/
```

Mali by ste vidieť:

- `yolov4-tiny.weights` (približne 23MB)
- `yolov4-tiny.cfg` (konfiguračný súbor)
- `coco.names` (súbor so štítkami tried)

### Test detekcie objektov

Otestujte spustením webového rozhrania DonkeyCar:

```bash
cd ~/mycar
```

```bash
python manage.py drive
```

Mali by ste vidieť detekciu objektov v kamerovom feed-e vo webovom rozhraní.

## Tipy na výkon

### Optimalizácia pre výkon v reálnom čase

1. **Ladenie rozlíšenia**: Vyvážte presnosť a rýchlosť úpravou vstupného rozlíšenia

2. **Frekvencia detekcie**: Spúšťajte detekciu každých niekoľko snímok namiesto každej snímky

3. **Spracovanie ROI**: Spracovávajte len oblasti záujmu v obrázku

## Riešenie problémov

### Problémy s inštaláciou OpenCV

Ak inštalačný skript zlyhá:

1. **Nedostatočné úložisko**: Uistite sa, že máte aspoň 16GB voľného miesta
2. **Problémy s pamäťou**: Zvýšte veľkosť swap súboru
3. **Problémy s napájaním**: Použite adekvátne napájanie

### CUDA nefunguje

Ak CUDA akcelerácia nefunguje:

```bash
# Skontrolujte inštaláciu CUDA
nvcc --version
```

```bash
# Overte cuDNN
cat /usr/local/cuda/include/cudnn_version.h | grep CUDNN_MAJOR -A 2
```

### Chyby načítania modelu

Ak sa YOLO modely nedajú načítať:

1. Overte integritu súborov:

   ```bash
   md5sum ~/mycar/models/yolov4-tiny.weights
   ```

2. Skontrolujte oprávnenia súborov:
   ```bash
   chmod 644 ~/mycar/models/*
   ```

## Referencie

- [Qengineering OpenCV inštalačný skript](https://github.com/Qengineering/Install-OpenCV-Jetson-Nano)
- [YOLOv4-Tiny článok](https://arxiv.org/abs/2004.10934)
- [OpenCV DNN modul dokumentácia](https://docs.opencv.org/master/d2/d58/tutorial_table_of_content_dnn.html)
- [NVIDIA Jetson Nano Developer Kit](https://developer.nvidia.com/embedded/jetson-nano-developer-kit)
