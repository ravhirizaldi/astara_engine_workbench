# ASTARA Engineering Workbench

**ASTARA Engineering Workbench** adalah perangkat lunak offline untuk simulasi
digital twin wahana Anthariksa dua tingkat dan pengujian *software-in-the-loop*
perangkat lunak penerbangan. Model mesinnya memakai nama seri Cendrawasih.

## Peringatan keselamatan

**HANYA UNTUK SIMULASI / BELUM DIVALIDASI.** Model propulsi dan aerodinamika
merupakan estimasi orde-rendah. Perangkat lunak ini bukan perangkat lunak
penerbangan tersertifikasi, bukan pengendali perangkat keras, dan bukan
pengganti CFD, uji terowongan angin, uji statik, keselamatan wilayah peluncuran,
atau kajian regulasi BRIN dan instansi terkait.

Perangkat lunak ini tidak memiliki antarmuka serial, jaringan, GPIO, pengapian, katup,
piroteknik, atau terminasi penerbangan.

## Menjalankan aplikasi

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 main.py
```

Menjalankan simulasi tanpa GUI:

```bash
python3 -m astara validate
python3 -m astara build-fsw
python3 -m astara simulate scenarios/anthariksa_reference_mission.json --seed 1
```

Hasil setiap simulasi disimpan di direktori `runs/` yang unik. Isinya mencakup
skenario, manifes, data kebenaran simulasi, telemetri perangkat lunak
penerbangan, log kejadian, grafik PNG, dan laporan PDF.

## Batas validitas

- Rentang referensi: ketinggian 10–100 km.
- Koefisien aerodinamika diperkirakan dari geometri sederhana, bukan CFD.
- Model propulsi memakai parameter `c*`, koefisien dorong, efisiensi nosel, dan
  laju aliran yang harus dikalibrasi.
- Sampel di luar batas Mach atau sudut serang ditandai `UNVALIDATED`.
- Hasil harus dikorelasikan dengan analisis dan data uji sebelum dipakai untuk
  keputusan rekayasa.
