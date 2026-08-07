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
pip install -r requirements.lock
python3 main.py
```

GUI Qt menjalankan solver di proses terpisah. Tampilan utama memuat lintasan,
metrik langsung, serta tiga panel ringan untuk ketinggian, kecepatan, dan gaya
dorong. Data tampilan dibatasi dan diperbarui lebih lambat daripada metrik agar
GUI tetap responsif; telemetri lengkap tetap ditulis oleh solver ke laporan.

Rendering CPU adalah pengaturan baku dan paling stabil di WSL. OpenGL hanya
diaktifkan secara eksplisit pada komputer dengan driver EGL/OpenGL yang bekerja:

```bash
ASTARA_UI_OPENGL=1 python3 main.py
```

Jika muncul galat Mesa, EGL, atau Zink, gunakan kembali CPU:

```bash
ASTARA_UI_OPENGL=0 python3 main.py
```

Dock **Fault Injection** dapat menyuntikkan gangguan sensor atau mesin ketika
simulasi desktop sedang berjalan. Pilih wahana, komponen, jenis gangguan, nilai
jika diperlukan, dan durasi. Durasi nol tetap aktif sampai **Clear** ditekan.
Semua perintah divalidasi dan dicatat sebagai kejadian simulasi; berkas skenario
dan kendaraan yang tersimpan tidak diubah.

Menjalankan simulasi tanpa GUI:

```bash
awb validate
awb build-fsw
awb simulate configs/scenarios/anthariksa_reference_mission.json --seed 1
```

Hasil setiap simulasi disimpan di direktori `runs/` yang unik. Isinya mencakup
skenario, manifes, data kebenaran simulasi, telemetri perangkat lunak
penerbangan, log kejadian, grafik PNG, dan laporan PDF.

## Batas validitas

- Rentang referensi: ketinggian 20–100 km.
- Koefisien aerodinamika diperkirakan dari geometri sederhana, bukan CFD.
- Model propulsi memakai parameter `c*`, koefisien dorong, efisiensi nosel, dan
  laju aliran yang harus dikalibrasi.
- Sampel di luar batas Mach atau sudut serang ditandai `UNVALIDATED`.
- Hasil harus dikorelasikan dengan analisis dan data uji sebelum dipakai untuk
  keputusan rekayasa.
