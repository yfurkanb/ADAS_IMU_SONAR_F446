# ADAS - IMU + Sonar Based Collision Warning System (STM32F446RE)

Bu proje, **STM32F446RE** üzerinde çalışan, gerçek zamanlı **IMU (MPU6050)** ve **Ultrasonic Sonar (HC-SR04)** verilerini işleyerek **çarpışma uyarı sistemi (ADAS)** oluşturan gömülü bir uygulamadır.

Proje kapsamında:
- Gerçek zamanlı sensör füzyonu
- Complementary filter ile Pitch/Roll hesaplama
- Hız tahmini (ax → m/s entegrasyonu)
- Sonar ile mesafe ölçümü
- **Time-To-Collision (TTC)** analizi
- Ani fren / çarpma algılama
- Çok seviyeli güvenlik durumu (SAFE / WARNING / CRITICAL)
- Seri port telemetri çıktısı

uygulanmıştır.

Bu proje; otomotiv, savunma ve otonom sistemlere giriş seviyesinde **gerçek bir ADAS algoritması** sunar.

---

# 📌 Özellikler

### ✔ MPU6050 IMU Sensör İşleme
- 3 eksen accelerometer + 3 eksen gyro
- Complementary filter ile Pitch/Roll hesaplama  
- İleri eksen ivmesinden hız tahmini
- Ani fren tespiti
- Çarpma algılama (2.5g üzeri ivme)

### ✔ HC-SR04 Ultrasonic Mesafe Ölçümü
- TIM3 Input Capture ile yüksek hassasiyetli ölçüm  
- 20 Hz örnekleme
- Gürültü azaltma için Low-Pass Filter (LPF)

### ✔ Güvenlik Durum Makinesi (Safety State Machine)
Durumlar:

| State | Açıklama |
|-------|----------|
| **SAFE** | Normal durum |
| **WARNING** | Mesafe veya TTC riskli |
| **CRITICAL** | Çarpma, ani fren veya kritik TTC |

Değerlendirilen kriterler:
- Mesafe < 1.2 m → WARNING  
- Mesafe < 0.5 m → CRITICAL  
- TTC < 1.0 s → WARNING  
- TTC < 0.5 s → CRITICAL  
- Toplam ivme > 2.5 g → CRITICAL  
- ax < -0.5 g → WARNING (ani fren)

### ✔ Telemetri Çıktısı
Seri port (115200 baud) üzerinden:
AX:1.02 AY:-0.02 AZ:-0.40 | Vx:39.20 m/s |
Dist:0.09 m TTC:0.28 s |
Pitch:-70.1 Roll:-133.7 | State:2

┌──────────────┐ ┌──────────────┐
│ MPU6050 │ I2C │ STM32F446RE │
└──────┬───────┘ └──────┬───────┘
│ │ TIM3 Input Capture
▼ ▼
IMU Driver HC-SR04 Driver
│ │
├──── Sensor Fusion ─┤
▼ ▼
Motion State Obstacle State
│ │
└────── Safety State Machine ───────┐
▼
Telemetry Output (UART)

---

# 🧩 Görev Yapısı (RTOS-siz, Tick-Based)

Her görev belirli bir frekansta çalışır:

| Görev | Frekans | İçerik |
|-------|---------|--------|
| IMU Task | 50 Hz | Pitch/Roll, hız entegrasyonu |
| Sonar Task | 20 Hz | Mesafe ölçümü |
| TTC Task | 20 Hz | Time-To-Collision |
| Safety Task | 50 Hz | State machine değerlendirmesi |
| Telemetry Task | 10 Hz | UART çıktı |

---

# 🛠 Donanım

## Kullanılan Kart ve Sensörler
- **STM32F446RE Nucleo**
- **MPU6050 (GY-521)** — I2C
- **HC-SR04** — Trigger + Echo (TIM3_CH1)

## Bağlantılar

### MPU6050
| MPU6050 | STM32F446RE |
|---------|-------------|
| VCC | 3.3V |
| GND | GND |
| SCL | PB8 |
| SDA | PB9 |

### HC-SR04
| HC-SR04 | STM32F446RE |
|---------|-------------|
| VCC | 5V |
| GND | GND |
| TRIG | PB0 |
| ECHO | PA6 (TIM3_CH1) |

⚠ Echo pininde **5V → 3.3V bölücü** kullanılması tavsiye edilir.  
Nucleo kartı genelde 5V toleranslıdır, ancak uzun vadede direnç bölücü güvenlidir.

---

# 📦 Proje Klasör Yapısı
/Core
/Inc
/Src
/Drivers
/Docs (isteğe bağlı)
/hcsr04.c / hcsr04.h
/mpu6050.c / mpu6050.h
README.md
.gitignore

---

# 🔧 Build & Çalıştırma

### 1️⃣ CubeIDE ile projeyi aç  
### 2️⃣ I2C → PB8/PB9 aktif  
### 3️⃣ TIM3 → CH1 → Input Capture Rising Edge  
### 4️⃣ GPIO PB0 → Output (TRIG)  
### 5️⃣ Build → Run  
### 6️⃣ Seri Portu 115200 baud ile aç

---

# 🧪 Örnek Çıktılar
AX:1.00 AY:-0.02 AZ:-0.40 | Vx:45.24 m/s |
Dist:0.09 m TTC:0.28 s |
Pitch:-70.2 Roll:-177.5 | State:2


---

# 🚀 Geliştirme Fırsatları

- Kalman Filter entegre etme  
- PID kontrollü fren simülasyonu  
- CAN-Bus telemetri desteği  
- RTOS portu (FreeRTOS)  
- VL53L0X/L1X ToF sensörü ile yüksek doğruluklu mesafe  

---

# 📄 Lisans

MIT License

---

# 👤 Geliştirici

**yfurkanb.**  
Otomotiv & Savunma odaklı gömülü sistemler geliştirme

---



