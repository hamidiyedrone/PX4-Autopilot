# Otonom Önleme Koşu Testleri, Telemetri Analizi ve Güdüm Tuning Kılavuzu (kosu_test.md)

Bu doküman; **Ukrayna Octopus / Vakkas VTOL Tailsitter Önleyici Dronun**, yüksek hızda (temel hız **28 m/s**, dinamik aralık **22 - 34 m/s**) uçan ve kaçınma manevraları yapan **Talon-1718 hedef İHA'sına** karşı gerçekleştireceği **25 kapsamlı otonom test koşusunu**, 50 Hz çok kanallı senkronize telemetri loglama sistemini, uçuş verisi analiz yöntemlerini ve **3D Blended APN (Augmented Proportional Navigation)** ile **IBVS (Image-Based Visual Servoing)** güdüm algoritmalarının tuning / revizyon kurallarını tanımlar.

> [!NOTE]
> Bu doküman ve beraberindeki `batch_run_25.py` test paketi; insan mühendisler ve yapay zeka ajanları (AI Flight Test Agents) tarafından 10+ saatlik gözetimsiz / otonom test koşturabilecek, sonuçları otomatik özetleyip raporlayabilecek şekilde yapılandırılmıştır.

---

## 1. Test Mimarisi ve Simülasyon Altyapısı

Önleme simülasyonu aşağıdaki bileşenlerle eşzamanlı çalışır:
- **PX4 Autopilot (SITL):** Önleyici dronun uçuş kontrolcüsü (`bin/px4`), durum kestirimi ve `src/modules/intercept` otonom önleme modülü.
- **Gazebo Harmonic (GZ):** Fiziksel simülasyon dünyası, aerodinamik kuvvetler, Storm32 3-eksenli gimbal, burun kamerası (`nose_camera`) ve MITLA 2.0 ağ fırlatıcı canister'ı (`net_launcher`).
- **target_sim.py:** Talon-1718 kinematik hedef motoru. Temel hız 28 m/s olmak üzere 22-34 m/s aralığında dinamik gaz dalgalanmaları, weave, break turn, dalış ve tırmanış manevraları üretir.
- **sim_detector.py & view_hud.py:** 50/80 FPS dinamik kamera modeli, optik 3D STL mesh projeksiyonu ve HUD arayüzü.
- **ground_relay.py:** Yer istasyonu hedef takip rölesi (MAVLink `FOLLOW_TARGET`).
- **batch_run_25.py:** 25 koşuyu gözetimsiz sırayla koşturan, 50 Hz telemetri kaydeden ve otomatik rapor üreten master test suite'i.

---

## 2. Hedef (Talon-1718) 25-Koşuluk Kapsamlı Manevra Test Matrisi

Hedef uçağın temel seyir hızı **28.0 m/s** (~101 km/h) olarak belirlenmiştir. Dronumuz azami 45 m/s hıza kadar ivmelenebilmekte olup, 28 m/s kaçan hedefe 35-40 m/s sprint ile yaklaşıp sepet mesafesinde (2.5m arkada) hız eşitleme yapar.

Aşağıdaki 25 senaryo 6 ana taktik grupta toplanmıştır:

### Grup A: Yüksek Hızlı Düz Takip & Gaz Dalgalanması (Speed Group)
| Koşu ID | Senaryo Adı | Talon Hızı [m/s] | İrtifa [m] | Profil Açıklaması | Test Edilen Kabiliyet |
| :---: | :--- | :---: | :---: | :--- | :--- |
| **RUN-01** | Baseline 28 m/s Level Pursuit | 28.0 | 60 | Düz ve yatay uçuş, sabit 28 m/s. | Temel yaklaşma, sanal sepete oturma ve ağ atımı. |
| **RUN-02** | Fast Sprint 32 m/s Pursuit | 32.0 | 80 | Yüksek hızlı düz kaçış (32 m/s). | İleri itki marjı, azami hızda sepet mesafesini koruma. |
| **RUN-03** | Low-Speed 24 m/s Pursuit | 24.0 | 50 | Alt hız sınırı düz takip (24 m/s). | Düşük hızda gövde pitch açısı ve sepet stabilitesi. |
| **RUN-04** | Periodic Throttle Pulse | 28.0 (24-32) | 65 | 24 m/s ile 32 m/s arası periyodik gaz darbeleri. | Dinamik kapanma hızı ($V_c$) regülasyonu. |

### Grup B: Kaçınma S-Dönüşleri & Yılan Dalgaları (Weave Group)
| Koşu ID | Senaryo Adı | Talon Hızı [m/s] | İrtifa [m] | Profil Açıklaması | Test Edilen Kabiliyet |
| :---: | :--- | :---: | :---: | :--- | :--- |
| **RUN-05** | Mild Weave at 28 m/s | 28.0 | 60 | Genlik: 20m, Dalga boyu: 450m (~15° bank). | Görüş hattı açısal hızı ($\dot{\lambda}$) hassas takibi. |
| **RUN-06** | Standard Tactical Weave | 28.0 | 65 | Genlik: 35m, Dalga boyu: 380m (~28° bank). | Yanal APN ivmesi ve sepet yanal sapma hatası. |
| **RUN-07** | Aggressive S-Turn Weave | 28.0 | 70 | Genlik: 50m, Dalga boyu: 300m (~38° bank). | Kuyruk sallama (fishtailing) sönümleme ve $R_{core}$ etkisi. |
| **RUN-08** | Fast Weave Sprint | 32.0 | 80 | Genlik: 35m, Hız: 32 m/s yüksek hızlı dalga. | Yüksek süratte viraj dışına savrulmama (N katsayısı). |

### Grup C: Sert Yüksek-G Kırılma Dönüşleri (Turn Group)
| Koşu ID | Senaryo Adı | Talon Hızı [m/s] | İrtifa [m] | Profil Açıklaması | Test Edilen Kabiliyet |
| :---: | :--- | :---: | :---: | :--- | :--- |
| **RUN-09** | Coordinated Orbit Left | 28.0 | 70 | Sola sürekli koordineli daire (30° bank). | Sabit yanal merkezkaç ivmesi altında sepet takibi. |
| **RUN-10** | High-G Break Turn Right | 28.0 | 60 | Sağa ani sert kırılma (45° bank, 0.55 rad/s). | Gimbal açısal hız sınırı (90°/s) ve FOV kaçırma kontrolü. |
| **RUN-11** | Fast Break Turn Left | 32.0 | 80 | 32 m/s hızda sert sola kırılma (45° bank). | Yüksek hızda merkezkaç ivmesi feedforward telafisi. |
| **RUN-12** | Break Turn with Speed Drop | 28 -> 20 | 65 | Dönüşün ortasında aniden 20 m/s'ye hava freni. | Dönüş içi ani bağıl hız değişimi ve fren klamplaması. |

### Grup D: Dikey Kaçışlar & Eğim Dinamiği (Vertical Group)
| Koşu ID | Senaryo Adı | Talon Hızı [m/s] | İrtifa [m] | Profil Açıklaması | Test Edilen Kabiliyet |
| :---: | :--- | :---: | :---: | :--- | :--- |
| **RUN-13** | Shallow Dive Evasion | 28.0 | 75 -> 45 | Sığ dikey dalış ($V_z = -3$ m/s, 28 m/s). | Eğimli uçuşta irtifa kaybına APN dikey tepkisi. |
| **RUN-14** | Steep Dive Evasion | 28.0 | 90 -> 35 | Dik dikey dalış ($V_z = -6$ m/s, 28 m/s). | Hızlı dalışta zemin emniyeti ve aşırı dalma önleme. |
| **RUN-15** | Zoom Climb Evasion | 28.0 | 50 -> 110 | Dik tırmanış ($V_z = +6$ m/s, 28 m/s). | Dikey eksen tırmanış APN kazancı ve itki payı. |
| **RUN-16** | Vertical Rollercoaster | 28.0 | 70 | Önce 40m'ye dalış, ardından 95m'ye dik tırmanış. | Dikey yönde yön değiştirme ve pitch salınım kontrolü. |

### Grup E: Gaz Tuzakları & Hız Şokları (Throttle Group)
| Koşu ID | Senaryo Adı | Talon Hızı [m/s] | İrtifa [m] | Profil Açıklaması | Test Edilen Kabiliyet |
| :---: | :--- | :---: | :---: | :--- | :--- |
| **RUN-17** | Hard Airbrake Trap | 28 -> 16 | 60 | Hedefe 15m kala aniden 16 m/s'ye motor kesme. | Geriye takla atmama (pitch-up snap clamp: max -4.5 m/s²). |
| **RUN-18** | Sprint Escape | 24 -> 34 | 65 | Hedefe 20m kala aniden 34 m/s tam gaz kaçış. | Maksimum ivme doyumu (`INT_ACC_MAX`) ve takip payı. |
| **RUN-19** | Cyclic Speed Jitter | 28 (23-33) | 60 | Her 10 saniyede bir $\pm 5$ m/s ani gaz adımları. | Kapanma hızı ivme türevi regülasyonu. |
| **RUN-20** | Decel & Re-accel Pulse | 28 -> 18 -> 32 | 65 | Hedef yavaşlayıp sepete çekerken aniden gaz açma. | Sepet oturma PID integral rüzgarı (anti-windup). |

### Grup F: Bileşik 3D Muharebe Kaçınma Manevraları (Combat Group)
| Koşu ID | Senaryo Adı | Talon Hızı [m/s] | İrtifa [m] | Profil Açıklaması | Test Edilen Kabiliyet |
| :---: | :--- | :---: | :---: | :--- | :--- |
| **RUN-21** | Spiral Dive Combat Evasion | 28.0 | 85 -> 40 | 35° bank dönüşüyle birlikte -5 m/s dalış. | 3D bileşik açısal hız altında sürekli kilitlenme. |
| **RUN-22** | Climbing S-Turn Evasion | 28.0 | 55 -> 110 | 35m genlikli S-dönüşü yaparken +5 m/s tırmanış. | Yatay + dikey bileşik APN güdüm performansı. |
| **RUN-23** | Evasive Reversals + Speed Drop | 28 -> 22 | 65 | Sola 40° sert dönüş, ardından Sağa 40° kırılma. | Tersine çevrimlerde gimbal ve gövde yönelim gecikmesi. |
| **RUN-24** | 3D Corkscrew Jink | 28 (25-32) | 55 -> 105 | Tirbüşon manevrası (Spiral tırmanış + gaz dalgası). | Çoklu serbestlik dereceli ekstrem takip. |
| **RUN-25** | Full Combat Evasion Gauntlet | 28 - 34 | 75 | Break Turn -> Spiral Dalış -> Zoom Climb zinciri. | Nihai stres ve yakalama dayanıklılık testi. |

---

## 3. Otomatik Çalıştırma ve Koşu Seçenekleri

### 3.1. 25 Koşunun Tamamını Gözetimsiz Başlatma (10+ Saatlik Test)
Aşağıdaki tek komutla tüm 25 koşu art arda koşturulur, her birinin telemetrisi diske yazılır ve özet rapor otomatik üretilir:
```bash
cd /home/kayra/Masaüstü/vakkas/PX4-Autopilot

# 25 koşunun tamamını çalıştır:
python3 Tools/simulation/interceptor_sim/tools/batch_run_25.py --all
```

### 3.2. Belirli Bir Koşuyu veya Aralığı Çalıştırma
```bash
# Sadece 7. koşuyu koştur (Aggressive S-Turn):
python3 Tools/simulation/interceptor_sim/tools/batch_run_25.py --run 7

# Belirli bir aralığı koştur (örn: 1'den 5'e kadar):
python3 Tools/simulation/interceptor_sim/tools/batch_run_25.py --range 1-5

# Belirli bir grubu koştur (örn: sadece dikey dalış/tırmanış grubu):
python3 Tools/simulation/interceptor_sim/tools/batch_run_25.py --group vertical
```

### 3.3. GUI ve HUD Ekranı ile Manuel İnceleme
Herhangi bir koşuyu görsel olarak ekranda izlemek için:
```bash
# 28 m/s standart hızda S-dönüşü görsel simülasyonu:
PATTERN=weave SPEED=28 ALT=65 ./Tools/simulation/interceptor_sim/sim.sh
```

---

## 4. 50 Hz Telemetri Kaydı ve Veri Şeması

Her koşuda `Tools/simulation/interceptor_sim/build/telemetry/run_XX_<group>.csv` dosyası üretilir.

### Kaydedilen Değişkenler:
1. `sim_time`: Simülasyon zamanı [saniye].
2. `phase`: Güdüm fazı (`STANDBY`, `APN_CRUISE`, `BLENDING`, `RENDEZVOUS`, `NET_FIRED`).
3. `target_source`: Hedef veri kaynağı (`GPS_FOLLOW_TARGET`, `VISION`, `DEAD_RECKONING`).
4. `int_pos_n`, `int_pos_e`, `int_pos_d`: Önleyici NED konumu [m].
5. `int_vel_n`, `int_vel_e`, `int_vel_d`: Önleyici NED hız vektörü [m/s].
6. `int_pitch_deg`, `int_tilt_deg`: Gövde pitch ve dikeyden toplam eğim açısı [derece].
7. `tgt_pos_n`, `tgt_pos_e`, `tgt_pos_d`: Hedef NED konumu [m].
8. `tgt_speed`: Hedef skaler hızı [m/s].
9. `basket_dist_rv`: Sanal sepete olan anlık mesafe $R_v$ [m].
10. `target_dist_rt`: Hedefe olan anlık mesafe $R_t$ [m].
11. `closing_speed_vc`: Anlık kapanma hızı $V_c = -\dot{R}$ [m/s].
12. `cmd_acc_norm`: Güdüm modülünden PX4'e gönderilen ivme normu [m/s²].
13. `cmd_gmb_pitch_deg`, `cmd_gmb_yaw_deg`: Seeker gimbal açı komutları [derece].
14. `net_fired`: Ağ atıldı mı (0 veya 1).

---

## 5. Uçuş Analiz Metodolojisi & Başarı Metrikleri

Her koşu sonunda otomatik analiz fonksiyonu çalışır ve aşağıdaki metrikleri puanlar:

### Metrik 1: En Yakın Sepet Mesafesi ($R_{v, min}$)
- **Kriter:** $R_v \le 2.2$ metre. Hedefin 2.5m arkasındaki yakalama sepetine tam oturma şartıdır.

### Metrik 2: Gövde Pitch Salınımı & Kararlılık (Limit Cycle Kontrolü)
- İleri uçuşta dron burun aşağı yatarken ($\text{Pitch} \approx -18^\circ \dots -28^\circ$) salınım genliği:
- **Kriter:** Tepe-tepe salınım $\Delta \theta_{pp} \le 12^\circ$ ve yön değiştirme frekansı $\le 1.2$ Hz olmalıdır.

### Metrik 3: Kapanma Hızı & Pitch-Up Snap (Geriye Takla) Önleme
- Kapanma anında boyuna frenleme ivmesi:
- **Kriter:** $-4.5 \text{ m/s}^2$ sınırını aşmamalıdır; araç burnunu kontrolsüzce göğe dikmemelidir.

### Metrik 4: Ağ Fırlatma Zarfı (Net Gun Deployment)
- Aşağıdaki 3 koşul art arda 8 kontrol çevrimi (yaklaşık 0.15s) sağlanmalıdır:
  1. $R_v \le 2.2$ m
  2. $|\mathbf{v}_{int} - \mathbf{v}_{tgt}| \le 2.5$ m/s
  3. Gövde burun paralelliği $\ge 0.95$ ($< 18^\circ$)
- **Başarı:** `net_fired == 1` olması durumunda test **YAKALANDI (CAPTURE CONFIRMED)** sayılır.

---

## 6. Güdüm Algoritmasını İyileştirme Karar Matrisi (Tuning Rehberi)

| Gözlenen Problem | Olası Kök Neden | İlgili Dosya & Parametre | Önerilen Düzeltme |
| :--- | :--- | :--- | :--- |
| **Virajda geride kalıyor (R_v > 3.5m)** | 28 m/s yüksek hızda APN yanal yönlendirmesi yetersiz | `INT_APN_N` (`module.yaml`) | `INT_APN_N` değerini `3.8`'den `4.2` veya `4.5`'e yükseltin. |
| **Yüksek hızda sepet arkasında sağa-sola fishtailing yapıyor** | Görüş hattı açı hızı regularizasyon çekirdeği küçük | `Intercept.cpp` (`R_core`) | `R_v * R_v + 6.25f` yerine `R_v * R_v + 10.0f` yapın ($R_{core} = 3.16m$). |
| **Hedef fren yapınca (RUN-17) araç geriye doğru takla atıyor** | Boyuna frenleme ivmesi çok sert | `Intercept.cpp` (`a_long_clamped`) | `math::max(a_long, -3.5f)` (Frenlemeyi -4.5'ten -3.5 m/s² seviyesine gevşetin). |
| **Düz uçuşta burun aşağı-yukarı dalgalanıyor** | İvme jerk limiti yüksek, pitch çevrimiyle çakışıyor | `Intercept.cpp` (`max_jerk`) | `max_jerk = 70.0f` yerine `45.0f` yapın. |
| **Sepete yaklaşıyor ama hız eşitleyemiyor** | Yakın faz Rendezvous PID türev kazancı zayıf | `Intercept.cpp` (`a_cmd_rendezvous`) | Hız farkı türev katsayısını `4.2f`'den `5.2f`'ye yükseltin. |

---

## 7. IBVS (Image-Based Visual Servoing) Entegrasyonu

Görsel güdüm üzerinde çalışan ekip arkadaşınızın kamera hata vektörünü doğrudan APN ile birleştirebilmesi için formülasyon:

1. **Piksel Hata Vektörü:**
   $$e_u = c_x - 0.5, \quad e_v = c_y - 0.5$$
2. **Bounding Box Büyüme Hızı ile Kapanma Hızı Kestirimi:**
   $$A = w \times h, \quad V_c \approx K_a \frac{\dot{A}}{A^{1.5}}$$
3. **Yanal IBVS İvme Komutu:**
   $$\mathbf{a}_{lat}^{ibvs} = - K_p^{ibvs} \begin{bmatrix} e_u \\ e_v \end{bmatrix} - K_d^{ibvs} \begin{bmatrix} \dot{e}_u \\ \dot{e}_v \end{bmatrix}$$
Bu lateral ivme, terminal fazda (`BLENDING` ve `RENDEZVOUS`) GPS verisi kaybolduğunda doğrudan `a_cmd_rendezvous` yerine ikame edilir.

---

## 8. Yapay Zeka Ajanına (AI Agent) Doğrudan Verilecek Görev Promptu

```text
SİSTEM YÖNERGESİ / TEST KOŞUSU TALİMATI:
Sen Vakkas VTOL Önleyici Dron projesinde otonom güdüm ve test mühendisisin.
Görevin:
1. `kosu_test.md` dokümanında tanımlanan 25 koşuluk test suite'ini `python3 Tools/simulation/interceptor_sim/tools/batch_run_25.py --all` komutuyla koştur.
2. Tüm koşular bittiğinde `Tools/simulation/interceptor_sim/build/telemetry/batch_25_summary.json` ve `batch_25_report.md` dosyalarını oku.
3. Başarısız olan (net_fired == 0) veya pitch salınımı yüksek olan koşuları tespit et.
4. `kosu_test.md` Bölüm 6'daki Karar Matrisine göre `src/modules/intercept/Intercept.cpp` veya `module.yaml` içindeki APN / Rendezvous parametrelerini güncelle.
5. Kodu `make px4_sitl_default` ile derle ve başarısız olan koşuları tekrar çalıştır.
6. Nihai başarı oranını ve parametre değişimlerini özetle.
```

---

## 9. Hızlı Başlangıç Komutları

```bash
# 1. Ortamı hazırla ve derle
cd /home/kayra/Masaüstü/vakkas/PX4-Autopilot
make px4_sitl_default

# 2. 25 koşuluk test paketini başlat (arka planda çalıştırmak için nohup eklenebilir)
python3 Tools/simulation/interceptor_sim/tools/batch_run_25.py --all

# 3. Canlı özet raporunu oku
cat Tools/simulation/interceptor_sim/build/telemetry/batch_25_report.md
```
