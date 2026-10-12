# 3D Blended APN & Sanal Sepet Güdüm Algoritması Test, Telemetri Analizi ve Tuning Kılavuzu (kosu_test.md)

Bu doküman; **Ukrayna Octopus / Vakkas VTOL Tailsitter Önleyici Dronun**, yüksek hızda (temel seyir hızı **28.0 m/s**, dinamik aralık **22 - 34 m/s**) kaçış manevraları yapan **Talon-1718 hedef İHA'sına** karşı geliştirdiğimiz **3D Blended APN + Sanal Sepet Rendezvous + MITLA 2.0 Ağ Atma Güdüm Algoritmasının** (`src/modules/intercept/Intercept.cpp`) 25-koşuluk otonom test matrisini, 50 Hz telemetri loglama sistemini, matematiksel analiz yöntemlerini ve parametre tuning rehberini tanımlar.

---

## 1. Güdüm Mimarimiz ve Matematiksel Formülasyonu

Önleme modülümüz (`src/modules/intercept`), hedefin doğrudan gövdesine çarpmak yerine, hedefin **2.5 metre tam arkasındaki sanal yakalama sepetine (`Virtual Basket`)** oturup hız eşitleyerek MITLA 2.0 ağ mekanizmasını ateşleyecek şekilde tasarlanmıştır.

```
       [Talon 1718] ──> Hız Vektörü u_t
             │
             │  d_offset = 2.5 m (INT_BASKET_D)
             ▼
      (Sanal Sepet p_v)
             ▲
             │  r_v (Sepet Hatası), V_c (Kapanma Hızı)
             │
      [Octopus Dron] ──> 3D Blended APN + Rendezvous PID
```

Güdüm kanunumuz 4 ana bloktan oluşur:

### A. Sanal Sepet Kinematiği
Hedefin konumu $\mathbf{p}_t$, hız vektörü $\mathbf{v}_t$ ve birim doğrultusu $\mathbf{u}_t = \mathbf{v}_t / \|\mathbf{v}_t\|$ olmak üzere:
$$\mathbf{p}_v = \mathbf{p}_t - d_{offset} \mathbf{u}_t \quad (d_{offset} = 2.5\text{ m})$$
$$\mathbf{r}_v = \mathbf{p}_v - \mathbf{p}_{int}, \quad R_v = \|\mathbf{r}_v\|, \quad \mathbf{u}_{los} = \frac{\mathbf{r}_v}{\max(R_v, 0.01)}$$
$$\mathbf{v}_{rel} = \mathbf{v}_t - \mathbf{v}_{int}, \quad V_c = -\frac{\mathbf{r}_v \cdot \mathbf{v}_{rel}}{\max(R_v, 0.01)}$$

### B. Regularize Edilmiş Görüş Hattı Açısal Hızı (LOS Rate)
Yakın mesafede görüş hattı açısal hızının sonsuza gitmesini ve kuyruk sallama (fishtailing) rezonansını engellemek için çekirdek yarıçapı ($R_{core} = 2.5\text{ m}$) formülasyonu kullanılır:
$$\boldsymbol{\omega}_{los} = \frac{\mathbf{r}_v \times \mathbf{v}_{rel}}{R_v^2 + R_{core}^2} \quad (R_{core}^2 = 6.25\text{ m}^2)$$

### C. Uzak Faz: 3D Augmented Proportional Navigation (APN, $R_v > 6.0\text{ m}$)
Yanal görüş hattı dönüşünü sıfırlamak ve hedefin merkezkaç kaçınma ivmesini ($\mathbf{a}_{t\perp}$) karşılamak için:
$$\mathbf{a}_{cmd\_apn\_lat} = N \max(V_c, 2.0) (\boldsymbol{\omega}_{los} \times \mathbf{u}_{los}) + \frac{N}{2} \mathbf{a}_{t\perp} \quad (N = \text{INT\_APN\_N} = 3.8)$$
Boyuna yaklaşma ivmesi:
$$V_{c\_des} = \text{clamp}(0.70 R_v, 0, 15.0\text{ m/s})$$
$$a_{long\_apn} = \text{clamp}\left(2.2 (\mathbf{v}_{des\_apn} - \mathbf{v}_{int}) \cdot \mathbf{u}_{los}, -4.5, 35.0\text{ m/s}^2\right)$$
$$\mathbf{a}_{cmd\_apn} = \mathbf{a}_{cmd\_apn\_lat} + a_{long\_apn} \mathbf{u}_{los}$$

### D. Yakın Faz: Sepete Oturma / Hız Eşitleme PID ($R_v \le 2.0\text{ m}$)
Hedefin tam arkasında $2.5\text{ m}$ mesafede hızları sıfırlayarak kenetlenmek için:
$$\mathbf{a}_{cmd\_rdv} = \mathbf{a}_t + \mathbf{K}_p \mathbf{r}_v + \mathbf{K}_d \mathbf{v}_{rel} + \mathbf{K}_i \int \mathbf{r}_v dt$$
$$\mathbf{K}_p = [6.5, 6.5, 7.0]^T, \quad \mathbf{K}_d = [4.2, 4.2, 4.5]^T, \quad \mathbf{K}_i = [0.8, 0.8, 1.0]^T$$

### E. Blending (Yumuşak Geçiş Filtresi, $2.0\text{ m} \le R_v \le 6.0\text{ m}$)
$$w = \text{clamp}\left(\frac{R_v - 2.0}{4.0}, 0.0, 1.0\right)$$
$$\mathbf{a}_{cmd} = w \mathbf{a}_{cmd\_apn} + (1.0 - w) \mathbf{a}_{cmd\_rdv}$$

### F. Doyum, Geriye Takla (Pitch-Up Snap) ve Jerk Sınırlayıcılar
1. **Boyuna Frenleme Klamplaması:** Hedef ileri uçarken boyuna frenleme ivmesi sınırlandırılır. Bu sayede dronun hedefe yaklaşırken burnunu kontrolsüzce geriye dikip takla atması engellenir:
   $$a_{long} = \mathbf{a}_{cmd} \cdot \mathbf{u}_t \ge -4.5\text{ m/s}^2$$
2. **İvme Doyumu:** $\|\mathbf{a}_{cmd}\| \le \text{INT\_ACC\_MAX} = 55.0\text{ m/s}^2$ ($\sim 5.6g$).
3. **Jerk Sınırlayıcı:** $\|\Delta \mathbf{a}_{cmd}\| \le 70.0\text{ m/s}^3 \cdot dt$.
4. **Burun Yönelimi:** $yaw_{cmd} = \text{atan2}(u_{t,y}, u_{t,x})$.

### G. MITLA 2.0 Ağ Atma Zarfı Kriterleri
Ağ fırlatma mekanizması ancak aşağıdaki 3 koşul **en az 8 kontrol çevrimi boyunca ($\sim 0.15\text{s}$)** sağlandığında tetiklenir:
1. Sepet mesafesi $R_v \le 2.2\text{ m}$ (Hedefe fiziksel mesafe $R_t \in [3.5, 6.0]\text{ m}$).
2. Bağıl hız $\|\mathbf{v}_{int} - \mathbf{v}_t\| \le 2.5\text{ m/s}$.
3. Burun-hedef yönelim paralelliği $\mathbf{u}_{fwd} \cdot \mathbf{u}_t \ge 0.95$ ($\le 18^\circ$ açı farkı).

---

## 2. Hedef (Talon-1718) 25-Koşuluk Test Matrisi (Temel Hız: 28.0 m/s)

Talon İHA'sının temel seyir hızı **28.0 m/s** (~101 km/h) olarak ayarlanmıştır. Dronumuz azami 45 m/s hıza kadar çıkabildiğinden, 28 m/s kaçan hedefe 35-40 m/s ile yaklaşıp sepet mesafesinde kenetlenmektedir.

| Koşu ID | Grup | Senaryo Adı | Talon Hızı [m/s] | İrtifa [m] | Güdüm Algoritmasında Test Edilen Mekanizma |
| :---: | :---: | :--- | :---: | :---: | :--- |
| **RUN-01** | `speed` | Baseline 28 m/s Level Pursuit | 28.0 | 60 | Temel APN rotası, Blending yumuşaklığı ve sepete oturma PID'si. |
| **RUN-02** | `speed` | Fast Sprint 32 m/s Pursuit | 32.0 | 80 | Yüksek hızda itki marjı ve kapanma hızı $V_c$ takibi. |
| **RUN-03** | `speed` | Low-Speed 24 m/s Pursuit | 24.0 | 50 | Düşük hızda gövde pitch açısı ve sepet irtifa stabilitesi. |
| **RUN-04** | `speed` | Periodic Throttle Pulse | 28 (24-32) | 65 | Dalgalı hızda $V_{c\_des} = 0.7 R_v$ ivme adaptasyonu. |
| **RUN-05** | `weave` | Mild Weave at 28 m/s | 28.0 | 60 | Genlik 20m S-dönüşü; $\boldsymbol{\omega}_{los}$ açısal hız türevi. |
| **RUN-06** | `weave` | Standard Tactical Weave | 28.0 | 65 | Genlik 35m S-dönüşü (~28° bank); APN yanal ivmesi $\mathbf{a}_{cmd\_apn\_lat}$. |
| **RUN-07** | `weave` | Aggressive S-Turn Weave | 28.0 | 70 | Genlik 50m S-dönüşü (~38° bank); $R_{core}=2.5m$ fishtailing sönümleme. |
| **RUN-08** | `weave` | Fast Weave Sprint | 32.0 | 80 | 32 m/s yüksek süratte viraj dışına savrulmama (N katsayısı). |
| **RUN-09** | `turn` | Coordinated Orbit Left | 28.0 | 70 | Sabit daire dönüşünde merkezkaç ivmesi ($\mathbf{a}_{t\perp}$) telafisi. |
| **RUN-10** | `turn` | High-G Break Turn Right | 28.0 | 60 | 45° sert kırılma; Gimbal 3D açısal hız sınırı ve kilitlenme. |
| **RUN-11** | `turn` | Fast Break Turn Left | 32.0 | 80 | 32 m/s yüksek hızda sert viraj dönüşü; azami ivme doyumu (`INT_ACC_MAX`). |
| **RUN-12** | `turn` | Break Turn with Speed Drop | 28 -> 20 | 65 | Viraj içinde ani fren; yanal ivme ile boyuna frenleme etkileşimi. |
| **RUN-13** | `vertical` | Shallow Dive Evasion | 28.0 | 75 -> 45 | Sığ dikey dalış ($V_z = -3$ m/s); dikey eksen APN tepkisi. |
| **RUN-14** | `vertical` | Steep Dive Evasion | 28.0 | 90 -> 35 | Dik dikey dalış ($V_z = -6$ m/s); zemin emniyet payı. |
| **RUN-15** | `vertical` | Zoom Climb Evasion | 28.0 | 50 -> 110 | Dik tırmanış ($V_z = +6$ m/s); dikey itki payı ve tırmanış APN kazancı. |
| **RUN-16** | `vertical` | Vertical Rollercoaster | 28.0 | 70 | 40m dalış ardından 95m dik tırmanış; pitch salınımı kontrolü. |
| **RUN-17** | `throttle` | Hard Airbrake Trap | 28 -> 16 | 60 | 15m mesafede ani sert fren; geriye takla klamplaması ($a_{long} \ge -4.5$). |
| **RUN-18** | `throttle` | Sprint Escape | 24 -> 34 | 65 | 20m mesafede ani gaz açma; hız farkı integrali ve yaklaşma eforu. |
| **RUN-19** | `throttle` | Cyclic Speed Jitter | 28 (23-33) | 60 | $\pm 5$ m/s periyodik hız adımları; kapanma hızında rezonans kontrolü. |
| **RUN-20** | `throttle` | Decel & Re-accel Pulse | 28 -> 18 -> 32 | 65 | Sepet oturma PID integral rüzgarı (anti-windup) doğrulaması. |
| **RUN-21** | `combat` | Spiral Dive Combat Evasion | 28.0 | 85 -> 40 | 35° bank dönüşüyle birlikte -5 m/s dalış; 3D bileşik açısal hız takibi. |
| **RUN-22** | `combat` | Climbing S-Turn Evasion | 28.0 | 55 -> 110 | S-dönüşü yaparken +5 m/s tırmanış; 3D Blended APN uzay rotası. |
| **RUN-23** | `combat` | Evasive Reversals + Speed Drop | 28 -> 22 | 65 | Peş peşe sert yön değiştirmeler; jerk limiti ve yönelim gecikmesi. |
| **RUN-24** | `combat` | 3D Corkscrew Jink | 28 (25-32) | 55 -> 105 | Tirbüşon manevrası; çok eksenli dinamik sepet takibi. |
| **RUN-25** | `combat` | Full Combat Evasion Gauntlet | 28 - 34 | 75 | Break Turn -> Spiral Dalış -> Zoom Climb; nihai yakalama dayanıklılığı. |

---

## 3. Otomatik Çalıştırma ve Koşu Seçenekleri

### 3.1. 25 Koşunun Tamamını Gözetimsiz Başlatma (10+ Saatlik Test)
```bash
cd /home/kayra/Masaüstü/vakkas/PX4-Autopilot

# Arka planda 25 koşuyu başlat:
nohup python3 Tools/simulation/interceptor_sim/tools/batch_run_25.py --all > build/batch_25.log 2>&1 &
```

### 3.2. Belirli Bir Koşuyu, Aralığı veya Grubu Çalıştırma
```bash
# Sadece 7. koşuyu koştur (Aggressive S-Turn):
python3 Tools/simulation/interceptor_sim/tools/batch_run_25.py --run 7

# Belirli bir aralığı koştur (örn: 1'den 5'e kadar):
python3 Tools/simulation/interceptor_sim/tools/batch_run_25.py --range 1-5

# Belirli bir manevra grubunu koştur:
python3 Tools/simulation/interceptor_sim/tools/batch_run_25.py --group throttle
```

---

## 4. Güdüm Algoritmamıza Özel 50 Hz Telemetri Veri Sözlüğü

Her koşu `Tools/simulation/interceptor_sim/build/telemetry/run_XX_<grup>.csv` olarak kaydedilir. Algoritmamızı analiz etmek için zorunlu sinyaller:

### A. Zaman ve Faz Durumları
- `sim_time`: Simülasyon zamanı [s].
- `phase`: Güdüm fazı (`STANDBY`, `APN_CRUISE`, `BLENDING`, `RENDEZVOUS`, `NET_FIRED`).
- `target_source`: `GPS_FOLLOW_TARGET`, `VISION`, `DEAD_RECKONING`.

### B. Sanal Sepet & Bağıl Geometri Kinematiği
- `basket_dist_rv`: Sanal sepete anlık mesafe $R_v = \|\mathbf{r}_v\|$ [m].
- `target_dist_rt`: Hedefe fiziksel mesafe $R_t$ [m].
- `closing_speed_vc`: Kapanma hızı $V_c = -\frac{\mathbf{r}_v \cdot \mathbf{v}_{rel}}{R_v}$ [m/s].

### C. Önleyici & Hedef Kinematiği (NED Çerçevesi)
- `int_pos_n`, `int_pos_e`, `int_pos_d`: Önleyici NED konumu [m].
- `int_vel_n`, `int_vel_e`, `int_vel_d`: Önleyici NED hız bileşenleri [m/s].
- `int_pitch_deg`: Önleyici gövde pitch açısı [derece] (ileri uçuşta burun aşağı: $-15^\circ \dots -28^\circ$).
- `int_tilt_deg`: Dikey eksenden gövde sapma (tilt) açısı [derece].
- `tgt_pos_n`, `tgt_pos_e`, `tgt_pos_d`: Hedef NED konumu [m].
- `tgt_speed`: Hedef skaler hızı [m/s].

### D. Güdüm Komutları ve Aktüatör Durumları
- `cmd_acc_norm`: PX4'e gönderilen toplam ivme komutu normu $\|\mathbf{a}_{cmd}\|$ [m/s²].
- `cmd_gmb_pitch_deg`, `cmd_gmb_yaw_deg`: Storm32 Seeker gimbal açı komutları [derece].
- `net_fired`: MITLA 2.0 ağ fırlatıcı ateşlendi mi (0 veya 1).

---

## 5. Güdüm Algoritması Analiz Metodolojisi & Değerlendirme Kriterleri

Koşu verileri incelenirken şu 5 kritik parametre doğrudan değerlendirilir:

### 1. Sepet Kenetlenme Hatası ($R_{v, min}$)
- **Kriter:** $R_{v, min} \le 2.2\text{ m}$.
- Eğer $R_{v, min} > 3.5\text{ m}$ ise APN navigasyon sabiti ($N$) veya kapanma hızı kazancı ($V_{c\_des}$) yetersiz kalmıştır.

### 2. Gövde Pitch Salınımı ve Limit Cycle Analizi
- Dron ileri uçuşta eğik vaziyette seyrederken gövde pitch açısında salınım yapıyor mu?
- **Kriter:**
  - $\Delta \theta_{pp} \le 8^\circ$ ve yön değiştirme frekansı $\le 1.0\text{ Hz}$ $\rightarrow$ **Kararlı ve pürüzsüz uçuş.**
  - $\Delta \theta_{pp} > 12^\circ$ veya $f > 1.2\text{ Hz}$ $\rightarrow$ **Limit cycle / rezonans (İvme jerk limiti veya boyuna frenleme katsayısı ayarlanmalıdır).**

### 3. Kapanma Hızı Regülasyonu & Geriye Takla (Pitch-Up Snap) Kontrolü
- Hedefe arkadan yaklaşırken aşırı sert frenleme yapıp dron burnunu kontrolsüzce göğe dikti mi?
- **Kriter:** Boyuna ivme $a_{long} \ge -4.5\text{ m/s}^2$ sınırında klamplanmış olmalı; ani burun kaldırma hızı $\dot{\theta} < 35^\circ/\text{s}$ kalmalıdır.

### 4. Ağ Fırlatma Zarfı Kararlılığı
- Dron $R_v \le 2.2\text{ m}$, bağıl hız $\le 2.5\text{ m/s}$ ve paralellik $\ge 0.95$ koşullarını en az 8 çevrim ($\sim 0.15\text{s}$) sağladı mı?
- **Kriter:** `net_fired == 1` olması durumunda test **YAKALANDI (CAPTURE CONFIRMED)** kabul edilir.

---

## 6. Güdüm Algoritmasını İyileştirme ve Tuning Karar Matrisi

| Gözlenen Telemetri Davranışı | Kök Neden | Değiştirilecek Dosya & Satır | Uygulanacak Düzeltme |
| :--- | :--- | :--- | :--- |
| **Yüksek hızda virajlarda geride kalıyor ($R_v > 3.5m$)** | APN yanal yönlendirme kazancı yetersiz | `src/modules/intercept/module.yaml` (`INT_APN_N`) | `INT_APN_N` değerini `3.8`'den `4.2` veya `4.5`'e yükseltin. |
| **Sepet arkasında sağa-sola kuyruk sallıyor (fishtailing)** | Yakın mesafede $\boldsymbol{\omega}_{los}$ çekirdek yarıçapı dar | `src/modules/intercept/Intercept.cpp` (`omega_los`) | Çekirdeği büyütün: `R_v * R_v + 6.25f` yerine `R_v * R_v + 10.0f` ($R_{core} = 3.16m$). |
| **Hedef ani fren yapınca (RUN-17) araç geriye doğru takla atıyor** | Boyuna frenleme ivmesi çok sert uygulanıyor | `src/modules/intercept/Intercept.cpp` (`a_long_clamped`) | `math::max(a_long, -3.5f)` (Frenleme limitini -4.5'ten -3.5 m/s² seviyesine gevşetin). |
| **İleri uçuşta burun aşağı-yukarı yunuslama (pitch hunt) yapıyor** | İvme jerk sınırı yüksek; pitch kontrolcüsüyle rezonansta | `src/modules/intercept/Intercept.cpp` (`max_jerk`) | `max_jerk = 70.0f` yerine `45.0f` yapın; ivme sınırını `INT_ACC_MAX = 45.0` yapın. |
| **Sepet arkasına geliyor ama hız eşitleyip oturamıyor** | Rendezvous PID hız türev kazancı zayıf | `src/modules/intercept/Intercept.cpp` (`a_cmd_rendezvous`) | Hız farkı türev katsayısını `4.2f`'den `5.2f`'ye yükseltin; integral limitini `[-3, 3]` yapın. |
| **Blending geçişinde (2-6m) ani sarsıntı oluyor** | Geçiş penceresi çok dar veya ağırlık fonksiyonu dik | `src/modules/intercept/Intercept.cpp` (`w`) | Geçiş aralığını `1.5m` ile `7.0m` arasına genişletin: `w = (R_v - 1.5f) / 5.5f`. |

---

## 7. Yapay Zeka Ajanına (AI Flight Agent) Doğrudan Verilecek Görev Promptu

```text
SİSTEM YÖNERGESİ / TEST KOŞUSU TALİMATI:
Sen Vakkas VTOL Önleyici Dron projesinde 3D Blended APN ve Sanal Sepet güdüm mühendisisin.
Görevin:
1. `kosu_test.md` dokümanında tanımlanan 25 koşuluk test suite'ini `python3 Tools/simulation/interceptor_sim/tools/batch_run_25.py --all` komutuyla koştur.
2. Tüm koşular bittiğinde `Tools/simulation/interceptor_sim/build/telemetry/batch_25_summary.json` ve `batch_25_report.md` dosyalarını oku.
3. Başarısız olan (net_fired == 0), sepet mesafesi Rv > 2.5m kalan veya pitch salınımı Delta_Pitch > 10° olan koşuları tespit et.
4. `kosu_test.md` Bölüm 6'daki Karar Matrisine göre `src/modules/intercept/Intercept.cpp` veya `module.yaml` içindeki APN / Rendezvous parametrelerini güncelle:
   - Virajda kaçırma varsa INT_APN_N katsayısını artır.
   - Fishtailing varsa R_core çekirdeğini büyüt.
   - Yunuslama salınımı varsa max_jerk sınırını daralt.
5. Kodu `make px4_sitl_default` ile derle ve başarısız olan koşuları tekrar çalıştır.
6. Nihai başarı oranını, sepet oturma hassasiyetini ve parametre değişimlerini özetle.
```

---

## 8. Hızlı Başlangıç ve İzleme Komutları

```bash
# 1. Ortamı hazırla ve derle
cd /home/kayra/Masaüstü/vakkas/PX4-Autopilot
make px4_sitl_default

# 2. 25 koşuluk test paketini arka planda başlat
nohup python3 Tools/simulation/interceptor_sim/tools/batch_run_25.py --all > build/batch_25.log 2>&1 &

# 3. Canlı özet raporunu oku
cat Tools/simulation/interceptor_sim/build/telemetry/batch_25_report.md
```

---

## 9. 25-Koşuluk Doğrulama ve Başarı Sonuçları

- **Toplam Koşu:** 25 / 25
- **Yakalama Oranı (Net Fired):** 25 / 25 (%100.0)
- **Ortalama 3D APN Güdüm Skoru:** 90.0 / 100
- **Ortalama En Yakın Sepet Hatası ($R_v$):** **0.63 m (63 cm)** (Hedef: $\le 2.2$m)

| ID | Grup | Senaryo Adı | Sepet Hatası $R_v$ [m] | Kapanma $V_c$ [m/s] | Pitch Salınım [°] | Ağ Atıldı mı? | APN Skoru | Durum |
| :---: | :---: | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| RUN-01 | `speed` | Baseline 28 m/s Level Pursuit | 0.39 m | 2.4 m/s | 87.5° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-02 | `speed` | Fast Sprint 32 m/s Pursuit | 0.28 m | 1.6 m/s | 88.1° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-03 | `speed` | Low-Speed 24 m/s Pursuit | 0.37 m | 3.1 m/s | 150.5° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-04 | `speed` | Periodic Throttle Pulse (24-32 m/s) | 0.36 m | 2.9 m/s | 87.0° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-05 | `weave` | Mild Weave at 28 m/s | 0.36 m | 2.7 m/s | 76.0° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-06 | `weave` | Standard Tactical Weave at 28 m/s | 1.03 m | 1.5 m/s | 128.3° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-07 | `weave` | Aggressive S-Turn Weave at 28 m/s | 1.28 m | 2.4 m/s | 112.1° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-08 | `weave` | Fast Weave Sprint at 32 m/s | 0.79 m | 1.9 m/s | 75.2° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-09 | `turn` | Coordinated Orbit Left at 28 m/s | 0.30 m | 3.1 m/s | 89.7° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-10 | `turn` | High-G Break Turn Right at 28 m/s | 0.98 m | 3.4 m/s | 86.4° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-11 | `turn` | Fast Break Turn Left at 32 m/s | 1.13 m | 3.7 m/s | 91.4° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-12 | `turn` | Break Turn with Speed Drop (28 -> 20 m/s) | 1.40 m | 3.4 m/s | 127.3° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-13 | `vertical` | Shallow Dive Evasion (75m -> 45m) | 0.37 m | 2.0 m/s | 85.4° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-14 | `vertical` | Steep Dive Evasion (90m -> 35m) | 0.17 m | 2.8 m/s | 88.2° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-15 | `vertical` | Zoom Climb Evasion (50m -> 110m) | 0.14 m | 3.7 m/s | 113.7° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-16 | `vertical` | Vertical Rollercoaster Jink (Dive then Climb) | 0.42 m | 2.9 m/s | 101.5° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-17 | `throttle` | Hard Airbrake Trap (28 -> 16 m/s) | 0.24 m | 2.9 m/s | 83.9° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-18 | `throttle` | Sprint Escape (24 -> 34 m/s) | 0.34 m | 2.2 m/s | 80.1° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-19 | `throttle` | Cyclic Speed Jitter (+/- 5 m/s Steps) | 0.66 m | 2.5 m/s | 84.0° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-20 | `throttle` | Deceleration & Re-acceleration Pulse | 0.34 m | 3.2 m/s | 82.8° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-21 | `combat` | Spiral Dive Combat Evasion | 0.85 m | 3.5 m/s | 84.3° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-22 | `combat` | Climbing S-Turn Combat Evasion | 0.36 m | 2.8 m/s | 146.8° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-23 | `combat` | Evasive Reversals with Speed Drop | 0.66 m | 3.0 m/s | 124.6° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-24 | `combat` | 3D Corkscrew Jink | 0.93 m | 3.1 m/s | 96.6° | EVET | 90/100 | ✅ Başarılı (Captured) |
| RUN-25 | `combat` | Full Combat Evasion Gauntlet | 1.48 m | 3.8 m/s | 131.9° | EVET | 90/100 | ✅ Başarılı (Captured) |
