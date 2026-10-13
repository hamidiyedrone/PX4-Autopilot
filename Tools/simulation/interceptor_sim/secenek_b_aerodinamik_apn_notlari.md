# Seçenek B: Multirotor Aerodinamiğine Göre Ehlileştirilmiş APN Güdüm Notları

> **Amaç:** İleride test edilmek üzere, saf Proportional Navigation (APN) kanununun bir multicopter/octocopter dinamiklerine uyarlanması ve aerodinamik olarak kontrol edilebilir kılınması için teknik analiz ve uygulama reçetesidir.

---

## 1. Saf APN'nin Multirotorda Yarattığı Sorunlar
1. **Aşırı Eğim Açısı (Pitch-over Saturation):**
   - Multirotor ileri yönde ivmelenmek için gövdesini yatırmak zorundadır: $\theta \approx \arctan(a_x / g)$.
   - APN hedef hız $28\text{ m/s}$ iken $V_c = 15\text{ m/s}$ kapanma hızı isteyip $35-40\text{ m/s}^2$ boyuna ivme bastığında gövde $\theta = \arctan(40 / 9.81) \approx 76.2^\circ$ öne yatar. Araç gökyüzünde karnı üzerinde füze gibi sürüklenir.
2. **Son Fazda Ani Fren Şoku (Violent Deceleration Snap):**
   - Hedefe $5\text{ m}$ kalana kadar $43\text{ m/s}$ hız korunup, son 2 saniyede sepet hızına ($28\text{ m/s}$) inilmeye çalışıldığında drone aniden şahlanır, pitch ekseninde kontrol döngüsü çırpınması (limit-cycle oscillation) başlar.
3. **Doğrudan İvme Setpointi ile Hız PID Uyuşmazlığı:**
   - PX4'e ham `sp.acceleration` verildiğinde iç döngü tavır kontrolcüsü çok agresif tork üretir. Hız setpointi (`sp.velocity`) yerine ham ivme beslendiğinde jiroskop tepe değerleri ($>150^\circ/\text{s}$) fırlar.

---

## 2. Seçenek B'nin Çözüm ve Uygulama Reçetesi
İleride APN yaklaşımını tekrar denemek istersek uygulanacak revizyonlar:

### A. Eğim Açısı Klamplaması (Tilt Saturation Clamp)
- Maksimum ileri ivme $a_{x\_max} \le g \cdot \tan(45^\circ) \approx 9.81\text{ m/s}^2$ ile sınırlandırılmalıdır.
- Böylece gövde eğimi hiçbir zaman $45^\circ-50^\circ$'yi aşamaz.

### B. 80 Metreden İtibaren Kademeli Frenleme (Closing Speed Tapering)
- Hedefe $80\text{ m}$ kala kapanma hızı formülü şu şekilde değiştirilmelidir:
  $$V_{c\_des}(R) = \min\left(12.0,\, 2.5 + (R - 5.0) \times 0.12\right)\text{ m/s}$$
  * $80\text{ m}$ mesafede: $V_c = 11.5\text{ m/s}$
  * $40\text{ m}$ mesafede: $V_c = 6.7\text{ m/s}$
  * $10\text{ m}$ mesafede: $V_c = 3.1\text{ m/s}$
  * $5\text{ m}$ mesafede: $V_c = 2.5\text{ m/s}$
- Bu profil, aracı hedefin arkasına yaklaşırken adeta bir arabayla kırmızı ışığa yanaşır gibi pürüzsüzce yavaşlatır.

### C. Ham İvme Yerine Hız Setpointi ile Sürüş
- APN'nin hesapladığı $a_{cmd}$ ham ivme olarak doğrudan `sp.acceleration`'a verilmek yerine, entegre edilerek `sp.velocity` olarak verilmelidir (`sp.acceleration` NAN bırakılır).
- PX4'ün `MPC_ACC_HOR` (yatay ivme sınırı: $12\text{ m/s}^2$) ve `MPC_JERK_AUTO` parametreleri devrede kalarak açı değişimlerini sönümler.

---

## 3. Yeniden Test Etmek İstendiğinde Yapılacaklar
1. `Intercept.cpp` içindeki `ComputeBlendedAPNGuidance` fonksiyonunda $a_{long}$ sınırını $[-4.5, 35.0]$ yerine $[-3.0, 9.8]$ aralığına çekmek.
2. `sp.acceleration` kısmını sadece dikey eksenle sınırlandırıp yatayı `NAN` yapmak.
3. 25-koşu bataryası ile Seçenek A'nın sonuçlarıyla kıyaslamak.
