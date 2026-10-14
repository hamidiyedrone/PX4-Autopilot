# Octopus Interceptor & Talon Target Simulation

Gazebo Harmonic ve PX4 SITL tabanlı 3D önleme ve hedef takip simülasyon ortamı.

---

## 🚀 Gereksinimler ve Kurulum (Requirements)

Simülasyon araçları, Burun Kamerası HUD ekranı ve Kontrol Paneli arayüzü için gerekli Python bağımlılıkları:

```bash
pip install -r Tools/simulation/interceptor_sim/requirements.txt
```

### ⚠️ Protobuf Sürüm Uyumluluğu (Önemli Not)

Gazebo Harmonic mesaj tanımları (`gz.msgs10`) Protobuf C++ API'si ile derlenmiştir. Sisteminizde modern yapay zeka/makine öğrenmesi paketleri (TensorFlow, ONNX vb.) kuruluysa, Python ortamınızda **`protobuf >= 4.x`** bulunabilir.

Protobuf 4+ sürümlerinde `TypeError: Descriptors cannot be created directly` hatasını önlemek için iki yöntem mevcuttur:

1. **Otomatik Çözüm (Varsayılan):**
   `sim.sh`, `view.sh`, `target_sim.py` ve `sim_detector.py` betiklerimiz bu değişkeni otomatik olarak tanımlar. Özel bir işlem yapmanıza gerek yoktur:
   ```bash
   export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
   ```

2. **İzole Sanal Ortam (Virtualenv) Çözümü:**
   TensorFlow/ONNX gibi protobuf 4+ gerektiren paketler kullanmıyorsanız, protobuf'u doğrudan 3.20.x sürümüne sabitleyebilirsiniz:
   ```bash
   pip install "protobuf<=3.20.3"
   ```

---

## 🎮 Simülasyonu Çalıştırma

### Gazebo 3D Görsel Ekranı + Kamera HUD + Kontrol Paneli:
```bash
bash Tools/simulation/interceptor_sim/sim.sh --gui
```

### Simülasyonu Sonlandırma:
```bash
bash Tools/simulation/interceptor_sim/kill.sh
```
