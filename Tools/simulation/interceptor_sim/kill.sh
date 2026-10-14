#!/usr/bin/env bash
echo "Tüm simülasyon süreçleri temizleniyor..."
pkill -9 -f "px4" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "target_sim.py" 2>/dev/null || true
pkill -9 -f "ground_relay.py" 2>/dev/null || true
pkill -9 -f "sim_detector.py" 2>/dev/null || true
pkill -9 -f "view_hud.py" 2>/dev/null || true
pkill -9 -f "control_panel.py" 2>/dev/null || true
pkill -9 -f "gcs_gui.py" 2>/dev/null || true
pkill -9 -f "web_gcs.py" 2>/dev/null || true
pkill -9 -f "view.sh" 2>/dev/null || true
sleep 1
echo "Temizlik tamamlandı! Bütün süreçler sonlandırıldı."
