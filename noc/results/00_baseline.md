# 00 — Baseline: 06a Gazebo koşusu analizi

- **Veri:** `logs/06a-physical-interaction/gazebo/` (tek koşu, 2026-11-03 10:27, ~80.7 s)
- **Script:** `noc/analysis/baseline_06a.py` (+ parser `noc/analysis/logs.py`)
- **Çıktılar:** `noc/results/00_baseline/` → `metrics.json`, `log_inventory.json` (sha256 dahil), `tracking.png`, `fext_spring.png`, `rotors.png`, `loop_period.png`
- **Yeniden üretim:** `.venv/bin/python -m noc.analysis.baseline_06a`
- **Seed:** Analizde rastgelelik yok. Simülasyon için seed kaydedilmemiş; koşu tekrarlanırsa sonuç birebir aynı çıkmayabilir.

**Etiketler:** **[LOG]** = bir component logundan doğrudan okunan sinyal. Bu sinyal kendisi de bir kestirim olabilir; örneğin `pom` bir durum kestiricisidir. **[HESAP]** = loglardan offline hesaplanan değer, bir model varsayımına dayanır. Bu raporda **hiçbir temas kuvveti ölçümü yoktur** (bkz. §3, §4).

Zaman ekseni: `t = ts − ts₀`, burada `ts₀` = pom.log'daki ilk örnek (unix 1793701651.036).

---

## 0. Eksik / boş / geçersiz loglar

| Dosya | Durum |
|---|---|
| `phynt.log` | **BOŞ.** Yalnızca başlık satırı var, 0 veri satırı. Wrench observer (`efx…etz`) ve AF çıktısı loglanmamış. |
| `simulation_data.npz` | **GEÇERSİZ.** `f_ext` ve `tau_ext` 4000/4000 satırda sıfır. `pos`, `vel`, `avel` 4000/4000 satırda sıfır. `att` her satırda `[1,0,0,0]`. `p_nom` 4000/4000 satırda, `pos_d` 3995/4000 satırda sıfır. Bunlar `model_hexa_fa.py`'deki `read_*()` fonksiyonlarının exception durumunda döndürdüğü varsayılan değerlerle birebir aynı. Yani genomix okumaları neredeyse her adımda başarısız olmuş, **sıfırlar ölçüm değil**. Ayrıca npz'deki `t` değeri duvar saati değil, nominal `i·0.02 s`; gerçek Python döngü periyodu bu nedenle bilinmiyor. |
| `pom-measurements.log` | Parse edilmedi. Satırlarda ölçüm tipine göre değişken sayıda sütun var (40257 satırın tamamı başlıkla uyuşmuyor). Bu rapor için gerekmedi. |
| `rotorcraft.log` | Kullanılabilir. `cmd_v6`, `cmd_v7` sütunları `-` (zaten 6 rotor var). 3388 satırda `cmd_v*` = `-`. `imu_temp` = nan. 44 satırda `imu_rate` = 0. |
| `maneuver.log` | Kullanılabilir, ama yalnızca hareket sırasında loglanıyor: goto'lar arasında 2.9–8.7 s boşluk var. Sürekli bir döngü periyodu ölçmek için uygun değil. |
| `pom`, `uavpos`, `uavatt`, `opti` | Tam (bkz. `log_inventory.json`). |

Sonuç: Bu koşuda **f_ext için logda tek bir geçerli örnek yok.** §3 ve §4'teki kuvvetlerin hepsi [HESAP].

## Fazlar

- **Serbest hover pencereleri.** Maneuver referansı sabit (|v| < 1e-6), z > 0.5 m, temas görevi dışında. Referans durduktan sonraki ilk 1 s atıldı.
  H1 = [8.70, 10.66] s, ref (0, 0, 1)
  H2 = [15.68, 23.39] s, ref (1.2, 0, 1)
  H3 = [67.73, 69.66] s, ref (1.0, 0, 1)
- **Temas görevi.** Nominal x > 1.5 ve gövde x > 1.3 → [24.89, 63.26] s.
  Burada geometrik penetrasyon faz tanımı için kullanılmadı, çünkü pom'un konum gürültüsü penetrasyondan büyük (pom − opti, x için std 1.6 mm).
- **Kararlı temas.** Temas görevi içinde nominal sabit, ilk 1 s atıldı. Beş pencere: [26.4, 30.7], [36.7, 38.7], [44.7, 46.7], [52.7, 54.7], [60.8, 62.7] s.

---

## 1. Hover kararlılığı [LOG]

Konum, opti (Gazebo mocap) ile referans arasındaki farktır. Açılar pom'dan alındı.

| | H1 (0,0,1) | H2 (1.2,0,1) | H3 (1.0,0,1) |
|---|---|---|---|
| süre [s] | 1.96 | 7.71 | 1.93 |
| x − ref: ort / std / max\|·\| [mm] | −25.0 / 4.9 / 32.7 | +20.0 / 49.7 / 142.0 | −28.6 / 9.0 / 44.5 |
| y − ref [mm] | −3.2 / 2.1 / 10.5 | +17.4 / 13.6 / 45.2 | −13.4 / 11.2 / 30.3 |
| z − ref [mm] | +8.4 / 4.8 / 22.9 | +4.2 / 9.6 / 22.3 | +3.1 / 1.1 / 6.1 |
| roll / pitch / yaw std [°] | 0.54 / 0.61 / 0.74 | 0.69 / 0.91 / 1.65 | 0.80 / 1.00 / 0.91 |
| \|roll\|, \|pitch\|, \|yaw\| max [°] | 1.8 / 2.2 / 2.0 | 2.0 / 3.4 / 4.4 | 2.0 / 2.2 / 2.2 |
| ω std (wx/wy/wz) [°/s] | 7.2 / 8.3 / 5.2 | 7.8 / 7.3 / 7.8 | 9.9 / 10.9 / 5.5 |
| uavpos fz ort ± std [N] | 26.88 ± 0.34 | 26.77 ± 0.44 | 26.98 ± 0.38 |

Gözlemler:
- **Sabit x ofseti var:** H1'de −25 mm, H3'te −29 mm. Bunun nedeni `Kixy = 0`, yani xy'de integral terim yok.
- H2'nin x istatistiklerine yaklaşma manevrasının aşımı da giriyor: t ≈ 16 s'de x = 1.333, referansın 13 cm ötesi. Ardından yavaş bir sönüm geliyor (`tracking.png`).
- Hover pencereleri kısa (H1 ve H3 yaklaşık 2 s). Uzun süreli hover kararlılığı bu koşudan çıkarılamaz.
- pom ile opti uyumlu: konumda ortalama fark < 0.3 mm, std 1.3–2.0 mm. Açıda std 0.21–0.24°.

## 2. Pozisyon takip hatası [LOG]

`uavpos e_*` = pom − uavpos referansı. Bu referans phynt AF çıktısıdır.

| | serbest uçuş (temas görevi dışı, z > 0.5; 29.0 s) | temas görevi (38.1 s) |
|---|---|---|
| e_x ort / RMS / max\|·\| [mm] | −11.6 / 54.2 / 144.8 | **−178.8** / 179.0 / 197.5 |
| e_y [mm] | +6.6 / 22.1 / 55.0 | +18.3 / 51.2 / 135.2 |
| e_z [mm] | +6.6 / 16.3 / 100.4 | −4.2 / 15.2 / 42.0 |
| \|e\| RMS / max [mm] | 60.8 / 149.2 | 186.7 / 224.6 |

- **Temas görevindeki e_x ≈ −179 mm takip hatası değil, tasarım gereği.** Nominal gövde x'i 1.55 (`BODY_X_CONTACT`), yani referans duvarın içinde. Gövde duvarda 1.357–1.388 aralığında duruyor. Duvar boyunca asıl takip performansı e_y ve e_z ile ölçülüyor.
- e_y'nin en büyük değeri (135 mm) duvar boyunca y süpürmesindeki gecikme ve aşım (t ≈ 47–52 s). Sürtünme μ = 0.3.
- **AF referansı değiştirmiyor:** phynt referansı ile maneuver nominali arasındaki fark tüm koşu boyunca max |x| 5.1 mm, |y| 3.6 mm, |z| 3.6 mm (std < 0.5 mm). Admittance filtresi pratikte etkisiz.
  - Olası nedenler: WO kestirimi sıfır, ya da AF parametreleri uygulanmadı. **Doğrulanmadı**, çünkü phynt.log boş.
  - Bu nedenle bu koşu "AF + WO açık" baseline'ı değil, fiilen **saf pozisyon kontrolüyle duvara itme** koşusudur.

## 3. f_ext zaman serisi

- **[LOG] yok.** `phynt.log` boş, npz'deki `f_ext` ise exception sonucu yazılmış sıfırlar (§0).
- **[HESAP] Kontrolcü tarafı yeniden kurulum** (`fext_spring.png`, üst panel; dünya çerçevesi):
  `f̂_ext = m·a_pom − f_cmd(uavpos) + m·g·e_z`, m = 2.72 kg.
  Varsayımlar:
  - uavpos `fx, fy, fz` dünya çerçevesinde toplam kuvvet komutudur ve rotorlar bunu tam olarak üretir: saturasyon yok (§5 ile tutarlı), allocation ve kütle doğru.
  - `a_pom`, pom'un sabit-ivme modelinden gelen bir kestirimdir.

| [HESAP] [N] | x ort / std / max\|·\| | y | z |
|---|---|---|---|
| serbest uçuş, m·a dahil | −0.08 / 0.86 / 6.07 | +0.05 / 0.79 / 4.91 | −0.12 / 0.92 / 4.59 |
| temas görevi, m·a dahil | −0.91 / 0.98 / 6.80 | +0.09 / 0.91 / 3.93 | −0.40 / 0.94 / 4.52 |
| temas görevi, quasi-static (a = 0) | −0.88 / 0.26 / 1.73 | +0.09 / 0.46 / 2.53 | −0.37 / 0.60 / 2.89 |

- Serbest uçuştaki ~0.9 N std, bu kestiricinin gürültü tabanını gösteriyor. Kaynağı pom ivmesi ve komut gürültüsü.
- Temasta x ortalaması −0.9 N: duvar gövdeyi −x yönünde itiyor.
- Sayısal not: Temasta |f_cmd,x| ≈ 0.88 N, `Kpxy · |e_x|` = 5 × 0.179 = 0.89 ile örtüşüyor. Bu bir gözlem; uavpos kontrol yasasının bu biçimde olduğu doğrulanmadı.

## 4. Temas: f_ext ile hesaplanan yay kuvveti karşılaştırması

**Uyarı:** Bu bölümdeki iki taraf da [HESAP]. Bir yanda §3'teki f̂_ext (logda f_ext yok), diğer yanda idealize yay `F = K_WALL · max(δ, 0)`, K_WALL = 500 N/m (world `<ode><kp>`).
- δ = uç küre yüzeyinin duvar önüne göre x penetrasyonu. Uç merkezi gövdede `[0.6, 0, −0.125]` + R·…, yarıçap 0.04 m, duvar x = 2.00.
- Kullanılan poz: pom veya opti.

| [HESAP], kararlı temas pencereleri | ortalama | std | min | max |
|---|---|---|---|---|
| −f̂_x quasi-static [N] | 0.92 | 0.12 | 0.52 | 1.23 |
| −f̂_x, m·a dahil [N] | 0.94 | 0.73 | −1.72 | 3.68 |
| K·δ(opti) [N] | 0.004 | 0.009 | 0 | 0.064 |
| K·δ(pom) [N] | 0.15 | 0.27 | 0 | 1.61 |
| K·δ(pom) + f̂_x fark [N] | −0.76 | 0.34 | −1.23 | 0.91 |

Penetrasyon dağılımı, temas görevi boyunca [HESAP], yüzdelikler p1 / p5 / p50 / p95 / p99:
- opti: −3.07 / −1.24 / −0.03 / +0.04 / +0.08 mm. δ > 0 olan örnek oranı %36.
- pom: −5.67 / −3.46 / −0.24 / +1.92 / +2.79 mm. δ > 0 olan örnek oranı %42.
- Kararlı temasta pitch (pom): ort 3.8°, aralık 1.4–8.0°. Kinematiğe dahil edildi.

Sonuçlar:
1. **Yay modeli f̂ ile uyuşmuyor.**
   - −f̂_x ≈ 0.9 N'luk bir kuvvet K = 500 N/m'de yaklaşık **1.8 mm** penetrasyon gerektirir. Opti'ye göre penetrasyonun p99'u yalnızca **0.08 mm**.
   - Aynı varsayımlarla bu, etkin duvar sertliği için **≥ ~1.1·10⁴ N/m** alt sınırı verir [HESAP].
   - Varsayımlar: opti pozu base-link pozudur, SDF uç geometrisi doğrudur, çubuk rijittir.
   - K·δ(pom) ile −f̂_x arasındaki korelasyon −0.44, yani anlamlı bir pozitif ilişki yok.
2. **pom tabanlı penetrasyon kullanılamaz.** pom − opti x std'si (1.6 mm) beklenen penetrasyonla aynı mertebede. `plot_06a.py`'nin yaptığı gibi pom ile K·δ hesaplamak, kuvvet yerine kestirici gürültüsünü ölçer.
3. **Olası neden, doğrulanmadı.** World dosyasında `<physics type='unused'>` var ve `gz sim` ile başlatılıyor. Varsayılan motor (muhtemelen DART), `<ode><kp>/<kd>/<min_depth>` değerlerini uygulamıyor olabilir. Bu doğruysa `K_WALL = 500` bu simülasyonun duvar sertliği değildir. `params.py` bunu zaten ASSUMPTION olarak işaretliyor; bu koşu varsayımla çelişiyor.
4. Gerçek temas kuvveti için Gazebo'da bir temas sensörü ya da F/T sensörü gerekiyor. SDF'de "Contact sensor tip" yorumu var ama `<sensor>` tanımı yok.

## 5. Rotor hızlarının sınırlara yaklaşımı [LOG]

- Kaynak: `rotorcraft.log` `cmd_v0..5` (komut) ve `meas_v0..5` (ölçülen).
- Sınırlar `uavatt set_wlimit` ile aynı birimde: w_min = 16, w_max = 100. "Yakın" = sınıra aralığın %10'u kadar mesafe, yani ≥ 91.6 veya ≤ 24.4.
- `meas_v` yaklaşık 20 Hz'de güncelleniyor (`motor_rate` ort 20.4 Hz). Kısa tepeler kaçırılmış olabilir.

| cmd | serbest uçuş | temas görevi |
|---|---|---|
| rotor 1..6 min | 59.7 / 47.2 / 46.1 / 61.3 / 48.3 / 41.9 | 56.2 / 44.7 / 51.1 / 47.1 / 38.8 / 52.2 |
| rotor 1..6 max | 80.2 / 89.9 / 92.1 / 80.1 / 89.4 / 90.5 | 87.0 / 86.8 / 91.3 / 88.5 / 85.0 / 93.4 |
| rotor 1..6 ort | 71.1 / 71.1 / 71.4 / 70.8 / 71.2 / 71.4 | 72.3 / 68.9 / 74.9 / 68.9 / 66.8 / 76.5 |
| w_max'a en küçük pay | 7.9 | **6.6** (rotor 6) |
| w_min'e en küçük pay | 25.9 | 22.8 |
| herhangi bir rotor ≥ 91.6 olan örnek oranı | %0.10 | %0.41 |
| sınıra değen örnek | 0 | 0 |

- Ölçülen hızlarda (meas) w_max'a en küçük pay 6.3. Komut ile ölçülen arasındaki fark en fazla 4.3.
- **Saturasyon yok.** Temas sırasında yük asimetrik: rotor 3 ve 6 ortalaması +3.5…+5, rotor 5 ortalaması −4.
- Karşılaştırma için [HESAP]: `params.compute_G` ile hover hızı 71.3. Serbest uçuştaki log ortalaması 70.8–71.4, tutarlı.
- Bu koşudaki ~0.9 N itme kuvvetinde marj rahat. Daha yüksek temas kuvvetlerinde rotor 6 ve rotor 3 ilk sınıra yaklaşacak rotorlar.

## 6. Gerçek döngü periyodu [LOG ts]

- Ardışık `ts` farklarından hesaplandı.
- **Varsayım:** Log `ts` değeri döngünün çalıştığı anı yansıtıyor. GenoM loglarında `ts` bazen girdinin zaman damgası olabilir; bu doğrulanmadı.
- Beklenen değer: uavpos ve uavatt için 1 kHz, IMU `set_sensor_rate` için 1000. Gazebo fizik adımı 2 ms (`real_time_update_rate` 500).

| log | ort. oran [Hz] | dt medyan [ms] | dt ort [ms] | dt p99 [ms] | dt max [ms] | 0.9–1.1 ms aralığındaki dt oranı | > 20 ms boşluk |
|---|---|---|---|---|---|---|---|
| uavatt | 722 | 1.007 | 1.385 | 6.26 | 26.5 | %48 | 3 |
| uavpos | 716 (tekil ts: **573**) | 1.072 | 1.396 | 7.19 | 92.6 | %30 | 3 |
| pom | 669 | 1.046 | 1.494 | 6.36 | 86.3 | %39 | 3 |
| rotorcraft | 635 | 1.031 | 1.575 | 7.31 | 33.0 | %36 | 3 |
| opti | 100.0 | 10.000 | 10.000 (std 0.001) | 10.00 | 10.00 | — | 0 |
| maneuver | (yalnız hareket sırasında) | 5.00 | — | 10.8 | — | — | — |

- **Kontrol döngüleri nominal 1 kHz'e ulaşmıyor.** Medyan ≈ 1 ms, ama dağılımın uzun bir kuyruğu var:
  - aralıkların %21–26'sı medyanın 1.5 katından (~1.5 ms) uzun,
  - etkin ortalama oran 635–722 Hz.
- uavpos satırlarının **%20'si tekrarlanan ts** taşıyor (11579 satırda dt = 0). Bu, aynı zaman damgasıyla iki kez loglandığını gösteriyor. Neden doğrulanmadı.
- rotorcraft'ın kendi raporladığı `imu_rate`, uçuş sırasında ortalama 952 Hz (min 0, max 1094).
- > 20 ms boşluklar tüm component'lerde aynı anda görülüyor (t ≈ 3.11 s'de hepsinde ~23 ms). Diğerleri t ≈ 17.5, 28.0, 32.8–33.0 ve 64.6 s civarında. Bu, sistem genelinde duraklamalara işaret ediyor; kaynağı host veya Gazebo olabilir, doğrulanmadı.
- opti'nin dt'si tamamen düzenli (10.000 ms). Damgalar büyük olasılıkla simülasyon saatinden geliyor. Toplam süre pom ile aynı (80.69 s), yani ortalama RTF ≈ 1.
- Python kayıt döngüsünün (`simulation()`, 50 Hz nominal) gerçek periyodu **bilinmiyor**, çünkü npz'de duvar saati yok.

---

## Varsayımlar (ayrı liste)

1. m = 2.72 kg, g = 9.81 (`noc/common/params.py`). Rotor ataletleri ihmal edildi.
2. Euler açıları ZYX (yaw-pitch-roll). pom ve opti log `roll pitch yaw` değerleri bu konvansiyonda kabul edildi.
3. Uç küre merkezi gövdede `[0.6, 0, −0.125]`, r = 0.04. Duvar normali +x, yüzeyi x = 2.00. Çubuk rijit.
4. opti pozu = base-link pozu.
5. uavpos `fx, fy, fz` = dünya çerçevesinde toplam kuvvet komutu ve tam olarak gerçekleşiyor.
6. K_WALL = 500 ve D_WALL = 50 değerleri world dosyasından alındı; motorun bunları uyguladığı **doğrulanmadı**, §4'e göre büyük olasılıkla uygulanmıyor.
7. Faz eşikleri: |v_nom| < 1e-6, oturma süresi 1 s, temas görevi için x_nom > 1.5 ve x > 1.3, rotor "yakın" eşiği %10. Bunlar analiz seçimleri, fiziksel parametre değil.

## Bir sonraki koşudan önce düzeltilmesi gerekenler

- `model_hexa_fa.py` `read_*()`: exception yutulmamalı, hatayı loglamalı. Şu an sıfır döndürüp veriyi sessizce bozuyor.
- `phynt.log` neden boş kalıyor, bulunmalı.
- AF'nin neden referansı değiştirmediği bulunmalı.
- Tip'e bir Gazebo contact sensörü veya F/T sensörü eklenmeli; ancak o zaman temas kuvveti **ölçülmüş** olur.
- Fizik motoru ve duvar sertliği doğrulanmalı: motor açıkça seçilmeli ya da ölçülen δ–F'den K tanımlanmalı.
