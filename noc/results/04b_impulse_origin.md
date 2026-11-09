# 04b — F/T temas darbelerinin kökeni (t04 verisiyle, uçuşsuz)

- **Veri:** `logs/noc/t04_ft_check/`. Yeni uçuş yok.
- **Script:** `noc/analysis/a04c_impulse_origin.py`.
- **Çıktılar:** `noc/results/04b_impulse_origin/metrics.json`, `impulse_origin.png`.
- **Yeniden üretim:** `.venv/bin/python -m noc.analysis.a04c_impulse_origin` (permütasyon seed'i 0).
- **Durum:** uçuş sonrası ve kapı dışı analiz. Eşikler ve pencereler veri görüldükten sonra seçildi.

**Etiketler:** **[SİM]** Gazebo F/T sensörü. **[LOG]** loglanmış sinyal. **[HESAP]** offline türetilmiş değer.
**Zaman tabanı:** F/T başlık damgası sim zamanı. Component log'ları wall-clock; 0.1 s'lik zarfla eşlendi (04_ft_check.md §2).
Pencere CONTACT: 10.0 s wall = 6.45 s sim, RTF 0.645.

---

## Özet

1. **Darbeler rastgele aralıklı, çoğu 1–2 fizik adımı süren çarpmalar.** Ortalama 29 Hz, medyan süre 2 ms.
2. **AF, uavpos veya uavatt frekanslarıyla ilişki görünmüyor.** Spektrum geniş bant; 29 Hz bu döngülerin 10–130 katı.
3. **Uç gerçekten ayrılıyor.** opti'ye göre boşluk 2.3 mm'ye çıkıyor.
4. **Fizik motoru DART.** `<ode><kp>/<kd>/<min_depth>` bu motorda **okunmuyor**. Duvar pratikte rijit.
5. **Bu veri tek başına kontrol/simülatör ayrımını kapatmıyor.** Rijit temas gerekli koşul, kontrol ise yük sağlıyor. Ayrımı t06 verecek.

---

## 1. Darbelerin tekrarı, süresi ve spektrumu

CONTACT, sim zamanı, 3225 örnek, |Fn − bias| eşiği 0.5 N, adım 2 ms [SİM]:

| Büyüklük | Değer |
|---|---|
| darbe sayısı | 189 |
| ortalama tekrar | **29.3 Hz** |
| darbe aralığı medyan (p5–p95) | **16 ms** (4–137 ms) |
| aralık CV | **1.22** (belleksiz süreçte 1) |
| süre medyan (p95, maks.) | **2 ms** (8, 16 ms) |
| tek adımlık darbe oranı | %60 (iki adım: %25) |
| tepe medyan (en büyük) | −5.2 N (−50.0 N) |
| darbe impulsu medyan | −0.0129 N·s |
| Σ impuls / süre | **−0.608 N** |
| ortalama Fn − bias | −0.616 N |

- Kuvvetin ortalaması neredeyse tamamen darbelerden geliyor (%98.7). Darbeler arasındaki örnekler sıfıra yakın.
- Sonuç eşik 0.2–1 N arasında değişmiyor: 27.5–29.3 Hz.
- Periyodiklik yok: darbe dizisi otokorelasyonu ≤ 0.073 (beyaz gürültü bandı ±0.035).
  En yüksek değer 4 ms gecikmede; darbeler kısa kümeler halinde geliyor.
- Spektrum [HESAP, Welch 1024]: düzlük 0.91 (1 = beyaz). Varyans: < 10 Hz %3.7, 10–50 Hz %20.8, 50–100 Hz %20.8, 100–250 Hz %53.6.
  Baskın çizgi yok; en yüksek tepeler (30.3, 46.9, 42.0, 92.3 Hz) gürültü seviyesinde birbirine yakın.

## 2. Kontrol frekansları ve fizik adımıyla ilişki

| Döngü | Değer | Kaynak | Frekans |
|---|---|---|---|
| AF | √(K/M) = √(10/5) = 1.41 rad/s, ζ = 1 | `af_on_baseline.json` | **0.225 Hz** |
| uavpos | √(Kp/m) = √(5/2.72) = 1.36 rad/s, ζ = 0.95 | config, yasa 00b | **0.216 Hz** |
| uavatt | √(Kq/Jyy) = √(4/0.0114) = 18.7 rad/s, ζ = 1.87 | log regresyonu τy = −4.0 e_r − 0.8 e_w (R² = 1.0) | **2.98 Hz** |
| optitrack | | world `<hz>` | 100 Hz |
| kontrol log hızı | | uavpos/uavatt log | ~990 Hz (wall) |
| fizik adımı | 2 ms | world `max_step_size` | 500 Hz |
| **darbe tekrarı** | | §1 | **29.3 Hz, düzensiz** |

- Darbe tekrarı AF/uavpos'un yaklaşık 130 katı, uavatt'ın 10 katı. Spektrumda bu döngülere ait çizgi yok.
- Darbe süresi fizik adımıyla aynı ölçekte: %60'ı tam 1 adım. Motor seviyesinde çözülen kısa olaylarla uyumlu.
- Darbe dizisi ile komutlar arasındaki koherans [HESAP] 20 Hz altında ortalama 0.15 (fx), 0.25 (τy). Şans düzeyi yaklaşık 0.08; üstünde ama zayıf.
  Yavaş döngülerden gelen yük ile yavaş modülasyon olması mümkün; bu veriyle ayrılamaz.
- Not: Kontrol döngüleri wall-clock, tesis sim zamanı. RTF 0.64 iken kontrol, tesise göre yaklaşık 1.56 kat hızlı işliyor. Bu nedenle frekanslar sim zamanında kaymış olabilir, ama büyüklük mertebesi değişmez.

## 3. Darbe anlarında uç hızı ve pozisyon hatası

189 darbe; darbe başlangıcındaki ortalama ile pencere ortalaması. 2000 permütasyon, seed 0. Bonferroni eşiği 0.05/6 = 0.0083.
Sinyaller wall-clock kaynaklı; zaman eşleme jitter'ı yaklaşık 25 ms.

| Sinyal | Darbe anında | Pencere | p |
|---|---|---|---|
| uç hızı vx [mm/s, pom, +x duvara] | **+3.0** (std 18.7) | −1.8 (std 23.3) | **0.005** |
| duvar boşluğu [mm, opti] | **0.12** | 0.33 | **< 0.001** |
| konum hatası xd − x [mm, maneuver − pom] | 184.03 | 184.32 | 0.029 |
| uavpos e_x [mm] | −118.7 | −118.8 | 0.86 |
| uavpos fx [N] | 0.598 | 0.609 | 0.23 |
| uavatt τy [N·m] | −0.157 | −0.170 | 0.031 |

- Darbeler, uç duvara doğru hareket ederken ve boşluk küçükken oluşuyor. Olay ortalamasında uç hızı darbeden yaklaşık 20 ms sonra +6 mm/s, 80 ms sonra −6 mm/s (geri sekme).
  Bu, jitter ölçeğinde olduğu için zamanlama kaba okunmalı.
- Konum hatası ve komutlar darbe anında pencereden Bonferroni düzeyinde farklı değil. Darbe tetikleyen bir kontrol kaynağı görünmüyor; 0.1 N düzeyinde fark bu çözünürlükte görülemez.

**İlk çarpma:** temas goto'sundan 1.18 s sonra, tepe −349.7 N [SİM], ilk 20 ms impulsu −0.742 N·s, uç hızı 0.224 m/s (pom, wall-clock birimi) = **yaklaşık 0.35 m/s sim zamanında** [HESAP, /RTF].
Etkin kütle 0.742/0.35 = 2.1 kg [HESAP]. Rijit tek adım tahmini 2.1 × 0.35 / 0.002 = 368 N, ölçülen −350 N ile uyumlu.

## 4. Uç duvardan gerçekten ayrılıyor mu?

Duvar yüzeyi x = 2.000 m. Uç yüzeyi = gövde x + R·(0.6, 0, −0.125) + 0.04. Boşluk > 0 = temas yok.

| Kaynak | Örnek | Medyan | Maks. | Min. | ≤ 0 | ≤ 0.5 mm | ≤ 1 mm | ≤ 2 mm |
|---|---|---|---|---|---|---|---|---|
| opti (100 Hz, ham pose) | 999 | **+0.14 mm** | **+2.27 mm** | −0.13 mm | %21.6 | %76.6 | %89.6 | %99.5 |
| pom (kestirim) | 6765 | +0.45 mm | +5.0 mm | −2.8 mm | %39.3 | %51.2 | %63.9 | %87.1 |

- **Evet, uç ayrılıyor.** opti boşluğu 2.3 mm'ye çıkıyor; opti'nin serbest uçuşta ardışık fark std'si 0.22 mm. pom gürültülü olduğu için boşluk ölçümünde opti esas alındı.
- Temas oranı toleransa bağlı: **%22 (0 mm) ile %77 (0.5 mm) arasında**. Kesin tek bir oran verilemez: 100 Hz ile 2 ms'lik olaylar çözülemez, geometri (uç pozu, gövde çerçevesi) mm altı belirsizlik taşır.
- F/T ile karşılaştırma: |Fn − bias| > 0.1 N olan örnek **%12.3**, > 0.05 N %30.8. İki kaynak da uç temasının sürekli olmadığını gösteriyor.

## 5. Fizik motoru ve `<ode><kp>/<kd>`

**Kurulu sürümler:** gz-sim 9.5.0, gz-physics 8.3.0, DART 6.13.2 (`dpkg -l`, container `art/tk3lab:ionic-0.2`).

**Motor DART:**
- `gz sim` çağrısında `--physics-engine` yok (`noc/experiments/sim_noc.sh`).
- World'de `<engine>` yok; `<physics type='unused'>` motor seçmiyor.
- Physics system varsayılanı `gz-physics-dartsim-plugin`: [Physics.cc#L843-845](https://github.com/gazebosim/gz-sim/blob/gz-sim9_9.5.0/src/systems/physics/Physics.cc#L843-L845).
- World'de Physics plugin'i yok; varsayılan `server.config` yükleniyor ([SimulationRunner.cc#L1620-1666](https://github.com/gazebosim/gz-sim/blob/gz-sim9_9.5.0/src/SimulationRunner.cc#L1620-L1666)). Container'daki iki `server.config`'te de `<engine>` yok.

**`kp`, `kd`, `min_depth`, `max_vel` DART'ta uygulanmıyor:**
- `ConstructSdfCollision` yüzeyden yalnızca friction (`mu`, `mu2`, `slip1/2`), `bounce/restitution_coefficient` ve `contact/collide_bitmask` okuyor: [SDFFeatures.cc#L922-997](https://github.com/gazebosim/gz-physics/blob/gz-physics8_8.3.0/dartsim/src/SDFFeatures.cc#L922-L997).
- Aynı dosyada `kp`, `kd`, `min_depth`, `max_vel`, `soft_cfm`, `soft_erp` geçmiyor (grep: 0).
- Kurulu `libgz-physics8-dartsim-plugin.so` içinde de bu dizgiler yok (`strings`: 0); `bounce`, `restitution_coefficient`, `collide_bitmask` var.
- DART sabit temas parametreleri: ERP 0.01, maks. düzeltme hızı 1 mm/s, CFM 1e−5, hata payı 0: [ContactConstraint.cpp#L45-48](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/ContactConstraint.cpp#L45-L48). World'de `bounce` yok, sekme kapalı.

**Veriyle tutarlılık [HESAP]:** kp = 500 N/m olsaydı 0.6 N için ≥ 1.2 mm, 50 N için 100 mm, 350 N için 700 mm batma gerekirdi. opti'de en fazla 0.13 mm batma var. `params.py` içindeki `K_WALL`/`D_WALL` bu simülasyonda geçerli değil. Bu, 00_baseline.md §4'teki doğrulanmamış şüpheyi kapatıyor.

**Kanıt düzeyi:** Kaynak kodu ve kurulu ikili dosya. Bu koşuda yüklenen eklentinin doğrudan kaydı yok; `gz sim` log'u saklanmamış. İsteğe bağlı çalışma anı doğrulaması: `gz sim -v 4` çıktısında yüklenen dartsim plugin satırı (komut verildi).

## 6. Yorum ve sınırlar

**Veriyle desteklenen:**
- Duvar rijit, temas kısa çarpmalar olarak çözülüyor. Darbe genişliği 1–2 adım; tepe kuvvet m_etkin·Δv/dt ile ölçekleniyor.
- Darbeler AF/uavpos/uavatt ile aynı frekansta değil. Darbe anında kontrol sinyalinde belirgin fark yok.
- Kontrol yükü (yaklaşık 0.6 N) sürekli itiyor; bunun ortalaması darbelerin ortalamasıyla aynı (§1). Yani kontrol darbelerin **ortalamasını** belirliyor; darbe **şeklini** görünüşe göre temas çözücüsü belirliyor.

**Bu analizin kapatmadığı:** Ayrılma kaynağı doğrulanmadı. Temas sonrası gövde/kol dinamiği mi, DART'ın hata düzeltmesi mi veya sürekli temas için çözücü davranışı mı, bu veriyle ayrılamaz. Kontrol değişikliklerinin darbeleri azaltıp azaltmadığını t06 gösterecek.

**Sınırlar:** Tek uçuş, 6.45 s sim temas. Pencere ve eşikler analiz sırasında seçildi. RTF 0.64.
