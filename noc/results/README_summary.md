# Optimizasyon için doğrulanmış girdiler

Tarih 2026-11-09. Her satırda kaynak rapor ve rapor commit'i var.

**Tür etiketleri**

| Etiket | Anlamı |
|---|---|
| **ÖLÇÜLEN-F/T** | Gazebo force_torque sensörü (`ee_joint`). Kontrol zincirinden bağımsız tek kuvvet ölçümü. |
| **ÖLÇÜLEN-LOG** | Doğrudan loglanan veya geri okunan değer (zamanlama, parametre, konum). |
| **KESTİRİM** | Kontrol zincirinden kuvvet: uavpos `fx` (kuasi-statik) veya WO `efx`. Ölçüm değildir. |
| **HESAP** | Modelden veya loglardan offline türetilmiş değer. |

**Uçuşlar**

| Koşu | Rapor | Rapor commit | Uçuş kodu | Kapı |
|---|---|---|---|---|
| t02 (K_x basamak) | `02_live_k.md` | `4a848f7` (sonuç dosyaları `be511f3`) | `921cf13` | A: KALDI |
| t03 (K_x rampa) | `03_k_ramp.md` | `be511f3` | `cd05d95` | A2: GEÇTİ |
| t04 (F/T) | `04_ft_check.md` | `95dd080` | `ed28b44` | B1: GEÇTİ |
| 04b (t04 logları, uçuşsuz) | `04b_impulse_origin.md` | `a3e63a7` | – | kapı dışı |

Temel config: `noc/config/af_on_baseline.json` (`b39a541`).

## 1. Karar değişkeni: K_x

| Girdi | Değer | Tür | Kaynak | Commit |
|---|---|---|---|---|
| Canlı değişim çalışıyor | t02: 205/205 çağrı geri okundu, 6100/6100 döngüde `af.K_x` = hedef | ÖLÇÜLEN-LOG | `02_live_k.md` Kapı A | `4a848f7` |
| Rampada uygulanan K | t03: 100/100 çağrı, 5300/5300 döngü hedefle aynı | ÖLÇÜLEN-LOG | `03_k_ramp.md` C2 | `be511f3` |
| Denenen aralık | K_x = 10–20 N/m; **dışı denenmedi** | ÖLÇÜLEN-LOG | `02_live_k.md`, `03_k_ramp.md` | `4a848f7`, `be511f3` |
| Servo kesintisi | yok: çağrı pencerelerinde uavpos.log boşluğu ≤ 13.7 ms | ÖLÇÜLEN-LOG | `02_live_k.md` Kapı A | `4a848f7` |

## 2. Kuvvet modeli: F = d0 / (1/Kpxy + 1/K_x), Kpxy = 5

d0 = referans − gövde konumu, x'te.

| Girdi | Değer | Tür | Kaynak | Commit |
|---|---|---|---|---|
| Öngörü, K_x = 10, d0 = 0.184 | 0.613 N | HESAP | `04_ft_check.md` §4 | `95dd080` |
| **F/T doğrulaması, K_x = 10** | **0.616 N**, %95 GA 0.540–0.697 N; model farkı −%0.4 | ÖLÇÜLEN-F/T (tek uçuş) | `04_ft_check.md` §4 | `95dd080` |
| Aynı veri, ön kayıtlı 2 s zaman zarfı | 0.574 N; model farkı +%6.8 | ÖLÇÜLEN-F/T | `04_ft_check.md` §2, §4 | `95dd080` |
| d0 ölçülen (t04, temas) | 0.1843 ± 0.0019 m | ÖLÇÜLEN-LOG | `04_ft_check.md` §4 | `95dd080` |
| K_x = 10–20, fx ile karşılaştırma | hata −%6.5 … −%0.4 (F = 0.573–0.733 N) | KESTİRİM | `02_live_k.md` §3, `03_k_ramp.md` §4 | `4a848f7`, `be511f3` |
| WO efx (t04 temas) | −0.650 N; F/T'den fark −0.034 N, GA sıfırı içeriyor | KESTİRİM | `04_ft_check.md` §3.1 | `95dd080` |
| −uavpos fx (t04 temas) | −0.608 N; F/T'den fark +0.008 N, GA sıfırı içeriyor | KESTİRİM | `04_ft_check.md` §3.1 | `95dd080` |

**Doğrulamanın kapsamı:** F/T ile tek çalışma noktası (K_x = 10, d0 = 0.184, Kpxy = 5), çözünürlük yaklaşık ±%13. K_x = 12–20 yalnızca kontrol zincirindeki fx ile karşılaştırıldı. Kpxy hiç değiştirilmedi.

## 3. Kısıt: K_x değişimi rampa ile

| Girdi | Değer | Tür | Kaynak | Commit |
|---|---|---|---|---|
| Basamakta fx sıçraması (ΔK = 2–5 N/m) | çağrı başına 0.064–0.143 N; 4/5 değişimde bayrak | ÖLÇÜLEN-LOG | `02_live_k.md` §2 | `4a848f7` |
| Sıçrama modeli | Δfx ≈ m·ΔK·\|dev\|/M_af (öngörü 0.065–0.141 N) | HESAP | `02_live_k.md` §2 | `4a848f7` |
| Denenen rampa | **10 N/m/s**, 0.2 N/m/çağrı, 50 Hz, 1 s (10 ↔ 20); başka hız denenmedi | ÖLÇÜLEN-LOG | `03_k_ramp.md` | `be511f3` |
| Rampada çağrı başına \|Δfx\| | medyan 0.0053 N, maks. 0.034 N | ÖLÇÜLEN-LOG | `03_k_ramp.md` §2 | `be511f3` |
| Rampa penceresinde maks. \|Δfx\| | 0.036 N ve 0.053 N (sınır 0.064 N) | ÖLÇÜLEN-LOG | `03_k_ramp.md` §2 | `be511f3` |
| Rampada maks. \|Δxd\| | K fazında 0.26 mm (sınır 1 mm) | ÖLÇÜLEN-LOG | `03_k_ramp.md` §3 | `be511f3` |

Not: Pencere bazlı sınırın ayırt etme gücü düşük; t03 gürültüsünde bir pencerenin geçme olasılığı yaklaşık %83 [HESAP]. Asıl kanıt çağrı başına adım.

## 4. Ulaşılabilir kuvvet aralığı

Tablo tamamen seri modelden **HESAP**. Model, denenen noktaların dışına uzatılıyor.

| Girdi | Değer | Tür | Kaynak | Commit |
|---|---|---|---|---|
| Üst sınır, d0 = 0.184, Kpxy = 5, K_x → ∞ | **0.92 N** (= d0·Kpxy) | HESAP | bu dosya; model `04_ft_check.md` §4 | `95dd080` |
| Denenen aralık K_x = 10–20 | 0.613–0.736 N | HESAP | `03_k_ramp.md` §4 | `be511f3` |
| 2 N hedefi | **mevcut d0 ve Kpxy ile ulaşılamaz** | HESAP | bu dosya | `95dd080` |

2 N için gereken değerler [HESAP]:

| K_x [N/m] | Gereken d0 (Kpxy = 5) | Gereken Kpxy (d0 = 0.184 m) |
|---|---|---|
| 10 | 0.600 m | **yok** (K_x > 10.87 gerekir) |
| 12 | 0.567 m | 115.4 |
| 15 | 0.533 m | 39.5 |
| 20 | 0.500 m | 23.8 |
| → ∞ | 0.400 m | 10.9 |

Gövde temas konumu x ≈ 1.366 m [ÖLÇÜLEN-LOG, t04 pom], dolayısıyla d0 = 0.5 m için referans x ≈ 1.87 m olur [HESAP]. Bu d0 ve Kpxy değerleri uçurulmadı. Kararlılık, doyma ve 3.5 N güvenlik sınırı (`flight.SAFETY`) bu tabloda değerlendirilmedi.

## 5. Zamanlama

| Girdi | Değer | Tür | Kaynak | Commit |
|---|---|---|---|---|
| `set_af_parameters` p95 (ΔK = 0, n = 200) | 6.01 ms (maks. 7.85) | ÖLÇÜLEN-LOG | `02_live_k.md` §5 | `4a848f7` |
| `set_af_parameters` p95 (rampa, n = 100) | 8.18 ms (maks. 11.7) | ÖLÇÜLEN-LOG | `03_k_ramp.md` §6 | `be511f3` |
| `get_af_parameters` p95 | 4.01 ms (t02), 7.26 ms (t03) | ÖLÇÜLEN-LOG | `02_live_k.md` §5, `03_k_ramp.md` §6 | `4a848f7`, `be511f3` |
| Döngü periyodu | nominal 20 ms (50 Hz); dt p99 25.3 ms (t02), 29.9 ms (t03) | ÖLÇÜLEN-LOG | `02_live_k.md` §5, `03_k_ramp.md` §6 | `4a848f7`, `be511f3` |
| `phynt.set_wo_zero` | 2.5 s bloklar (uçuş başında bir kez) | ÖLÇÜLEN-LOG | `02_live_k.md` §5, `03_k_ramp.md` §6 | `4a848f7`, `be511f3` |
| RTF | uçuş 0.680, temas 0.645 (yalnız t04; t02/t03'te sim zamanı kaydı yok) | ÖLÇÜLEN-LOG | `04_ft_check.md` C2, `04b_impulse_origin.md` | `95dd080`, `a3e63a7` |

Kontrol wall-clock'ta, tesis sim zamanında çalışıyor; RTF 0.64'te wall-clock hızları sim zamanında yaklaşık 1/RTF kat büyük [HESAP].

## 6. Bilinen sınırlamalar

| Sınırlama | Durum | Tür | Kaynak | Commit |
|---|---|---|---|---|
| **Rijit temas** | Motor DART; `kp/kd/min_depth` okunmuyor. Duvar pratikte rijit. | kaynak kod ve ikili dosya | `04b_impulse_origin.md` §5 | `a3e63a7` |
| Darbeli F/T | ~29 Hz, çoğu 1–2 fizik adımı, tepe −50 N; medyan ≈ 0 | ÖLÇÜLEN-F/T | `04b_impulse_origin.md` §1 | `a3e63a7` |
| Uç duvardan ayrılıyor | boşluk 2.3 mm'ye kadar | ÖLÇÜLEN-LOG (opti) | `04b_impulse_origin.md` §4 | `a3e63a7` |
| **Darbelerin kökeni kesin ayrılmadı** | 04b: darbe şekli DART rijit temasından, ortalama kuvvet kontrolden. Kontrol varyantlarıyla test edilmedi; t06 yapılmayacak (karar 2026-11-09). Optimizasyon ortalama kuvvet üzerinden. | sınırlama | `04b_impulse_origin.md` §6 | `a3e63a7` |
| İlk çarpma | yaklaşık 350 N (−349.7 N, 2 s temas goto) | ÖLÇÜLEN-F/T | `04b_impulse_origin.md` §3 | `a3e63a7` |
| Maneuver duvar saatinde | RTF ≈ 0.64; komut hızı sim zamanında yaklaşık 1.56 kat büyük | HESAP | `04b_impulse_origin.md` §2, §3 | `a3e63a7` |
| **Ölçüm kuralı** | Kuvvet = sim zamanına hizalı, 0.5 s hareketli ortalamalı F/T normal kuvveti; ham F/T yalnız tanı için. Filtre sonrası fx ile RMSE 0.22 N. | kural | `README.md`, `04_ft_check.md` §3.3 | bu commit, `95dd080` |
| **Oturma süresi ölçülemedi** | Güvenilir değer yok. Örnekler: basamak 1.87–2.47 s (t02), rampa 1.02 s ve ≥ 9.96 s censored (t03), geçiş başına tek örnek. | ÖLÇÜLEN-LOG, gürültü ile örtülü | `02_live_k.md` §4, `03_k_ramp.md` §5 | `4a848f7`, `be511f3` |
| Oturma modeli | %2 oturma: K = 10 için ≈ 4.1 s, K = 20 için ≈ 2.8 s | HESAP | `03_k_ramp.md` §5 | `be511f3` |
| **B sabit** | 14.14 N·s/m; K_x değişince güncellenmedi | ÖLÇÜLEN-LOG | `02_live_k.md` başlık | `4a848f7` |
| Sönüm oranı | K = 20'de ζ = 0.71 (K = 10'da 1.0) | HESAP | `02_live_k.md` §3 | `4a848f7` |
| WO–fx farkı | Tutmadan tutmaya 0.0–0.1 N değişiyor; AF sapması WO'yu izliyor | KESTİRİM | `02_live_k.md` §3, `03_k_ramp.md` §4 | `4a848f7`, `be511f3` |
| Gecikme | F/T'ye göre WO ve fx gecikmesi belirlenemedi | ÖLÇÜLEN-F/T, yöntem sınırlı | `04_ft_check.md` §3.2 | `95dd080` |
| Kapı A | Tanım gereği KALDI; ölçüt sonradan değiştirilmedi | – | `02_live_k.md` | `4a848f7` |
