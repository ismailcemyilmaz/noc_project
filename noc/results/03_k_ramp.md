# 03 — Uçuşta K_x rampası (t03) ve Kapı A2

- **Veri:**
  - `logs/noc/t03_k_ramp/` (t03 uçuşu, 2026-11-04 23:08, 5300 örnek @ 50 Hz, abort/hata yok)
  - `logs/noc/t03_check/` (uçuşsuz `--check`, 23:07)
  - Karşılaştırma için `logs/noc/t02_live_k/` (t02, basamaklar)
- **Script'ler:** koşu `noc/tests/t03_k_ramp.py` (git rev `cd05d95`), analiz `noc/analysis/a03_k_ramp.py`
- **Ölçütler:** `noc/config/t03_k_ramp_criteria.json`, uçuştan önce commit edildi.
  - `f71c8cd`: Kapı A2 ölçütleri C1–C5.
  - `24165f5`: Değişiklik 1; yalnızca gate'e girmeyen 2σ oturma kuralı.
  - Analiz sırasında okunan dosyanın sha256'sı `c09533c9…`. Uçuştan sonra dosyada değişiklik yok
    (`git diff HEAD -- noc/config/` boş).
- **Çıktılar:**
  - `noc/results/03_k_ramp/metrics.json` (sha256 envanteri, ölçüt sha256 ve commit dahil)
  - `summary.md` (otomatik tablolar)
  - `k_ramp.png`
- **Yeniden üretim:** `.venv/bin/python -m noc.analysis.a03_k_ramp`
- **Seed:** yok (koşuda RNG yok; simülatör seed'i kaydedilmiyor).
- **Senaryo:**
  - 06a temas noktası (1.55, 0, 1).
  - K_x = 10 (20 s) → rampa 10 N/m/s (1 s, 50 çağrı × 0.2 N/m) → 20 (20 s) → rampa → 10 (20 s).
  - M, B (14.14 N·s/m), J ve K[1:] sabit.

**Etiketler:** **[LOG]** loglanmış sinyal (bazıları kendisi kestirim), **[HESAP]** offline türetilmiş değer.
**Bu raporda temas kuvveti ölçümü yok.** `F = uavpos fx` [HESAP, kuasi-statik] ve WO `efx` [LOG, kestirim]
aynı zincirden geliyor; bağımsız değiller.

---

## Kapı A2 sonucu: **GEÇTİ**

| Ölçüt (önceden commit edildi) | Sonuç | Değer |
|---|---|---|
| C1 abort, hata ve okuma hatası yok | ✅ | abort yok, hata yok, okuma hatası 0 (beklemede fazında da 0) |
| C2 uygulanan K her döngüde hedefle aynı | ✅ | 5300/5300 döngü, 100/100 çağrı tam geri okumayla doğrulandı |
| C3 rampa sırasında maks. \|Δfx\| ≤ 0.064 N | ✅ | rampa 1: **0.036 N**, rampa 2: **0.053 N** |
| C4 K fazında maks. \|Δxd\| ≤ 1 mm/örnek | ✅ | **0.26 mm** |
| C5 kalıcı F öngörünün ±%15'i içinde | ✅ | −1.5 %, −0.4 %, −3.1 % |

t02'nin sonucu (Kapı A **KALDI**) değişmedi; bu rapor onu yeniden değerlendirmiyor.

**C3 hakkında not:** C3 geçti, ama bu koşuda C3'ün ayırt etme gücü düşük (bkz. §2). Rampadaki
maksimumlar, aynı koşunun çağrı içermeyen pencerelerindeki gürültü dağılımının içinde kalıyor.
"Rampa basamak etkisini ortadan kaldırdı" sonucunu destekleyen asıl kanıt C3 değil, §2'deki çağrı
başına adım ölçümü.

---

## 1. Beklemede kaynaklar ve preflight

- **Preflight** (`preflight.json`): phynt_des ve ext_wrench `status: pending` olarak kaydedildi.
  Bu koşuda ikisi de hatasız okundu; EMPTY_POSTER görülmedi. Diğer 7 kaynak OK.
- **Goto öncesi:** Her iki kaynağın ts'si 1793830127.764'te donuk [LOG]. Bu, t02'deki
  başlangıç davranışıyla aynı (02_live_k.md §1).
- **Hazır olma:** İlk goto'dan **13.5 ms** sonra iki kaynak da hazır (taze ts, bütün alanlar
  geçerli). Sınır 2 s.
- **Okuma hatası:** Ne beklemede fazında ne sonrasında okuma hatası var (`read_error_counts_all = {}`).

## 2. K değişimi sırasında fx: rampa ve basamak

| koşu | değişim | pencere | maks. \|Δfx\| [LOG] | öngörülen adım [HESAP] |
|---|---|---|---|---|
| t03 | 10 → 20 rampa | rampa (1.00 s) | 0.036 N | çağrı başına m·0.2·\|dev\|/M_af ≈ 0.0065 N |
| t03 | 20 → 10 rampa | rampa (1.08 s) | 0.053 N | ≈ 0.0065 N |
| t02 | 10 → 12 basamak | çağrı ±1 s | 0.064 N | 0.065 N |
| t02 | 12 → 15 | çağrı ±1 s | 0.103 N | 0.087 N |
| t02 | 15 → 20 | çağrı ±1 s | 0.143 N | 0.120 N |
| t02 | 20 → 15 | çağrı ±1 s | 0.098 N | 0.113 N |
| t02 | 15 → 10 | çağrı ±1 s | 0.133 N | 0.141 N |

**Gürültü seviyesi:** Tutmaların içinde, K değişiminden en az 2 s uzakta kalan 1 s'lik
pencerelerde maks. \|Δfx\| [LOG]:

| koşu | pencere sayısı | medyan | p90 | maks. | 0.064 N'u aşan pencere oranı |
|---|---|---|---|---|---|
| t02 | 39 | 0.033 N | 0.052 N | 0.064 N | 5 % |
| t03 | 47 | 0.044 N | 0.074 N | 0.127 N | **17 %** |

- **t03'te fx gürültüsü t02'den yüksek.** Plotun 4. panelinde tutmalarda 0.10–0.15 N'luk tek
  satırlık sıçramalar görülüyor. Nedeni bu verilerle belirlenemedi. Döngü yükü bu koşuda biraz daha
  yüksek: set p95 8.2 ms (t02: 6.0), döngü dt p99 29.9 ms (t02: 25.3). Ancak bunun fx'e nasıl
  yansıdığı gösterilmedi.
- **C3'ün gücü düşük.** Sadece gürültü olsa bile bir 1 s'lik pencerenin C3'ü geçme olasılığı bu
  koşuda yaklaşık %83. Pencereler bağımsız varsayılırsa iki pencerenin birlikte geçme olasılığı
  yaklaşık %69 [HESAP]. Yani C3'ün geçmesi rampanın düzgünlüğünü tek başına kanıtlamıyor.
- **Asıl kanıt çağrı başına adım.** 100 rampa çağrısının her birinde, çağrıdan hemen önceki ve
  sonraki uavpos satırı arasındaki \|Δfx\| [LOG]: medyan **0.0053 N**, maks. 0.034 N.
  - Medyan, öngörülen 0.0065 N'luk ivme ileri besleme adımıyla aynı büyüklükte.
  - t02'de tek çağrıdaki adım 0.064–0.143 N idi (çağrı anında, ±1 ms).
  - Rampa bu adımı yaklaşık 50 küçük adıma bölüyor; her biri gürültünün altında kalıyor.
- **Rampa maksimumlarının konumu.** Rampa 1'in maksimumu en yakın çağrıdan 4.1 ms, rampa 2'ninki
  0.7 ms uzakta. Çağrılar 20 ms arayla geldiği için her nokta bir çağrıya en fazla 10 ms uzakta;
  bu yakınlık tek başına bir şey kanıtlamıyor.

## 3. Pozisyon referansı (C4)

- K fazındaki maks. \|Δxd\| = 0.26 mm, t = 89.74 s'de [LOG].
- Bu noktada K fazının en uzun uavpos.log boşluğu var: 34.6 ms. Yani 0.26 mm'lik değişim tek bir
  satır adımında değil, 34.6 ms'lik bir aralıkta oluşmuş. Zaman başına bakıldığında bir sıçrama
  değil.
- Rampa pencerelerindeki maks. \|Δxd\|: 0.05 ve 0.15 mm. t02'deki basamaklarda en fazla 0.06 mm idi.

## 4. Kalıcı durum (her tutmanın son 10 s'si)

| koşu | K_x | F = fx [HESAP] | öngörü | hata | \|WO efx\| [LOG] | AF sapması [LOG] | K·\|dev\| [HESAP] |
|---|---|---|---|---|---|---|---|
| t03 | 10 | 0.604 N | 0.613 | −1.5 % | 0.686 | −68.4 mm | 0.684 |
| t03 | 20 | 0.733 N | 0.736 | −0.4 % | 0.729 | −35.9 mm | 0.718 |
| t03 | 10 | 0.594 N | 0.613 | −3.1 % | 0.638 | −64.4 mm | 0.644 |
| t02 | 10 | 0.587 N | 0.613 | −4.3 % | 0.591 | −59.4 mm | 0.594 |
| t02 | 20 | 0.725 N | 0.736 | −1.5 % | 0.823 | −41.7 mm | 0.834 |
| t02 | 10 (son) | 0.573 N | 0.613 | −6.5 % | 0.620 | −63.6 mm | 0.636 |

Öngörü (seri yay modeli) [HESAP]: F = d₀ / (1/Kpxy + 1/K_x), d₀ = 0.184 m, Kpxy = 5.

- **F seri modeli izliyor.** İki koşuda aynı K değerleri için fark en fazla 0.02 N.
- **AF sapması yine F'yi değil WO kestirimini izliyor.** K·\|dev\| ile \|WO efx\| arasındaki fark
  en fazla 0.011 N.
- **WO ve fx farkı bu koşuda başka yerde büyük.** Fark ilk K = 10 tutmasında 0.08 N; bu yüzden
  sapma öngörüden (−61.3 mm) 7 mm fazla. t02'de büyük fark K = 20'deydi.
- WO ile fx arasındaki farkın tutmadan tutmaya değişmesi 02_live_k.md §3'teki gözlemle tutarlı.
  İkisi de kestirim; hangisinin gerçek kuvvete daha yakın olduğu bu veriyle söylenemez.

## 5. Oturma süresi (2σ bandı, değişiklik 1 kuralı)

Kural: Bant, kalıcı kısımdaki 0.5 s hareketli ortalamanın ortalaması ± 2σ. Oturma süresi,
referans anından kalıcı kısım başlamadan önceki son band dışı örneğe kadar geçen süre.
"≥" ile gösterilen değer censored: sinyal kalıcı kısım başlarken hâlâ bandın dışında, değer bir
alt sınır.

| koşu | geçiş | referans anı | oturma | 2σ bandı (±) |
|---|---|---|---|---|
| t03 | temas → K = 10 | temas goto'sunun sonu | 4.51 s | 11.2 mm |
| t03 | 10 → 20 rampa | son rampa çağrısı | **1.02 s** | 9.6 mm |
| t03 | 20 → 10 rampa | son rampa çağrısı | **≥ 9.96 s (censored)** | 5.7 mm |
| t02 | temas → K = 10 | temas goto'sunun sonu | 8.37 s | 6.2 mm |
| t02 | 10 → 12 | set çağrısı | 2.12 s | 6.7 mm |
| t02 | 12 → 15 | set çağrısı | 2.47 s | 7.1 mm |
| t02 | 15 → 20 | set çağrısı | 2.37 s | 6.3 mm |
| t02 | 20 → 15 | set çağrısı | 2.09 s | 3.4 mm |
| t02 | 15 → 10 | set çağrısı | 1.87 s | 4.4 mm |

- **Rampa ile basamak arasında oturma farkı bu veriyle kurulamaz.**
  - t03 10 → 20: 1.02 s. Bu, t02'deki 1.9–2.5 s'den kısa.
  - Ama t03'ün ikinci geçişi censored çıktı. Plotta sapma 72–76 s arasında hedefin altına iniyor,
    sonra yavaşça kalıcı ortalamaya geri dönüyor.
  - Bantlar t03'te t02'den geniş (9.6–11.2 mm'ye karşı 3.4–7.1 mm). Sapmadaki yavaş kaymalar,
    örneğin 55–70 s arasındaki ±5 mm'lik dalgalanma, σ'yı büyütüyor.
  - Her geçiş türünden tek örnek var.
- **Model notu** [HESAP, AF ikinci mertebe varsayımı, B sabit]:
  - K = 20: ζ = 0.71, ω_n = 2.0 rad/s, %2 oturma ≈ 4/(ζω_n) ≈ 2.8 s.
  - K = 10: ζ = 1.0, ω_n = 1.41 rad/s, %2 oturma ≈ 5.8/ω_n ≈ 4.1 s.
  - Ölçülen değerler bu büyüklükte ya da gürültü/kayma ile örtülü.

## 6. Zamanlama ve döngü

- **Rampa çağrıları (n = 100):**
  - `set`: medyan 2.49, p95 8.18, maks. 11.7 ms;
  - `get`: medyan 1.53, p95 7.26 ms.
  - Hepsi 20 ms'lik döngünün içinde.
- **Döngü:** dt p99 29.9 ms.
- **Tek büyük gecikme:** `phynt.set_wo_zero` 2.52 s bloklandı (t = 20 s; t01 ve t02'de de aynı).
  K fazından önce kapandı.
- **Güvenlik:** Tetikleme yok.

## Sonuç

- Önceden commit edilen ölçütlere göre **Kapı A2 GEÇTİ.**
- **Ölçümün desteklediği:**
  - Rampa, uygulanan K'yı her döngüde doğru değere getiriyor.
  - Pozisyon referansını sıçratmıyor.
  - t02'deki çağrıyla senkron 0.06–0.14 N'luk fx adımını, çağrı başına yaklaşık 0.005 N'luk
    adımlara bölüyor; bu adımlar gürültünün altında.
  - Kalıcı F, iki koşuda da seri modelin %7'si içinde.
- **Ölçümün desteklemediği:**
  - "C3 geçti, öyleyse rampa düzgün" çıkarımı; bu koşuda gürültü sınıra çok yakın.
  - Rampa ile basamak arasında oturma süresi farkı.

## Karar bekleyenler (kalıcı değişiklik yapılmadı)

- **H1 — fx gürültüsü:** t03'te t02'ye göre daha yüksek. Kaynağı (döngü yükü, uçuştan uçuşa
  değişim) ayrı bir hover/temas tekrar koşusuyla ayrıştırılabilir.
- **H2 — gelecek Δfx kriterleri:** Sabit bir eşik yerine, aynı koşunun çağrısız pencerelerinden
  türetilen bir referansla karşılaştırma ya da çağrı başına adım ölçüsü kullanılabilir.
  Yeni bir deneyden önce ön kayda girmeli.
- **H3 — oturma için tekrar:** Her geçiş türü için birden fazla tekrar ya da daha uzun tutmalar
  gerekli.
- **H4 — WO–fx farkı:** Tutmadan tutmaya 0.0–0.1 N değişiyor. AF sapması WO'yu izlediği için
  sapma hedeflerini doğrudan etkiliyor.
