# 02 — Uçuşta canlı K_x değişimi (t02) ve Kapı A

- **Veri:**
  - `logs/noc/t02_live_k/` (t02 uçuşu, 2026-11-04 22:11, 6100 örnek @ 50 Hz, abort/hata yok)
  - `logs/noc/t02_check/` (uçuşsuz `--check` koşusu, 22:09; konsol çıktısı `logs/noc/t02_live_k/console_check.log`)
- **Script'ler:** koşu `noc/tests/t02_live_k.py` (git rev `921cf13`), analiz `noc/analysis/a02_live_k.py`
- **Çıktılar:** `noc/results/02_live_k/metrics.json` (sha256 envanteri dahil), `live_k.png`
- **Yeniden üretim:** `.venv/bin/python -m noc.analysis.a02_live_k`
- **Config:** `noc/config/af_on_baseline.json`; kalkıştan önce okunan AF parametreleri birebir eşleşti
  (mass 5, K = [10, 100, 100, 50, 50, 50], B = 2√(M·K) K_x = 10 için, J).
- **Seed:** yok (koşuda RNG yok; simülatör seed'i kaydedilmiyor).
- **Senaryo:** 06a temas noktası (1.55, 0, 1). t = 30 s'den itibaren 12 s'lik tutmalar:
  K_x = 10 → 12 → 15 → 20 → 15 → 10. Ardından değeri değişmeyen 200 `set_af_parameters` çağrısı
  (döngü başına bir). Yalnızca K[0] değişiyor; **B sabit kalıyor** (14.14 N·s/m).

**Etiketler:** **[LOG]** loglanmış sinyal (bazıları kendisi kestirim), **[HESAP]** offline türetilmiş değer.
**Bu raporda temas kuvveti ölçümü yok.** `F = uavpos fx` [HESAP, kuasi-statik] ve WO `efx` [LOG, kestirim]
aynı zincirden geliyor; bağımsız değiller.

---

## Kapı A sonucu: **KALDI**

| Kriter | Sonuç | Değer |
|---|---|---|
| Koşu tamamlandı (abort/hata yok) | ✅ | `abort = null`, `error = null`, okuma hatası 0 |
| Tekrar `set_af_parameters` p95 < 20 ms | ✅ | p95 **6.01 ms**, ort. 2.26, maks. 7.85 ms (n = 200) |
| Uygulanan K doğrulandı | ✅ | 205/205 çağrı geri okundu; 6100/6100 döngüde `af.K_x == hedef` |
| Her pencerede F, öngörünün ±%15'i içinde | ✅ | hata −6.5 % … −0.9 % |
| Servo kesintisiz **ve sıçrama yok** | ❌ | kesinti yok; **4/5 K değişiminde "jump" bayrağı** |

Kapıyı düşüren tek kriter "jump". Kesinti yok: çağrı pencerelerinde uavpos.log satır aralığı en fazla
13.7 ms, eşik 100 ms. §2'de gösterildiği gibi bu sıçrama bir servo arızası değil. K'nın basamak
şeklinde değiştirilmesinin AF ivme çıkışında yarattığı, nicel olarak öngörülen bir süreksizlik.
Kriter bunu ayırt edecek biçimde tanımlanmadığı için sonuç tanım gereği **KALDI**. Kriteri geriye
dönük olarak değiştirmedim (bkz. "Karar bekleyenler").

---

## 1. Preflight'taki `external_wrench` hatası (S_posterLib_EMPTY_POSTER)

`phynt/external_wrench port: S_posterLib_EMPTY_POSTER` hatası **uçuş koşusunda değil, `--check`
koşusunda** (22:09, yerde, `start()`'tan 1.0 s sonra) görüldü. Uçuşun kendi preflight'ı
(`preflight.json`, `start()`'ın hemen ardından) **9/9 OK** döndü.

Uçuş verisi [LOG, npz]:
- `ext_wrench.valid.force`: **6100/6100** döngü geçerli; okuma hatası 0.
- t = 0–1.99 s (100 döngü): `external_wrench.ts` ve `phynt_des.ts` **donuk**, ikisi de start anındaki
  aynı değerde (1793826664.2376). Bu süre boyunca yaş en fazla 2.53 s.
- phynt.log'un ilk veri satırı t = 2.008 s'de, yani ilk `maneuver.goto` (t = 2.00) anında.
- t ≥ 2.0 s: `external_wrench` yaşı en fazla **4 ms** (`t_wall − ts`). Uçuşun geri kalanında güncel.

**Yorum:**
- phynt'in çıkış portları, ilk geçerli referans gelene kadar güncellenmiyor. 06a/CLAUDE.md'deki
  "idle'da maneuver.desired alanları None" gözlemiyle tutarlı.
- Referanstan önceki durum koşudan koşuya farklı:
  - t02_check: poster boş (EMPTY_POSTER), phynt_des alanları geçersiz, phynt.log yalnızca başlık (75 B);
  - t01_check: poster var ama alanlar None;
  - t02 uçuşu: start anından kalma tek bir değer.
- **Sonuç:** Bu bir başlangıç durumu. Uçuşta ilk goto'dan sonra ext_wrench hiç kesilmedi.
- **Doğrulanmayan varsayım:** Check koşusunda posterin neden hiç yazılmadığı (uçuşta ise bir kez
  yazılmış olması) bu verilerle açıklanamıyor. Neden bilinmiyor, ama uçuşu etkilemiyor.

---

## 2. Çağrı anındaki fx sıçraması: kaynak

Analizin "jump" tanımı: çağrının ±1 s penceresinde, satır başına maks. |Δfx|, referans penceresindeki
değerin (son K = 10 tutması, 0.0325 N) **3 katını** (0.098 N) aşarsa bayrak.

| K değişimi | maks. \|Δfx\| [LOG] | çağrıya göre zaman | Δaxd [LOG] | m·Δaxd [HESAP] | öngörü m·ΔK·\|dev\|/M_af [HESAP] | bayrak |
|---|---|---|---|---|---|---|
| 10 → 12 | 0.064 N | 0.000 s | +0.022 m/s² | 0.060 N | 0.065 N | – |
| 12 → 15 | 0.103 N | −0.001 s | +0.038 m/s² | 0.103 N | 0.087 N | JUMP |
| 15 → 20 | 0.143 N | +0.001 s | +0.053 m/s² | 0.143 N | 0.120 N | JUMP |
| 20 → 15 | 0.098 N | 0.000 s | −0.035 m/s² | 0.096 N | 0.113 N | JUMP |
| 15 → 10 | 0.133 N | 0.000 s | −0.050 m/s² | 0.135 N | 0.141 N | JUMP |

(m = 2.72 kg, M_af = 5 kg; |dev| = önceki pencerenin kalıcı AF sapması.)

- **Sıçrama çağrıyla senkron ve gerçek.** Beş pencerenin hepsinde maks. |Δfx| çağrı anında (±1 ms)
  oluşuyor. Çağrı içermeyen 20 adet 2 s'lik pencerede (tutmaların içinden) maks. |Δfx| en fazla
  **0.064 N** (medyan 0.036). Bu pencerelerin hiçbiri eşiği aşmıyor.
- **Mekanizma** [HESAP, model varsayımı: AF `M ẍ + B ẋ + K x = f_ext`]:
  - K basamak şeklinde değişince AF durumu (x, ẋ) sürekli kalır, ama ivme çıkışı
    ΔK·dev/M_af kadar sıçrar.
  - uavpos bu ivmeyi `axd` ileri beslemesi olarak alır, fx = m·axd + …
  - Ölçülen Δfx, m·Δaxd'ye üç basamakta eşit. Öngörü ile fark ±%20.
- **Pozisyon referansı sıçramıyor:** maks. |Δxd| 6·10⁻⁵ m, referans penceresinde 4·10⁻⁵ m.
- **Büyüklük:** Sıçrama tek satırlık, en fazla 0.143 N. Bu, o andaki F ≈ 0.7 N'un yaklaşık %20'si.
- **Tekrar bloğu:** 200 aynı değerli çağrıda (ΔK = 0) maks. |Δfx| 0.034 N, bayrak yok. Bu da
  sıçramanın çağrının kendisinden değil, K değişiminden kaynaklandığını gösteriyor.

---

## 3. K penceresi başına kalıcı durum (her tutmanın son 6 s'si)

| K_x | F = fx [HESAP] | öngörü F (d₀ = 0.184) | hata | \|WO efx\| [LOG] | AF sapması [LOG] | öngörü sapma | K·\|dev\| [HESAP] | d₀ ölçülen [LOG] | k_eff = F/d₀ [HESAP] | k_seri | ζ (B sabit) [HESAP] |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 10 | 0.587 N | 0.613 | −4.3 % | 0.591 | −59.4 mm | −61.3 | 0.594 | 0.1832 m | 3.20 | 3.33 | 1.00 |
| 12 | 0.636 N | 0.649 | −2.1 % | 0.682 | −53.1 mm | −54.1 | 0.637 | 0.1829 m | 3.48 | 3.53 | 0.91 |
| 15 | 0.684 N | 0.690 | −0.9 % | 0.694 | −44.1 mm | −46.0 | 0.661 | 0.1837 m | 3.72 | 3.75 | 0.82 |
| 20 | 0.725 N | 0.736 | −1.5 % | 0.823 | −41.7 mm | −36.8 | 0.834 | 0.1841 m | 3.94 | 4.00 | 0.71 |
| 15 | 0.662 N | 0.690 | −4.1 % | 0.773 | −51.7 mm | −46.0 | 0.776 | 0.1815 m | 3.65 | 3.75 | 0.82 |
| 10 | 0.573 N | 0.613 | −6.5 % | 0.620 | −63.6 mm | −61.3 | 0.636 | 0.1825 m | 3.14 | 3.33 | 1.00 |

Öngörü (seri yay modeli) [HESAP]: F = d₀ / (1/Kpxy + 1/K_x), sapma = F/K_x, Kpxy = 5.

- **F, seri modeli izliyor.** K_x 10 → 20 iken F 0.587 → 0.725 N (+%24); öngörü +%20.
  k_eff her pencerede k_seri'nin %94–99'u.
- **AF sapması F'yi değil, WO kestirimini izliyor.** K·|dev| ile |WO efx| arasındaki fark en fazla
  0.045 N (K = 12). Seri model ise fx üzerinden kuruldu.
- **WO ile fx arasındaki fark büyüyor.** Fark K = 20 ve ikinci K = 15 penceresinde 0.10–0.11 N'a
  çıkıyor. Bu yüzden sapma bu iki pencerede öngörüden 4.9 ve 5.7 mm fazla.
- **Histerezis:** Aynı K için sapma yukarı ve aşağı yönde farklı (K = 15: −44.1 / −51.7 mm;
  K = 10: −59.4 / −63.6 mm). Yukarıdaki WO–fx farkı bunu açıklıyor. İkisi de kestirim olduğundan
  hangisinin gerçek temas kuvvetine yakın olduğu bu veriyle söylenemez.
- **d₀ sabit:** 0.1815–0.1841 m arasında; t01'deki 0.184 değeriyle tutarlı.

## 4. Oturma süresi: belirlenemedi

- `a02` oturma süresini yalnızca ilk pencere için verdi: 8.44 s. Bu süre temas goto'sunun sonundan
  ölçüldü; bant ±5.9 mm.
- Diğer beş pencere için `None` döndü. Bant max(%10·basamak, 2 mm) = 2 mm.
- Bunun nedeni gürültü. 0.5 s hareketli ortalamadan sonra bile sapmanın kalıcı kısımdaki std'si
  1.7–3.5 mm. Ayrıca ±1.3 mm/s'ye varan yavaş kaymalar var (grafikte görülüyor). Basamaklar ise
  yalnızca 2–12 mm.
- 5 mm'lik bantla sonuç: K → 15 (aşağı) 1.9 s, K → 10 (aşağı) 1.8 s; diğerleri yine
  belirlenemedi.
- **"Oturmadı" denemez; mevcut çözünürlükle ölçülemiyor.**
- Model notu [HESAP]: B sabit tutulduğu için ζ, K = 20'de 0.71'e düşüyor.

## 5. Zamanlama ve döngü

- **Planlı 5 değişim:** `set` 1.08–5.08 ms, `get` 0.60–3.65 ms [LOG, perf_counter].
- **Tekrarlar (n = 200):**
  - `set`: medyan 1.69, p95 6.01, maks. 7.85 ms;
  - `get`: medyan 0.83, p95 4.01 ms.
- **Döngü dt p99:** 25.3 ms.
- **Tek büyük gecikme:** `phynt.set_wo_zero` 2.55 s bloklandı (t = 20 s; t01'deki gibi). Bu yüzden
  temas goto'su t = 22.55 s'de (wall) gönderildi. Durum K penceresinden (t ≥ 30 s) önce kapandı.
- **phynt.log:** 90 472 satır. K pencerelerinde maks. boşluk 84 ms (t = 30.35 ve 94.3 s; ikisi de
  çağrı anı değil).

## Karar bekleyenler (kalıcı değişiklik yapılmadı)

- **G1 — jump kriteri:** Δfx'ten öngörülen m·ΔK·dev/M_af terimi çıkarılsın ya da pozisyon/hız
  referansının (xd, vxd) sürekliliği ayrı kontrol edilsin. Kapı A, t02 verisiyle yeniden
  değerlendirilir. Mevcut veriyle sonuç büyük olasılıkla GEÇTİ olur, ama bu kriter bu veriyi
  gördükten sonra seçilmiş olur. Bu yüzden ayrı bir kararla kaydedilmeli.
- **G2 — K rampası:** K'yı basamak yerine rampa ile değiştirmek ivme süreksizliğini ortadan kaldırır.
  Yeni bir koşu gerektirir.
- **G3 — oturma ölçütü:** Daha uzun tutmalar (≥ 20 s) ya da gürültüden türetilen bir bant (örn. 2σ).
- **G4 — preflight:** `external_wrench` ilk goto'ya kadar "beklemede" kabul edilsin, ya da kontrol
  ilk referanstan sonra yapılsın.
- **G5 — B ölçeklemesi:** K değişirken B = 2√(M·K) de güncellensin mi (ζ = 1 korunur)?
  Kontrol tasarımına ait bir karar.
