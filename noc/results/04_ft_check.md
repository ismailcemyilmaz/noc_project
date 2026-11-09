# 04 — Gazebo F/T sensörü ile temas kuvveti (t04) ve Kapı B1

- **Veri:**
  - `logs/noc/t04_ft_check/` (t04 uçuşu, 2026-11-09 13:33, 27868 F/T mesajı, abort/hata yok)
  - `logs/noc/t04_check/` (uçuşsuz `--check`)
- **Script'ler:** koşu `noc/tests/t04_ft_check.py` (git rev `ed28b44`), analiz `noc/analysis/a04_ft_check.py`,
  ek analiz `noc/analysis/a04b_ft_supplement.py` (uçuştan sonra yazıldı, kapı dışı).
- **Ölçütler:** `noc/config/t04_ft_criteria.json`, commit `18a4436` (2026-11-04), uçuştan önce.
  sha256 `5f50e0a5b70e…`. Uçuştan sonra dosyada ve `a04_ft_check.py` içinde değişiklik yok.
- **Çıktılar:** `noc/results/04_ft_check/` altında `metrics.json`, `summary.md`, `ft_check.png`, `supplement.json`.
- **Yeniden üretim:** `.venv/bin/python -m noc.analysis.a04_ft_check` ve
  `.venv/bin/python -m noc.analysis.a04b_ft_supplement logs/noc/t04_ft_check noc/results/04_ft_check/supplement.json`
- **Seed:** koşuda RNG yok; simülatör seed'i kaydedilmiyor. Ek analizdeki bootstrap seed'i 0.

**Etiketler:** **[SİM]** Gazebo force_torque sensörü (`ee_joint` kısıt kuvveti). **[LOG]** loglanmış sinyal
(WO `efx` ve uavpos `fx`; ikisi de kontrol zincirinden gelen kestirim veya komut). **[HESAP]** offline türetilmiş değer.
**[EK]** uçuştan sonra seçilmiş yöntem; kapıya girmez.

F/T, duvardaki temas kuvveti değil, `ee_joint` üzerinden iletilen kuvvettir. Aradaki fark m_ee·(a − g) ve sürtünme terimleridir.

---

## Kapı B1 sonucu: **GEÇTİ**

| Ölçüt (önceden commit edildi) | Sonuç | Değer |
|---|---|---|
| C1 kayıt ve okuma zinciri hatasız | ✅ | 27868 mesaj, 0 hatalı, logger durdurulana kadar canlı, abort yok, okuma hatası 0 |
| C2 damgalar sürekli, > 100 Hz | ✅ | maks. boşluk 2.0 ms (≤ 50 ms), 500.0 Hz (damga), 339.8 Hz (host), RTF 0.680 |
| C3 bias serbest uçuşta ölçüldü | ✅ | n = 2253 (≥ 500), bias −0.0003 N, std 0.0371 N, uç yüzeyi maks. x 1.876 m (≤ 1.95) |
| C4 temas kuvveti > 5·std | ✅ | Fn − bias = −0.5741 N = 15.5·std |
| C5 öncesi/sonrası tutarlı | ✅ | Δön −0.5741, Δson −0.5749, oran 1.00 ([1/3, 3] içinde) |
| C6 güvenlik ve K, t02/t03 ile aynı | ✅ | safety v2, aynı; AF 3800/3800 döngü eşleşti; `set_af_parameters` çağrısı 0 |

Beklenen işaret tuttu: Fn − bias < 0. Yerde `--check` [SİM]: |F| = 0.0982 N, m_ee·g = 0.0981 N (+%0.1), 3141 mesaj, 0 hatalı.

**Kapı sonucu aşağıdaki iki bulgudan etkilenmiyor** (§1 ve §2). Kısa zarfla C4 = 16.8·std, C5 oranı 1.00.
Blok bootstrap ile %95 GA sıfırı içermiyor.

---

## 1. Bulgu: Temas kuvveti impulsif

CONTACT penceresinde F/T [SİM], sürekli bir kuvvet değil, darbeler dizisi:

| | Değer (taahhüt edilen zaman eşlemesi) |
|---|---|
| ortalama − bias | **−0.5741 N** |
| medyan − bias | **−0.0081 N** |
| std | 2.83 N (FREE1: 0.037 N) |
| \|Fn − bias\| < 0.05 N olan örnek | %69.8 |
| Fn − bias < −1 N olan örnek | %8.1 |
| CONTACT içinde en küçük değer | −50.2 N |
| tüm uçuşta en küçük değer (ilk çarpma, temas goto + 1.18 s) | −349.7 N |

- Sonuç: Ortalama kuvvet makul, ama anlık F/T kuvveti sabit bir temas kuvveti gibi yorumlanamaz. Uç duvara çarpıp ayrılıyor
  (veya motor kısıtı aralıklı çözüyor). Hangisi olduğu bu veriyle ayrılamıyor.
- Fizik adımı 2 ms, sensör 500 Hz. Yani her adım örnekleniyor ve darbe atlanmıyor.
- 2 s'lik alt pencere ortalamaları kararlı değil: −0.757, −0.498, −0.578, −0.632, −0.406 N.
- Ortalamanın %95 GA'sı [EK, dairesel blok bootstrap, blok 0.5 s]: **[−0.658, −0.496] N**, yani yaklaşık ±%14.
  Bu çözünürlüğün altındaki model farkları bu koşuyla ayırt edilemez.

## 2. Bulgu: Taahhüt edilen zaman eşlemesi F/T'yi ~0.5 s erken yerleştiriyor

Ölçüt, F/T damgasını wall-clock'a (host − damga) değerinin 2 s'lik yuvarlanan alt zarfıyla eşliyor. RTF ≈ 0.64 olduğunda
(host − damga) sürekli artıyor. Ortalanmış pencerenin minimumu da pencerenin başındaki değeri veriyor. Böylece F/T örnekleri
olduğundan erken eşleniyor.

| | host − eşlenen t (medyan) | F/T ilk temas − pom temas | F/T son temas − pom ayrılma |
|---|---|---|---|
| taahhüt edilen 2 s zarf | **534 ms** | **−0.59 s** | **−0.41 s** |
| kısa zarf 0.1 s [EK] | 25 ms | −0.016 s | +0.064 s |

Kenar kontrolü bağımsızdır: pom uç yüzeyi (temas seviyesi − 2 mm) ile F/T'nin ilk ve son |Fn − bias| > 1 N darbesi karşılaştırıldı.
**Kısa zarf pom kenarlarıyla uyumlu; taahhüt edilen zarf değil.** Ölçütler değiştirilmedi. Kapı yukarıdaki gibi taahhüt edilen tanımla karar verildi;
aşağıda iki eşleme de veriliyor.

Bunun etkisi:
- FREE ve CONTACT pencereleri F/T'de yaklaşık 0.5 s kayıyor. CONTACT ortalaması −0.574 N'dan −0.616 N'a değişiyor (%7).
- Çapraz korelasyon gecikmesi aynı miktarda kayıyor (§3.2).

## 3. F/T ile WO efx ve uavpos fx karşılaştırması (CONTACT)

Karşılaştırılan büyüklükler: −uavpos fx (işareti F/T ile aynı olsun diye) ve WO efx. Fark = kestirim − F/T.

### 3.1 Bias (CONTACT ortalaması, gecikme kaydırması yok)

| Sinyal | Kaynak | Ortalama [N] | Fark, taahhüt edilen eşleme [N] | Fark, kısa zarf [EK] [N] |
|---|---|---|---|---|
| F/T Fn − bias | [SİM] | −0.5741 / −0.6156 (kısa zarf) | – | – |
| WO efx | [LOG, kestirim] | −0.6496 | **−0.075** (%95 GA −0.153, +0.009) | **−0.034** (−0.109, +0.047) |
| −uavpos fx | [LOG, komut] | −0.6081 | **−0.034** (−0.112, +0.050) | **+0.008** (−0.068, +0.089) |

- Dört durumda da GA sıfırı içeriyor. **Bu koşu ile bias, ~±0.08 N içinde sıfırdan ayırt edilemiyor.**
- a04'ün ön kayıtlı tablosu (lag kaydırmalı, ham 100 Hz, ortalama hata): WO −0.0386, −fx −0.0083 N.
  Bu değerler 0.89 s ve −0.40 s kaydırma içerdiğinden yukarıdaki doğrudan farklarla aynı değil.

### 3.2 Gecikme

Ön kayıtlı yöntem (a04, ham, 100 Hz, ±1 s; pozitif = kestirim F/T'den geride):

| Kestirim | gecikme | korelasyon |
|---|---|---|
| WO efx | +0.89 s | **0.085** |
| −uavpos fx | −0.40 s | **0.125** |

**Bu iki değer gecikme olarak kullanılamaz.** Korelasyon yaklaşık 0.1; ham F/T darbeleri çapraz korelasyonu bastırıyor ve zaman eşlemesi 0.5 s
kaydırılmış. [EK] F/T ve kestirimlere aynı simetrik hareketli ortalama uygulanırsa (10 ms bin ortalaması, ardından MA):

| MA genişliği | Eşleme | WO gecikme (korr.) | −fx gecikme (korr.) |
|---|---|---|---|
| 0.2 s | taahhüt edilen | +1.00 (0.26) | −0.25 (0.54) |
| 0.5 s | taahhüt edilen | +1.00 (0.35) | −0.21 (0.65) |
| 1.0 s | taahhüt edilen | +1.00 (0.43) | −0.08 (0.74) |
| 0.2 s | kısa zarf | +1.00 (0.22) | −0.82 (0.48) |
| 0.5 s | kısa zarf | +1.00 (0.32) | −0.77 (0.59) |
| 1.0 s | kısa zarf | +1.00 (0.43) | −0.60 (0.70) |

**Sonuç: bu koşudan güvenilir bir gecikme çıkmıyor.**
- WO: tepe her durumda arama sınırında (+1 s); gerçek bir tepe değil.
- −fx: Değer filtreye (−0.60…−0.82) ve eşlemeye bağlı. Negatif gecikme, gerçek bir öncü sinyal olarak okunmamalı. Olası açıklama
  (doğrulanmadı): fx temastan önceki goto ivmesini de içeriyor, korelasyon temas kenarı dışındaki hareketten geliyor olabilir.
- Temas kenarlarından %50 eşik yöntemi de denendi ve atıldı: F/T aralıklı olduğu için eşik pencere sonuna kadar yeniden geçiliyor.

### 3.3 RMSE

Ön kayıtlı (a04, ham F/T, lag kaydırmalı, taahhüt edilen eşleme):

| Kestirim | RMSE span | RMSE span (kaydırmasız) | RMSE CONTACT |
|---|---|---|---|
| WO efx | 2.177 N | 2.190 N | 2.190 N |
| −uavpos fx | 2.172 N | 2.195 N | 2.196 N |

Bu değerler kestirim hatası değil, F/T darbelerinin genliğidir (F/T std ≈ 2.8 N). Span ilk çarpmayı (−350 N) da içeriyor.

[EK] Aynı MA uygulanmış, kaydırmasız, CONTACT penceresi, kısa zarf:

| MA genişliği | WO efx RMSE | −fx RMSE |
|---|---|---|
| 0.2 s | 0.400 N | 0.392 N |
| 0.5 s | 0.250 N | 0.218 N |
| 1.0 s | 0.193 N | 0.131 N |

RMSE MA genişledikçe düşüyor; bu, farkın önemli bir kısmının yüksek frekanslı (F/T darbeleri dahil) bileşenden geldiğiyle uyumlu.
Bu değerler kestirim hatasının kendisi değil, F/T darbe artığını da içeren bir üst sınırdır. Taahhüt edilen eşlemede 1.0 s: WO 0.192 N, −fx 0.143 N. Span RMSE'si tek bir −350 N darbeye ve eşlemeye duyarlı (0.5 s MA: 0.52 ile 0.69 N); bu yüzden
span kıyaslamasına güvenilmemeli.

## 4. Seri yay modeli ile F/T karşılaştırması

Model [HESAP]: F = d₀ / (1/Kpxy + 1/K_x), Kpxy = 5, K_x = 10 (`af_on_baseline.json`). Bu, t02/t03'tekiyle aynı model.
- d₀ = 0.184 m (t02/t03 değeri): F = **0.6133 N**
- d₀ ölçülen [LOG, maneuver x − pom x, CONTACT]: 0.18429 ± 0.00187 m: F = **0.6143 N**

| F/T zaman eşlemesi | F/T \|Fn − bias\| [SİM] | %95 GA | Model − F/T (d₀ = 0.184) | Model − F/T (d₀ ölçülen) |
|---|---|---|---|---|
| taahhüt edilen 2 s | 0.5741 N | 0.496–0.658 | **+0.0392 N (+%6.8)** | +0.0402 N (+%7.0) |
| kısa zarf 0.1 s [EK] | 0.6156 N | 0.540–0.697 | **−0.0022 N (−%0.4)** | −0.0013 N (−%0.2) |

Seri yay modeli her iki eşlemede de F/T'nin %95 GA'sı içinde. Ama model hatası bu koşunun çözünürlüğünden (yaklaşık ±%13–14) daha iyi söylenemez
ve eşlemeye göre %0 ile %7 arasında değişiyor. Kısa zarftaki %0.4'lük fark, GA genişliği düşünüldüğünde rastlantısal olarak küçük olabilir; modelin %0.4 doğrulukta
olduğu anlamına gelmez.

## 5. Varsayımlar ve sınırlar

- Component log'larının `ts` değerleri wall-clock kabul edildi. Kenar kontrolü bunu pom için destekliyor; phynt/uavpos için doğrudan test edilmedi.
- Uçuş zamanlaması wall-clock'ta; RTF 0.64 yüzünden CONTACT penceresi sim zamanında yaklaşık 6.4 s. Pencerenin ilk 2 s'si oturmamış olabilir (ilk alt pencere ortalaması en büyük).
- `ee` kütlesi 0.01 kg eklendi: toplam 2.73 kg (06a/uavpos 2.72 kullanıyor). Tek koşu; tekrar yok.
- Bu rapor t02, t03 sonuçlarını yeniden değerlendirmiyor.
- [EK] seçimleri (0.1 s zarf, 0.2/0.5/1.0 s MA, 0.5 s bootstrap bloğu) veriyi gördükten sonra yapıldı.

## 6. Öneri (uygulanmadı)

- Zaman eşlemesi tanımı ve F/T gecikme ölçümü için yeni bir ön kayıt: kısa veya yalnızca ileri bakan zarf; gecikme için temas darbesi ile sürekli bir kuvvet kenarı içeren bir senaryo.
- Temastaki impulsif davranışın nedeni (duvar/kısıt parametreleri, 2 ms adım) ayrı bir inceleme gerektiriyor. Bu sonuç F/T'yi anlık kuvvet hedefi olarak kullanmadan önce ele alınmalı.
