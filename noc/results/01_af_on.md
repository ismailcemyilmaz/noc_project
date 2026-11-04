# 01 — AF açık: rotor_measure düzeltmesi (F1) ve bloklamayan goto (F2)

- **Veri:**
  - `logs/noc/t01_af_on/` (t01 uçuşu, 2026-11-04 17:21, 3745 örnek)
  - `logs/noc/t00/` (t00, karşılaştırma)
  - `logs/noc/t01_check/` (uçuşsuz kontrol)
- **Script'ler:**
  - koşu: `noc/tests/t01_af_on.py`, setup: `noc/common/setup_06a.py`
  - analiz: `noc/analysis/a01_af_on.py`
- **Çıktılar:** `noc/results/01_af_on/metrics.json` (sha256 envanteri dahil), `compare.png`
- **Yeniden üretim:** `.venv/bin/python -m noc.analysis.a01_af_on`
- **Seed:** yok (analizde rastgelelik yok, simülatör seed'i kaydedilmiyor).
- **Tek fark:** t00'dan farkı yalnızca iki şey.
  - `uavatt.rotor_measure ← rotorcraft/rotor_measure` bağlantısı.
  - `maneuver.goto(..., ack=True)`.
  - 06a parametreleri, waypoint'ler ve kazançlar aynı; `src/06a` değişmedi.

**Etiketler:**
- **[LOG]**: loglanmış sinyal. Bazıları kendisi bir kestirimdir, örneğin WO.
- **[HESAP]**: offline türetilmiş değer.

**Bu raporda doğrudan temas kuvveti ölçümü yok.** Elimizdeki iki kuvvet de kestirim:
- WO `external_wrench` [LOG, kestirim],
- kuasi-statik `F = uavpos fx` [HESAP].

Bu ikisi bağımsız değil. WO, uavatt'ın rotor hızlarından hesapladığı wrench'i kullanıyor, uavpos fx ise aynı zincirin komutu. Uyuşmaları "komut edilen ≈ üretilen wrench" anlamına geliyor, duvar kuvvetinin doğrulaması değil.

---

## Özet

| | t00 (06a olduğu gibi) | t01 (F1 + F2) |
|---|---|---|
| `wrench_measure.force` geçerli [LOG, npz] | 0 % | **99.97 %** |
| `external_wrench.force` geçerli [LOG, npz] | 0 % | **100 %** |
| phynt.log veri satırı | 0 | **59 485** (log_info: total 64 113, miss 4 832 = %7.5) |
| Kalıcı temasta F = uavpos fx [HESAP] | 0.916 ± 0.146 N | **0.622 ± 0.130 N** |
| Kalıcı temasta WO f_ext,x [LOG] | yok | **−0.606 ± 0.115 N** |
| Kalıcı temasta AF sapması (uavpos xd − maneuver x) [LOG] | 0 mm | **−55.0 ± 12.8 mm** |
| e_x [LOG] | −180.4 mm | −128.8 mm |
| Gövde x [LOG] | 1.370 m | 1.366 m |
| Etkin sertlik F / d₀ [HESAP] | 5.08 N/m (≈ Kpxy) | **3.38 N/m** (seri model: 3.33) |
| Python döngüsü dt p99 / > 100 ms boşluk | 22.6 ms / 10 (goto bloğu 2–5 s) | 24.7 ms / 1 (`set_wo_zero` 2.6 s) |
| goto çağrı süresi | 2.0–5.1 s | 0.8–7 ms |
| Güvenlik tetiklemesi | yok | t = 74.88 s, **inişte yere temas** (§6) |

- **F1 çalışıyor:**
  - WO çıktı üretiyor,
  - AF referansı WO kuvvetiyle tutarlı biçimde geri çekiyor,
  - phynt.log doluyor.
- **F2 çalışıyor:** döngü hareket sırasında da örnekliyor.

Kalıcı temas pencereleri maneuver.log'a göre belirlendi: nominal statik, x_nom = 1.55, ilk 1 s atıldı. t01'de pencereler [27.2, 30.7], [36.8, 38.7], [44.7, 46.7], [52.8, 54.7], [60.8, 62.7] s. uavpos.log'dan 8510 satır, npz'den 564 örnek.

## 1. wrench_measure ve external_wrench

- t01'de `uavatt.wrench_measure.force` 3745 örneğin 1'i hariç geçerli. `phynt.external_wrench` örneklerin tamamında geçerli.
- t00'da ikisi de %0'dı (bkz. 00b_diagnosis Q1).
- Uçuşsuz kontrol, yerde rölantide (rotor 15.26): `wrench_measure.force` = (0.000, 0.000, 1.22) N, `external_wrench` ≈ 0.
- WO gürültüsü [LOG, phynt.log efx]:
  - serbest uçuşta (12–20 s) ort −0.012 N, std 0.095 N,
  - `set_wo_zero` sonrası hover'da (22.7–24.5 s) ort −0.049 N, std 0.070 N.

## 2. phynt.log

- 59 485 veri satırı, 22 sütun (`ts efx … az`). Satır uyuşmazlığı yok.
- `log_info` değerleri:

| snapshot | total | miss |
|---|---|---|
| after_start | 0 | 0 |
| contact_1 | 20 647 | 1 521 |
| contact_2 | 34 123 | 2 692 |
| before_stop | 64 113 | 4 832 |

- `/tmp/phynt.log` uçuş boyunca büyüyor: 75 B'tan 10.0 MB'a.
- 00b Q3 hipotezi destekleniyor: girdiler geçerli olunca log yazılıyor. Kesin nedensellik phynt kaynağına bakılmadan doğrulanamaz.
- Yeni gözlem: kayıtların **%7.5'i kaçırılıyor** (miss). phynt.log'dan yapılacak analizlerde örnekleme boşlukları olacak.

## 3. Kalıcı temasta F, AF sapması, uavpos fx

| [birim] | t00 | t01 |
|---|---|---|
| uavpos fx [N] (kuasi-statik F) | 0.916 ± 0.146 | 0.622 ± 0.130 |
| uav_input.thrust x, npz [N] [LOG] | — | 0.623 ± 0.131 |
| WO f_ext,x [N] [LOG] | — | −0.606 ± 0.115 |
| Kpxy·e_x [N] [HESAP] | −0.902 | −0.644 |
| Kvxy·e_vx [N] [HESAP] | −0.014 ± 0.141 | +0.022 ± 0.151 |
| AF sapması [mm] | 0 | −55.0 ± 12.8 |
| d₀ = x_nom − x_gövde [mm] | 180.4 | 183.8 |

Pencere bazında [LOG]:

| pencere [s] | AF sapma ort [mm] | son 0.5 s [mm] | WO efx [N] | efx/K_x [mm] | fx [N] |
|---|---|---|---|---|---|
| 27.2–30.7 | −43.4 | −58.9 | −0.634 | −63.4 | 0.635 |
| 36.8–38.7 | −50.9 | −51.1 | −0.581 | −58.1 | 0.687 |
| 44.7–46.7 | −69.5 | −65.7 | −0.589 | −58.9 | 0.592 |
| 52.8–54.7 | −56.7 | −54.6 | −0.525 | −52.5 | 0.604 |
| 60.8–62.7 | −63.7 | −65.4 | −0.678 | −67.8 | 0.584 |

**AF, K_x ve WO kuvvetiyle dinamik olarak tutarlı** [HESAP]:
- `5·d̈ + 14.14·ḋ + 10·d = efx(phynt.log)` modeli loglanan WO kuvvetiyle sürüldü.
- Temas boyunca loglanan sapmayı **RMS 2.1 mm farkla, korelasyon 0.988** ile yeniden üretiyor.
- Pencere ortalamalarında sim ile log arasındaki fark ≤ 4.8 mm.
- Statik `efx/K_x` ile pencere ortalaması arasındaki fark (ilk pencerede −63 mm'ye karşı −43 mm) AF'nin geçici rejiminden geliyor:
  - ω_n = √(K/m) = 1.41 rad/s, kritik sönüm, %2 oturma süresi ≈ 4.1 s,
  - kalıcı pencereler ise 1.9–3.5 s uzunluğunda.

## 4. Öngörü ile karşılaştırma

Seri model [HESAP]: `F = d₀ / (1/Kpxy + 1/K_af,x)`, `sapma = F / K_af,x`, Kpxy = 5, K_af,x = 10 → k_seri = 3.33 N/m. Duvar sertliği ihmal edildi; 00_baseline'a göre ≥ ~10⁴ N/m.

| | d₀ [m] | F öngörü [N] | sapma öngörü [mm] | F t01 [N] | sapma t01 [mm] |
|---|---|---|---|---|---|
| verilen d₀ | 0.19 | 0.633 | 63.3 | 0.622 (−0.011, −1.8 %) | 55.0 (−8.3 mm, −13 %) |
| ölçülen d₀ | 0.1838 | 0.613 | 61.3 | 0.622 (+0.009, +1.5 %) | 55.0 (−6.3 mm, −10 %) |

- **F öngörüyle ±2 % içinde uyuşuyor.**
- Sapma ortalaması öngörüden 6–8 mm küçük. Bunun nedeni §3'teki AF geçici rejimi: pencereler oturma süresinden kısa. Son 0.5 s ortalamaları −51…−66 mm aralığında, öngörüye yakın.
- Pencereden pencereye ±7 mm oynama, WO kuvvetinin pencereler arasında 0.53–0.68 N değişmesinden geliyor. Sürtünme ve duvar boyunca hareketten sonraki artık etkiler olabilir; doğrulanmadı.
- `compare.png` alt panelinde t ≈ 38.7 s ve 51.3 s'de görülen tek örneklik 0'a sıçramalar bir analiz artefaktı. maneuver.log hareket yokken yazılmıyor; yeni goto başladığı anda iki log arasındaki interpolasyon bu sıçramayı üretiyor. Kalıcı pencere ortalamalarına etkisi ihmal edilebilir: ~2000 satırda 1 satır.
- e_x bileşimi kontrolü: |e_x| = d₀ − |sapma| = 183.8 − 55.0 = 128.8 mm, loglanan değer 128.8 mm. ✓

## 5. Kv ve etkin sertlik: yorum

- **Ölçüm:**
  - Kalıcı temasta Kvxy·e_vx terimi ortalama +0.02 N; std 0.15 N, salınım kaynaklı.
  - Statik kuvvet neredeyse tamamen Kpxy·e_x terimiyle taşınıyor.
  - Etkin sertlik t00'da 5.08 N/m (≈ Kpxy, AF girdisiz), t01'de 3.38 N/m (≈ seri 3.33).
- **Yorum (model, tek Kv değeriyle; Kv taraması yapılmadı):**
  - Kuasi-statik dengede e_v → 0. Bu yüzden Kv statik sertliğe girmiyor: `k_eff = (1/Kpxy + 1/K_af)⁻¹ ≤ min(Kpxy, K_af)`.
  - Kv'yi artırmak **statik** etkin sertliği değiştirmez. Statik sertliğin üst sınırı AF kapalıyken Kpxy (5 N/m), AF açıkken seri değer (3.33 N/m). Bu iki değer verilerle uyumlu.
  - Kv, frekansa bağlı (dinamik) sertliği `|Kpxy + jω·Kvxy|` olarak ve sönümü artırır. Etkisini geçici rejimde ve temas anındaki darbede görmeyi beklerim.
  - Bu bir model çıkarımı. Doğrulamak için Kv taraması (Kpxy ve K_af sabit) ve kalıcı penceresi oturma süresinden uzun (≥ 5 s) bir senaryo gerekir.

## 6. vel alanlarının durumu [LOG, npz]

Faz tanımları:
- **hareket:** goto başlangıcı + süre (t01'de 2078 örnek; t00'da 0, goto blokladığı için),
- **statik:** geri kalan,
- **temas:** görev fazı,
- **kalıcı temas:** §3 pencereleri.

| kaynak | faz | t01 vel None oranı | t01 pos geçerli | t01 vel ≠ 0 |
|---|---|---|---|---|
| maneuver.desired | hareket | 0.05 % | 99.95 % | 99.6 % |
| | statik | **98.6 %** | 1.4 % | 1.4 % |
| | temas (görev) | 43.3 % | 56.7 % | 56.5 % |
| | kalıcı temas | **100 %** | 0 % | 0 % |
| phynt.get_reference | hareket | 0.05 % | 99.7 % | 99.6 % |
| | statik | 19.7 % | 1.5 % | 1.5 % |
| | kalıcı temas | 0 % (vel = 0) | **0 %** | 0 % |
| phynt.desired (AF çıktısı) | tüm fazlar | **0 %** | **100 %** | 100 % |

Sonuçlar:
- **E2 doğrulandı.**
  - maneuver, hareket sırasında pos/vel/acc yayımlıyor. Hareket bitince `desired` alanlarını boş bırakıyor: kalıcı temasta %100 None.
  - phynt'in referans kopyasında da pos None, vel = 0.
  - t00'daki "pos None" gözlemi bundan geliyordu. t00 ayrıca yalnızca statik fazlarda örnek alabilmişti.
- **phynt.desired, t01'de her fazda pos ve vel içeriyor.** AF aktifken çıktıyı kendi durumundan üretiyor. Bu yüzden boş referans AF'nin sapma üretmesini **engellemedi**. 00b'deki "engelleyebilir" endişesi bu koşuda gerçekleşmedi.
- t00'da phynt.desired statik fazda pos None'dı (pos geçerli 2.5 %). Bu fark AF'nin kuvvet girdisi alıp almamasıyla ilişkili görünüyor; mekanizma doğrulanmadı.

## 7. Güvenlik tetiklemesi (t = 74.88 s)

- Neden: `external_wrench |f| = 3.54 N > 3.5` (5 ardışık örnek).
- Durum: t01 son waypoint'te (inişte, nominal z = 0) gövde z = 0.237 m'de, yani yerdeydi.
  - Uçuş referansı zemin altında kaldığı için kontrolcü aracı yere bastırıyor, WO da bu yer tepkisini görüyor.
  - Benzer bir durum kalkıştan önce de var (t = 4 s'de ext_f,z = 3.50 N). Ama orada `set_wo_zero` öncesi olduğu için kontrol devre dışıydı.
- Eylem:
  - geri çekilme: x < 1 m olduğu için x kaydırılmadı, mevcut noktaya goto, 3.0 s,
  - iniş goto: 5.1 s,
  - ardından `stop()`.
  - Duvar temasıyla ilgisi yok. Analiz edilen pencereler (≤ 62.7 s) etkilenmedi; kayıt son ~5 s eksik.
- **Güvenlik mantığının yer temasını dışlaması gerekiyor.** Önerim: WO kuvvet kontrolü yalnızca z > 0.5 m iken aktif olsun, ya da yalnızca duvar temas fazında (aktif waypoint x = 1.55).

---

## Kalıcı hale getirme seçenekleri (UYGULANMADI, onayını bekliyor)

1. **P1:** `noc/common/setup_06a.py`'yi (rotor_measure bağlantısı) bundan sonraki tüm noc deneylerinin varsayılan setup'ı yapmak. `src/06a` dokunulmadan kalır.
2. **P2:** `ack=True` goto'yu noc deney akışlarında varsayılan yapmak.
3. **P3:** Güvenlik monitörünü düzeltmek (§7): WO kuvvet kontrolü yalnızca z > 0.5 m'de ya da duvar fazında aktif olsun.
4. **P4:** t01'i yeni referans koşu ("AF-on baseline") olarak kabul etmek. `noc/common/params.py` veya bir config dosyasına Kpxy, Kvxy ve K_af gibi değerleri bu koşunun kaynağıyla yazmak.
5. İsteğe bağlı: Kv taraması ve uzun kalıcı pencereli senaryo (§5).
