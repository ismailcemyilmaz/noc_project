# 00b — Teşhis: WO, AF, phynt.log, Kpxy·e_x (t00_diag koşusu)

- **Veri:** `logs/noc/t00/` (t00_diag, 2026-11-04 16:45, 4000 örnek), `logs/noc/t00_check/` (uçuşsuz kontrol)
- **Script'ler:** koşu `noc/tests/t00_diag.py`, analiz `noc/analysis/a00_diag.py`
- **Çıktılar:** `noc/results/00b_diagnosis/metrics.json`, `diag.png`, `io_timing.png`
- **Yeniden üretim:** `.venv/bin/python -m noc.analysis.a00_diag`. Dosyaların sha256 değerleri `metrics.json → inventory` altında.
- **Seed:** analizde rastgelelik yok; simülatör için kaydedilmiş bir seed yok.

**Etiketler:**
- **[LOG]**: ya genomix ile Python'dan okunan port değeri (npz), ya da bir component logu.
- **[HESAP]**: loglardan offline türetilmiş değer.

Bu raporda **temas kuvveti ölçümü yok**. WO çıktısı da yok (Q1). Kuvvet olarak verilen her değer [HESAP].

Zaman: `t = ts − ts₀`, burada ts₀ pom.log'un ilk örneğidir.

---

## Özet

| Soru | Cevap |
|---|---|
| Q1 WO temas kuvvetini görüyor mu? | **Hayır, WO hiç çıktı üretmiyor.** `phynt/external_wrench` 4000 okumanın hiçbirinde yazılmamış: ts = 0, force = None. WO'nun girdisi `uavatt/wrench_measure` her döngüde zaman damgası alıyor (3975 farklı ts), ama `force` ve `torque` **hep None**. Muhtemel kök neden: 06a `setup()` içinde **`uavatt` → `rotor_measure` giriş portu hiç bağlanmamış**. |
| Q2 AF sapması K_x ve f_ext ile tutarlı mı? | AF parametreleri doğru yüklenmiş (K = [10, 100, 100, 50, 50, 50]). Ama sapma **tam 0**: kararlı temasta std 0, tüm uçuşta \|·\| ≤ 5.3 mm. f_ext yaklaşık −0.92 N [HESAP] olsaydı beklenen sapma \|f\|/K_x ≈ **92 mm** olurdu. Sapma sıfır, çünkü AF'ye giden kuvvet girdisi (WO) yok. Q1 ile tutarlı. |
| Q3 phynt.log neden boş? | `log()` çağrısı başarılı, başlık yazılıyor (75 bayt). Ama `log_info` dört snapshot'ın hepsinde **total = 0, miss = 0**. Yani phynt hiç log kaydı üretmiyor; kayıt üretilip kaybolma ya da flush/kopyalama sorunu değil. Hipotez (doğrulanmadı): main task, girdiler geçersiz olduğu için log yazan hesap adımına hiç girmiyor. |
| Q4 Temas kuvveti Kpxy·e_x ile açıklanıyor mu? | **Evet.** Kararlı temasta uavpos komutu tam olarak `fx = −5·e_x − 7·e_vx` (fit a = −5.000, b = −7.000, residual 2·10⁻⁶ N, n = 8570). Kpxy·\|e_x\| = 0.90 N, f_cmd,x = 0.92 ± 0.15 N. Kazançlar kütleyle çarpılmıyor: m·Kpxy·\|e_x\| = 2.45 N verirdi, uymuyor. |

Ek bulgular (aşağıda ayrıntılı):
- **(E1)** `maneuver.goto` bloklayıcı.
- **(E2)** Referansın `pos` alanı statik fazlarda None.
- **(E3)** Eski npz'deki sıfırların açıklaması.

---

## Q1 — Wrench observer

| Kaynak [LOG, npz] | ts geçerli | force geçerli | farklı ts sayısı |
|---|---|---|---|
| `phynt.external_wrench` (çıkış) | 0 / 4000 | 0 / 4000 | 0 |
| `uavatt.wrench_measure` (çıkış; 06a'da phynt/wrench_measure girişine bağlı) | 4000 / 4000 | **0 / 4000** | 3975 |

- `get_wo_parameters` dört snapshot'ta da aynı: K = [1, 1, 1, 0, 0, 0], fc = [20, 20, 20, 1, 1, 1], **bias = 0**.
  - Bias, t ≈ 20.7 s'deki `set_wo_zero` çağrısından (2.45 s blokladı) sonra da 0. WO hesap yapmıyorsa bu beklenen bir sonuç.
- uavatt dokümanı (`noc/docs/genom/uavatt/README.adoc`) şöyle diyor:
  - `wrench_measure`: "the current wrench exerted by the UAV according to the **propeller measurements** and the geometric model".
  - main task `rotor_measure` (in, `or_rotorcraft::output`) portunu okuyor.
- 06a `setup()` uavatt için yalnızca `state` ve `uav_input` portlarını bağlıyor. **`rotor_measure` bağlı değil.**
  - Lab'ın diğer script'leri aynı tipteki portu `rotorcraft/rotor_measure`'a bağlıyor: `src/01-intro/tk3-hexarotor-fa-control.py:70`, `src/04-state-estimation/state_estimation.py:125`. (Bu iki dosya physical-interaction sadeleştirmesinde ağaçtan kaldırıldı: `git show b39a541:<yol>`.)
- Çıkarım zinciri (son halkası doğrulanmadı):
  1. rotor_measure bağlı değil,
  2. → uavatt wrench_measure.force = None,
  3. → phynt WO'nun girdisi yok,
  4. → external_wrench yazılmıyor,
  5. → AF kuvvet görmüyor.
- Karşılaştırma için [HESAP]: kontrolcü tarafından bulunan kuasi-statik kestirim `f̂_x = −f_cmd,x`. Kararlı temasta **−0.92 ± 0.14 N** (n = 1260 npz örneği). Baseline'daki değerle aynı (−0.92 N).

## Q2 — Admittance filter

- `get_af_parameters` her snapshot'ta aynı:
  - mass 5, K = [10, 100, 100, 50, 50, 50], B = 2√(m·K),
  - J = diag(0.05), af.force = af.torque = 0.
  - Yani 06a'daki değerler uygulanmış.
- Sapma [LOG, component logları]: `uavpos.xd − maneuver.x`:
  - tüm uçuşta ort −0.9 µm, std 0.40 mm, aralık −5.3 / +3.0 mm (zaman hizalama ve interpolasyon kaynaklı),
  - **kararlı temasta tam 0** (n = 8570).
- Beklenen değer [HESAP, statik admittans, işaret konvansiyonu doğrulanmadı]: \|dev_x\| = \|f_ext,x\| / K_x ≈ 0.92 / 10 = **92 mm**. Gözlenen 0 mm.
- Sonuç: AF parametre tarafında doğru kurulu. Kuvvet girdisi olmadığı için çıktısı nominalle aynı. **AF bu koşuda K_x ile "tutarsız" değil, girdisiz.**
- Ayrıca bkz. E2: statik fazlarda phynt'e gelen referansın `pos` alanı None. WO düzeltilse bile bu, AF'nin konum sapması üretmesini engelleyebilir. Doğrulanmadı.

## Q3 — phynt.log

| snapshot | log_info total / miss | /tmp/phynt.log [bayt] |
|---|---|---|
| after_start | 0 / 0 | 75 |
| contact_1 (t_nom 28 s) | 0 / 0 | 75 |
| contact_2 (t_nom 45 s) | 0 / 0 | 75 |
| before_stop | 0 / 0 | 75 |

- 75 bayt, yalnızca başlık satırının boyutu. stop() sonrası kopyalanan `logs/noc/t00/phynt.log` da 75 bayt, 0 veri satırı.
- Dışlanan nedenler:
  - **kaçırılan kayıt** (miss = 0),
  - **dosyanın kopyalanmadan önce boşalması** (/tmp'deki dosya da 75 bayt),
  - **log() çağrısının başarısız olması** (başlık yazılmış).
- Kalan açıklama: phynt main task hiç log kaydı üretmiyor.
  - Hipotez: kayıt, WO/AF hesap adımında yazılıyor ve girdiler (wrench_measure.force, referans pos) geçersiz olduğu için o adıma girilmiyor.
  - phynt kaynak kodu bu repoda yok. **Doğrulanmadı.** Bir sonraki koşuda `log_info.total > 0` olup olmadığına bakarak test edilebilir.

## Q4 — Kpxy·e_x

Kararlı temas pencereleri (maneuver.log'a göre statik, x_nom = 1.55, ilk 1 s atıldı): [26.8, 30.7], [36.7, 38.7], [44.8, 46.7], [52.8, 54.7], [60.7, 62.7] s. uavpos.log'dan 8570 satır.

| [LOG] uavpos | değer |
|---|---|
| e_x | −0.1804 ± 0.0054 m |
| e_vx | −0.0020 ± 0.020 m/s |
| f_cmd,x | 0.916 ± 0.146 N (0.43 … 1.36) |
| f_cmd,x + 5·e_x + 7·e_vx | 7.8·10⁻⁸ ± 2.1·10⁻⁶ N |

| [HESAP] | değer |
|---|---|
| Kpxy·\|e_x\| | 0.902 ± 0.027 N |
| m·Kpxy·\|e_x\| | 2.45 N (uymuyor) |
| Fit (kararlı temas) `fx = a·e_x + b·e_vx + c` | a = −5.000, b = −7.000, c = 3·10⁻⁷ |
| Fit (tüm uçuş, n = 50081) | a = −4.79, b = −7.69, c = 0.012; residual 0.33 N (hareket sırasındaki feedforward terimlerden) |

- uavpos'un x yasası statik fazda `f = −Kpxy·e − Kvxy·ė` (Kixy = 0). Kazançlar doğrudan N/m ve N·s/m biriminde.
- Temas kuvvetinin kontrolcü tarafındaki karşılığı bu yasayla belirleniyor: **F ≈ Kpxy·|e_x| ≈ 5 × 0.18 = 0.90 N**.
- e_x = −0.18 m, 06a'nın referansı duvarın 0.15 m + geometri kadar içine koymasından geliyor (`BODY_X_CONTACT = 1.55`, gövde x ≈ 1.37).
- Bu bir kontrolcü-tarafı açıklama. Duvarın gerçek tepkisi ölçülmedi (bkz. 00_baseline §4).

---

## Ek bulgular

**E1 — `maneuver.goto` bloklayıcı.**
- Her çağrı hareket süresi boyunca blokluyor: 2.0–5.1 s, `events.json`. `set_wo_zero` de 2.45 s blokluyor.
- Sonuçlar:
  - Python döngüsü hareket sırasında **hiç örnek alamıyor**: dt > 100 ms olan 10 boşluk var, en büyüğü 5.1 s.
  - Döngü ardından "yetişmek" için uyumadan koşuyor: 1413 örnek < 5 ms aralıklı, gecikme ortalaması 1.9 s, en fazla 5.1 s.
  - npz'nin `t_nominal` sütunu gerçek zamanı yansıtmıyor, analizde `t_wall` kullanıldı.
- Bu durum 06a'nın kendi `simulation()` döngüsü için de geçerli.

**E2 — Statik fazlarda referansın `pos` alanı None.**
- Gözlemler [LOG, ham snapshot]:
  - `phynt.get_reference()` ve `maneuver.desired()`, her goto bittikten sonra `pos: null`, `vel: 0`, `acc: 0` döndürüyor.
  - Port ts değeri goto başına bir kez değişiyor (maneuver 9, phynt 14 farklı ts).
  - `phynt.desired.pos` yalnızca ilk goto'dan önce geçerli (100/4000 örnek). Okunan hiçbir referans örneğinde vel ≠ 0 yok.
- Okumalar yalnızca statik fazlarda yapılabildi (E1). Yani **hareket sırasındaki port içeriği bilinmiyor**.
- Buna rağmen uavpos.xd nominal pozisyonu tutuyor.
- Hipotez, doğrulanmadı: maneuver hareket bitince `pos` alanı olmayan bir "dur" durumu yayımlıyor, uavpos da son geçerli pozisyonu tutuyor.
  - Bu doğruysa, statik temas sırasında phynt'in AF'si değiştirecek bir pozisyon referansı almıyor.

**E3 — Eski npz (06a, `simulation_data.npz`).** Önceki sorudaki "pos_d mi p_nom mu sıfır" sorusu:
- **İkisi de sıfır**: p_nom 4000/4000, pos_d 3995/4000.
- Nedenleri:
  - 06a `read_desired_phynt()` / `read_nominal()` fonksiyonları `d['pos']['x']` okuyor. Port `pos` alanı None olunca bu `TypeError` veriyor, exception yutuluyor ve sıfır yazılıyor.
  - `read_state()` her çağrıda başarısız (`pom.frame()` port adı istiyor).
  - `read_external_wrench()` için `get_wo_data` servisi hiç yok.
- **Kesinlik durumu:** Bu açıklama t00'da aynı fazlarda gözlenen None alanlarına dayanan bir çıkarım. Eski koşunun exception metinleri kaydedilmediği için kesin değil.

---

## Önerilen düzeltmeler (UYGULANMADI, onayını bekliyor)

1. **F1 — uavatt `rotor_measure` bağlantısı.** `uavatt.connect_port({'local': 'rotor_measure', 'remote': 'rotorcraft/rotor_measure'})`.
   - Nereye: 06a'ya dokunmadan noc tarafındaki bir setup sarmalayıcısına; ya da senin onayınla 06a'ya.
   - Beklenti:
     - `wrench_measure.force` geçerli olur, external_wrench yazılır,
     - kararlı temasta WO yaklaşık −0.9 N görür,
     - `log_info.total > 0` olur.
   - Uzak port adı 01-intro script'lerinden alındı, uavatt ile denenmedi. t00_check'e bir doğrulama satırı eklenir.
2. **F2 — Bloklamayan goto.** Kayıt döngüsünde `maneuver.goto(..., ack=True)` (06a `servo` çağrılarında kullanılan genomix seçeneği). Böylece hareket sırasında da örnek alınır ve E2 doğrudan test edilebilir.
   - Alternatif: kaydı ayrı bir thread'de yapmak.
3. **F3 — E2'nin doğrulanması.** F1 ve F2 ile yeni bir t00 koşusu. Hareket ve temas sırasında `phynt.get_reference().pos` ile `phynt.desired().pos` alanlarına bakılır.
   - Statik fazda pos hâlâ None ise AF için referans kaynağı ayrıca ele alınmalı.
   - Seçenekler: hareket bitince `phynt.set_position`, ya da maneuver'ın sürekli yayın yapan bir modu.
4. **F4 — 06a okuma fonksiyonları.** Hata yutan `read_*` fonksiyonlarının yerine `noc/common/telekyb_io` kullanılması.
   - Yalnızca 06a'yı değiştirmeye karar verirsen geçerli.

**Not:** F1 uygulanınca AF devreye girer. Bu, temas dengesini değiştirir: referans yaklaşık K_x⁻¹·|f| kadar geri çekilir, kuvvet düşer. Sonraki koşu artık "aynı baseline" sayılmaz.
