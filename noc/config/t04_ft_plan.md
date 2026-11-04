# t04 — Gazebo F/T sensörü ile temas kuvveti: plan

Yazıldığı tarih 2026-11-04. Uygulama ve uçuştan önce yazıldı ve commit edildi.
Ölçütler: `noc/config/t04_ft_criteria.json` (Kapı B1).

## Amaç

Şimdiye kadarki iki kuvvet de aynı kontrol zincirinden gelen kestirimdi: WO `efx` ve uavpos `fx`.
Bu deneyde ek olarak fizik motorunun eklemde ilettiği kuvvet kaydedilecek. Bu kuvvet uç ile gövde
arasındaki sabit eklem `ee_joint` üzerindeki force_torque sensöründen okunacak. Sonra üç sinyal
zaman hizalanıp karşılaştırılacak.

## Dosyalar (mevcutlar değişmez)

| Dosya | İçerik |
|---|---|
| `gazebo/models/noc-tilthex/` | `mrsim-tilthex` kopyası. `ee-vbar`, `ee-hbar`, `ee-tip` (visual + collision) yeni `ee` link'ine taşınır. `base`–`ee` arasında fixed `ee_joint` kurulur. Uç geometrisi ve pozları birebir aynı kalır. `ee` link pozu model kökünde (base ile aynı çerçeve). |
| `ee` kütlesi | **0.01 kg**, kütle merkezi (0.3, 0, −0.125). Atalet ixx = 1e−6, iyy = izz = 3e−4 kg·m² (0.6 m çubuk, m L²/12). Orijinal modelde uç kütlesizdi, toplam kütle 2.72 → 2.73 kg (+0.37 %). 06a/uavpos hâlâ 2.72 kullanıyor. |
| sensör | `ee_joint` üzerinde `force_torque`: `<frame>child</frame>`, `<measure_direction>child_to_parent</measure_direction>`, 500 Hz, topic `/noc/ee_ft`, `always_on`. |
| `gazebo/worlds/noc-wall-world.world` | `hexa-fa-wall-world.world` kopyası. `model://noc-tilthex` kullanır, adı `hr6`. optitrack `hr6::base → HR_6` aynı. Ek olarak `gz-sim-forcetorque-system` eklentisi. |
| `noc/experiments/sim_noc.sh` | 06a `simulation.sh` kopyası. Yeni dünyayı açar. `GZ_SIM_RESOURCE_PATH=/shared-workspace/gazebo/models:$GZ_SIM_RESOURCE_PATH`. |
| `noc/experiments/log_ft.sh` + `ft_stamp.py` | `gz topic -e -t /noc/ee_ft --json-output` çıktısının her mesajına host alış zamanını ekler. Satır formatı: `host_ts<TAB>json`. |
| `noc/common/ft_log.py` | F/T log parser. t04 canlı okumada, a04 offline kullanır. |
| `noc/tests/t04_ft_check.py` | setup_06a + flight + telekyb_io. `--check` seçeneği var. Logger'ı kendisi başlatır. |
| `noc/analysis/a04_ft_check.py` | Kapı B1, zaman hizalama, gecikme, bias, RMSE. |

## İşaret ve çerçeve beklentisi (ön kayıt)

- `child_to_parent` ayarında sensör, `ee` link'inin `base`'e uyguladığı wrench'i, `ee` çerçevesinde
  verir. `ee` çerçevesi body çerçevesiyle aynı.
- Duvar ucu −x yönünde iter. Uç bu kuvveti gövdeye iletir. Beklenen: bias düzeltilmiş F/T Fx < 0.
  WO `efx` ile aynı işaret, uavpos `fx` ile ters işaret olmalı.
- Normal bileşen: F/T kuvveti pom attitude ile dünya çerçevesine döndürülür ve dünya x bileşeni
  alınır.
- Bias şunları içerir: uç ağırlığı (yaklaşık 0.098 N, body −z'de) ve hover'daki duruş etkisi.
  Uçun ivmeden kaynaklanan kuvveti (m_ee·a) bias'ta yok; bu bir gürültü terimi.

## Senaryo (nominal s)

| t | olay |
|---|---|
| 2 | goto (0, 0, 1), 5 s |
| 10 | goto (1.2, 0, 1), 4 s (uç yüzeyi x ≈ 1.84, duvardan 16 cm uzakta) |
| 20 | `phynt.set_wo_zero(2 s)` |
| 26–34 | **FREE1**: serbest hover, bias penceresi |
| 34 | temas goto (1.55, 0, 1), 2 s |
| 44–54 | **CONTACT**: sabit temasın kalıcı penceresi |
| 54 | geri çekilme goto (1.2, 0, 1), 3 s |
| 60–68 | **FREE2**: geri çekildikten sonra serbest hover |
| 68 | iniş goto (0, 0, 0), 5 s |
| 76 | stop |

- K_x = 10 sabit, AF parametreleri hiç değiştirilmiyor.
- Kalkıştan önce AF `af_on_baseline.json` ile karşılaştırılıyor; her döngüde `get_af_parameters`
  geri okunup kontrol ediliyor (t02/t03 ile aynı).
- Güvenlik: `flight.Safety` v2, `SAFETY` aynen.
- Beklemede kaynaklar (phynt_des, ext_wrench) t03'teki gibi.

## Ön kontroller

- **F/T akışı:** `start()` sonrasında 3 s içinde en az 50 F/T mesajı gelmezse kalkış yapılmaz.
- **`--check` (uçuşsuz):** setup + start + preflight + F/T akışı. Yerde F/T ortalaması ve uç
  ağırlığı kontrolü: |F| ≈ m_ee·g = 0.098 N, ±%20. Bu yalnızca raporlanır.

## Raporlanan, kapı dışı

- **Gecikme:** F/T normal kuvvetinin WO `efx` ve −uavpos `fx`'e göre gecikmesi.
- **RMSE:** Gecikme düzeltildikten sonra, temas aralığında ve kalıcı pencerede.
- **Bias** değerleri, RTF.
- **Bağımsızlık düzeyi** (§ ön kayıt):
  - F/T, fizik motorunun eklem kısıt kuvvetidir.
  - WO kestirimi rotor hızı + model + durumdan hesaplanıyor; uavpos `fx` ise kontrolcü komutu.
  - Üçü de aynı simülasyonun aynı temas olayına tepki veriyor. Bu yüzden zaman olarak bağımsız
    değiller, ama F/T kontrol zincirinin hiçbir çıktısını kullanmıyor.
  - F/T duvardaki temas kuvveti değil, `ee_joint` üzerinden iletilen kuvvet. Arada
    m_ee·(a − g) ve sürtünme terimleri var.

## F/T Ionic'te çalışmazsa

- **Yapılmayacak:** Model veya dünyada büyük bir değişiklik.
- **Yapılacak:**
  1. Nedeni yazılacak: hata mesajı, topic yok, sıfır değer vb.
  2. Alternatif olarak `ee-tip-col` üzerinde contact sensörü (`gz-sim-contact-system`) önerilecek;
     kullanıcı onaylamadan uygulanmayacak.
