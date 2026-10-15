# F-16 MATLAB simülasyonu

## Klasörler ve başlangıç

- `src/`: 3 DOF ve 6 DOF simülasyon kodu, Dünya ve hava aracı modelleri.
- `tests/`: kısa regresyon testleri.
- `validation/`: trim, mod, dinamik ve Stevens karşılaştırmaları.
- `validation/outputs/`: tek MAT sonuç dosyası ve üç karşılaştırma grafiği.
- `validation/references/`: kullanıcının sağladığı kitap görseli.
- `veriler/`: aerodinamik veri çalışma kitabı.

MATLAB'i proje kökünde açıp önce `setup_f16` çalıştırın. Bu komut gerekli
klasörleri MATLAB yoluna ekler. Tam doğrulama için doğrudan
`run_all_validations(true)` çağrılabilir; yol kurulumunu kendisi yapar.

## Kodu kitapla birlikte okuma

Üç modelin başlatıcısı birbirinden bağımsızdır. Her dosya kendi varsayılan
seçeneklerini, başlangıç durumunu, RK4 zaman döngüsünü ve sonuç hazırlığını içerir:

- `run_f16_flat.m`: düz Dünya 6 DOF; `F16DynamicsFlat.m` çağırır.
  Hız gövde, konum NED eksenlerindedir.
- `run_f16_rotating.m`: dönen WGS-84 Dünya 6 DOF; `F16DynamicsRotating.m`
  çağırır. Hız ve konum ECEF eksenlerindedir.
- `run_f16_3dof.m`: boylamsal 3 DOF; `F16Dynamics3DOF.m` çağırır.

Kitapta her modelin başlangıçtan sonuçlara kadar akışı kendi dosyasından
izlenebilir. Ortak başlatıcı yoktur; model seçimi ilgili fonksiyonu çağırarak
yapılır. Eski `run_f16(options)` çağrıları yerine `run_f16_flat(options)`
veya `run_f16_rotating(options)` kullanılmalıdır.

`F16AircraftLoads.m` iki 6 DOF modelinin ortak aerodinamik, itki ve eyleyici
hesaplarını yapar. `F16Frames.m` koordinat dönüşümlerini; `AeroModel.m`,
`F16ThrustRK4.m` ve `F16Actuators.m` ilgili alt modelleri içerir.
Boylamsal 3 DOF modeli ayrı `run_f16_3dof.m` ve `F16Dynamics3DOF.m`
dosyalarındadır.

Genel integrasyon yöntemi `src/RK4.m` dosyasındadır. Üç bağımsız başlatıcı da
her zaman adımında bu fonksiyonu çağırır:

```matlab
stateNext = RK4(derivative, time, state, dt, extra1, extra2, ...);
% F-16 örneği: ek girdiler seçenekler ve model parametreleridir.
stateNext = RK4(@F16DynamicsFlat, time, state, dt, cfg, p);
```

`derivative(t,x,...)`, durumla aynı boyutta türev döndürür. RK4, türevi
başlangıçta, iki orta nokta durumunda ve adım sonunda değerlendirip
`xNext = x + dt*(k1 + 2*k2 + 2*k3 + k4)/6` ile bir adım ilerler.
Zamana veya duruma bağlı kontrol komutları her ara aşamada yeniden hesaplanır.
Zaman döngüsü, kuaterniyon normalizasyonu, yüzey sınırları ve yer teması
kontrolü simülasyon sürücülerinde kalır; `RK4.m` hava aracından bağımsızdır.

Kodda **B** gövde, **N** yerel kuzey-doğu-aşağı (NED), **E** Dünya'ya bağlı
ECEF, **I** eylemsiz çerçevedir. `C_WB`, gövde bileşenlerini seçilen Dünya
çerçevesine (`W=N` veya `W=E`) taşır. `omegaBI` gövdenin eylemsiz açısal
hızıdır; `omegaRel` dönen Dünya'ya göre hızdır. İkisi düz Dünya modelinde
aynıdır. Kitaptaki denklemleri okurken bu ayrım özellikle dönen Dünya
kolundaki açısal momentum ve kuaterniyon türevlerinde önemlidir.

`F16Parameters.m` içindeki `stateIndex` haritası, ana çözücüde aşağıdaki
17 bileşenli durumun sayısal dizinlerini adlandırır:

```matlab
state = [velocity; omegaBI; position; q_WB; enginePower; surfaceDeg];
% boyutlar:  3         3          3       4        1            3
```

`velocity` düz modelde gövde, dönen modelde ECEF hızıdır. `position` düz
modelde NED, dönen modelde ECEF konumudur. `q_WB` skaler bileşeni önce gelen
Hamilton kuaterniyonudur. Motor gücü 0–100, yüzey açıları derecedir.
`test_f16` temel regresyonları, `check_f16_trim` dengeli düz uçuşu,
`compare_stevens_trim` yayımlı trim satırlarıyla karşılaştırmayı gösterir.

## Kitaptaki boylamsal 3 DOF modeli

`Kitap_GIT/kitap.typ` dosyasının **Boylamsal Hareket Denklemleri** bölümü
(yaklaşık 3120–3171. satırlar) düz, dönmeyen Dünya'da kuzey-aşağı
düzlemindeki hareketi tanımlar. Buradaki 3 DOF, iki öteleme ve bir yunuslama
serbestlik derecesidir; noktasal kütle modeli değildir. Kullanım:

```matlab
longitudinal = run_f16_3dof(struct('duration',10,'dt',0.01, ...
    'controls',[-1;0.5], 'initialElevatorDeg',0));
plot(longitudinal.time,longitudinal.altitude)
```

`controls=[elevatör_derece;gaz_oranı]` sırasındadır. Fonksiyon kolu
`controls=@(t,state) [elevator;throttle]` biçiminde her RK4 ara aşamasında
değerlendirilir. Durum sırası `[u;w;q;p_N;p_D;theta;motor_gücü;elevatör]`:
`u,w` gövde yer hızı (m/s), `q` yunuslama hızı (rad/s), `p_N,p_D` NED
konumu (m), `theta` dikilme (rad), motor gücü 0–100 ve elevatör derecedir.
Başlangıç alanları `initialVelocityBody=[u;w]`,
`initialPitchRateRadS`, `initialNED=[p_N;p_D]`, `initialPitchDeg`,
`initialEnginePower` ve `initialElevatorDeg` olarak verilir. `windNED`
üç bileşenlidir; doğu bileşeni sıfır olmalıdır.

`F16Dynamics3DOF.m` içindeki `uDot,wDot`, kitaptaki
`otelemeDinamigi3DOF`; `pitchRateDot`, `donmeDinamigi3DOF`; `pitchDot`,
`donmeKinematigi3DOF`; `positionDot`, `otelemeKinematigi3DOF` denklemlerini
uygular. Rüzgâr `duzDunyaRuzgarIliskisi3DOF` ile gövdeye çevrilir.
Hücum açısı ortak aerodinamik modelde `atan2(w_a,u_a)` ile bulunur; bu,
kitaptaki `arctan(w_a/u_a)` ifadesinin bölge ve sıfır payda açısından
güvenli biçimidir. Yan hız ve kayış açısı sıfır, yatış ve baş sabittir.
Yer/iniş takımı kuvveti bu F-16 uygulamasında modellenmez.

3 DOF ve 6 DOF, aynı başlangıç durumunda boylamsal türevleri verir;
uzun süreli yörüngelerin bire bir çakışması beklenmez. Örneğin 6 DOF'taki
motor rotoru jiroskopiği yunuslama sırasında yanal açısal hız üretebilir;
3 DOF bu yanal hareketi tanımı gereği dışarıda bırakır.

İki bağımsız 6 DOF başlatıcısı sabit adımlı RK4 ile çalışır:

```matlab
flat = run_f16_flat(struct('clock','simulation', ...
    'dt',0.01, 'duration',10));
ellipsoid = run_f16_rotating(struct('clock','simulation', ...
    'dt',0.01, 'duration',10));
% Gerçek zaman seçeneği her iki Dünya modelinde de kullanılabilir:
liveFlat = run_f16_flat(struct('clock','realtime', ...
    'dt',0.01,'duration',10));
liveEllipsoid = run_f16_rotating(struct('clock','realtime', ...
    'dt',0.01,'duration',10));
plot(flat.time, flat.altitude, ellipsoid.time, ellipsoid.altitude)
legend('Düz dünya','Dönen WGS-84 dünya'); xlabel('s'); ylabel('m');
```

`clock='simulation'` mümkün olduğu kadar hızlı çalışır. `clock='realtime'` her
adım sonunda duvar saatine göre bekler; hesaplama süresi adım aralığını aşarsa
gerçek zamana yetişeceği garanti edilmez. Her iki modda da fiziksel zaman adımı
aynı, sabit `dt` değeridir. `duration`, `dt` değerinin tam katı olmalıdır.

`controls=[elevator;aileron;rudder;throttle]` ilk üç bileşende derece, son
bileşende 0–1 oranıdır. İstenirse `controls=@(t,state) [0;0;0;0.5]` ile her
RK4 alt adımında çağrılan bir girdi işlevi verilebilir.
`cgFraction` ortalama aerodinamik kord üzerindeki ağırlık merkezi konumudur
(varsayılan 0,35). Tablo 3.6-3'teki ileri CG dönüşünde 0,30 seçilir;
Şekil 3.6-6/7'deki Örnek 3.6-2 dönüşü aynı trim değildir.
Üç kontrol yüzeyi Stevens/Lewis A.4'e göre ayrı RK4 durumlarıdır: hepsinin
birinci derece zaman sabiti 0,0495 s; elevator ±25° ve 60°/s, aileron ±21,5°
ve 80°/s, rudder ±30° ve 120°/s sınırındadır. Başlangıçta yüzeyler ilk komuta
eşit alınır; geçişi görmek için `initialActuatorDeg=[0;0;0]` verilebilir.
Elevator katsayı tablosu yalnız ±24°'ye uzandığından 24°–25° arasında aynı
kenar katsayısı kullanılır. Aileronun ±21,5° etkisi mevcut doğrusal
`da/20` terimiyle hesaplanır.

Başlangıç koşulları `initialVelocityBody` (m/s), `initialOmegaBody` (rad/s),
`initialEulerDeg`
(yatış, dikilme, baş; derece), `initialNED` (düz dünya; m) ve `initialLLADeg`
(enlem, boylam derece; irtifa m) alanlarıyla ayarlanır. `windNED` (m/s) ve
`stopAtGround` seçenekleri vardır. Varsayılan başlangıç 5000 m irtifada,
150 m/s gövde ileri hızıyladır; başlangıç koşulu trimli uçuş değildir.

Çıktıda `time`, `state`, `altitude`, `velocityBody`, `attitudeEulerDeg`,
`latitudeLongitudeDeg` (yalnız elipsoid model), `actuatorDeg` ve
`wallSeconds` bulunur.
`state` sütunları sırasıyla `[hız(3); açısal hız(3); konum(3);
kuaterniyon(4); motor gücü; gerçek yüzey açıları(3)]` düzenindedir. Düz
dünyada hız gövde, konum yerel
NED; dönen dünyada hız ve konum ECEF koordinatlarındadır. Açısal hız her iki
modelde gövde koordinatlarındaki eylemsiz `omega_BI` değeridir. Başlangıçtaki
`initialOmegaBody` seçilen dünya çerçevesine göre hızdır; dönen model onu
başlangıçta `omega_BI` değerine dönüştürür.

## Denklem ve veri sınırları

Hareket denklemleri `Kitap_GIT/kitap.typ` düz dünya bölümündeki 2894–2951 ve
dönen dünya bölümündeki 2970–3058 civarına dayanır. Kuaterniyon sözleşmesi
skaler önce Hamilton biçimidir: `q_WB` gövdeden NED/ECEF'e **etkin** dönüşü
gösterir, `C_WB*v_B=v_W` ve `qdot=0.5*q⊗[0;omega_BW^B]`. Kitaptaki `q_NB`
adlandırması ve matris yönü bazı satırlarda birbirleriyle uyuşmadığı için
buradaki açık sözleşme kullanılır. Dönen model, ECEF hız denkleminde Coriolis
ve merkezkaç terimlerini içerir; açısal dinamik eylemsiz açısal hızla hesaplanır.

WGS-84 geometri parametreleri ve dönüş hızı kullanılır. Kitabın WGS-84
yerçekimi bölümü henüz tamamlanmadığı için burada Somigliana yüzey normal
ağırlığı irtifayla ters kare olarak ölçeklenmiştir. ECEF kuvvet denklemindeki
merkezkaç terimiyle çift sayım olmaması için fiziksel çekime merkezkaç vektörü
bir defa geri eklenir. Bu, yüksek irtifa/yörünge için hassas bir gravite modeli
değildir. Düz model sabit 9.80665 m/s² kullanır.

Aerodinamik katsayılar mevcut `AeroModel.m` tablolarından gelir; hücum açısı
−10°…45° ve kayış açısı −30°…30° tablo sınırlarına kısılır. Elevatörün
`CX/Cm` katsayı sorgusu ±24° ile sınırlıdır; kanatçık ve dümen etkileri
doğrusal kontrol türevlerinden hesaplanır.
Fiziksel hava hızına 200 m/s sınırı uygulanmaz. Atmosfer modeli 0–20 km
arasında SI standart atmosfer yaklaşımıdır; bunun dışında sınır değerine
kısılır. Motor verisi Stevens Ek A itki tablolarından alınmıştır;
Mach 0 satırı da kullanılır. Veri 0–15.24 km ve
Mach 0–1 aralığında interpolasyon yapar, dışarıda kenar değerini kullanır.
Motor gücü RK4 durumudur. Model bu tablonun dışındaki uçuş zarfında
doğrulanmış sayılmaz.
`validate_stevens_tables()` Wiley Ek A.7/A.8'deki 744 aerodinamik ve
108 itki sayısını, ayrıca 120 ara noktadaki altı katsayıyı doğrudan denetler.
Bu incelemede DLDR/DNDA tablolarındaki 129 aktarım hatası düzeltilmiştir.
`test_stevens_validation()` motor güç geçişlerini de denetler.
Özellikle kitabın Ek A.6 atmosfer yaklaşımı yerine
ayrı bir SI standart atmosfer yaklaşımı kullanılır; tablo dışına
ekstrapolasyon yerine sınırda sabitleme uygulanır.
Rotorun +gövde X eksenindeki sabit 160 slug·ft²/s açısal momentumu Euler
denkleminde jiroskop çapraz terimiyle hesaba katılır. Rotorun güçle değişen
devri ve hızlanma tepkisi modellenmemiştir.

Atalet tensörü `Jxz=982 slug·ft²` ürün ataleti için −`Jxz` tensör girdisini
kullanır. `Jxx=9496 slug·ft²`, [NASA F-16 kontrol örneğindeki](https://ntrs.nasa.gov/api/citations/20180008442/downloads/20180008442.pdf)
değerle uyumludur; [Stevens ve Lewis Appendix A](https://onlinelibrary.wiley.com/doi/epdf/10.1002/9781119174882.app1)
farklı bir `Jxx` değeri (9456) verir. Üretim modelleri kendi SI sabitleriyle Stevens tablolarına karşılaştırılır. Gerçek uçuş verisiyle karşılaştırma bu doğrulamaya dahil değildir.

Simülasyonun giriş noktaları `run_f16_flat.m`, `run_f16_rotating.m` ve
`run_f16_3dof.m` dosyalarıdır. Önceki joystick/Euler
denemesi ve kullanılmayan yardımcılar kaldırılmıştır; kaynak aerodinamik
çalışma kitabı `veriler/AeroModel.xlsx` korunur. `test_f16` temel geometri,
atmosfer, veri, servo ve iki Dünya modelini denetler.

## Test ve doğrulama

```matlab
setup_f16
test_f16
r = run_all_validations(false); % sayısal doğrulama
r = run_all_validations(true);  % MATLAB ile üç grafik
```

Doğrulama kodu ve açıklaması [validation/](validation/README.md) altında;
üretilen grafikler `validation/outputs/` klasöründedir. `false` seçeneği
MATLAB grafik ortamına ihtiyaç duymaz. Grafikleri MATLAB üretir; kayıtlı sonuçlardan
yalnız grafikleri yenilemek için `plot_stevens_validation()` çağrılır.
Trim ve boylamsal modlar düz Dünya, dönen Dünya ve 3-DOF modelleriyle karşılaştırılır. Dönüşte yalnız iki 6-DOF modeli kullanılır. Koşullar, uyuşmazlıklar ve kesilen dönüş izleri [doğrulama açıklamasında](validation/README.md) yer alır. Tek sonuç kümesi tutulur.
