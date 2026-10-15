# Stevens ile model karşılaştırması

Proje kökünden tek komut:

```matlab
r = run_all_validations;       % tüm kontroller, sonuçlar ve üç MATLAB grafiği
```

Grafiksiz hesap için `run_all_validations(false)`, kayıtlı sonuçlardan yeniden
çizmek için `setup_f16; plot_stevens_validation` kullanılır.

## Karşılaştırılan modeller

- Düz Dünya: `src/F16DynamicsFlat.m`.
- Dönen Dünya: `src/F16DynamicsRotating.m`; WGS-84, 41°K / 29°D.
- Boylamsal 3-DOF: `src/F16Dynamics3DOF.m`. Yalnız düz uçuş ve boylamsal modlar.

Kaynak işaretleri doğrudan Stevens'ın yayımlı sayılarıdır. Ayrı bir Stevens
simülatörü, alternatif sabitlerle yeniden üretilen referans eğrisi veya duyarlık
karşılaştırması kullanılmaz. Her model kendi üretim denklemleriyle çağrılır.

## Koşullar ve yöntem

**Düz uçuş:** Tablo 3.6-2'nin 16 hız satırı, deniz seviyesi, CG=0,35c. Tablo
3.6-3'ün 502 ft/s ve CG=0,30/0,35/0,38c satırları da karşılaştırılır. Her model
kendi trimini çözer. Artıklar ve kumanda/aerodinamik sınırları denetlenir.
130 ft/s satırı geçerli trim değildir; hata özetine katılmaz.

**Modlar:** Tablo 4.2-3'ün dört hız/irtifa koşulu, CG=0,30c. Motor ve irtifa
dinamiği dahil edilir; yüzeyler trim konumunda sabittir. 6-DOF modellerinde 13,
3-DOF'ta 7 etkin durum kullanılır. Dutch roll, roll ve spiral yalnız 6-DOF
modellerinde vardır. Kaynağın periyodu `2*pi/abs(lambda)` doğal periyodudur.
Örnek 3.8-1/2'nin dört durumlu matrisleri ve Tablo 3.5-2'nin türevleri ayrıca
MAT dosyasındadır. Asimetrik türev örneği 3-DOF'a uygulanmaz.

Dönen Dünya sonuçları belirtilen konumda **anlık trim ve lineerleştirmedir**.
Ortak durumdaki `p,q,r` yerel NED'e göre hızlardır (`omega_BN`). Üretim ECEF
durumuna geçerken hem Dünya dönüşü hem NED taşıma hızı eklenir. Açısal ivme
dönüşümünde bu hızların türevleri de hesaba katılır. `trim.initialOmegaBody`,
6 DOF başlatıcılarının beklediği Dünya'ya göre hızı (`omega_BE`) ayrıca verir.
Ortak koordinat dönüşümü bağımsız bir üretim ECEF adımıyla; öteleme ise
bağımsız NED Newton denklemiyle test edilir. Ayrıca ayrı NED durumlarıyla
12 saniyelik yörünge integre edilir ve ECEF üretim koşusuyla karşılaştırılır.
NED kontrolü aynı yük tablolarını kullanır; ECEF dinamik fonksiyonunu veya
koordinat adaptörünü kendi integrasyonunda çağırmaz. Bu modelde küresel, zamandan
bağımsız bir sabit denge olduğu varsayılmaz.

Önceki trim kodu `omega_BE` türevini sıfırlıyordu. NED'e göre sabit dönüş
istenirken NED taşıma ivmesini atlamak hataydı. Eski başlangıcın gerçek
`omega_BN` türevi yaklaşık `[-2,85e-7; 6,11e-6; 1,27e-6] rad/s²` idi.
Düzeltilmiş dönüş triminde bu artık `1e-10 rad/s²` altında sınanır.
Kullanılan dönüşüm:

```text
omega_BI = omega_BN + C_BN * (omega_IE_N + omega_NE_N)
dot(omega_BN) = dot(omega_BI)
              + omega_BN × C_BN * (omega_IE_N + omega_NE_N)
              - C_BN * (dot(omega_IE_N) + dot(omega_NE_N))
```

Burada ikinci satırdaki çarpım `omega_BN × [C_BN*(...)]` biçimindedir.
Üretim `F16DynamicsRotating` denklemleri değiştirilmedi; hata trim/karşılaştırma
koordinat dönüşümündeydi. Dünya dönüşü sıfıra, eğrilik ihmal edilecek düzeye
getirildiğinde dönen modelin düz modele yaklaştığı da regresyon testidir.

**Dönüş:** Örnek 3.6-2'nin 502 ft/s, deniz seviyesi, CG=0,35c, 0,3 rad/s koşulu.
Başlangıç yer izi kuzeye bakar. İki 6-DOF modeli ayrı ayrı trimlenir; kumandalar
sabit tutulur ve gerçek kuaterniyon/ECEF denklemleri RK4 ile ilerletilir. Kitabın
19 konum örneği aynı zamanlarla eşleştirilir. Dönen Dünya'nın ECEF konumları
başlangıç noktasının teğet NED düzlemine izdüşürülür. 3-DOF dönüşe katılmaz.

Hedef süre 180 s, adım 0,01 s'dir. İrtifa −1 m'nin altına veya alfa/beta aero
tablosunun dışına çıkarsa koşu kesilir. 1 m tolerans, deniz seviyesindeki
başlangıcın sayısal gürültüyle kesilmesini önler. **−1 m üretim modelinin
çökme, yer teması veya aerodinamik geçerlilik sınırı değildir; karşılaştırma
kodunun seçtiği durdurma eşiğidir.** Ayrılma tanısında denklemler 12 s'ye
kadar sürdürülür; bu uzatma fiziksel yer altı uçuşu olarak yorumlanmaz.
Eksik kaynak zamanları `NaN`
kalır; 180 s eşleşmesi sayılmaz. Aynı koşu 0,005 s adımla da yapılır; yalnız
aynı zamanlardaki noktalar eşleştirilir. `stepCheckMaxPositionFt`,
`stepCheckUntilS` ve `fineEndTimeS` adım kontrolünü kaydeder. Konum farkı
0,05 ft'yi veya bitiş zamanı farkı 0,02 s'yi aşarsa `stepConverged=false`
olur. Bu toleranslar sayısal tekrar edilebilirlik içindir, fiziksel doğruluk
ölçütü değildir. Daireye zorlama veya kontrolcü yoktur.

## Sonuçları okuma

Düz Dünya ve 3-DOF'un düz uçuş trimleri çakışır. 15 geçerli satırda en büyük
farklar gazda 0,000883, hücum açısında 0,0236°, elevatörde 0,0359°'dir.
Dönen Dünya'da bunlar sırasıyla 0,000499, 0,0384° ve 0,00785°'dir.
Dönen modelin daha küçük çıkan bazı hataları tek başına daha doğru olduğu
anlamına gelmez; kaynak düz Dünya varsayımlarını kullanır.

Deniz seviyesinde 900 ft/s fugoid periyodu düz Dünya'da 183,329 s, dönen
Dünya'da 183,482 s, 3-DOF'ta 183,329 s; Stevens'ta 183,4 s'dir.
50.000 ft / 600 ft/s koşulunda üç modelde de geçerli trim bulunmaz. Bu noktaya
modal sonuç yazılmaz. 50.000 ft / 900 ft/s koşulu Mach≈0,93 ile kaynak modelin
0,81 sınırını aşar; grafik, o kitap satırının sayısal karşılaştırmasıdır.

Dönüşte düz Dünya koşusu **180 s** tamamlandı. Kitabın 19 noktasındaki en
büyük yatay konum farkı **16,653 ft (5,076 m)**. Bu koşu kitabın konumlarını
bire bir üretmez; sapma grafikte ve MAT dosyasında korunur. Kitabın basılı
yer izi nominal sabit hızlı daireyle tam çakışmaz; kalan farkın tamamı
yuvarlama hatası diye açıklanamaz.

Dönen Dünya'da düzeltilmiş NED trimi sonrası sabit kumandalarla uçuş,
**11,84 s**'de irtifa −1 m sınırını geçtiği için kesildi. 10 s'deki yatay
konum farkı **6,297 ft (1,919 m)**; 20–180 s örnekleri karşılaştırılmış
sayılmaz. Eski 12,11 s sonucu hatalı trim tanımıyla hesaplanmıştı ve artık
kullanılmıyor. Düzeltme erken ayrılmayı ortadan kaldırmadı; bu ayrılmayı
yalnız eksik taşıma ivmesine bağlamak yanlış olur.

Adım yarıya indiğinde ortak zamanlardaki en büyük konum değişimi düz
Dünya'da **1,35e−9 ft**, dönen Dünya'da **2,10e−7 ft**; kesilme zamanı da
yalnız 0,005 s değişti. ECEF üretim ivmesi, bağımsız NED Newton denklemindeki
`-(2*omega_IE_N+omega_NE_N) × v_N` terimiyle `1e-10 m/s²` toleransında
uyuştu. Bu kontroller, kalan ayrılmayı bir koordinat dönüşümü veya zaman
adımı hatası olarak açıklamıyor.

Dönen Dünya'nın yerel denge koşulu dönüş boyunca değişir: Coriolis'in gövde
bileşenleri, yerel NED yönelimi ve ağırlık vektörü sabit kalmaz. Başlangıçta
trim çözmek, aynı sabit kumandalarla bütün daireyi yatay tutmayı garanti
etmez. Dönüş çevresindeki anlık Jacobian'ın pozitif kutbu yaklaşık
**+0,576 s⁻¹** olduğundan küçük sapmalar büyüyebilir. Bu yerel kutup hesabı,
küresel yörüngenin kararlılık ispatı değildir. Kaynağın düz Dünya sonucuyla
dönen Dünya'nın 180 s boyunca uyuştuğu iddia edilmez.

Bunu ayıran `results.turn.frameAudit` kontrolünde konum, hız, yatış,
dikilme ve kumandalar korunup baş açısı 90° değiştirildi. Başlangıçta
sıfırlanan gövde ivmesi yaklaşık `[0,0016; −0,0158; −0,0061] m/s²` oldu
(norm yaklaşık **0,017 m/s²**). Aynı fark hesabında yalnız Dünya dönüş
terimleri çıkarıldığında norm yaklaşık **0,0029 m/s²** kaldı. Bu dört RHS
sorgusu bir başka uçak modeli veya doğrulama eğrisi değildir; aynı sabit
kumandaların farklı baş açılarında neden artık denge sağlamadığını ölçer.
Bu tanıyı yalnız bir ihtimal olarak bırakmamak için iki ek sınama yapıldı:

1. **Bağımsız NED yörüngesi:** 12 s sonunda irtifa **−1,11382 m**. ECEF
   üretim yörüngesiyle en büyük konum farkı **1,17e−7 m**, hız farkı
   **7,88e−8 m/s**. İrtifa ayrılması iki koordinat sisteminde de oluşuyor.
2. **Değişen denge artığının etkisi:** Baş açısı `psi0+0,3*t` olan nominal
   yatay hareket boyunca `A(t)` Jacobian'ı ve `b(t)=f(x_nom)-dot(x_nom)`
   artığı hesaplandı. `dot(delta)=A(t)*delta+b(t)`, `delta(0)=0` hesabı
   12 s sonunda **−1,10191 m** irtifa değişimi öngördü. Tam doğrusal olmayan
   sonuçla fark **%1,07**. Öteleme denge artığının katkısı **−1,15542 m**,
   açısal denge artığının katkısı **+0,05351 m**. Böylece ilk ayrılmanın
   çoğu değişen öteleme dengesinden geliyor; yalnız eski açısal trim
   hatasından kaynaklanmıyor. Öteleme katkısının **−0,67272 m**'si doğrudan
   baş açısıyla değişen Coriolis teriminden, **−0,48270 m**'si diğer
   öteleme artıklarından geliyor. Coriolis katkısı, nominal gövde hızında
   `-2*(omega_IE_B(t)-omega_IE_B(0)) × v_B` farkı ayrı sağ taraf olarak
   kullanılarak hesaplandı.

İkinci hesap bir kontrolcü veya ek uçak modeli değildir; sabit kumandalı
koşunun ilk sapmasını açıklayan lineer hata denklemidir. Katkılar bu
lineer hesapta toplanabilir; ayrı doğrusal olmayan uçuşların farkları
değildir. `results.rotatingTrajectoryCheck` tüm bu sonuçları saklar;
`turn.png` alt sırasında irtifa tahmini, NED/ECEF farkı ve Coriolis dahil
terimlerin katkıları gösterilir. Üst iki sıradaki Stevens karşılaştırması
yine yalnız düz ve dönen Dünya modelleridir.

**Sonuç:** NED taşıma ivmesinin atlanması gerçek bir trim hatasıydı ve
düzeltildi. Kalan erken sapma ise baş açısıyla değişen denge artığının açık
çevrim hareketinde büyümesiyle sayısal olarak açıklanıyor. Bu sonuç,
180 s sabit kumandalı yatay dönüşün doğrulandığı anlamına gelmez. Başlangıçta
çözülen anlık trimi bütün dönüş boyunca sabit denge gibi yorumlamak yanlıştı.
Kontrolcü eklenmedi; üretim fiziği veya kumandalar eşleşme uğruna değiştirilmedi.

Farkların değerlendirilmesinde aynı ortak aerodinamik tablolar, SI atmosferi,
yerçekimi ve sabitlerin kullanıldığı dikkate alınmalıdır. Düz modelde sabit
9,80665 m/s²; dönen modelde enlem/irtifaya bağlı normal yerçekimi, Coriolis ve
geometri vardır. Stevens'ın atmosfer ve yuvarlatılmış sabitleri bire bir aynı
değildir. Salt karşılaştırma hatasını azaltmak amacıyla üretim parametreleri
kitaba uydurulmaz.

Özellikle 50.000 ft'de üretim atmosferinin yoğunluğu **0,186479 kg/m³**,
Stevens Ek A.6'nın `0,002377*(1−0,703e−5*h_ft)^4,14` bağıntısı ise birim
dönüşümünden sonra **0,203922 kg/m³** verir: **%8,55 fark**. Bu fark dinamik
basıncı, gerekli hücum açısını ve modal türevleri etkiler. Deniz seviyesinde
uyuşup yüksek irtifada ayrılmanın somut nedenlerinden biridir. Bütün modal
farkları yalnız atmosfere bağlamak için parametreleri tek tek değiştiren
ayrı bir neden analizi gerekir; burada böyle bir sonuç iddia edilmez.

## Kod ve çıktı düzeni

- `check_f16_trim`: üç model için ortak trim çözücüsü.
- `f16_validation_dynamics`: üretim denklemlerini ortak karşılaştırma
  koordinatlarına dönüştürür; ayrı bir uçak modeli içermez.
- `linearize_f16_full`: sayısal Jacobian.
- `compare_stevens_trim`, `compare_stevens_modes`, `validate_stevens_turn`:
  üç karşılaştırma.
- `stevens_example_362`, `references/`: kitap verileri.
- `validate_stevens_tables`: 852 kaynak tablo sayısı ve 720 ara değer kontrolü.
- `plot_stevens_validation`: yalnız MATLAB ile üç grafik.

`outputs/` yalnız `validation_results.mat`, `trim.png`, `modes.png`, `turn.png`
içerir. Test günlükleri, ara dosyalar ve eski karşılaştırmalar tutulmaz.
`tests/test_f16` temel üretim kontrollerini; `tests/test_stevens_validation`
kaynak verilerini, üç modelin trimlerini, Jacobian adımını, koordinat
dönüşümünü, bağımsız NED yörüngesini ve ilk ayrılmanın zorlanmış varyasyon
hesabını denetler. Önceki DLDR/DNDA aktarım hatalarının düzeltmeleri korunur.

## Kaynaklar

- [Stevens/Lewis/Johnson, Wiley Ek A.7/A.8](https://onlinelibrary.wiley.com/doi/epdf/10.1002/9781119174882.app1)
- [Paylaşılan dönüş trimi görseli](references/3_6_2.png)
- [Kitap tabloları: 3.5-2, 3.6-2/3, 3.8-1/2, 4.2-3](https://studylib.net/doc/28069257/aircraft-control-and-simulation)
- [NASA/NESC F-16 kontrol koşulu: açısal hızlar yerel çerçeveye göre](https://nescacademy.nasa.gov/flightsim/2015/atmospheric/acc11)
