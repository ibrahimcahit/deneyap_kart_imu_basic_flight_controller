# Deneyap Kart ve LSM6DSM IMU ile 3 eksenli uçuş kontrolcü

Deneyap Kartı üzerinde bulunan LSM6DSM IMU kullanılarak, üç eksenli uçuş stabilitesi sağlanabilmektedir.

## Gereklilikler

Deneyap Kart kütüphanesini indirmeniz gerekmektedir:

* [Deneyap Kart kurulumu](https://docs.deneyapkart.org/) 
* [LSM6DSM Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsm.pdf)

## Bağlantı Şeması

![image](https://raw.githubusercontent.com/ibrahimcahit/deneyap_kart_imu_basic_flight_controller/main/scheme.jpg)

## Nasıl Çalışıyor?

İlk başta gerekli kütüphaneleri import ediyoruz:

	#include "deneyap.h"
	#include "lsm6dsm.h"
	#include "ServoESP32.h"

Sonrasında kodda, kullanıcı tarafından değiştirilebilen iki yer çıkıyor. İlki, Servo yönlerini değiştirmek için. Eğer servonuz ter dönüyorsa ve çevirmek istiyorsanız, basitçe "*1*" olan değeri "*-1*" yapabilirsiniz. Bu sayede IMU dan gelen Raw değeri negatif ya da pozitif oalrak değiştirebilirsiniz. 

	// (!) Motorlar ters ise 1 olan degeri -1 yap (!)   
	//************************************************
	float a = 1; //Sag aileron                    // *
	float b = 1; //Sol aileron                    // *
	float c = 1; //Elevator                       // *
	float d = 1; //Rudder                         // *
	//************************************************

Diğer kısımda ise, herhangi bir servonun başlangıçta merkezlenmediğini düşünüyorsanız, değerleri ikişer ikişer büyültebilir ya da küçültebilirsiniz. Bu sayede, eğer IMU nun ilgili ekseninden gelen veri 0 a uzaksa, bu veriyi olabildiğince 0 a yakınlaştırmış oluruz. 

	// Gyro degerlerinde sapma varsa, asagidaki degerleri 
	// pozitif ve negatif olarak buyultup kucultebilirsiniz   
	//*******************************************************
	float e = 0; //Sag aileron                           // *
	float f = 0; //Sol aileron                           // *
	float g = 0; //Elevator                              // *
	float h = 0; //Rudder                                // *
	//********************************************************

Sonrasında, Servo pinlerini değişkenlere atıyoruz. 

	int servo_sag_aileron_pin = D0;  // Sag Alieron servo motoru
	int servo_sol_aileron_pin = D1;  // Sol Alieorn servo motoru
	int servo_elevator_pin = D4;     // Elevator servo motoru
	int servo_rudder_pin = D8;       // Rudder servo motoru

IMU dan okunan değerleri kaydetmek için değişkenler oluşturuyoruz

	float IMU_sag_aileron, IMU_sol_aileron, IMU_elevator, IMU_rudder;

IMU dan Z eksenini ve dolayısıyla YAW hareketini alabilmek için, değişkenler oluşturuyoruz. Buradaki `tstep`, ölçümler arasındaki adım aralığını temsil ediyor. 

	float gyroz;
	float norm_gyroz;
	float IMU_rudder_raw = 0;
	float tstep = 0.01;

Servolar için map edilmiş değerleri tutacak değişkenleri tanımlıyoruz

	int16_t sag_aileron, sol_aileron, elevator, rudder;

LSM6DSM class ı içinde, IMU objemizi oluşturuyoruz

	LSM6DSM IMU;

Servolarımızı tanımlıyoruz

	Servo servo_sag_aileron;
	Servo servo_sol_aileron;
	Servo servo_elevator;
	Servo servo_rudder;

`Setup ()` içinde klasik şeyleri yapıyoruz. Serial iletişimi başlatıyor, Servoları ilgili portlara takıyor ve IMU yu devreye alıyoruz.

	void setup() {

	  Serial.begin(115200);
	  Serial.println("Seri port açıldı");

	  servo_sag_aileron.attach(servo_sag_aileron_pin);
	  servo_sol_aileron.attach(servo_sol_aileron_pin);
	  servo_elevator.attach(servo_elevator_pin);
	  servo_rudder.attach(servo_rudder_pin);
	  Serial.println("Servolar aktif edildi");

	  IMU.begin();
	  Serial.println("IMU devrede");

	  delay (100);
	}

Loop kısmında ilk başta, delay süresini `millis` ile ESP32'nin sistem saatine eşitliyoruz. 

 	timer = millis();

Sonrasında X ve Y eksenlerindeki, yani Elevator ve Aileron hareketleri için gerekli olan sensör okumalarını, IMU muzdan istiyoruz. Burada aileron için iki tane okuma yaptım, zira iki aileron birbirine ters çalışıyor. İki okuma yapmak, daha sağlıklı olacaktır diye düşündüm. 

`a`, `b` ve `c` parametrelerini, kodun başından hatırlayacaksınız. Bunlar okunan değerlerin işaretlerini, dolayısıyla  servo yönlerini değiştirmemize olanak tanıyan parametreler. 

`IMU.readFloatAccelX()` fonksiyonu, bize **float** olarak X eksenindeki ivmelenmeyi veriyor. İvmelenme, lineer hareket olduğundan bunu kolayca kullanabiliyoruz. 

	IMU_sag_aileron = IMU.readFloatAccelX() * -100 * a;
	IMU_sol_aileron = IMU.readFloatAccelX() * -100 * b; 
	IMU_elevator = IMU.readFloatAccelY() * 100 * c;

Gelelim Z eksenine ve dolayısıyla YAW hareketine. Burada işler, X ve Y eksenine göre biraz daha karışık. Z ekseni, direkt yer çekimine karşı olduğu için ivmeölçer ile öçlmek sağlıklı olmuyor. Gyro ile okunan değerlerde ise sensörün, bir sonraki okuma için önceki gyro değerini başlangıç olarak almaması gerekiyor. 

Zira Gyro, 360 derecelik bir alanda size, başlangıç noktasına göre konumunuzu verir. Başlangıç noktası ise, her yeni okumada önceki okunan değer olur. Diyelim ki Gyro ile bir konumda okuma yaptınız ve 200 değerini aldınız. Yeni okumada artık o konum, sıfır olacaktır. 

İlk başta `IMU.readFloatGyroZ()` ile Z eksenindeki Gyro değerini alıyoruz. Okuduğumuz değeri,  Gyronun hassasiyetine göre (LSM6DSM kütüphanemizde bu değer standart olarak 250 dps olarak belirlenmiştir. *dps = degree per second* ) normalize etmek için bir sabit ile çarpmamız gerekli. Bizim durumumuzda bu sabit `0.007633f` sayısıdır.  

- `0.007633f` --> 250dps
- `0.015267` --> 500dps
- `0.030487f` --> 1000 dps
- `0.060975f` --> 2000dps

Üstte de bahsetmiştim, Gyroda başlangıç noktası hep güncellenir. Bu nedenle değişimi bulmak adına, güncel değeri önceki okunan değer ile topluyoruz, her seferinde. İşlemler sonunda çıkan değer, ondalıklı oluyor. Bunu düzeltmek için 100 ile çarpıyoruz. `d` değeri ise, servo yönü için.

	gyroz = IMU.readFloatGyroZ();
	norm_gyroz = gyroz * 0.007633f;
	IMU_rudder_raw = IMU_rudder_raw + norm_gyroz * tstep;
	IMU_rudder = IMU_rudder_raw * 100 * d;

Artık elimizde tüm değerler var. Birkaç basit `Serial.print` işlemi ile bunları seri ekrana yazdırıyoruz. 

	 Serial.print("*********************");
	  Serial.println();

	  Serial.print(" Sag Aileron = ");
	  Serial.println(IMU_sag_aileron);
	  Serial.print(" Sol Aileron = ");
	  Serial.println(IMU_sol_aileron);
	  Serial.print(" Elevator =    ");
	  Serial.println(IMU_elevator);
	  Serial.print(" Rudder =     ");
	  Serial.println(IMU_rudder);

Şimdi, geldik değerleri servolara yazmaya. Aslında servolar da, IMU sensörümüz de, dairesel hareketlere göre çalışırlar. Bu nedenle IMU da okunan bir aralığı, Servolarımızın çalışma aralığına eşitleyebiliriz. 

Servo kütüphanemizde bulunan map() fonksiyonu sayesinde,  IMU da okunan değerleri direkt olarak servolarımıza atayabiliriz. 

IMU, X ve Y eksenin de `-100 <-> 100` arasında okuma yapar. Z ekseni içinse `-80 <-> 80` arası okuma yapar.  Servolar ise standart olarak 0 ile 180 derece arası çalışır. Dolayısıyla bu iki aralığı birbirine map edebilir ve IMU dan gelen verinin direkt olarak servoalrı çalıştırmasını sağlayabiliriz. 

Burada bulunan dört parametre, `e`, `f`, `g` ve `h` okunan değeri sıfıra eşitlemek için var. Yani bir nevi kalibrasyon görevi görüyor. Eğer IMU normal konumda 0 yerine başka bir değer okuyorsa, kodun başında bu değişkenleri büyültüp küçülterek bunu nötrleyebilirsiniz.

	sag_aileron = map(IMU_sag_aileron + e, 100, -100, 0, 180);
	sol_aileron = map(IMU_sol_aileron + f, 100, -100, 0, 180);
	elevator = map(IMU_elevator + g, 100, -100, 0, 180);
	rudder = map(IMU_rudder + h, 80, -80, 0, 180);

Finalde ise, map edilen değerleri servolara yazıyoruz.

	servo_sag_aileron.write(sag_aileron);
	servo_sol_aileron.write(sol_aileron);
	servo_elevator.write(elevator);
	servo_rudder.write(rudder);

## Hata Bİldirimi ve Sorun Çözme

Bulduğunuz sorunları Issue kısımından bildirebilirsiniz. Ya da bu çalışmayı fork ederek düzeltmeler yapabilirsiniz. :)
