# Gömülü Sistemler Portfolyosu
Bu repository, **İTÜ Rake** bünyesinde geliştirdiğimiz otonom araç projesi için **Elektronik Kaptanı** olarak yazdığım gömülü sistem sürücülerini, motor kontrol algoritmalarını ve haberleşme protokollerini içermektedir.

## Proje Hakkında
IGVC (Intelligent Ground Vehicle Competition) ve TEKNOFEST İKA (İnsansız Karar Aracı) için hazırlanan otonom aracımızın elektronik alt sistemlerinin tasarımı ve yazılımı tarafımca yönetilmiştir. Bu kodlar, aracın sensör verilerini toplama, ana bilgisayar ile haberleşme ve motor sürüş işlemlerini gerçekleştirmektedir.

## Kullanılan Teknolojiler ve Donanımlar
* **Mikrodenetleyici:** STM32F407, Arduino UNO/NANO
* **Programlama Dili:** C / C++
* **Haberleşme Protokolleri:** CAN Bus, UART(w/ DMA)
* **Sensörler & Aktüatörler:**
    * REV NEO Fırçasız DC Motorlar & Sürücüleri
    * Step motorlar
    * IMU 
 

## Öne Çıkan Çalışmalar
Bu repoda aşağıdaki modüllerin implementasyonları bulunmaktadır:

1.  **CAN Bus Motor Kontrolü:**
    * STM32 üzerinden CAN protokolü ile motor sürücülerine hız ve tork komutlarının gönderilmesi ve alınması.
    * Hata ayıklama ve veri doğrulama mekanizmaları.

2.  **UART Haberleşmesi:**
    * STM32 ve Jetson Orin arasında USB-TTL dönüştürücü ile odometri verisi aktarımı.
    * veri paketleme/Parse etme yapıları.

3.  **Step Motor Kontrolü**
    * Timer'da üretilen pwm sinyalleri ile pozisyon veya görüntü kontrollü step fonksiyonları.

4.  **Güvenlik Algoritmaları:**
    * Acil durdurma (E-Stop) senaryoları ve Watchdog mekanizmaları.
      



## Lisans ve Referans
Bu proje **Rake** çalışmaları kapsamında geliştirilmiştir. Kodlar **MIT Lisansı** ile açık kaynak olarak paylaşılmıştır.
