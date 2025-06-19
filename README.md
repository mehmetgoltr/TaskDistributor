# TaskDistributor
Bu proje, iki mobil robotun GPS verilerini kullanarak merkezi bir bilgisayardan görev aldığı ve hedefe yöneldiği basit ama işlevsel bir çoklu robot koordinasyon sistemini simüle eder.

🛠️ Sistemde yer alan bileşenler:

2 adet Raspberry Pi 5 (robotlar)

Her robotta bir GPS modülü ve bir ultrasonik sensör

Görevleri dağıtan merkezi bilgisayar

ROS 2 (Jazzy) ile iletişim

✅ Temel Özellikler:

Gerçek zamanlı GPS verisi okuma (gps_reader.py)

Merkezi görev dağıtımı (task_distributor.py)

Engel algılama ve motor kontrolü ile hedefe yönelme (robot_controller.py)

🧰 Donanım Gereksinimleri
2 × Raspberry Pi 5 (Ubuntu yüklü)

2 × GY-NEO6MV2 GPS modülü

2 × HC-SR04 Ultrasonik sensör

4 × DC motor + Motor sürücü

1 × Merkezi Bilgisayar (Ubuntu veya Windows)

Güç kaynağı, jumper kablolar, breadboard vb.

🛠️ Yazılım Kurulumu
1. Projeyi Klonla
bash
Kopyala
Düzenle
git clone https://github.com/kullanici_adi/coklu-robot-gps-task.git
cd coklu-robot-gps-task
2. Bağımlılıkları Yükle
Robotlarda (RPi5):

bash
Kopyala
Düzenle
sudo apt update
sudo apt install python3-serial python3-gpiod
Merkezi Bilgisayarda:

bash
Kopyala
Düzenle
sudo apt update
sudo apt install python3-colcon-common-extensions
3. ROS 2 Ortamı
Tüm cihazlarda:

bash
Kopyala
Düzenle
source /opt/ros/jazzy/setup.bash
🚦 Kullanım
Robotlarda (Her Raspberry Pi'de)
GPS modülünü USB üzerinden bağlayın.

Ultrasonik sensörü GPIO pinlerine bağlayın.

GPS veri yayıncısını başlatın:

bash
Kopyala
Düzenle
python3 gps_reader.py
Robot kontrolcüsünü başlatın:

bash
Kopyala
Düzenle
python3 robot_controller.py
Merkezi Bilgisayarda
Robotlara ping atarak iletişim kurulduğundan emin olun.

Görev dağıtıcısını başlatın:

bash
Kopyala
Düzenle
python3 task_distributor.py
📁 Dosya Yapısı
graphql
Kopyala
Düzenle
coklu-robot-gps-task/
├── gps_reader.py           # GPS verisi okuma ve ROS yayını
├── robot_controller.py     # Motor kontrolü, engel algılama, yön bulma
├── task_distributor.py     # Kutuların konumuna göre görev atama
├── config/
│   └── kutular.json        # Kutuların (görevlerin) GPS konumu
└── README.md
📷 Örnek Çıktılar
task_distributor.py (Merkezi PC Terminali):

csharp
Kopyala
Düzenle
[INFO] Task Distributor başlatıldı.
[INFO] Robot1 konumu: 41.0253, 29.0032
[INFO] Robot2 konumu: 41.0241, 29.0029
[INFO] Görev atandı: box1 -> Robot1
[INFO] Görev atandı: box2 -> Robot2
[INFO] Görevler başarıyla gönderildi.
robot_controller.py (RPi5 Terminali):

less
Kopyala
Düzenle
[INFO] Konum güncellendi: (41.0253, 29.0032)
[INFO] Hedefe uzaklık: 4.6 metre
[INFO] İleri gidiliyor...
[INFO] Engel tespit edildi! Sağa kaçılıyor...
🎯 Proje Amacı
Bu projenin amacı, GPS tabanlı lokalizasyon ve merkezi karar verme yoluyla görevlerin çoklu robotlara atanmasını sağlamaktır. Robotların birbirinden bağımsız hareket ederken görev paylaşımı yapabildiği bir temel senaryo uygulanmıştır.

🤝 Katkı Sağla
Her türlü katkıya açığız. Kodlara katkı sağlamak veya geri bildirimde bulunmak isterseniz lütfen pull request gönderin.

📄 Lisans
MIT Lisansı
