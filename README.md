# MZI Task Distributor
Bu çalışma, ROS 2 ara katman yazılımı kullanılarak geliştirilen, GPS koordinasyonuna dayalı, düşük maliyetli bir çoklu robot sisteminin tasarım ve uygulamasını sunmaktadır. 

Sistemde yer alan bileşenler:

En az 2 adet Raspberry Pi 5

Her robotta bir GPS modülü ve bir ultrasonik sensör

Görevleri dağıtan merkezi bilgisayar

ROS 2 (Jazzy) ile iletişim


# ✅ Temel Özellikler:

Gerçek zamanlı GPS verisi okuma (gps_reader.py)

Merkezi görev dağıtımı (task_distributor.py)

Engel algılama ve motor kontrolü ile hedefe yönelme (robot_controller.py)

## Donanım Gereksinimleri
Raspberry Pi 5 (Ubuntu yüklü)

GY-NEO6MV2 GPS modülü

HC-SR04 Ultrasonik sensör

DC motor + Motor sürücü

Merkezi Bilgisayar (Ubuntu)


## Yazılım Kurulumu
1. Projeyi Klonlayın
 ``` 
git clone https://github.com/mehmetgoltr/TaskDistributor
cd TaskDistributor
 ``` 
2. Bağımlılıkları Yükleyin
Robotlarda (RPi5):
 ``` 
sudo apt update
sudo apt install python3-serial python3-gpiod
 ``` 
Merkezi Bilgisayarda:
 ``` 
sudo apt update
sudo apt install python3-colcon-common-extensions
 ``` 
3. ROS 2 Ortamı
Tüm cihazlarda:
 ``` 
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30
export ROS_DISCOVERY_SERVER=<Cihazın IP Adresi>
 ```

## Kullanım
GPS veri yayıncısını başlatın:

 ``` 
python3 gps_reader.py
 ```

Farklı bir terminalde Robot kontrolcüsünü başlatın:
 ``` 
python3 robot_controller.py
 ``` 
Merkezi Bilgisayarda:
 ``` 
python3 task_distributor.py
 ``` 
📁 Dosya Yapısı
 ``` 
coklu-robot-gps-task/
├── gps_reader.py           # GPS verisi okuma ve ROS yayını
├── robot_controller.py     # Motor kontrolü, engel algılama, yön bulma
├── task_distributor.py     # Kutuların konumuna göre görev atama
├── config/
│   └── kutular.json        # Kutuların (görevlerin) GPS konumu
└── README.md
 ``` 

## Proje Amacı
Bu projenin amacı, GPS tabanlı lokalizasyon ve merkezi karar verme yoluyla görevlerin çoklu robotlara atanmasını sağlamaktır. Robotların birbirinden bağımsız hareket ederken görev paylaşımı yapabildiği bir temel senaryo uygulanmıştır. 
Bu proje, Marmara Üniversitesi Teknoloji Fakültesi Elektrik-Elektronik Mühendisliği Bölümü kapsamında, EEM7060.1 Çoklu Robot Sistemleri dersi için geliştirilmiştir.

## İletişim
Mehmet Göl

Şule Zeynep Aydın

İzemnur Budak

Dr. Savaş Öztürk

📄 Lisans
MIT Lisansı
