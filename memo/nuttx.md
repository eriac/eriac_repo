# 環境構築
sudo apt-get install -y git autoconf libtool gperf flex bison libncurses-dev gcc-arm-none-eabi make pkg-config

mkdir NuttX && cd NuttX
git clone https://bitbucket.org/nuttx/nuttx
git clone https://bitbucket.org/nuttx/apps
git clone https://bitbucket.org/nuttx/tools

cd tools/kconfig-frontends
./configure
make
sudo make install
sudo /sbin/ldconfig

# STLINK
sudo apt-get install -y libusb-1.0-0-dev cmake screen
sudo apt-get install dh-autoreconf
git clone https://github.com/texane/stlink stlink.git
cd stlink
make
sudo cp build/Release/st-* /usr/local/bin
sudo cp etc/udev/rules.d/49-stlinkv* /etc/udev/rules.d/

# F3用にビルド
cd NuttX/nuttx
make distclean
./tools/configure.sh stm32f3discovery/nsh
make menuconfig
platformを選択したら、exitで抜ける
make

# bluepill(F103)ようにbuild
tools/configure.sh -l stm32f103-minimum/usbnsh
make

# LPC1768用にビルド
cd nuttx
tools/configure.sh mbed/nsh
make menuconfig
make CROSSDEV=arm-none-eabi-
cp nuttx.bin /media/ubuntu/MBED/
screen /dev/ttyACM0 115200
(ctl+a k)

# st 書き込み
st-flash write nuttx.bin 0x8000000

# sim
cd NuttX/nuttx
make distclean
./tools/configure.sh sim/nsh
  Copy files
  Refreshing...
make clean
make
./nuttx

# 参考
全般
http://nopnop2002.webcrow.jp/NUTTX-STM32F3/NUTTX-Development.html

opencdcで書き込み
https://tsubakicraft.wordpress.com/2019/02/09/nuttx%e3%82%92stm32f4-discovery%e3%81%a7%e5%8b%95%e3%81%8b%e3%81%97%e3%81%a6%e3%81%bf%e3%82%8b/

STLINK driver
https://daichiahl.wordpress.com/2016/01/23/ubuntuでstm32-armマイコンの開発環境を整える/