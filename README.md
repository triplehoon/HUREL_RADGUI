# HUREL_RADGUI

## library version check

open3d 0.1.7

spdlog 1.4.0
fmt 6.0.0 (important) -fPIC option needed

# Install enviroment
Ubuntu 18 as in English

# Package installation    

## 폴더 설정      
cd ~    
mkdir codes // to install libs     
mkdir repos  // to install my repo      

## cmake:
sudo apt get install build-essential git     
// you need to install recent cmake.        
sudo apt purge cmake     
cd ~/codes      
wget https://github.com/Kitware/CMake/releases/download/v3.26.3/cmake-3.26.3.tar.gz
tar -xvf cmake*.tar 
cd cmake*
./bootstrap
make -j 49 && sudo make install   
sudo apt install cmake-curses-gui      

## vscode:      
vscode 인터넷에서 설치      

## openmp: 멀티쓰레딩 툴    
기본으로 설치 됨     

## fmt 6.0.0 설치:     
mkdir fmt   
cd fmt   
wget https://github.com/fmtlib/fmt/releases/download/6.0.0/fmt-6.0.0.zip   
unzip fmt-6.0.0.zip   
cd fmt-6.0.0.zip   
mkdir build    
cd build   
ccmake ..    
c, c, g                         
make -j 40 && sudo make install    


## oneapi 설치:   
참고: https://www.intel.com/content/www/us/en/docs/oneapi/installation-guide-linux/2023-0/apt.html#GUID-560A487B-1B5B-4406-BB93-22BC7B526BCD      

//download the key to system keyring     
wget -O- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \  
| gpg --dearmor | sudo tee /usr/share/keyrings/oneapi-archive-keyring.gpg > /dev/null    
     
//add signed entry to apt sources and configure the APT client to use Intel repository:    
echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" | sudo tee /etc/apt/sources.list.d/oneAPI.list        
sudo apt install intel-basekit    

## glfw        
cd codes       
mkdir glfw       
cd glfw        
wget https://github.com/glfw/glfw/releases/download/3.3.8/glfw-3.3.8.zip       
unzip glfw-3.3.8       
cd glfw-3.3.8       
mkdir build       
cd build       
cmake ..       
make -j 40 && sudo make install       

## pcl 설치       
### prerequisit       
flann and qhull       
sudo apt install libflann-dev libqhull-dev libvtk7-dev libboost-all-dev       

cd ~/codes       
mkdir pcl       
cd pcl       
wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.13.0.zip       
cd pcl-1.**       
mkdir build       
cd build       
ccmake ..              
c,c,g,        
make -j 40 && sudo make install       


## open3d 0.17 설치:    
wget https://github.com/isl-org/Open3D/archive/refs/tags/v0.17.0.tar.gz       
tar -xvf v0.17.0.tar.gz      
cd Open3D-0.17.0/     

CMakeList 에서 GLIBCXX_USE_CXX11_ABI ON 으로 다 변경 (혹은 one api 설치 제대로)

mkdir build       
cd build        
ccmake ..         
c,       
use_system_fmt as ON     
use_system_qhull as ON   
use_system_flann as ON    
make -j 40 && sudo make install      

## opencv 4.2: 영상화 툴     
https://webnautes.tistory.com/1186     
링크 참고해서 그대로 실행       

## rtabmap 설치       
cd ~/codes       
mkdir rtabmap       
cd rtabmap       
git clone https://github.com/introlab/rtabmap.git       
cd rtabmap       
mkdir build       
cd build       
ccmake ..       
make -j 40 && sudo make install       


## usb 설정       
sudo apt install libusb-dev       
cd ~/codes       
mkdir cyusb_linux       
cd cyusb_linux       
git clone https://github.com/cpboyd/cyusb_linux.git       

cd cyusb_linux       
make -j 40 && sudo make install       
