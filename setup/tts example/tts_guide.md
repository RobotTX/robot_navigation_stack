### festival ###
#how to use
* festival english - echo "text" | festival --tts
#installment
sudo apt-get install festival
* female voice
mkdir us1  
cd us1
wget http://www.speech.cs.cmu.edu/cmu_arctic/packed/cmu_us_slt_arctic-0.95-release.tar.bz2
tar xf cmu_us_slt_arctic-0.95-release.tar.bz2
sudo mv cmu_us_slt_arctic /usr/share/festival/voices/english/cmu_us_slt_arctic_clunits
sudo vi /etc/festival.scm
add (set! voice_default 'voice_cmu_us_slt_arctic_clunits) to the end

### ekho ###
#how to use
* ekho chinese - ekho "string"
ekho -v Cantonese "煲冬瓜" #用粤语说
#installment
sudo apt-get install libespeak-dev
sudo apt-get install libsndfile1-dev 
sudo apt-get install libpulse-dev
sudo apt-get install libncurses5-dev (required by --enable-festival)
sudo apt-get install libestools-dev (optionally required by --enable-festival)
sudo apt-get install festival-dev  (optionally required by --enable-festival)
sudo apt-get install libvorbis-dev (optional)
sudo apt-get install libmp3lame-dev (optional)
download from https://sourceforge.net/projects/e-guidedog/files/Ekho/7.5/ekho-7.5.tar.xz/download
cd ekho-7.5
./configure
make
sudo make install

### mp3 ###
#how to use
* play mp3/wav - sudo play file.wav
#installment
sudo apt install sox
sudo apt install libsox-fmt-all