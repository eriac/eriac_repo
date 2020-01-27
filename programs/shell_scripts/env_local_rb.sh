pip install jupyter 
pip installnumpy
pip install scikit-learn
pip install matplotlib
pip install scipy
pip install pandas 

pip install chainer
#pip install cupy

#openMPI3.0?
cd
wget https://www.open-mpi.org/software/ompi/v3.0/downloads/openmpi-3.0.0.tar.gz
tar -xvf openmpi-*
cd openmpi-*
./configure --prefix="/home/$USER/.openmpi"
make
sudo make install
echo export PATH="$PATH:/home/$USER/.openmpi/bin" >> /home/$USER/.bashrc
echo export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/home/$USER/.openmpi/lib/" >> /home/$USER/.bashrc

pip install mpi4py
