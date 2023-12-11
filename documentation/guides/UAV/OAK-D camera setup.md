# Camera dependencies

Do the following steps.

```bash
sudo wget -qO- https://docs.luxonis.com/install_depthai.sh | bash 
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash 

sudo apt install python3-pip 
sudo git clone --recursive https://github.com/luxonis/depthai.git 
sudo git clone --recursive https://github.com/luxonis/depthai-python.git 
 
cd depthai 
python3 install_requirements.py 
```
