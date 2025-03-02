Use miniconda to setup python env

Step 1 – Prerequsities
    sudo apt install -y make build-essential git

Step 2 – Download and install miniconda
    mkdir -p ~/miniconda3
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
    bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
    rm ~/miniconda3/miniconda.sh

    source ~/miniconda3/bin/activate
    # initialize conda on all available shells 
    conda init --all

Step 4 – Install required python packages 

    cd /home/qd/CarlaSimulator/PythonClient
    conda env create -f environment.yml
    conda activate controller2d

    pip install Pillow>=3.1.2
    pip install numpy>=1.14.5
    pip install protobuf==3.6.0
    pip install pygame>=1.9.4
    pip matplotlib==2.2.2
    pip install matplotlib==2.2.2
    pip install future>=0.16.0
    pip install scipy>=0.17.0

    ## 1: Run autopilot to travel town 1 

        ###  In terminal 1:  Start Carla server with /Game/Maps/Town01, -fps=20
        cd /home/qd/CarlaSimulator
        ./CarlaUE4.sh /Game/Maps/Town01 -windowed -carla-server -benchmark -fps=20
        
        ###  In terminal 2:         
        cd /home/qd/CarlaSimulator/PythonClient
        python manual_control.py --autopilot --map

        ##########################################
        ### autopilot demo works on March 2nd! ###
        ##########################################

    ### 2: Other built-in towns and tracks in Carla
        # /Game/Maps/Town01
        # /Game/Maps/Town02
