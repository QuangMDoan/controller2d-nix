Assumption: Carla is installed (untar) to ~/CarlaSimulator directory

Step 0 – Prerequsities
    sudo apt install -y make build-essential git

Step 1 – Install Carla for Ubuntu
    - Download CarlaUE4Ubuntu.tar.gz from https://drive.google.com/drive/folders/1P6rHxOeH2RBNOqA0q2R03WYrNUPysccx
    - Move CarlaUE4Ubuntu.tar.gz to home directory (~/)
    - Run tar -xzf CarlaUE4Ubuntu.tar.gz
    
Step 2 – Download and install miniconda for PythonClient
    mkdir -p ~/miniconda3
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
    bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
    rm ~/miniconda3/miniconda.sh

    source ~/miniconda3/bin/activate

    # initialize conda on all available shells 
    conda init --all

Step 4 – Install required packages for Carla python client

    cd ~/CarlaSimulator
    rm -rf ~/CarlaSimulator/PythonClient

    cd ~/CarlaSimulator
    git clone git@github.com:QuangMDoan/controller2d-nix.git PythonClient

    conda env create -f environment.yml
    conda activate controller2d
    python -m pip install -r requirements.txt

    ## 1: Run autopilot to travel town 1 

        ###  In terminal 1:  Start Carla server with /Game/Maps/Town01, -fps=20
        cd ~/CarlaSimulator
        ./CarlaUE4.sh /Game/Maps/Town01 -windowed -carla-server -benchmark -fps=20
        
        ###  In terminal 2:         
        cd ~/CarlaSimulator/PythonClient
        python manual_control.py --autopilot --map

        ##########################################
        ### autopilot demo works on March 2nd! ###
        ##########################################

    ### 2: Other built-in towns for demo in Carla
        # /Game/Maps/Town01
        # /Game/Maps/Town02

    ### 3: Run hand-written controller implemented in controller2d.py
        cd ~/CarlaSimulator
        ./CarlaUE4.sh /Game/Maps/RaceTrack -windowed -carla-server -benchmark -fps=30

        cd ~/CarlaSimulator/PythonClient/Course1FinalProject
        conda activate controller2d
        python module_7.py

        ## Once Carla traveled through all racetrack waypoints or time is up 
        ### We can assess the quality of the controller2d by running the grader below 

        cd ~/CarlaSimulator/PythonClient/Course1FinalProject
        conda activate controller2d
        python grade_c1m7.py racetrack_waypoints.txt controller_output/trajectory.txt