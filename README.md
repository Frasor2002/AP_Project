<div align="center">

# Automated Planning: Theory and Practice Project

</div>

Project for the "Automated Planning: Theory and Practice" course at University of Trento.

# Cloning the repository
To clone the repository use the following commands.
```
git clone https://github.com/Frasor2002/AP_Project.git
cd AP_Project
```

## How to run

Each folder contains a different problem for the project.
Docker needs to be install in order for the experiments to be runned.
### Problem1 execution
To run the experiment the docker image must be built in order to run planutils.
```
cd problem1

docker build --rm  --tag myplanutils . --file Dockerfile

docker run  -v.:/computer -it --privileged --rm myplanutils bash
```
After starting the container, run the following commands in the bash shell:
```
cd ../computer
./run_experiment.sh
```
After a few seconds the logs and plans will be found in the newly created logs directory in the problem1 folder.
### Problem2 execution
To run the experiment the docker image must be built in order to run planutils.
```
cd problem2

docker build --rm  --tag myplanutils . --file Dockerfile

docker run  -v.:/computer -it --privileged --rm myplanutils bash
```
After starting the container, run the following commands in the bash shell:
```
cd ../computer
./run_experiment.sh
```
After a few seconds the logs and plans will be found in the newly created logs directory in the problem2 folder.
### Problem3 execution
The third problem usesa similar image to the first two but also needs java so the building may take a little more.
To run the experiment the docker image must be built in order to run planutils.
Additionaly the singularity image must be put into the folder since its too heavy to be pushed to github.
The file used can be found here: https://drive.google.com/file/d/1j4GX9KYpY2tt5wrloL5aInAejNJ-sMa6/view?usp=sharing.
```
cd problem3

docker build --rm  --tag myplanutils . --file Dockerfile

docker run  -v.:/computer -it --privileged --rm myplanutils bash
```
After starting the container, run the following commands in the bash shell:
```
cd ../computer
./run_experiment.sh
```
After a few seconds the logs and plans will be found in the newly created logs directory in the problem3 folder.
The time taken by the planners this time is manually timed in the bash script.
### Problem4 exection
Problem4 uses the same image of the first three but with an additional planner installation. The process is very similar.
To run the experiment the docker image must be built in order to run planutils.
```
cd problem4

docker build --rm  --tag myplanutils . --file Dockerfile

docker run  -v.:/computer -it --privileged --rm myplanutils bash
```
After starting the container, run the following commands in the bash shell:
```
cd ../computer
./run_experiment.sh
```
After a few seconds the logs and plans will be found in the newly created logs directory in the problem4 folder.
### Problem5 execution
Problem5 integrates with PlanSys2 and so the process to execute experiment is a bit longer. First the docker image must be built and the container run.
```
cd problem5

docker build --rm  --tag ros-humble . --file Dokerfile-humble

docker run -v .:/root/plansys2_ws/src/problem5 -v /tmp/.X11-unix/:/tmp/.X11-unix/ --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --network=host --name ubuntu_bash --env="DISPLAY" --rm -i -t ros-humble bash
```
Inside the container, the launch script must be run to access the plansys terminal.
```
cd plansys2_ws/
./src/problem5/run.experiment.sh 
```
After execution is done a new terminal is opened and a few other commands must be given to get the plan and run it.
```
source src/problem5/launch/commands

run
```


