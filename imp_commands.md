1) ~/flingbot$ python run_sim.py --eval --tasks flingbot-normal-rect-eval.hdf5 --load flingbot.pth --num_processes 1 --gui --dump_visualizations

2) difference
/usr/lib/cuda/version.txt belongs to the version installed as part of your Linux distribution;
/usr/local/cuda has the version of CUDA installed manually, independently of the distribution by you or your system administrator.

3) $ python -c "import torch; print(torch.cuda.is_available())"

4.1) build image
$ docker build -t flingbot .


4) $ export FLINGBOT_PATH=${PWD}
 $ docker run -v $FLINGBOT_PATH:/workspace/flingbot -v /home/hong_data/miniforge3:/home/hong_data/miniforge3 --runtime=nvidia --gpus all --shm-size=24gb  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -it flingbot 

5) In docker container:
		a. # export PATH=/home/hong_data/miniforge3/bin:$PATH
		b. # conda init bash
		c. # . ~/.bashrc
		d. # conda activate flingbot && cd flingbot
		e. # . ./prepare.sh && ./compile.sh

6) $ sudo systemctl daemon-reload
(flingbot) hong_data@hongdata:~/flingbot$ sudo systemctl restart docker


7) $ conda activate flingbot && cd ~/flingbot

8) $ python environment/tasks.py --path new-normal-rect-tasks.hdf5 --num_processes 8 --num_tasks 10 --cloth_type square --min_cloth_size 64 --max_cloth_size 104



8.1) $docker run \
  -v /home/hong_data/flingbot/PyFlex:/workspace/PyFleX \
  -v /home/hong_data/miniforge3:/home/hong_data/miniforge3 \
  -it yunzhuli/pyflex_16_04_cuda_9_1:latest


9) Looks like everything is running able to see the cloth in softgym, but

(flingbot) hong_data@hongdata:~/flingbot$ python environment/tasks.py --path new-normal-rect-tasks.hdf5 --num_processes 8 --num_tasks 10 --cloth_type square --min_cloth_size 64 --max_cloth_size 104
2024-11-09 18:49:39,790	INFO worker.py:1752 -- Started a local Ray instance.
:job_id:01000000
:job_id:01000000
:task_name:generate_tasks_helper
:task_name:generate_tasks_helper
Flex: Could not open CUDA driver, ensure you have the latest driver (nvcuda.dll) installed and an NVIDIA GPU present on the system - ./../../../src/cuda/util.cpp:145
Flex: Could not initialize CUDA driver - ../../../src/cuda/flex.cu:4624
Could not initialize Flex, exiting.
(flingbot) hong_data@hongdata:~/flingbot$ 





1) Few more points after building Softgym:

2) When compiling if you run into pybind11 issue:
Check whether your python version is supported with pyblind11. conda install pybind11 installed the latest 2.13 pybind11 which no longer supports python =3.6. Hence installed pybind version 2.6

3) $ docker run \
    -v /home/hong_data/Pranay/clothbot/softgym:/workspace/softgym \
    -v /home/hong_data/miniforge3:/home/hong_data/miniforge3 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -it xingyu/softgym:latest bash


4) Training Flingbot
 python run_sim.py --tasks flingbot-rect-train.hdf5 --num_processes 8 --log flingbot-train-from-scratch --action_primitives fling --dump_visualizations


5) Evaluation on Tshirt
$ python run_sim.py --eval --tasks flingbot-shirt-eval.hdf5 --load latest_ckpt.pth --num_processes 4 --dump_visualizations

6) Inference on real-world Cloth Images
$ python run_real_world_inference.py  --eval --load flingbot.pth