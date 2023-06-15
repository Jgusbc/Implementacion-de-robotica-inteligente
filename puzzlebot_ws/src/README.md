INSTALLATION: 
	Step 0 
		Open a terminal in this folder 
	Step 1 
		sudo apt update
		sudo apt install -y python3-pip
		pip3 install --upgrade pip
	Step 2
		cd yolov5
		sudo apt install -y libfreetype6-dev
		pip3 install -r requirements.txt
	Step 3 
		cd ~
		sudo apt-get install -y libopenblas-base libopenmpi-dev
		wget https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl -O torch-1.9.0-cp36-cp36m-linux_aarch64.whl
		pip3 install torch-1.8.0-cp36-cp36m-linux_aarch64.whl
	Step 4
		sudo apt install -y libjpeg-dev zlib1g-dev
		git clone --branch v0.9.0 https://github.com/pytorch/vision torchvision
		cd torchvision
		sudo python3 setup.py install
	Step 5 
		# Download pip wheel from location mentioned above
		$ wget https://nvidia.box.com/shared/static/jy7nqva7l88mq9i8bw3g3sklzf4kccn2.whl -O onnxruntime_gpu-1.10.0-cp36-cp36m-linux_aarch64.whl
		$ pip3 install onnxruntime_gpu-1.10.0-cp36-cp36m-linux_aarch64.whl

Activity 3A
	
	Test the default execution through a command (modify the path and adapt it to your system)

	python3 detect.py --weights best.engine --data 'data/tf.yaml' --source /yolov5/samples/data/test/images --name /yolov5/samples/data/test/class1 --device 0

Activity 3B

	You'll find a template in yolo_rec/src/standalone_yolo.py, test it and use it as a baseline to implement your ROS code
