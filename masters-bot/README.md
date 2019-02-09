******Autopilot System Design for Natural Disaster******


****Introduction****

This project aims at designing an Autonomous Car that can traverse to the destination at the time of a natural disaster when the roads and surroundings are destroyed. A 2-D LiDAR and a Stereo Camera are the basic sensors used to detect obstacles. A 2-D LiDAR is used for fundamental obstacle detection and avoidance. Even though LiDAR is fast and accurate, it fails to detect glass, smoke, fog and objects below its scanning plane. To avoid the drawbacks of LiDAR during natural disasters, this project also uses a stereo camera. FCN-AlexNet model was trained in such a way that it can detect cracks on the roads, rocks, vegetation, water, people, cars, buildings and other objects in the surrounding area. The sensor fusion of LiDAR and Camera helps in obstacle detection and avoiding them by varying the velocity and steering angle of the car accordingly. The car is  able to reach to the destination by avoiding all the obstacles.

****Launch Project****

To run the car with only Lidar autocore.launch file is used. Use the following commands to launch:

```
cd {Path-to-project/master-bot}
source devel/setup.bash
roslaunch autocore.launch
```

To run the car with integration of Lidar and ZED camera object_classify_display.launch   file is used. Use the following commands to launch:

```
cd {Path-to-project/master-bot}
source devel/setup.bash
roslaunch autocore.launch
```

**How to connect to car using SSH:**

The car used in the project is connected to the WiFi. Before connecting using SSH make sure your laptop’s Wi-Fi is connected to car’s Wi-Fi network. Than use following command to connect your laptop to the car using SSH:

```
ssh ubuntu@192.168.0.100
```


****Installations****

**Installing NVIDIA Driver on the Host**

At this point, JetPack will have flashed the Jetson with the latest L4T BSP, and installed CUDA toolkits to both the Jetson and host PC. However, the NVIDIA PCIe driver will still need to be installed on the host PC to enable GPU-accelerated training. Run the following commands from the host PC to install the NVIDIA driver from the Ubuntu repo:

$ sudo apt-get install nvidia-384	# use nvidia-375 for alternate version
$ sudo reboot
Afer rebooting, the NVIDIA driver should be listed under lsmod:

```
$ lsmod | grep nvidia
nvidia_uvm            647168  0
nvidia_drm             49152  1
nvidia_modeset        790528  4 nvidia_drm
nvidia              12144640  60 nvidia_modeset,nvidia_uvm
drm_kms_helper        167936  1 nvidia_drm
drm                   368640  4 nvidia_drm,drm_kms_helper
```

To verify the CUDA toolkit and NVIDIA driver are working, run some tests that come with the CUDA samples:

```
$ cd /usr/local/cuda/samples
$ sudo make
$ cd bin/x86_64/linux/release/
$ ./deviceQuery
$ ./bandwidthTest --memory=pinned
```

**Installing cuDNN on the Host**

The next step is to install NVIDIA cuDNN libraries on the host PC. Download the libcudnn and libcudnn packages from the NVIDIA cuDNN webpage:

https://developer.nvidia.com/cudnn

Then install the packages with the following commands:

```
$ sudo dpkg -i libcudnn<version>_amd64.deb
$ sudo dpkg -i libcudnn-dev_<version>_amd64.deb
```

**Installing NVcaffe on the Host**

NVcaffe is the NVIDIA branch of Caffe with optimizations for GPU. NVcaffe requires cuDNN and is used by DIGITS for training DNNs. To install it, clone the NVcaffe repo from GitHub, and compile from source, using the caffe-0.15 branch.

note: for this tutorial, NVcaffe is only required on the host (for training). During inferencing phase TensorRT is used on the Jetson and doesn't require caffe.

First clone the caffe-0.15 branch from https://github.com/NVIDIA/caffe

```
$ git clone -b caffe-0.15 https://github.com/NVIDIA/caffe
```

Build caffe with the instructions from here:

http://caffe.berkeleyvision.org/installation.html#compilation

Caffe should now be configured and built. Now edit your user's ~/.bashrc to include the path to your Caffe tree (replace the paths below to reflect your own):

```
export CAFFE_ROOT=/home/dusty/workspace/caffe
export PYTHONPATH=/home/dusty/workspace/caffe/python:$PYTHONPATH
```

Close and re-open the terminal for the changes to take effect.

**Installing DIGITS on the Host**

NVIDIA DIGITS is a Python-based web service which interactively trains DNNs and manages datasets. As highlighed in the DIGITS workflow, it runs on the host PC to create the network model during the training phase. The trained model is then copied from the host PC to the Jetson for the runtime inference phase with TensorRT.

For automated installation, it's recommended to use DIGITS through NVIDIA GPU Cloud, which comes with a DIGITS Docker image that can run on a GPU attached to a local PC or cloud instance. Alternatively, to install DIGITS from source, first clone the DIGITS repo from GitHub:

```
$ git clone https://github.com/nvidia/DIGITS
```

Then complete the steps under the Building DIGITS documentation.

https://github.com/NVIDIA/DIGITS/blob/digits-6.0/docs/BuildDigits.md

Starting the DIGITS Server
Assuming that your terminal is still in the DIGITS directory, the webserver can be started by running the digits-devserver Python script:

```
$ ./digits-devserver 
  ___ ___ ___ ___ _____ ___
 |   \_ _/ __|_ _|_   _/ __|
 | |) | | (_ || |  | | \__ \
 |___/___\___|___| |_| |___/ 5.1-dev

2017-04-17 13:19:02 [INFO ] Loaded 0 jobs.`
```

DIGITS will store user jobs (training datasets and model snapshots) under the digits/jobs directory.


*To access the interactive DIGITS session, open your web browser and navigate to 0.0.0.0:5000.*

note: by default the DIGITS server will start on port 5000, but the port can be specified by passing the --port argument to the digits-devserver script.


**Building from Source on Jetson**

Cloning the Repo
To obtain the repository, navigate to a folder of your choosing on the Jetson. First, make sure git and cmake are installed locally:

```
$ sudo apt-get install git cmake
```

Then clone the jetson-inference repo:

```
$ git clone https://github.com/dusty-nv/jetson-inference
$ cd jetson-inference
$ git submodule update --init
```

Configuring with CMake
When cmake is run, a special pre-installation script (CMakePreBuild.sh) is run and will automatically install any dependencies.

```
$ mkdir build
$ cd build
$ cmake ../
```

note: the cmake command will launch the CMakePrebuild.sh script which asks for sudo while making sure prerequisite packages have been installed on the Jetson. The script also downloads the network model snapshots from web services.

Compiling the Project
Make sure you are still in the jetson-inference/build directory, created above in step #2.

```
$ cd jetson-inference/build			# omit if pwd is already /build from above
$ make
```

Depending on architecture, the package will be built to either armhf or aarch64, with the following directory structure:

```
|-build
   \aarch64		    (64-bit)
      \bin			where the sample binaries are built to
      \include		where the headers reside
      \lib			where the libraries are build to
   \armhf           (32-bit)
      \bin			where the sample binaries are built to
      \include		where the headers reside
      \lib			where the libraries are build to
      
```
binaries residing in aarch64/bin, headers in aarch64/include, and libraries in aarch64/lib.

Digging Into the Code
For reference, see the available vision primitives, including imageNet for image recognition and detectNet for object localization.

```
/**
 * Image recognition with GoogleNet/Alexnet or custom models, using TensorRT.
 */
class imageNet : public tensorNet
{
public:
	/**
	 * Network choice enumeration.
	 */
	enum NetworkType
	{
		ALEXNET,
		GOOGLENET
	};

	/**
	 * Load a new network instance
	 */
	static imageNet* Create( NetworkType networkType=GOOGLENET );
	
	/**
	 * Load a new network instance
	 * @param prototxt_path File path to the deployable network prototxt
	 * @param model_path File path to the caffemodel
	 * @param mean_binary File path to the mean value binary proto
	 * @param class_info File path to list of class name labels
	 * @param input Name of the input layer blob.
	 */
	static imageNet* Create( const char* prototxt_path, const char* model_path, const char* mean_binary,
							 const char* class_labels, const char* input="data", const char* output="prob" );

	/**
	 * Determine the maximum likelihood image class.
	 * @param rgba float4 input image in CUDA device memory.
	 * @param width width of the input image in pixels.
	 * @param height height of the input image in pixels.
	 * @param confidence optional pointer to float filled with confidence value.
	 * @returns Index of the maximum class, or -1 on error.
	 */
	int Classify( float* rgba, uint32_t width, uint32_t height, float* confidence=NULL );
};
```
Both inherit from the shared tensorNet object which contains common TensorRT code.



****Semantic Segmentation****

As an example of image segmentation, we'll work with an aerial drone dataset that separates ground terrain from the sky. The dataset is in First Person View (FPV) to emulate the vantage point of a drone in flight and train a network that functions as an autopilot guided by the terrain that it senses.

To download and extract the dataset, run the following commands from the host PC running the DIGITS server:

```
$ wget --no-check-certificate https://nvidia.box.com/shared/static/ft9cc5yjvrbhkh07wcivu5ji9zola6i1.gz -O NVIDIA-Aerial-Drone-Dataset.tar.gz

HTTP request sent, awaiting response... 200 OK
Length: 7140413391 (6.6G) [application/octet-stream]
Saving to: ‘NVIDIA-Aerial-Drone-Dataset.tar.gz’

NVIDIA-Aerial-Drone-Datase 100%[======================================>]   6.65G  3.33MB/s    in 44m 44s 

2017-04-17 14:11:54 (2.54 MB/s) - ‘NVIDIA-Aerial-Drone-Dataset.tar.gz’ saved [7140413391/7140413391]

$ tar -xzvf NVIDIA-Aerial-Drone-Dataset.tar.gz 
```

The dataset includes various clips captured from flights of drone platforms, but the one we'll be focusing on in this tutorial is under FPV/SFWA. Next we'll create the training database in DIGITS before training the model.

**Generating Pretrained FCN-Alexnet for transfer learning**

Fully Convolutional Network (FCN) Alexnet is the network topology that we'll use for segmentation models with DIGITS and TensorRT. See this Parallel ForAll article about the convolutionalizing process. A new feature to DIGITS5 was supporting segmentation datasets and training models. A script is included with the DIGITS semantic segmentation example which converts the Alexnet model into FCN-Alexnet. This base model is then used as a pre-trained starting point for training future FCN-Alexnet segmentation models on custom datasets.

To generate the pre-trained FCN-Alexnet model, open a terminal, navigate to the DIGITS semantic-segmantation example, and run the net_surgery script:

```
$ cd DIGITS/examples/semantic-segmentation
$ ./net_surgery.py
Downloading files (this might take a few minutes)...
Downloading https://raw.githubusercontent.com/BVLC/caffe/rc3/models/bvlc_alexnet/deploy.prototxt...
Downloading http://dl.caffe.berkeleyvision.org/bvlc_alexnet.caffemodel...
Loading Alexnet model...
...
Saving FCN-Alexnet model to fcn_alexnet.caffemodel

```


FCN-Alexnet Patches for TensorRT
There exist a couple non-essential layers included in the original FCN-Alexnet which aren't supported in TensorRT and should be deleted from the deploy.prototxt included in the snapshot.

At the end of deploy.prototxt, delete the deconv and crop layers:

```
layer {
  name: "upscore"
  type: "Deconvolution"
  bottom: "score_fr"
  top: "upscore"
  param {
    lr_mult: 0.0
  }
  convolution_param {
    num_output: 21
    bias_term: false
    kernel_size: 63
    group: 21
    stride: 32
    weight_filler {
      type: "bilinear"
    }
  }
}
layer {
  name: "score"
  type: "Crop"
  bottom: "upscore"
  bottom: "data"
  top: "score"
  crop_param {
    axis: 2
    offset: 18
  }
}

```
And on line 24 of deploy.prototxt, change pad: 100 to pad: 0. Finally copy the fpv-labels.txt and fpv-deploy-colors.txt from the aerial dataset to your model snapshot folder on Jetson. Your FCN-Alexnet model snapshot is now compatible with TensorRT. Now we can run it on Jetson and perform inference on images.

Running Segmentation Models on Jetson
To test a custom segmentation network model snapshot on the Jetson, use the command line interface to test the segnet-console program.

First, for convienience, set the path to your extracted snapshot into a $NET variable:

```
$ NET=20170421-122956-f7c0_epoch_5.0

$ ./segnet-console drone_0428.png output_0428.png \
--prototxt=$NET/deploy.prototxt \
--model=$NET/snapshot_iter_22610.caffemodel \
--labels=$NET/fpv-labels.txt \
--colors=$NET/fpv-deploy-colors.txt \
--input_blob=data \ 
--output_blob=score_fr

```

**Annotation on own Dataset**

For this project we have collected and annotated images by our own. For annotation PixelAnnotation tool is used.

config.json file for pixelAnnotation tool is added for the reference :

```
{
    "labels": {
        "cracks": {
            "categorie": "construction",
            "color": [
                243,
                121,
                0
            ],
            "id": 29,
            "id_categorie": 2,
            "name": "cracks"
        },
		"vehicle": {
            "categorie": "vehicle",
            "color": [
                0,
                0,
                142
            ],
            "id": 26,
            "id_categorie": 7,
            "name": "vehicle"
        },
		"person": {
            "categorie": "human",
            "color": [
                220,
                20,
                60
            ],
            "id": 24,
            "id_categorie": 6,
            "name": "person"
        },
		"road": {
            "categorie": "road",
            "color": [
                128,
                64,
                128
            ],
            "id": 7,
            "id_categorie": 1,
            "name": "road"
        },
		"sky": {
            "categorie": "sky",
            "color": [
                70,
                130,
                180
            ],
            "id": 23,
            "id_categorie": 5,
            "name": "sky"
        },
		"water": {
            "categorie": "nature",
            "color": [
                85,
                255,
                255
            ],
            "id": 14,
            "id_categorie": 4,
            "name": "water"
        },
		"vegetation": {
            "categorie": "nature",
            "color": [
                107,
                142,
                35
            ],
            "id": 21,
            "id_categorie": 4,
            "name": "vegetation"
        },
		"other": {
            "categorie": "void",
            "color": [
                170,
                85,
                127
            ],
            "id": 35,
            "id_categorie": 0,
            "name": "other"
        },
		"building": {
            "categorie": "construction",
            "color": [
                70,
                70,
                70
            ],
            "id": 11,
            "id_categorie": 2,
            "name": "building"
        }
		
    }
}

```

