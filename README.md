This repo contains all code related to the CTR actuation unit in the COMET lab. There are several folders, each with their own substructure. 

data_collection_optitrack: This is an archive of files that we used to do data collect on the OptiTrack system in the SRL. This is mostly outdated code and kept around just for records. 

data_collection_cSharp_api: This folder contains all scripts needed for communication with the NDI Polaris Vega oin PracticePoint. This is the current method of data collection. 

docs: This folder hosts a webpage with information about the OptiTrack system. It has not been updated to include any additional systems. To edit the webpage, you need to make commits directly to the gh-pages branch, which will run the Jenkins auto-compile. 

image_data: Contains images of tubes. These images were used to measure curvature.

kinematics_and_control: This contains all scripts related to the forward, inverse, and velocity kinematics. Forward kineamtics has been thouroughly tested. Inverse and velocity kinematics are still a WIP. 

parameter_filtering: All code related to the Kalman filtering project



For more information about the CTR device, please see this [Notion Page](https://jewel-louse-b60.notion.site/Concentric-Tube-Robot-Educational-Platform-7b1a04f343eb4cfabc8706df61d023d7). 