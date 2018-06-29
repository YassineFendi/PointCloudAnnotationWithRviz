# PointCloudAnnotationWithRviz
Annotation Tool for point cloud based on Velodyne VLP-32C


Comparing with labeling 2D images, annotate 3D point clouds is more difficult due to the complexity of visualization tools. Lack of annotated 3D point clouds in large scale is one of the bottlenecks for improving the recognition of 3D point cloud. This work is an annotation tool based on the well-known visualization framework RVIZ. This tool is enabled for manually labeling point cloud based on an Euclidean clustering algorithm (which is not given here). basically this tool is developed to create Datasets for the classification of categories ( car,pedistrian...) for autonomous cars.


The input is a PCD file with fields : x,y,z,rgb intensity and id.

The output is a PCD file with fields : x,y,z,rxy,ith,jth,intensity and id. (rxy,ith,jth are not considered for now).

Previously, a ground segmentation and objects clustering program was implemented to the bag that contain Data. In the proposed annotation tool, we used only clustered data of non-ground objects. We convert the bag file data that contain the clustered objects to PCD files. When loading a PCD file, the program convert all the point cloud to interactive markers. So that we can interact with the point cloud on Rviz.
The user can select any object on the point cloud, with a simple mouse click in Rviz interface. The selected object will be automatically saved separately.     
Multiple objects can be selected in the same scene and can be saved to the same category. 

Each PCD file contains a frame of the clustered bag file. That allow us to select easily any object but also to have different views and poses of the same object. This will help us to have a bigger dataset.
The selected object is saved as a PCD file. So we will have a new point cloud with a smaller size. We then can quickly and efficiently extract features since our ultimate goal is to classify objects into different categories (car, pedestrian, truck, traffic sign...) for the perception of autonomous cars.

To run the programm from terminal : 
1 - rosrun pack main_node
2 - run Rviz and change the fixed frame to base_link
