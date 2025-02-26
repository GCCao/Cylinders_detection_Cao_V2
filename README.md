# Cylinders_detection_Cao
 
 This MATLAB implementation presents an automated cylinder detection algorithm for complex pipelines from unstructured point clouds. The method involves iterative clustering segmentation to simplify data processing, reliable candidate cylinder estimation using three-point random sampling, high-precision fitting for accurate localization, and multi-filtering mechanisms to minimize false detections. It achieves high precision, recall, and F1 scores, outperforming existing methods. This code is designed for reverse engineering and industrial applications, providing a robust solution for cylinder detection in various pipeline scenarios.

All the code is written in the MATLAB environment. The Cylinders_detection_Cao directory contains three folders: adjustment_Cylindrical_Surfaces, detect_cylinder_from_Cao_Github, and PipelinesData_Cao. You only need to run the demo_cylinder_detection_test_Cao.m script in the detect_cylinder_from_Cao_Github folder to observe the detection results. The PipelinesData_Cao folder contains the relevant datasets.

Citation:
Cao, G. (2025). Automated Detection of Cylindrical Structures in Complex Pipelines Using Iterative Point Cloud Segmentation and High Precision Fitting. The Visual Computer. 