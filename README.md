# ChangeSim: Towards End-to-End Online Scene Change Detection inIndustrial Indoor Environments


This repository provides ChangeSim dataset, codes and files for evaluation. Please refer to [paper](arxiv주소) for more information about the dataset.


## Dataset download

The data is divided into train/test set and reference/query. 

[Reference_Sequence_Train]()(00.0 GB)

[Reference_Sequence_Test](https://kaistackr-my.sharepoint.com/:u:/g/personal/jhyuk_kaist_ac_kr/Ecy15_DweZ9EkNdKOFueMn0Bxsq7XkAYNtgHZ-klPZ9M3A?e=5OBH4y)(30.2 GB)

[Query_Sequence_Train](https://kaistackr-my.sharepoint.com/:u:/g/personal/jhyuk_kaist_ac_kr/EW1W0h1RzEhBrTUn7zcx2vUBw-W0yQ2JZGB2rREdeICEjw?e=0KRm3J)(42.8 GB)

[Query_Sequence_Test](https://kaistackr-my.sharepoint.com/:u:/g/personal/jhyuk_kaist_ac_kr/Ecy15_DweZ9EkNdKOFueMn0Bxsq7XkAYNtgHZ-klPZ9M3A?e=5J9Kd3)(30.3 GB)

### Data directory structure
```
ROOT
|
--- ENV_NAME_0                             # environment folder
|       |
|       ---- Easy                          # difficulty level
|       |      |
|       |      ---- P000                   # trajectory folder
|       |      |      |
|       |      |      +--- depth_left      # 000000_left_depth.npy - 000xxx_left_depth.npy
|       |      |      +--- depth_right     # 000000_right_depth.npy - 000xxx_right_depth.npy
|       |      |      +--- flow            # 000000_000001_flow/mask.npy - 000xxx_000xxx_flow/mask.npy
|       |      |      +--- image_left      # 000000_left.png - 000xxx_left.png 
|       |      |      +--- image_right     # 000000_right.png - 000xxx_right.png 
|       |      |      +--- seg_left        # 000000_left_seg.npy - 000xxx_left_seg.npy
|       |      |      +--- seg_right       # 000000_right_seg.npy - 000xxx_right_seg.npy
|       |      |      ---- pose_left.txt 
|       |      |      ---- pose_right.txt
|       |      |  
|       |      +--- P001
|       |      .
|       |      .
|       |      |
|       |      +--- P00K
|       |
|       +--- Hard
|
+-- ENV_NAME_1
.
.
|
+-- ENV_NAME_N
```
