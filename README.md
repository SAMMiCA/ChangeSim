# ChangeSim: Towards End-to-End Online Scene Change Detection inIndustrial Indoor Environments


This repository provides ChangeSim dataset, codes and files for evaluation. Please refer to [paper](arxiv주소) for more information about the dataset.


## Dataset download

The data is divided into train/test set and reference/query. 

[Reference_Sequence_Train](https://kaistackr-my.sharepoint.com/:u:/g/personal/jhyuk_kaist_ac_kr/Efxa73-liStOkSKpXVC4ARABY11jS0LP8O3HzhOdZ4fNNA?e=aYTREN)(52.8 GB)

[Reference_Sequence_Test](https://kaistackr-my.sharepoint.com/:u:/g/personal/jhyuk_kaist_ac_kr/EQy9p83-nvlMtY90aAnEItkBfJ7-1c3vpAe4dv5xsVlwKA?e=1OAD2l)(30.2 GB)

[Query_Sequence_Train](https://kaistackr-my.sharepoint.com/:u:/g/personal/jhyuk_kaist_ac_kr/EW1W0h1RzEhBrTUn7zcx2vUBw-W0yQ2JZGB2rREdeICEjw?e=0KRm3J)(42.8 GB)

[Query_Sequence_Test](https://kaistackr-my.sharepoint.com/:u:/g/personal/jhyuk_kaist_ac_kr/Ecy15_DweZ9EkNdKOFueMn0Bxsq7XkAYNtgHZ-klPZ9M3A?e=5J9Kd3)(30.3 GB)

### Data directory structure
```
Reference_Sequence_
|
--- Warehouse_0                              # Environment folder
|       |
|       ---- Seq_0                           # Sequece
|       |      |
|       |      +--- rgb                      # 0.png - xxxx.png      
|       |      +--- depth                    # 0.png - xxxx.png
|       |      +--- semantic_segmentation    # 0.png - xxxx.png     
|       |      ---- raw                   
|       |      |     |
|       |      |     +--- rgb                # 0.png - xxxx.png
|       |      |     +--- depth              # 0.png - xxxx.png
|       |      |     ---- poses.g2o 
|       |      |     ---- rtabmap.yaml
|       |
|       +--- Seq_1
|
+-- Warehouse_1
.
.
+-- Warehouse_N



Query_Sequence_
|
--- Warehouse_0                              # Environment folder
|       |
|       ---- Seq_0                           # Sequece
|       |      |
|       |      +--- rgb                      # 0.png - xxxx.png      
|       |      +--- depth                    # 0.png - xxxx.png
|       |      +--- semantic_segmentation    # 0.png - xxxx.png
|       |      +--- change_segmentation      # 0.png - xxxx.png
|       |      +--- pose                     # 0.txt - xxxx.txt
|       |      ---- t0                   
|       |      |     |
|       |      |     +--- rgb                # 0.png - xxxx.png
|       |      |     +--- depth              # 0.png - xxxx.png
|       |      |     +--- idx                # 0.txt - xxxx.txt
|       |      ---- cloud_map.ply
|       |      ---- trajectory.txt
|       |
|       +--- Seq_0_dust
|       .
|       .
+-- Warehouse_1
.
.
+-- Warehouse_N

