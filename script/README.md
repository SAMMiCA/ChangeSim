
# Data directory structure
```
[Folder Name]
      |
      --- Reference
             |
             --- Ref_Seq_Train
             |        |
             |        +-- Warehouse_0
             |        .         
             |        +-- Warehouse_5
             |
             --- Ref_Seq_Test
             |        |
             |        +-- Warehouse_6
             |        .
             |        +-- Warehouse_9
            
          Query
             |
             +-- Query_Seq_Train
             +-- Query_Seq_Test
            
          script
             |
             --- visualization.py
             --- dataloader.py
            
          utils
             |     
             --- Object_Labeling.py
             --- dict_indexing.py
             --- idx2color.txt
```


# Visualization
```
python visualization.py
```
<p align="center"><img src="../fig/visualization.png"></p>
