
# Data directory structure

Compose the folder in this way

```
[Folder Name]
        |
        --- Mapping
            |
            --- Ref_Seq_Train
            |        |
            |        +--- Warehouse_0
            |        .         
            |        +--- Warehouse_5
            |
            +-- Ref_Seq_Test
            |        |
            |        +--- Warehouse_6
            |        .
            |        +--- Warehouse_9
            Localization
            |
            +-- Query_Seq_Train
            +-- Query_Seq_Test
            script
            |
            --- visualization.py
            --- dataloader.py
            |
            --- utils
            |     |
            |     --- Object_Labeling.py
            |     --- dict_indexing.py
            |     --- idx2color.txt

```


# Visualiation
```
python visualization.py

```
