# SML Messages Package

This package contains ROS2 message definitions which are used to transmit Task Orders in the SML during 2026. 

## Contents

### Orders 

```
Order.msg
---
int OT_PRODUCE=1
int OT_RECYCLE=2

int order_type
int product_id
```

### Stations

```
Station.msg
---
int ST_STORAGE=1
int ST_WORKBENCH=2
int ST_CUSTOMER=3
int ST_HYBRID=4

string station_name
int station_type
int station_id
int[] material_ids
```

### Task

```
Task.msg
---
Orders[] order_list
Stations[] arena_layout
```


## Example Tasks 

### Entry Tier - Battery Assembly

Task: Build the battery from two source materials. 


```
order_list = 
{
   order_type = 1 ; product_id = 34
}

arena_layout = 
{
   station_type = 1; station_id = 1; material_ids = {3}
   station_type = 1; station_id = 3; material_ids = {4}
   station_type = 2; station_id = 4; material_ids = {}
   station_type = 3; station_id = 5; material_ids = {}
}
```