# Action type constants
int32 FETCH_BLOCK    = 1   # go to shelf, arm pick block, place on bot base
int32 GOTO_WORKBENCH = 2   # drive to wb, arm place all blocks from base onto wb
int32 LOAD_TOY       = 3   # arm pick assembled toy from wb, place on bot base
int32 DELIVER        = 4   # drive to cc_1, arm place toy at counter
int32 FETCH_RECYCLE  = 5   # go to cc_1, arm pick toy, place on bot base
int32 PLACE_RECYCLE  = 6   # drive to wb, arm place toy, wait for breakdown
int32 LOAD_BLOCKS    = 7   # arm pick each block from wb onto bot base
int32 RETURN_BLOCK   = 8   # drive to shelf, arm place block from base onto shelf

# Fields
int32  action
int32  block_type
string station
string label