# RecorderStatus

#Stages
uint8 RECORDING=0
uint8 INSUFFICIENT_DISK_SPACE=1
uint8 PREPARING_UPLOAD=2
uint8 UPLOADING=3
uint8 COMPLETE=4
uint8 CLEANUP=5

# Which stage the action is in
uint8 stage
