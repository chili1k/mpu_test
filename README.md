# Schematic

```
MPU6050            ARDUINO    button  
vcc -------------- vcc--------/ ----------------|
                                                |
gnd -------------- gnd----|                     |
scl -------------- scl    |                     |
sda -------------- scl   | | pull down          |
                         | | resistor 1k-10k    |
                   pin3---|---------------------|
```
