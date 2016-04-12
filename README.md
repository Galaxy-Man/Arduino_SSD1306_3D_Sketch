# Arduino_SSD1306_3D_Sketch
display 3d sketch in ssd1306 OLED with arduino

DC - 9
CS - 12    //or to vcc
RST - 10

and put a potentiometer between +5 and GND, the center pin to A0, this is for camera angle adjustment

the 3d obj struct is defined as follow:
in MyVertex every 3 int8 define a vertex location
in MyEdge every 2 int8 define a vertex connection
quat is the initial rotation represented by quaternion
offset is the translation