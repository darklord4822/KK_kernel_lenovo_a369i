Lenovo a369i source 3.4.67
===============
When using these sources to indicate in credits SRT team!!!
===============
Working
1) RIL
2) Camera
3) Touch
4) Sounds
5) LCM
6) Proximity
and other.
NOT Working:
1) Accelerometer
==============
Build Command:

cd KK_kernel_lenovo_a369i
./mk -o=TARGET_BUILD_VARIANT=user a369i n k

Then, to create the boot.img:

./pack_bootimage.sh
