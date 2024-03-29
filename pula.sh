sleep 1
echo "Pula mea"
OCMD=reattach_nxp ssh corneliu
sleep 1
sudo /usr/local/bin/mavproxy.py --master /dev/serial/by-id/usb-NXP_SEMICONDUCTORS_PX4_FMUK66_v3.x_0-if00
