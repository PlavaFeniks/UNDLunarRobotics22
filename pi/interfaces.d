source-directory /etc/network/interfaces.d
allow-hotplug can0
iface can0 can static
	bitrate 1000000
	up /sbin/ifconfig $IFACE txqueuelen 1000
	up /sbin/ip link set $IFACE down
	up /sbin/ip link set $IFACE up type can
auto wlan0
