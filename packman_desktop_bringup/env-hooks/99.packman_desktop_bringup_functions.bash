function packman-core {
  if [ -z "$1" ]; then
    echo -e "\033[31mPlease provide a hostname as first argument\033[0m"
  else
    if [ -z "$2" ]
    then
        echo -e "\033[31mPlease provide a [network_interface, e.g. eno1, eth0] as second argument\033[0m"
    else
       ruvu_bringup_connect_to_external_ros_master "http://$1:11311" "$2"
    fi
  fi
}
