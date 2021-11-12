#!/usr/bin/env bash
rosbash_init_node "dt100_vm_node" "$@"  # parse the command line arguments

# get params
rosbash_param bridge_adapter_ "bridge_adapter"
rosbash_param add_network_interface_ "add_network_interface" "True"
rosbash_param headless_ "headless" "True"

# just print this out
echo "Launching DT100 Virtual Machine..."

# set bridged adapter 
VBoxManage modifyvm "Windows_XP_32_DT100" --nic1 none
VBoxManage modifyvm "Windows_XP_32_DT100" --nic1 bridged --bridgeadapter1 $bridge_adapter_

# start vm
if $headless_; then
  VBoxManage startvm "Windows_XP_32_DT100" --type headless
else
  VBoxManage startvm "Windows_XP_32_DT100"
fi
echo "here3"
# exit gracefully by returning a status 
trap ctrlc SIGINT

exit 0
