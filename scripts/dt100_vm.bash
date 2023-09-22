#!/usr/bin/env bash
rosbash_init_node "dt100_vm_node" "$@"  # parse the command line arguments

# get params
rosbash_param bridge_adapter_ "bridge_adapter"
rosbash_param headless_ "headless" "True"

# print params
echo "dt100_vm_node parameters loaded:"
echo "bridge_adapter_: ${bridge_adapter_}"
echo "headless_: ${headless_}"

# set bridged adapter
VBoxManage modifyvm "Windows_XP_32_DT100" --nic1 none
VBoxManage modifyvm "Windows_XP_32_DT100" --nic1 bridged --bridgeadapter1 $bridge_adapter_

# start vm
echo "Launching DT100 Virtual Machine..."
if $headless_; then
  VBoxManage startvm "Windows_XP_32_DT100" --type headless
else
  VBoxManage startvm "Windows_XP_32_DT100"
fi

# exit gracefully by returning a status 
trap ctrlc SIGINT

exit 0
