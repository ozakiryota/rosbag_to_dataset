#!/bin/bash

user_name=user
user_pw=$user_name
user_id=${local_uid:-9001}
group_id=${local_gid:-9001}

useradd -u $user_id -o -m -G sudo $user_name
groupmod -g $group_id $user_name
echo $user_name:$user_pw | chpasswd
echo "$user_name ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

chown -R $user_name /home/$user_name

exec /usr/sbin/gosu $user_name "$@"