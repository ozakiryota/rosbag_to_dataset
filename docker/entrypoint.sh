#!/bin/bash

user_name=user
user_id=${local_uid:-9001}
group_id=${local_gid:-9001}

useradd -u $user_id -o -m $user_name
groupmod -g $group_id $user_name
chown -R $user_name /home/$user_name

exec /usr/sbin/gosu $user_name "$@"
