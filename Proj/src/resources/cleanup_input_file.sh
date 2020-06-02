#!/usr/bin/env sh

[ ! -r "$1" ] && {
  echo "${1} is not a readable file"
  exit 1
}

sed "s/\s*(//;s/,//g;s/)//" "$1" > "${1}.clean"
