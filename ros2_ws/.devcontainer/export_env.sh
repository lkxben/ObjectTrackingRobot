#!/bin/bash
echo "UID=$(id -u)" > .env
echo "GID=$(id -g)" >> .env
echo "USER=$(id -un)" >> .env
echo "GROUP=$(id -gn)" >> .env