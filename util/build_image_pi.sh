#!/usr/bin/env bash
docker stop glados-pi > /dev/null 2>&1 || true
docker rm glados-pi > /dev/null 2>&1 || true

cd ~
docker build -t glados-pi -f glados/.devcontainer/pi.Dockerfile .