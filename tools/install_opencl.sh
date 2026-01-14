#!/usr/bin/env bash

wget https://apt.llvm.org/llvm.sh
chmod +x llvm.sh
sudo ./llvm.sh 18
sudo apt install -y llvm-18 llvm-18-dev llvm-18-tools clang-18
sudo apt install -y libclang-18-dev libedit-dev