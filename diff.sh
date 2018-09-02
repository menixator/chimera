#!/usr/bin/env bash

diff -u --color=always <(./print_unique_asm.sh $1) <(./print_unique_asm.sh $2)

