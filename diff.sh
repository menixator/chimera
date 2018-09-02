#!/usr/bin/env bash

diff -u --color=always --label "Test $1" <(./print_unique_asm.sh $1) --label "Test $2" <(./print_unique_asm.sh $2)

