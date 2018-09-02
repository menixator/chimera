#!/usr/bin/env bash

comm -12 --output-delimiter='' <(./print_unique_asm.sh $1) <(./print_unique_asm.sh $2)
