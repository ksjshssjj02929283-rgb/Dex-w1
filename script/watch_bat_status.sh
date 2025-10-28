#!/bin/bash

echo "电池电量:"
str=$(ethercat upload --type uint8 -p 0 0x6000 3)
str1=$(echo "$str" | cut -d ' ' -f 2 )
echo "$str1%"

echo "电池当前温度:"
str=$(ethercat upload --type uint8 -p 0 0x6000 4)
str1=$(echo "$str" | cut -d ' ' -f 2 )
echo "$str1 ℃"

echo "电池当前电压:"
str=$(ethercat upload --type uint16 -p 0 0x6000 5)
str1=$(echo "$str" | cut -d ' ' -f 2 )
result=$(echo "scale=2; $str1 / 10" | bc)
result=$(echo "scale=2; $str1 / 10" | bc)
echo "$result V"

echo "电池当前电流:"
str=$(ethercat upload --type uint16 -p 0 0x6000 6)
str1=$(echo "$str" | cut -d ' ' -f 2 )
result=$(echo "scale=2; $str1 / 10" | bc)
echo "$result A"