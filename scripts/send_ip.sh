#!/bin/bash

BOT_TOKEN="8640688801:AAH16v7dNsnVziP6Fm8i11OB0QEzImaD-1E"
CHAT_ID="1188719245"

IP=$(hostname -I | awk '{print $1}')

if [ -z "$IP" ]; then
  IP=$(ip route get 1 | awk '{print $7; exit}')
fi

curl -s "https://api.telegram.org/bot$BOT_TOKEN/sendMessage" \
-d "chat_id=$CHAT_ID" \
-d "text=🧀 Jetson IP: $IP"
