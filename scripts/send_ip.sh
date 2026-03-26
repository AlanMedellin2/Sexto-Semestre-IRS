#!/bin/bash

BOT_TOKEN="8640688801:AAH16v7dNsnVziP6Fm8i11OB0QEzImaD-1E"
CHAT_ID="-1003848044218"

# esperar red real
while ! ping -c 1 google.com &> /dev/null; do
  sleep 2
done

# IP local
LOCAL_IP=$(hostname -I | awk '{print $1}')

if [ -z "$LOCAL_IP" ]; then
  LOCAL_IP=$(ip route get 1 | awk '{print $7; exit}')
fi

# IP pública
PUBLIC_IP=$(curl -s --max-time 10 https://api.ipify.org 2>/dev/null || echo "no disponible")

HOSTNAME=$(hostname)
TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')

MESSAGE="🤖🧀 *QuezoJetson Encendida*
📛 Hostname: \`${HOSTNAME}\`
🏠 IP Local: \`${LOCAL_IP}\`
🌐 IP Pública: \`${PUBLIC_IP}\`
🕐 Hora: ${TIMESTAMP}"

curl -s -X POST "https://api.telegram.org/bot${BOT_TOKEN}/sendMessage" \
  --data-urlencode "chat_id=${CHAT_ID}" \
  --data-urlencode "text=${MESSAGE}" \
  --data-urlencode "parse_mode=Markdown"

echo "IP enviada: ${LOCAL_IP}"
