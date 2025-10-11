#!/bin/bash

STOP_SERVICES=(
    "farmng-amiga.service"
    "farmng-filter.service"
    "farmng-foxglove.service"
)

# -- Stop amiga services that interfere with our ros2 nodes
for svc in "${STOP_SERVICES[@]}"; do
  curl -sS -L -X POST http://localhost:8001/systemctl_action/ \
    -H 'Content-Type: application/json' \
    -d '{
      "account_name": "adminfarmng",
      "service_id": "'"$svc"'",
      "action": "stop"
    }'
done