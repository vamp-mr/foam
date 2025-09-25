docker run -it \
  --name foam \
  -v "$PWD:/foam" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /mnt/wslg:/mnt/wslg \
  --env DISPLAY \
  --env WAYLAND_DISPLAY \
  --env XDG_RUNTIME_DIR \
  --env PULSE_SERVER \
  foam-image
