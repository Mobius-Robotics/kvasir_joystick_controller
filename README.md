kvasir-joystick-controller
========================

Allows control of Kvasir via a Joystick, such as an Xbox 360 gamepad, via ROS2 and pygame.

This package depends on `kvasir_hw_interface`, which can be found in our organization's repositories
and must be present (and have been compiled) in the same ROS2 workspace.

Installation
------------

Other than being ran manually via the included launch file, copying
`scripts/joystick_controller.service` to `/etc/systemd/system/` allows the script to automatically
run on bootup, provided that you also enable (and start) the service:
```shell
sudo systemctl daemon-reload && sudo systemctl enable joystick_controller.service && sudo systemctl start joystick_controller.service
```
