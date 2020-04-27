Easy "clone and go" repository for a libopencm3 based project with FreeRTOS.

# Instructions
 1. `git clone` with `--recurse-submodules`
 1. `make -C libopencm3` – only needed once
 1. `make -C freertos` – only needed once
 1. `make -C firmware`
 1. `make -C firmware firmware.st-flash` for flashing

# Directories
* `firmware` contains your application
* `shared` contains libraries.