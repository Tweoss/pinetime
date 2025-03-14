default-profile := 'release'
anaktoria-ip := '10.32.81.227'

build-run profile=default-profile:
    just build {{profile}}
    just run {{profile}}

build profile=default-profile:
    #!/usr/bin/env bash
    set -euxo pipefail
    path_profile={{ if profile == "dev" { "debug" } else { "release" } }}
    cargo rustc --profile {{profile}} --target armv7-none-eabi.json -Z build-std="core,compiler_builtins,alloc" -- -C link-arg=-Tlink.x
    arm-none-eabi-objcopy target/armv7-none-eabi/$path_profile/pinetime -O binary target/armv7-none-eabi/$path_profile/pinetime.bin

run profile=default-profile:
    #!/usr/bin/env bash
    set -euxo pipefail
    path_profile={{ if profile == "dev" { "debug" } else { "release" } }}
    scp target/armv7-none-eabi/$path_profile/pinetime anaktoria@{{anaktoria-ip}}:/home/anaktoria/francis-pinetime/pinetime
    # scp target/armv7-none-eabi/$path_profile/pinetime.bin anaktoria@{{anaktoria-ip}}:/home/anaktoria/francis-pinetime/pinetime.bin
    just gdb

flash:
    ssh anaktoria@{{anaktoria-ip}} "cd francis-pinetime; echo $'program pinetime.bin\nreset run' | nc -q 0 localhost 4444"

gdb:
    # enter continue to run program
    ssh -t anaktoria@{{anaktoria-ip}} "shopt -s huponexit; cd francis-pinetime; arm-none-eabi-gdb pinetime -ex 'target extended-remote localhost:3333' -ex 'monitor arm semihosting enable' -ex 'monitor reset halt' -ex 'load'"
    # ssh -t anaktoria@{{anaktoria-ip}} "shopt -s huponexit; cd francis-pinetime; arm-none-eabi-gdb pinetime -ex 'target extended-remote localhost:3333' -ex 'monitor arm semihosting enable' -ex 'monitor reset halt' -ex 'load'"

build-copy-boot profile=default-profile:
    just build-boot {{profile}}
    just copy-boot {{profile}}

build-boot profile=default-profile:
    #!/usr/bin/env bash
    set -euxo pipefail
    path_profile={{ if profile == "dev" { "debug" } else { "release" } }}
    cargo rustc --profile {{profile}} --target armv7-none-eabi.json --package bootloader -Z build-std="core,compiler_builtins,alloc" -- -C link-arg=-Tlink.x
    arm-none-eabi-objcopy target/armv7-none-eabi/$path_profile/boot -O binary target/armv7-none-eabi/$path_profile/boot.bin

copy-boot profile=default-profile:
    #!/usr/bin/env bash
    set -euxo pipefail
    path_profile={{ if profile == "dev" { "debug" } else { "release" } }}
    cp target/armv7-none-eabi/$path_profile/boot.bin '/Volumes/No Name/kernel.img'
    sync
    diskutil eject "NO NAME"

profile profile:
    #!/usr/bin/env bash
    set -euxo pipefail
    path_profile={{ if profile == "dev" { "debug" } else { "release" } }}
    # copy paste addr:count pairs into profile-{{profile}}.txt
    arm-none-eabi-objdump -d target/armv7-none-eabi/$path_profile/pinetime > target/armv7-none-eabi/$path_profile/pinetime.dump
    python3 lookup.py target/armv7-none-eabi/$path_profile/pinetime.dump profile-$path_profile.txt
