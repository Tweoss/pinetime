default-profile := 'release'
anaktoria-ip := '10.27.162.141'

build-run profile=default-profile:
    just build {{profile}}
    just run {{profile}}

build profile=default-profile:
    #!/usr/bin/env bash
    set -euxo pipefail
    path_profile={{ if profile == "dev" { "debug" } else { "release" } }}
    cargo rustc --profile {{profile}} --target thumbv7em-none-eabihf -Z build-std="core,compiler_builtins,alloc" -- -C link-arg=-Tlink.x

run profile=default-profile:
    #!/usr/bin/env bash
    set -euxo pipefail
    path_profile={{ if profile == "dev" { "debug" } else { "release" } }}
    just gdb

build-copy-bin:
    arm-none-eabi-objcopy target/thumbv7em-none-eabihf/release/pinetime -O binary target/thumbv7em-none-eabihf/release/pinetime.bin
    scp target/thumbv7em-none-eabihf/release/pinetime.bin anaktoria@{{anaktoria-ip}}:/home/anaktoria/francis-pinetime/pinetime.bin
    ssh anaktoria@{{anaktoria-ip}}

setup-openocd:
    ssh -L 3333:localhost:3333 anaktoria@{{anaktoria-ip}} -t "cd francis-pinetime; bash -l"

flash:
    ssh anaktoria@{{anaktoria-ip}} "cd francis-pinetime; echo $'program pinetime.bin\nreset run' | nc -q 0 localhost 4444"

size:
    arm-none-eabi-size -Ax target/thumbv7em-none-eabihf/release/pinetime

dump:
    arm-none-eabi-objdump --architecture arm -Mforce-thumb --target binary -D target/thumbv7em-none-eabihf/release/pinetime.bin

gdb:
    arm-none-eabi-gdb target/thumbv7em-none-eabihf/release/pinetime -ex 'target extended-remote localhost:3333' -ex 'monitor arm semihosting enable' -ex 'set confirm off' -ex 'monitor reset halt' -ex 'load' -ex 'continue'

build-copy-boot profile=default-profile:
    just build-boot {{profile}}
    just copy-boot {{profile}}

build-boot profile=default-profile:
    #!/usr/bin/env bash
    set -euxo pipefail
    path_profile={{ if profile == "dev" { "debug" } else { "release" } }}
    cargo rustc --profile {{profile}} --target thumbv7em-none-eabihf --package bootloader -Z build-std="core,compiler_builtins,alloc" -- -C link-arg=-Tlink.x
    arm-none-eabi-objcopy target/thumbv7em-none-eabihf/$path_profile/boot -O binary target/thumbv7em-none-eabihf/$path_profile/boot.bin

copy-boot profile=default-profile:
    #!/usr/bin/env bash
    set -euxo pipefail
    path_profile={{ if profile == "dev" { "debug" } else { "release" } }}
    cp target/thumbv7em-none-eabihf/$path_profile/boot.bin '/Volumes/No Name/kernel.img'
    sync
    diskutil eject "NO NAME"

profile profile:
    #!/usr/bin/env bash
    set -euxo pipefail
    path_profile={{ if profile == "dev" { "debug" } else { "release" } }}
    # copy paste addr:count pairs into profile-{{profile}}.txt
    arm-none-eabi-objdump -d target/thumbv7em-none-eabihf/$path_profile/pinetime > target/thumbv7em-none-eabihf/$path_profile/pinetime.dump
    python3 lookup.py target/thumbv7em-none-eabihf/$path_profile/pinetime.dump profile-$path_profile.txt
