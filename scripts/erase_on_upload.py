Import("env")

# PlatformIO prepends upload_flags before the esptool subcommand. --erase-all
# belongs to write_flash, so insert it directly after that subcommand.
# Merely building does not erase anything; erasure happens during upload.
if env.subst("$UPLOAD_PROTOCOL") == "esptool":
    flags = list(env["UPLOADERFLAGS"])
    flags.insert(flags.index("write_flash") + 1, "--erase-all")
    env.Replace(UPLOADERFLAGS=flags)
