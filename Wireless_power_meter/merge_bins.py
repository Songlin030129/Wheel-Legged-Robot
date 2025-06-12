Import('env')
import os
OUTPUT_DIR = "$BUILD_DIR{}".format(os.path.sep)
APP_BIN = "$BUILD_DIR/${PROGNAME}.bin"

def copy_merge_bins(source, target, env):
    firmware_src = str(target[0])
    flash_images = env.Flatten(env.get("FLASH_EXTRA_IMAGES", [])) + ["$ESP32_APP_OFFSET", APP_BIN]
    name = firmware_src.split(os.path.sep)[2]
    flash_size = env.GetProjectOption("board_upload.flash_size", "4MB")
    board = env.BoardConfig()
    f_flash = board.get("build.f_flash", "40000000L")
    flash_freq = '40m'
    if (f_flash == '80000000L'):
        flash_freq = '80m'
        
    #这里要修改为对应的esp32型号！
    mcu = board.get("build.mcu", "esp32c3")

    firmware_dst = "{}{}_{}.bin".format(OUTPUT_DIR, name, flash_size)
    if os.path.isfile(firmware_dst):
        os.remove(firmware_dst)
    cmd = " ".join([
        "$PYTHONEXE", "$OBJCOPY", '--chip', mcu, 'merge_bin', '--output', firmware_dst, '--flash_mode', 'dio',
        '--flash_size', flash_size, '--flash_freq', flash_freq] + flash_images)
    env.Execute(cmd)
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", [copy_merge_bins])