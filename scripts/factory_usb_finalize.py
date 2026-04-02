import os
import subprocess

Import("env")


def _clear_otadata_after_serial_upload(source, target, env):
    protocol = env.subst("$UPLOAD_PROTOCOL").strip()
    if protocol != "esptool":
        print("[factory-usb-finalize] Skip: upload_protocol is not esptool")
        return

    upload_port = env.subst("$UPLOAD_PORT").strip()
    if not upload_port:
        print("[factory-usb-finalize] Skip: upload port not set")
        return

    platform = env.PioPlatform()
    esptool_dir = platform.get_package_dir("tool-esptoolpy")
    if not esptool_dir:
        raise RuntimeError("tool-esptoolpy package not found")

    python_exe = env.subst("$PYTHONEXE").strip()
    if not python_exe:
        raise RuntimeError("PYTHONEXE is not available")

    esptool_py = os.path.join(esptool_dir, "esptool.py")
    mcu = env.BoardConfig().get("build.mcu", "esp32")

    cmd = [
        python_exe,
        esptool_py,
        "--chip",
        mcu,
        "--port",
        upload_port,
        "erase_region",
        "0xE000",
        "0x2000",
    ]

    print("[factory-usb-finalize] Clearing otadata so the next boot starts from factory")
    subprocess.run(cmd, check=True)


env.AddPostAction("upload", _clear_otadata_after_serial_upload)
