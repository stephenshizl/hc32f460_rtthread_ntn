# Copyright (c) 2026, youyoutech.
# SPDX-License-Identifier: Apache-2.0
"""Generate the signed-image App project from the maintained BSP project."""
import argparse
from pathlib import Path
import re
import xml.etree.ElementTree as ET

BSP = Path(__file__).resolve().parents[1]
BOOT = BSP / "boot"
GEN = BSP / "build" / "ntn-ota"


def add_file(group, path, file_type="1"):
    files = group.find("Files")
    item = ET.SubElement(files, "File")
    ET.SubElement(item, "FileName").text = Path(path).name
    ET.SubElement(item, "FileType").text = file_type
    ET.SubElement(item, "FilePath").text = str(Path(path))


def layout_int(name):
    return int(re.search(r"^#define " + name + r"\s+(0x[0-9A-Fa-f]+|[0-9]+)$",
                         (BOOT / "ota_layout.h").read_text(), re.M)[1], 0)


def app_project():
    tree = ET.parse(BSP / "project.uvprojx")
    target = tree.find("./Targets/Target")
    target.find("TargetName").text = "ntn-ota-app"
    options = target.find("TargetOption")
    common = options.find("TargetCommonOption")
    common.find("OutputName").text = "ntn-ota-app"
    common.find("OutputDirectory").text = "./build/ntn-ota/app/"
    common.find("ListingPath").text = "./build/ntn-ota/app/"
    ads = options.find("TargetArmAds")
    compiler = ads.find("Cads/VariousControls")
    vector = layout_int("NTN_OTA_PRIMARY") + layout_int("NTN_OTA_HEADER_SIZE")
    maximum = layout_int("NTN_OTA_APP_MAX")
    common.find("Cpu").text = re.sub(r"IROM1\([^)]*\)", f"IROM1({vector:#010x},{maximum:#x})",
                                    common.find("Cpu").text)
    compiler.find("Define").text += f", NTN_OTA_APP, VECT_TAB_OFFSET={vector:#010x}"
    compiler.find("IncludePath").text += ";boot;build/ntn-ota"
    linker = ads.find("LDads")
    linker.find("ScatterFile").text = ".\\boot\\ota_app.sct"
    linker.find("TextAddressRange").text = f"{vector:#010x}"
    for group in target.findall("Groups/Group"):
        files = group.find("Files")
        for item in files.findall("File"):
            if item.findtext("FileName") == "hc32_ll_icg.c":
                files.remove(item)
        if group.findtext("GroupName") == "Applications":
            add_file(group, "boot/ota_identity.c")
    tree.write(BSP / "ntn-ota-app.uvprojx", encoding="utf-8", xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-id", required=True, help="32 hexadecimal digits")
    args = parser.parse_args()
    if not re.fullmatch("[0-9a-fA-F]{32}", args.build_id):
        parser.error("--build-id must contain exactly 32 hex digits")
    GEN.mkdir(parents=True, exist_ok=True)
    values = ", ".join(f"0x{byte:02x}" for byte in bytes.fromhex(args.build_id))
    (GEN / "ota_build_id.h").write_text(f"#define NTN_OTA_BUILD_ID {{{values}}}\n", encoding="ascii")
    app_project()


if __name__ == "__main__":
    main()
