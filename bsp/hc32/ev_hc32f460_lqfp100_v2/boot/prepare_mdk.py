# Copyright (c) 2026, youyoutech.
# SPDX-License-Identifier: Apache-2.0
"""Generate the signed-image App project from the maintained BSP project."""
import argparse
import hashlib
import io
from pathlib import Path
import re
from urllib.request import urlopen
import xml.etree.ElementTree as ET
import zipfile

BSP = Path(__file__).resolve().parents[1]
BOOT = BSP / "boot"
GEN = BSP / "build" / "ntn-ota"
MCUBOOT = GEN / "deps" / "mcuboot-2.4.0"
MCUBOOT_SHA256 = "e45751e9e7c75aa924a30f9261e1bc6f601bc04c5270b9f6fb456c303d68ea0b"


def fetch_mcuboot():
    url = "https://codeload.github.com/mcu-tools/mcuboot/zip/refs/tags/v2.4.0"
    with urlopen(url, timeout=60) as response:
        data = response.read()
    if hashlib.sha256(data).hexdigest() != MCUBOOT_SHA256:
        raise ValueError("MCUboot archive checksum mismatch")
    with zipfile.ZipFile(io.BytesIO(data)) as archive:
        archive.extractall(MCUBOOT.parent)


def add_file(group, path, file_type="1"):
    files = group.find("Files")
    item = ET.SubElement(files, "File")
    ET.SubElement(item, "FileName").text = Path(path).name
    ET.SubElement(item, "FileType").text = file_type
    ET.SubElement(item, "FilePath").text = str(Path(path))


def layout_int(name):
    return int(re.search(r"^#define " + name + r"\s+(0x[0-9A-Fa-f]+|[0-9]+)$",
                         (BOOT / "ota_layout.h").read_text(), re.M)[1], 0)


def project(stage, vector, maximum):
    tree = ET.parse(BSP / "project.uvprojx")
    target = tree.find("./Targets/Target")
    name = f"ntn-ota-{stage}"
    target.find("TargetName").text = name
    options = target.find("TargetOption")
    common = options.find("TargetCommonOption")
    common.find("OutputName").text = name
    common.find("OutputDirectory").text = f".\\build\\ntn-ota\\{stage}\\"
    common.find("ListingPath").text = f".\\build\\ntn-ota\\{stage}\\"
    common.find("AfterMake/RunUserProg1").text = "0"
    ads = options.find("TargetArmAds")
    compiler = ads.find("Cads/VariousControls")
    common.find("Cpu").text = re.sub(r"IROM1\([^)]*\)", f"IROM1({vector:#010x},{maximum:#x})",
                                    common.find("Cpu").text)
    compiler.find("Define").text += f", NTN_OTA_{stage.upper()}, VECT_TAB_OFFSET={vector:#010x}"
    compiler.find("IncludePath").text += ";boot;build/ntn-ota"
    linker = ads.find("LDads")
    linker.find("ScatterFile").text = f".\\boot\\ota_{stage}.sct"
    linker.find("TextAddressRange").text = f"{vector:#010x}"
    return tree, target


def save_project(tree, stage):
    tree.write(BSP / f"ntn-ota-{stage}.uvprojx", encoding="utf-8", xml_declaration=True)


def app_project():
    vector = layout_int("NTN_OTA_PRIMARY") + layout_int("NTN_OTA_HEADER_SIZE")
    tree, target = project("app", vector, layout_int("NTN_OTA_APP_MAX"))
    for group in target.findall("Groups/Group"):
        files = group.find("Files")
        for item in files.findall("File"):
            if item.findtext("FileName") == "hc32_ll_icg.c":
                files.remove(item)
        if group.findtext("GroupName") == "Applications":
            add_file(group, "boot/ota_identity.c")
            add_file(group, "boot/ota_app.c")
            add_file(group, "boot/ota_flash.c")
    save_project(tree, "app")


def bl1_project(bl2_bin, watchdog):
    raw = bl2_bin.read_bytes()
    size = layout_int("NTN_OTA_BL2_SIZE")
    if not 8 <= len(raw) <= size:
        raise ValueError("BL2 binary outside assigned region")
    digest = hashlib.sha256(raw.ljust(size, b"\xff")).digest()
    values = ", ".join(f"0x{byte:02x}" for byte in digest)
    (GEN / "ota_bl2_digest.h").write_text(f"#define NTN_OTA_BL2_DIGEST {{{values}}}\n", encoding="ascii")
    tree, target = project("bl1", 0, layout_int("NTN_OTA_BL1_SIZE"))
    groups = target.find("Groups")
    keep = {"startup_hc32f460.s", "system_hc32f460.c", "hc32_ll_icg.c",
            "hc32_ll_fcg.c", "hc32_ll_utility.c", "hc32_ll_usart.c", "hc32_ll_gpio.c",
            "hc32_ll_clk.c", "hc32_ll_swdt.c"}
    if watchdog:
        target.find("TargetOption/TargetArmAds/Cads/VariousControls/MiscControls").text = (
            "--preinclude boot/ota_icg.h")
    for group in list(groups):
        files = group.find("Files")
        for item in list(files):
            if item.findtext("FileName") not in keep:
                files.remove(item)
        if len(files) == 0:
            groups.remove(group)
    group = ET.SubElement(groups, "Group")
    ET.SubElement(group, "GroupName").text = "BL1"
    ET.SubElement(group, "Files")
    add_file(group, "boot/ota_bl1.c")
    add_file(group, "packages/hc32-f4-series-latest/hc32f460/src/hc32_ll_hash.c")
    save_project(tree, "bl1")


def bl2_project(public_key):
    from cryptography.hazmat.primitives import serialization
    from cryptography.hazmat.primitives.asymmetric import ec

    public = serialization.load_pem_public_key(public_key.read_bytes())
    if not isinstance(public, ec.EllipticCurvePublicKey) or not isinstance(public.curve, ec.SECP256R1):
        raise ValueError("Expected an ECDSA P-256 public key")
    der = public.public_bytes(serialization.Encoding.DER, serialization.PublicFormat.SubjectPublicKeyInfo)
    data = ", ".join(f"0x{byte:02x}" for byte in der)
    (GEN / "ota_boot_key.c").write_text(
        '#include "bootutil/sign_key.h"\n'
        f'static const unsigned char key[] = {{{data}}};\n'
        'static const unsigned int key_len = sizeof(key);\n'
        'const struct bootutil_key bootutil_keys[] = {{key, &key_len}};\n'
        'const int bootutil_key_cnt = 1;\n', encoding="ascii")

    if not (MCUBOOT / "boot" / "bootutil").is_dir():
        raise ValueError(f"Extract the pinned MCUboot source to {MCUBOOT}")
    tree, target = project("bl2", layout_int("NTN_OTA_BL2_BASE"), layout_int("NTN_OTA_BL2_SIZE"))
    target.find("uAC6").text = "1"
    target.remove(target.find("pCCUsed"))
    ads = target.find("TargetOption/TargetArmAds")
    ET.SubElement(ads.find("Aads"), "uClangAs").text = "0"
    ads.find("Cads/wLevel").text = "1"
    ads.find("Cads/Optim").text = "4"
    for controls in target.findall(".//Cads/VariousControls"):
        misc = controls.find("MiscControls")
        misc.text = (misc.text or "").replace("--c99", "")
    groups = target.find("Groups")
    for group in list(groups):
        name = group.findtext("GroupName")
        if name in {"Tests", "Finsh"}:
            groups.remove(group)
            continue
        files = group.find("Files")
        for item in list(files):
            file_name = item.findtext("FileName")
            if ((name == "Applications" and file_name != "app_flash.c") or
                    file_name.startswith("dev_i2c_") or file_name in
                    {"hc32_ll_icg.c", "drv_i2c.c", "fal_rtt.c"}):
                files.remove(item)

    compiler = target.find("TargetOption/TargetArmAds/Cads/VariousControls")
    compiler.find("MiscControls").text = "-Oz -ffunction-sections -include boot/ota_bl2_config.h"
    bootutil = MCUBOOT / "boot" / "bootutil"
    tinycrypt = MCUBOOT / "ext" / "tinycrypt" / "lib"
    asn1 = MCUBOOT / "ext" / "mbedtls-asn1"
    compiler.find("IncludePath").text += ";" + ";".join(
        str(p.relative_to(BSP)) for p in (bootutil / "include", bootutil / "src",
                                         tinycrypt / "include", asn1 / "include"))
    group = ET.SubElement(groups, "Group")
    ET.SubElement(group, "GroupName").text = "MCUboot"
    ET.SubElement(group, "Files")
    sources = ("bootutil_area", "bootutil_find_key", "bootutil_img_hash", "bootutil_loader",
               "bootutil_misc", "bootutil_public", "fault_injection_hardening", "image_ecdsa",
               "image_validate", "loader", "swap_misc", "swap_scratch", "tlv")
    for path in (BOOT / "ota_bl2.c", BOOT / "ota_flash.c", BOOT / "ota_serial.c", GEN / "ota_boot_key.c",
                 *(bootutil / "src" / f"{name}.c" for name in sources),
                 *(tinycrypt / "source" / f"{name}.c" for name in
                   ("ecc", "ecc_dsa", "ecc_platform_specific", "sha256", "utils")),
                 asn1 / "src" / "asn1parse.c", asn1 / "src" / "platform_util.c"):
        add_file(group, path.relative_to(BSP))
    save_project(tree, "bl2")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fetch-mcuboot", action="store_true")
    parser.add_argument("--build-id", help="32 hexadecimal digits")
    parser.add_argument("--public-key", type=Path, help="P-256 public key PEM for BL2")
    parser.add_argument("--bl2-bin", type=Path, help="BL2 raw image to authenticate in BL1")
    parser.add_argument("--watchdog", action="store_true", help="Enable the ICG SWDT profile in BL1")
    args = parser.parse_args()
    if not (args.fetch_mcuboot or args.build_id or args.public_key or args.bl2_bin):
        parser.error("specify --fetch-mcuboot, --build-id, --public-key, or --bl2-bin")
    if args.build_id and not re.fullmatch("[0-9a-fA-F]{32}", args.build_id):
        parser.error("--build-id must contain exactly 32 hex digits")
    GEN.mkdir(parents=True, exist_ok=True)
    if args.fetch_mcuboot:
        fetch_mcuboot()
    if args.build_id:
        values = ", ".join(f"0x{byte:02x}" for byte in bytes.fromhex(args.build_id))
        (GEN / "ota_build_id.h").write_text(f"#define NTN_OTA_BUILD_ID {{{values}}}\n", encoding="ascii")
        app_project()
    if args.public_key:
        bl2_project(args.public_key)
    if args.bl2_bin:
        bl1_project(args.bl2_bin, args.watchdog)


if __name__ == "__main__":
    main()
