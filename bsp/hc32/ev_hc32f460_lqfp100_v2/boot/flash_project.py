# Copyright (c) 2026, youyoutech. SPDX-License-Identifier: Apache-2.0
"""Generate download-only uVision targets alongside the released HEX files."""
from copy import deepcopy
from hashlib import sha256
import shutil
from xml.etree import ElementTree as ET

from prepare_mdk import BSP, GEN


def generate(targets):
    project = ET.parse(BSP / "project.uvprojx")
    options = ET.parse(BSP / "project.uvoptx")
    target_list = project.find("Targets")
    template = target_list.find("Target")
    target_list.clear()
    template.find("Groups").clear()
    common = template.find("TargetOption/TargetCommonOption")
    common.find("AfterMake/RunUserProg1").text = "0"
    common.find("CreateHexFile").text = "0"
    common.find("OutputDirectory").text = ".\\"
    template.find(".//UpdateFlashBeforeDebugging").text = "0"
    template.find(".//UseTargetDll").text = "0"
    template.find(".//UseExternalTool").text = "1"
    template.find(".//DriverSelection").text = "4102"
    shutil.copyfile(BSP / "boot/flash_station.py", GEN / "flash_station.py")
    shutil.copyfile(BSP / "boot/flash_batch.py", GEN / "flash_batch.py")
    opt_root = options.getroot()
    opt_template = opt_root.find("Target")
    for item in opt_root.findall("Target") + opt_root.findall("Group"):
        opt_root.remove(item)
    for name, filename in targets.items():
        target = deepcopy(template)
        target.find("TargetName").text = name
        target.find(".//OutputName").text = filename
        digest = sha256((GEN / filename).read_bytes()).hexdigest()
        launcher = f"download-{name}.cmd"
        (GEN / launcher).write_text(
            '@echo off\ncd /d "%~dp0"\n'
            f'python flash_station.py "{filename}" --sha256 {digest} > "download-{name}.log" 2>&1\n'
            f'set "result=%errorlevel%"\ntype "download-{name}.log"\nexit /b %result%\n',
            encoding="ascii")
        target.find(".//Flash3").text = f'"cmd.exe" (/d /c call "$P\\{launcher}")'
        target_list.append(target)
        opt = deepcopy(opt_template)
        opt.find("TargetName").text = name
        opt.find(".//IsCurrentTarget").text = "1" if name == "ALL" else "0"
        for reg in opt.findall(".//SetRegEntry"):
            if reg.findtext("Key") == "JL2CM3":
                settings = reg.find("Name")
                settings.text = " ".join(arg for arg in settings.text.split(" ") if not arg.startswith("-U"))
        opt_root.append(opt)
    project.write(GEN / "flash.uvprojx", encoding="utf-8", xml_declaration=True)
    options.write(GEN / "flash.uvoptx", encoding="utf-8", xml_declaration=True)
