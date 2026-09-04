"""Small shared Drake parsing utilities."""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


def register_package_xml(parser: Any, package_xml: Path) -> None:
    """Register one ROS-style package.xml with a Drake Parser."""
    path = Path(package_xml)
    root = ET.parse(path).getroot()
    name = root.findtext("name")
    if name is None:
        raise ValueError(f"Missing <name> in {path}")
    parser.package_map().Add(name.strip(), str(path.parent.resolve()))
