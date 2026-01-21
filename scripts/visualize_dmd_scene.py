import argparse
import logging
from pathlib import Path
import xml.etree.ElementTree as ET

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

from pydrake.geometry import StartMeshcat
from pydrake.multibody.parsing import (
    LoadModelDirectives,
    ProcessModelDirectives,
)
from pydrake.planning import RobotDiagramBuilder
from pydrake.visualization import (
    ApplyVisualizationConfig,
    VisualizationConfig,
)


def register_package_xml(parser, package_xml_path: Path):
    """
    Register a ROS-style package.xml with Drake's PackageMap.

    Args:
        parser: Drake Parser instance
        package_xml_path: Path to package.xml
    """
    if not package_xml_path.exists():
        raise ValueError(f"package.xml does not exist: {package_xml_path}")

    tree = ET.parse(package_xml_path)
    root = tree.getroot()

    name_elem = root.find("name")
    if name_elem is None or not name_elem.text:
        raise ValueError(
            f"Could not find <name> tag in {package_xml_path}"
        )

    package_name = name_elem.text.strip()
    package_dir = str(package_xml_path.parent)

    parser.package_map().Add(package_name, package_dir)
    logger.info("Registered package '%s' at %s", package_name, package_dir)


def visualize_dmd_file(dmd_file: Path, package_xmls: list[Path]):
    """
    Visualize a single .dmd.yaml file in Meshcat.

    Args:
        dmd_file: Path to .dmd.yaml file
        package_xmls: List of paths to package.xml files
    """
    if not dmd_file.exists():
        raise ValueError(f".dmd.yaml file does not exist: {dmd_file}")

    meshcat = StartMeshcat()
    meshcat.Delete()

    builder = RobotDiagramBuilder()
    parser = builder.parser()

    # Register all provided package.xml files
    for pkg_xml in package_xmls:
        register_package_xml(parser, pkg_xml)

    # Load and process directives
    directives = LoadModelDirectives(str(dmd_file))
    ProcessModelDirectives(directives, parser)

    builder.plant().Finalize()

    ApplyVisualizationConfig(
        config=VisualizationConfig(),
        plant=builder.plant(),
        scene_graph=builder.scene_graph(),
        builder=builder.builder(),
        meshcat=meshcat,
    )

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    diagram.ForcedPublish(context)

    logger.info("Visualizing: %s", dmd_file)
    logger.info("Meshcat server running. Press Ctrl+C to exit.")

    # Keep objects alive
    try:
        while True:
            pass
    except KeyboardInterrupt:
        logger.info("Exiting.")


def main():
    parser = argparse.ArgumentParser(
        description="Visualize a single Drake .dmd.yaml file in Meshcat"
    )
    parser.add_argument(
        "dmd_file",
        type=str,
        help="Path to the .dmd.yaml file to visualize",
    )
    parser.add_argument(
        "--package-xml",
        type=str,
        action="append",
        default=[],
        help=(
            "Path to a package.xml file. "
            "May be specified multiple times."
        ),
    )

    args = parser.parse_args()

    dmd_file = Path(args.dmd_file)
    package_xmls = [Path(p) for p in args.package_xml]

    visualize_dmd_file(dmd_file, package_xmls)


if __name__ == "__main__":
    main()
