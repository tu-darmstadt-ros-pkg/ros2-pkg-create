#!/usr/bin/env python3

import argparse
import glob
import os
import shutil
from typing import Dict

import jinja2


def parseCli():

    parser = argparse.ArgumentParser(
        description="Creates a ROS 2 package with configurable components",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument("pkg_name", type=str, help="package name")
    parser.add_argument("--action", type=str, default=None, help="action name")

    args = parser.parse_args()

    return args


def loadJinjaTemplates(templates_dir: str) -> Dict[str, jinja2.Template]:

    templates = {}

    env = jinja2.Environment(loader=jinja2.FileSystemLoader(templates_dir), trim_blocks=False)
    template_files = glob.glob(os.path.join(templates_dir, "**/*.jinja2"), recursive=True)

    for f in template_files:
        templates[f] = env.get_template(os.path.relpath(f, templates_dir))

    return templates


def renderJinjaTemplates(templates: Dict[str, jinja2.Template], **kwargs) -> Dict[str, str]:

    docs = {}

    for template_file, template in templates.items():
        docs[template_file] = template.render(kwargs)

    return docs


def main():

    args = parseCli()

    template_dir = "template_cpp_pkg"

    shutil.copytree(template_dir, args.pkg_name, dirs_exist_ok=True)

    template_dir = args.pkg_name

    templates = loadJinjaTemplates(template_dir)

    docs = renderJinjaTemplates(
        templates,
        package_name=args.pkg_name,
        action=args.action
    )

    for template_file, doc in docs.items():
        doc_file = template_file.replace(".jinja2", "")
        with open(doc_file, "w") as f:
            f.write(doc)
        os.remove(template_file)
        print("Created file: {}".format(doc_file))


if __name__ == "__main__":

    main()
