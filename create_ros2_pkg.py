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

    args = parser.parse_args()

    return args


def loadJinjaTemplates(templates_dir: str) -> Dict[str, jinja2.Template]:

    templates = {}

    env = jinja2.Environment(loader=jinja2.FileSystemLoader(templates_dir), trim_blocks=False)
    template_files = glob.glob(os.path.join(templates_dir, "*.jinja2"))
    for f in template_files:
        templates[f] = env.get_template(os.path.basename(f))

    return templates


def renderJinjaTemplates(templates: Dict[str, jinja2.Template], package_name: str) -> Dict[str, str]:

    docs = {}

    context = {
        "package_name": package_name,
    }

    for template_file, template in templates.items():
        doc_file = template_file.replace(".jinja2", "")
        docs[doc_file] = template.render(context)

    return docs


def main():

    args = parseCli()

    template_dir = "template_cpp_pkg"

    shutil.copytree(template_dir, args.pkg_name, dirs_exist_ok=True)

    template_dir = args.pkg_name

    templates = loadJinjaTemplates(template_dir)

    docs = renderJinjaTemplates(templates, args.pkg_name)

    for doc_file, doc in docs.items():
        with open(doc_file, "w") as f:
            f.write(doc)
        print("Created file: {}".format(doc_file))


if __name__ == "__main__":

    main()
