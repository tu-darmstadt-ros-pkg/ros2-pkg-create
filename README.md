# ros2-pkg-create

### Installation

```bash
pip install ros2-pkg-create
```

### Usage

```
usage: ros2-pkg-create [-h] [--defaults] --template {ros2_cpp_pkg} [--package_name PACKAGE_NAME] [--description DESCRIPTION] [--maintainer MAINTAINER] [--maintainer-email MAINTAINER_EMAIL] [--author AUTHOR] [--author-email AUTHOR_EMAIL]
                       [--license {Apache-2.0,BSL-1.0,BSD-2.0,BSD-2-Clause,BSD-3-Clause,GPL-3.0-only,LGPL-2.1-only,LGPL-3.0-only,MIT,MIT-0}] [--node-name NODE_NAME] [--node-class-name NODE_CLASS_NAME] [--is-component] [--is-lifecycle] [--has-launch-file]
                       [--launch-file-type {xml,py,yml}] [--has-params] [--has-subscriber] [--has-publisher] [--has-service-server] [--has-action-server] [--has-timer] [--version]
                       destination

Creates a ROS2 package from templates

positional arguments:
  destination           Destination directory

options:
  -h, --help            show this help message and exit
  --defaults            Use defaults for all options
  --template {ros2_cpp_pkg}
                        Template
  --package_name PACKAGE_NAME
                        Package name
  --description DESCRIPTION
                        Description
  --maintainer MAINTAINER
                        Maintainer
  --maintainer-email MAINTAINER_EMAIL
                        Maintainer email
  --author AUTHOR       Author
  --author-email AUTHOR_EMAIL
                        Author email
  --license {Apache-2.0,BSL-1.0,BSD-2.0,BSD-2-Clause,BSD-3-Clause,GPL-3.0-only,LGPL-2.1-only,LGPL-3.0-only,MIT,MIT-0}
                        License
  --node-name NODE_NAME
                        Node name
  --node-class-name NODE_CLASS_NAME
                        Class name of node
  --is-component        Make it a component?
  --is-lifecycle        Make it a lifecycle node?
  --has-launch-file     Add a launch file?
  --launch-file-type {xml,py,yml}
                        Type of launch file
  --has-params          Add parameter loading
  --has-subscriber      Add a subscriber?
  --has-publisher       Add a publisher?
  --has-service-server  Add a service server?
  --has-action-server   Add an action server?
  --has-timer           Add a timer callback?
  --version             show program's version number and exit
```
