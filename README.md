# ros2-pkg-create

### Installation

```bash
pip install --extra-index-url https://test.pypi.org/simple/ ros2-pkg-create

# bash-completion
activate-global-python-argcomplete
eval "$(register-python-argcomplete ros2-pkg-create)"
```

### Usage

```
usage: ros2-pkg-create [-h] [--defaults] [--use-local-templates] --template {ros2_cpp_pkg,ros2_interfaces_pkg} [--package-name PACKAGE_NAME] [--description DESCRIPTION] [--maintainer MAINTAINER] [--maintainer-email MAINTAINER_EMAIL]
                       [--author AUTHOR] [--author-email AUTHOR_EMAIL] [--license {Apache-2.0,BSL-1.0,BSD-2.0,BSD-2-Clause,BSD-3-Clause,GPL-3.0-only,LGPL-2.1-only,LGPL-3.0-only,MIT,MIT-0}] [--node-name NODE_NAME]
                       [--node-class-name NODE_CLASS_NAME] [--is-component] [--no-is-component] [--is-lifecycle] [--no-is-lifecycle] [--has-launch-file] [--no-has-launch-file] [--launch-file-type {xml,py,yml}] [--has-params]
                       [--no-has-params] [--has-subscriber] [--no-has-subscriber] [--has-publisher] [--no-has-publisher] [--has-service-server] [--no-has-service-server] [--has-action-server] [--no-has-action-server] [--has-timer]
                       [--no-has-timer] [--auto-shutdown] [--no-auto-shutdown] [--interface-types {Message,Service,Action}] [--msg-name MSG_NAME] [--srv-name SRV_NAME] [--action-name ACTION_NAME] [--has-docker-ros] [--version]
                       destination

Creates a ROS 2 package from templates

positional arguments:
  destination           Destination directory

options:
  -h, --help            show this help message and exit
  --defaults            Use defaults for all options
  --use-local-templates
                        Use locally installed templates instead of remotely pulling most recent ones
  --template {ros2_cpp_pkg,ros2_interfaces_pkg}
                        Template
  --package-name PACKAGE_NAME
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
  --no-is-component
  --is-lifecycle        Make it a lifecycle node?
  --no-is-lifecycle
  --has-launch-file     Add a launch file?
  --no-has-launch-file
  --launch-file-type {xml,py,yml}
                        Type of launch file
  --has-params          Add parameter loading
  --no-has-params
  --has-subscriber      Add a subscriber?
  --no-has-subscriber
  --has-publisher       Add a publisher?
  --no-has-publisher
  --has-service-server  Add a service server?
  --no-has-service-server
  --has-action-server   Add an action server?
  --no-has-action-server
  --has-timer           Add a timer callback?
  --no-has-timer
  --auto-shutdown       Automatically shutdown the node after launch (useful in CI/CD)?
  --no-auto-shutdown
  --interface-types {Message,Service,Action}
                        Interfaces types
  --msg-name MSG_NAME   Message name
  --srv-name SRV_NAME   Service name
  --action-name ACTION_NAME
                        Action name
  --has-docker-ros      Add the docker-ros CI integration?
  --version             show program's version number and exit
```
